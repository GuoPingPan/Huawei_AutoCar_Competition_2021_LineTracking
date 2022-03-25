#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <vector>
using namespace std;
using namespace cv;

/** 参数定义 **/
const float fx = 760.4862674784594; // 相机内参
const float fy = 761.4971958529285;
const float cx = 631.6715834996345;
const float cy = 329.3054436037627;

const float y = 129.5585;   // 固定相机离地面高度，单位mm，通过计算得到，不一定准

float offset;
float thetaX;
float thetaY;
float scale;
int imgWidth;  // 图片大小
int imgHeight;
float up_rate;
float down_rate;
int sizeThres;

ros::Publisher pathpub;
ros::Publisher leftpoint;
ros::Publisher rightpoint;

/** 将图片中的 点uv 转化为 现实坐标中的 点xy **/
cv::Point3f getXYZ(cv::Point2f& point){
    cv::Point3f point3d;
    point3d.x = (fy * y/(point.y-cy))/scale;
    point3d.y = (-(point.x-cx)*point3d.x/fx);
    point3d.z = 0.0;
    // std::cout<<point3d<<std::endl;
    return point3d;
}

/** 将二维边线转化为三维边线 **/
void transTo3D(vector<cv::Point2f>& uv,vector<cv::Point3f>& posearray){
    for(auto p:uv){
        cv::Point3f pose = getXYZ(p);
        posearray.push_back(pose);
    }
}

/** 配合方法一、二：处理单边情况 **/
void pubMiddleline(vector<cv::Point2f>& uv,bool is_left){
    vector<cv::Point3f> xy;
    sensor_msgs::PointCloud path;
    transTo3D(uv,xy);
    // path.points.resize(xy.size());
    int i = 0;
    for(auto p:xy){
        if(is_left){
            geometry_msgs::Point32 point;
            point.x = p.x;
            point.y = p.y - offset;
            point.z = 0.0;
            path.points.push_back(point);
        }
        else{
            geometry_msgs::Point32 point;
            point.x = p.x;
            point.y = p.y + offset;
            point.z = 0.0;
            path.points.push_back(point);

        }

    }
    ROS_INFO("pathpose:%d",path.points.size());
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera";
    pathpub.publish(path);
    ROS_INFO("publish successfully");
}

/** 方法四：混合处理不分单双边线 **/
void pubMiddlelineCombine(vector<cv::Point2f>& uvLeft,vector<cv::Point2f>& uvRight){
    
    // 1.将数据转化为三维坐标系下
    vector<cv::Point3f> xyLeft,xyRight;
    ROS_INFO("int two middle");
    transTo3D(uvLeft,xyLeft);
    transTo3D(uvRight,xyRight);

    // 2.参数定义
    int r = 0;
    int l = 0;
    float max_distance = 0;
    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "camera";
    
    float lastparallel = 0; // 记录上一点的横向坐标
    bool init = 0;

    // 计算中线，只要有一边线的点都用完了就退出循环
    while (l<xyLeft.size() && r<xyRight.size())
    {
        // ROS_INFO("l: %d, r:%d",l,r);
        // 1.初始化
        if(!init){
            lastparallel =( xyLeft[l].y + xyRight[r].y )/2;
            init = 1;
            continue;
        }
        // 2.两边线纵向距离相等时
        if(xyLeft[l].x == xyRight[r].x){
            if(xyLeft[l].x > max_distance)
                max_distance = xyLeft[l].x;
            float tmp = ( xyLeft[l].y + xyRight[r].y )/2;
            if(abs(tmp - lastparallel)>0.1){// 与上一点横坐标的偏差
                l++;
                r++;
                continue;
            }
            geometry_msgs::Point32 p;
            p.x = xyLeft[l].x;
            p.y = tmp;
            p.z = 0.0;
            pc.points.push_back(p);
            l++;
            r++;
            lastparallel = tmp;
        }
        // 3.左边线纵向距离大于右边线
        else if(xyLeft[l].x > xyRight[r].x){
            if(xyLeft[l].x > max_distance)
                max_distance = xyLeft[l].x;
            float tmp = xyLeft[l].y - offset;
            // cout<<"LEFT:"<<tmp<<endl;

            if(abs(tmp - lastparallel)>0.1){
                l++;
                continue;
            }
            geometry_msgs::Point32 p;
            p.x = xyLeft[l].x;
            p.y = tmp;
            p.z = 0.0;
            pc.points.push_back(p);
            l++;
            lastparallel = tmp;
        }
        // 4.右边线纵向距离大于左边线
        else{
            if(xyRight[l].x > max_distance)
                max_distance = xyRight[l].x;
            // cout<<"RIGHT:"<<xyRight[r].y<<endl;
            float tmp = xyRight[r].y + offset;
            // cout<<"RIGHT:"<<tmp<<endl;
            if(abs(tmp - lastparallel)>0.1){
                r++;
                continue;
            }
            geometry_msgs::Point32 p;
            p.x = xyRight[r].x;
            p.y = tmp;
            p.z = 0.0;
            pc.points.push_back(p);
            r++;
            lastparallel = tmp;
        }

    }
    // ROS_INFO("OUT1");
    // 将另一边剩下的轨迹点加入
    // while (l<xyLeft.size())
    // {
    //     if(xyLeft[l].x > max_distance)
    //         max_distance = xyLeft[l].x;
    //     float tmp = xyLeft[l].y - offset;
        
    //     geometry_msgs::Point32 p;
    //     p.x = xyLeft[l].x;
    //     p.y = tmp;
    //     p.z = 0.0;
    //     pc.points.push_back(p);
    //     l++;
    // }
    
    // while (r<xyRight.size())
    // {
    //     if(xyRight[r].x > max_distance)
    //         max_distance = xyRight[r].x;
    //     // cout<<"RIGHT BEFORT:"<<xyRight[r].y<<endl;
    //     float tmp = xyRight[r].y + offset;
    //     // cout<<"RIGHT AFTER:"<<tmp<<endl;
    //     geometry_msgs::Point32 p;
    //     p.x = xyRight[r].x;
    //     p.y = tmp;
    //     p.z = 0.0;
    //     pc.points.push_back(p);
    //     r++;
    // }
    // ROS_INFO("OUT2");
    ROS_INFO("MAX DISTANCE %f",max_distance);

    pathpub.publish(pc);
}

bool humanline_detection(Mat& binary, Mat& RGB, int imgHeight, int imgWeight) {
    int jump = 1;
    int begin_imgHeight = imgHeight * 3.0 / 5;
    int end_imgHeight = imgHeight * 4.0 / 5;
    int end_imgWeight = imgWeight * 3.0 / 4;
    int all_point = 0;
    int white_point = 0;
    for (int x = begin_imgHeight; x < end_imgHeight; x+=jump) {
        for (int y = 0; y < end_imgWeight; y+=jump) {
            all_point++;
            if (RGB.at<Vec3b>(x, y)[0] > 230 && RGB.at<Vec3b>(x, y)[1] > 230 && RGB.at<Vec3b>(x,y)[2] > 230) {
                white_point++;
            }
        }
    }
    cout<< "rate:"<< (float)white_point / all_point<<"   ";
    //return (float)white_point / all_point;
    if ((float)white_point / all_point > 0.15)
        return true;
    return false;
}
void remove_white(Mat& binary, Mat& RGB, int imgHeight, int imgWeight) {
    int begin_imgHeight = (int)(imgHeight / 2.0);
    for (int x = begin_imgHeight; x < imgHeight; x++) {
        uchar* data = binary.ptr<uchar>(x);
        for (int y = 0; y < imgWeight; y++) {
            if (RGB.at<Vec3b>(x, y)[0] > 230 && RGB.at<Vec3b>(x, y)[1] > 230 && RGB.at<Vec3b>(x, y)[2] > 230) {
                data[y] = 0;
            }
        }
    }
    return;
}

int main(int argc ,char **argv){
    ros::init(argc,argv,"linktracking");
    ros::NodeHandle nh("~");
    pathpub = nh.advertise<sensor_msgs::PointCloud>("path",1);
    leftpoint = nh.advertise<sensor_msgs::PointCloud>("leftpath",1);
    rightpoint = nh.advertise<sensor_msgs::PointCloud>("rightpath",1);
    string video;
    nh.param<string>("video",video,"../dataset/test.mp4");
    nh.param<float>("offset",offset,0.45);
    nh.param<float>("thetaX",thetaX,0.2);
    nh.param<float>("thetaY",thetaY,0.1);
    nh.param<float>("scale",scale,1000.0);
    nh.param<int>("imgWidth",imgWidth,1280);
    nh.param<int>("imgHeight",imgHeight,720);
    nh.param<float>("up_rate",up_rate,0.5);
    nh.param<float>("down_rate",down_rate,0.9);
    nh.param<int>("size_thres",sizeThres,10);

    ROS_INFO("offset : %f",offset);
    ROS_INFO("thetaX : %f",thetaY);
    ROS_INFO("thetaY : %f",thetaX);
    ROS_INFO("scale : %f",scale);
    ROS_INFO("imgWidth : %d",imgWidth);
    ROS_INFO("imgHeight : %d",imgHeight);

    /** 1.获得视频数据 **/
    cv::VideoCapture cap = cv::VideoCapture(video);
    cap.set(3, imgWidth);//CV_CAP_PROP_FRAME_WIDTH
    cap.set(4, imgHeight);//CV_CAP_PROP_FRAME_HEIGHT
    cap.set(cv::CAP_PROP_FPS,30);

    ROS_INFO("%s",video.c_str());
    cv::Mat frame;
    cv::Mat gray;
    cv::Mat binary;
    bool is_find_human_line = true;

    if(!cap.isOpened()){
        ROS_INFO("Don't load the Image!");
        return 1;
    }

    int left_num=1,right_num=1;
    while (ros::ok()&&cap.isOpened()) {
        /** 2.获得帧并二值化 **/
        cap.read(frame);
        if(!frame.data){
            ROS_INFO("Don't get this the frame");
            continue;
        }
        

        
        // method 2:自适应二值化
        cv::cvtColor(frame, gray, 7);//CV_RGB2GRAY
        cv::threshold(gray, binary, 150, 255, 0);//CV_THRESH_BINARY
        if(!is_find_human_line)
            remove_white(binary,frame,imgHeight,imgWidth);
        cv::erode(binary, binary, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(binary,binary,11);
        if(is_find_human_line){
            if(humanline_detection(binary,frame,imgHeight,imgWidth)){
                is_find_human_line = false;
            }

        }

        /** 3.获得两边轨迹点 **/

        int vline = binary.cols / 2;
        //动态中线
        int base_vline = frame.cols / 2;
        int float_range = frame.cols / 4;
        float float_rate = ((long)left_num - right_num) / ((long)left_num + right_num + 0.1);
        vline = (int)(base_vline + float_range * float_rate);  

        vector<cv::Point2f> uvRight;
        vector<cv::Point2f> uvLeft;
        // right side
        for (int i = int(imgHeight*up_rate); i < int(imgHeight*down_rate)-1; ++i) {
            uchar *data = binary.ptr<uchar>(i);
            for (int j = vline; j < binary.cols - 1; ++j) {
                if (data[j] == 0 && data[j + 1] == 255) {
                    right_num++;
                    for (int tmp = 0; tmp < 10; tmp++) {
                        frame.at<Vec3b>(i, j - tmp)[0] = 0;
                        frame.at<Vec3b>(i, j - tmp)[1] = 0;
                        frame.at<Vec3b>(i, j - tmp)[2] = 255;
                        }
                    uvRight.push_back(cv::Point2f(j, i));
                    break;
                }
            }
       } 
        // left side
        for (int i = int(imgHeight*up_rate); i < int(imgHeight*down_rate)-1; ++i) {
            uchar *data = binary.ptr<uchar>(i);
            for (int j = vline; j > 0; --j) {
                if (data[j] == 0 && data[j - 1] == 255) {
                    left_num++;
                    for (int tmp = 0; tmp < 10; tmp++) {
                        frame.at<Vec3b>(i, j + tmp)[0] = 0;
                        frame.at<Vec3b>(i, j + tmp)[1] = 255;
                        frame.at<Vec3b>(i, j + tmp)[2] = 0;
                    }
                    uvLeft.push_back(cv::Point2f(j, i));
                    break;
                }
            }
        }
        //绘制中线
        for (int i = int(imgHeight * up_rate); i < int(imgHeight * down_rate) - 1; ++i) {
            for (int tmp = -1; tmp < 2; tmp++) {
                frame.at<Vec3b>(i, vline+tmp)[0] = 0;
                frame.at<Vec3b>(i, vline+tmp)[1] = 255;
                frame.at<Vec3b>(i, vline+tmp)[2] = 255;
            }

        }
        //绘制基本中线
        for (int i = int(imgHeight * up_rate); i < int(imgHeight * down_rate) - 1; ++i) {
            frame.at<Vec3b>(i, base_vline)[0] = 128;
            frame.at<Vec3b>(i, base_vline)[1] = 128;
            frame.at<Vec3b>(i, base_vline)[2] = 128;
        }

        cv::imshow("frame",binary);
        cv::waitKey(0);
        

        /** 4.绘制两边线 **/
        ROS_INFO("right.size = %d,left.size = %d",uvRight.size(),uvLeft.size());
    
        if(uvRight.size()>sizeThres && uvLeft.size()>sizeThres){
            pubMiddlelineCombine(uvLeft,uvRight);
            ROS_INFO("combine");
        }
        else if(uvRight.size()>=uvLeft.size()&&uvRight.size()>sizeThres){
            pubMiddleline(uvRight,0);
            ROS_INFO("right");
        }
        else if(uvLeft.size()>uvRight.size()&&uvLeft.size()>sizeThres){
            pubMiddleline(uvLeft,1);
            ROS_INFO("left");
        }
            

    }
    cv::destroyAllWindows();
    cap.release();
    return 0;
}
