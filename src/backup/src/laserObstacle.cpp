#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include <vector>

using namespace std;

float distThres;
int sizeThres;

float offset;
float disThres; 
float thetaX;
float thetaY;
// int sizeThres;
int sizeSum;


ros::Publisher stop_pub;
ros::Publisher filter_pub;
ros::Publisher laser_path;

std_msgs::Header header;


/** 方法四：混合处理不分单双边线 **/
void pubMiddlelineCombine(vector<cv::Point2f>& xyLeft,vector<cv::Point2f>& xyRight){

    // 1.参数定义
    int r = 0;
    int l = 0;
    float max_distance = 0;
    sensor_msgs::PointCloud pc;
    pc.header = header;
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

    laser_path.publish(pc);
}


/** 画出中间线 **/
void pubMiddleline(vector<cv::Point2f>& xy){
    sensor_msgs::PointCloud path;
    int i=0;
    path.points.resize(xy.size());
    for(auto p:xy){
        if(p.y>0){
            path.points[i].x = p.x;
            path.points[i].y = p.y-offset;
            path.points[i].z = 0.0;
        }
        else if(p.y<0){
            path.points[i].x = p.x;
            path.points[i].y = p.y+offset;
            path.points[i].z = 0.0;
        }
        ++i;
    }
    path.header = header;
    laser_path.publish(path);
}




/**
 * 1.将雷达数据进行分组
 * 2.将两边的雷达数据进行分类计算
 * 3.计算两边雷达数据最小y值之差
 *  
*/

void obstacleWithbridge(const sensor_msgs::LaserScan::ConstPtr& scan){
    ros::Time t1;
    int count = scan->scan_time / scan->time_increment;
    vector<cv::Point2f> xyLeft,xyRight;
    sensor_msgs::PointCloud pc;
    pc.header.frame_id = "laser";
    pc.header.stamp = ros::Time::now();

    for(int i = 0;i<count;++i){
        if(scan->ranges[i]>distThres)    // 距离阈值
            continue;
        float angle = scan->angle_min+scan->angle_increment*i;
        cv::Point2f point;
        point.x = scan->ranges[i]*cos(angle);
        point.y = scan->ranges[i]*sin(angle);
        if(point.y>0){
            xyLeft.push_back(point);
        }
        else{
            xyRight.push_back(point);
        }


        geometry_msgs::Point32 point3d;
        point3d.x = point.x;
        point3d.y = point.y;
        point3d.z = 0.0;

        pc.points.push_back(point3d);
    }

    // 具有两条边线的情况
    if(xyRight.size()>sizeThres&&xyLeft.size()>sizeThres){
        sort(xyLeft.begin(),xyLeft.end(),[](const cv::Point2f p1,const cv::Point2f p2){return p1.x>p2.x;});
        sort(xyRight.begin(),xyRight.end(),[](const cv::Point2f p1,const cv::Point2f p2){return p1.x>p2.x;});
        float l_min_y = xyLeft.begin()->y; 
        float r_min_y = xyRight.begin()->y;
        if(abs(l_min_y-r_min_y)<distThres){
            ROS_INFO("theres is a robot");
            std_msgs::Int32 stop;
            stop.data = 1;
            stop_pub.publish(stop);
        } 
    }
    else if(xyLeft.size()>sizeThres){
        ROS_INFO("theres is a robot");
        std_msgs::Int32 stop;
        stop.data = 1;
        stop_pub.publish(stop);
    }
    else if(xyRight.size()>sizeThres){
        ROS_INFO("theres is a robot");
        std_msgs::Int32 stop;
        stop.data = 1;
        stop_pub.publish(stop);
    }

}



void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    ros::Time t1;
    int count = scan->scan_time / scan->time_increment;
    vector<cv::Point2f> obstacle;
    sensor_msgs::PointCloud pc;
    pc.header.frame_id = "laser";
    pc.header.stamp = ros::Time::now();

    for(int i = 0;i<count;++i){
        float angle = scan->angle_min+scan->angle_increment*i;
        // if(scan->ranges[i]>distThres&&((angle>-M_PI&&angle<-M_PI/2)||(angle>M_PI/2&&angle<M_PI)))    // 距离阈值
            // continue;
        if(scan->ranges[i]<distThres&&angle>-M_PI/2&&angle<M_PI/2){

            cv::Point2f point;
            point.x = scan->ranges[i]*cos(angle);
            point.y = scan->ranges[i]*sin(angle);
            obstacle.push_back(point);

            geometry_msgs::Point32 point3d;
            point3d.x = point.x;
            point3d.y = point.y;
            point3d.z = 0.0;

            pc.points.push_back(point3d);


        }

    }

    pc.points.resize(int(obstacle.size()));
    filter_pub.publish(pc);

    if(int(obstacle.size())>sizeThres){
        std_msgs::Int32 stop;
        stop.data = 1;
        stop_pub.publish(stop);
    }
}

int main(int argc,char**argv){

    ros::init(argc,argv,"laserTracking");
    ros::NodeHandle nh("~");
    nh.param<float>("dist_thres",distThres,0.60);
    nh.param<int>("size_thres",sizeThres,10);

    ROS_INFO("dist_thres: %f",distThres);
    ROS_INFO("size_thres: %d",sizeThres);
    ros::Subscriber lasersub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1,laserCallback);
    stop_pub = nh.advertise<std_msgs::Int32>("stop",1);
    filter_pub = nh.advertise<sensor_msgs::PointCloud>("filter_laser",1);

    ros::spin();

    return 0;
}