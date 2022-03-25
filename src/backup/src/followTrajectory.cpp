#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "tf/transform_listener.h"
#include "Eigen/Dense"
#include "sensor_msgs/PointCloud.h"
using namespace std;

int main(int argc,char **argv){

    ros::init(argc,argv,"followTrajectory");
    ros::NodeHandle nh;
    string trajectory;
    string target_frame;
    string source_frame;
    ros::param::param<string>("trajectory",trajectory,"");
    ros::param::param<string>("target_frame",target_frame,"/laser");
    ros::param::param<string>("source_frame",source_frame,"/map");

    /** 1.读取点云信息 **/
    if(trajectory.size()==0){
        ROS_INFO("Can't load the trajectory!");
        return 1;
    }
    ifstream in(trajectory);
    vector<tf::Vector3> xy;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("trajectory",1);

    string s;
    while(!in.eof()){
        getline(in,s);
        cout<< s<<endl;
        if(!s.size())
            break;
        int pos = s.find(' ');
        string x = s.substr(2,pos);
        string y = s.substr(pos+3,s.length());
        float xv = x[0] == '-'? -atof(x.substr(1,x.length()).c_str()) : atof(x.c_str());
        float yv = y[0] == '-'? -atof(y.substr(1,y.length()).c_str()) : atof(y.c_str());
        tf::Vector3 p(xv,yv,0.0);
        cout<< p.getX()<<" "<<p.getY()<<" "<<p.getZ() << endl;
        xy.push_back(p);
        if(!in.good())
            break;
    }


    /** 2.将轨迹转到雷达坐标系下 **/
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Rate rate(10);
    while (ros::ok())
    {
        try{
        /**
         * @details 这里理解一个几个概念，source_frame,target_frame,parent_frame,child_frame
         *          source_frame 和 target_frame作用于坐标变换  parent_frame和child_frame作用
         *          要实现source_frame(假设为parent_frame)下的某个点转化到target_frame(则为child_frame)下的表示可以使用通过parent
        */
            listener.waitForTransform(target_frame,source_frame,ros::Time(0),ros::Duration(10.0));//wait for 10s
            listener.lookupTransform(target_frame, source_frame,ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            continue;
        }
        // double yaw = tf::getYaw(transform.getRotation());
        // double now_x = transform.getOrigin().getX();
        // double now_y = transform.getOrigin().getY();
        // transform.setOrigin(tf::Vector3(1,0,0));
        // tf::Quaternion q;
        // q.setRPY(0,0,3.14/2);
        // transform.setRotation(q);
        sensor_msgs::PointCloud pc;
        pc.header.stamp = ros::Time::now();
        pc.header.frame_id = target_frame;
        pc.points.resize(xy.size());
        int i = 0;
        for(auto point:xy){
            cout<< "origin:"<<point.getX()<<" "<<point.getY()<<" "<<point.getZ() << endl;
            tf::Vector3 point_trans = transform * point;
            cout<< "trans:"<< point_trans.getX()<<" "<<point_trans.getY()<<" "<<point_trans.getZ() << endl;
            pc.points[i].x = point_trans.getX();
            pc.points[i].y = point_trans.getY();
            pc.points[i].z = 0.0;
            ++i;
        }
        pub.publish(pc);
        rate.sleep();
    }
    

    return 0;
}