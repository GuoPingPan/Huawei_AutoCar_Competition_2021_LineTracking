#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <fstream>
#include <string>


int main(int argc,char **argv){

    ros::init(argc,argv,"saveTractory");
    ros::NodeHandle nh;
    tf::TransformListener listener;
    std::ofstream fout("Tractory.txt");
    ros::Rate rate(10);
    std::string target_frame;
    std::string source_frame;
    ros::param::param<std::string>("target_frame",target_frame,"/map");
    ros::param::param<std::string>("source_frame",source_frame,"/laser");

    while (ros::ok()){
        tf::StampedTransform transform;
        try{
            /**
             * @details 这里理解一个几个概念，source_frame,target_frame,parent_frame,child_frame
             *          source_frame 和 target_frame作用于坐标变换  parent_frame和child_frame作用
             *          要实现source_frame(假设为parent_frame)下的某个点转化到target_frame(则为child_frame)下的表示可以使用通过parent
            */
            listener.waitForTransform(target_frame,source_frame,ros::Time(0),ros::Duration(10.0));//wait for 10s
            listener.lookupTransform(target_frame,source_frame,ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            continue;
        }
        fout<<"x: "<<transform.getOrigin().getX()<<" "<<"y: "<<transform.getOrigin().getY()<<std::endl;
        rate.sleep();
    }
    fout.close();

    return 0;
}