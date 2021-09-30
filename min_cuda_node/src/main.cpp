#include <ros/ros.h>
#include "min_cuda_node/hello.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "min_cuda");
    ros::NodeHandle nh;

    ros::Rate rate(10.f);
    while (nh.ok())
    {
        callHello();
        rate.sleep();
    }
    return 0;
}