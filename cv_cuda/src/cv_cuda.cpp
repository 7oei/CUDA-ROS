#include <ros/ros.h>
#include "cv_cuda/cv_cuda.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cv_cuda");
    ros::NodeHandle nh;

    ros::Rate rate(10.f);

    // 係数行列 A
    float dataA[][3] = { {1.0, -2.0, 3.0}, {3.0, 1.0, -5.0}, {-2.0, 6.0, -9.0} };
    // 行列 B
    float dataB[] = {1.0, -4.0, -2.0};
    cv::Mat matA(3, 3, CV_32FC1, dataA);
    cv::Mat matB(3, 1, CV_32FC1, dataB);
    cv::Mat matX(3, 1, CV_32FC1);

    // AX = B を解く. 両辺にAの逆行列を前掛けする
    matX = matA.inv()*matB;



    while (nh.ok())
    {
        cv::MatConstIterator_<float> iter = matX.begin<float>();
        while(iter != matX.end<float>()) printf("%.1f\n", *iter++);
        cvcallHello();
        rate.sleep();
    }
    return 0;
}