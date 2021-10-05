#include <ros/ros.h>
#include "normal_cuda/normal_cuda.hpp"
#include <random>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "normal_cuda");
    ros::NodeHandle nh;

    ros::Publisher normal_publisher;
    normal_publisher = nh.advertise<sensor_msgs::PointCloud2> ("normal", 1);

    ros::Rate rate(10.f);

    //近傍点群の定義(20点くらい)(3*N)
    int N=20;
    int axis=0;//0=x,1=y,2=z
    int param_data=0;


	pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
    normals->points.clear();
	normals->points.resize(N);

    while (nh.ok())
    {

        nh.getParam("/normal/normal_cuda/axis", axis);

        std::vector<std::vector<float>> neighbor_points(N, std::vector<float>(3, 0));

        std::random_device rnd;     // 非決定的な乱数生成器を生成
        std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
        std::uniform_int_distribution<> rand5(0, 500);

        std::random_device rnd2;     // 非決定的な乱数生成器を生成
        std::mt19937 mt2(rnd2());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
        std::uniform_int_distribution<> rand1(0, 100);
        for(int i=0;i<N;i++){
            if(axis==0){
                neighbor_points[i][0]=rand1(mt2)/100.0f;
                neighbor_points[i][1]=rand5(mt)/100.0f;
                neighbor_points[i][2]=rand5(mt)/100.0f;
            }
            if(axis==1){
                neighbor_points[i][0]=rand5(mt)/100.0f;
                neighbor_points[i][1]=rand1(mt2)/100.0f;
                neighbor_points[i][2]=rand5(mt)/100.0f;
            }
            if(axis==2){
                neighbor_points[i][0]=rand5(mt)/100.0f;
                neighbor_points[i][1]=rand5(mt)/100.0f;
                neighbor_points[i][2]=rand1(mt2)/100.0f;
            }
            normals->points[i].x = neighbor_points[i][0];
            normals->points[i].y = neighbor_points[i][1];
            normals->points[i].z = neighbor_points[i][2];
            // std::cout<<std::endl;
        }
        //関数実行
        // float matrix[3][3];
        // covariance(neighbor_points,matrix);

        // float eigen_vector[3][3];
        // float eigen_value[3];
        // eigen(neighbor_points,eigen_vector,eigen_value);

        float normal_vector[3];
        normal(neighbor_points,normal_vector);
        for(int i=0;i<N;i++){
            normals->points[i].normal_x = normal_vector[0];
            normals->points[i].normal_y = normal_vector[1];
            normals->points[i].normal_z = normal_vector[2];
            normals->points[i].curvature = 0;
        }
        pcl_conversions::toPCL(ros::Time::now(),normals->header.stamp);
        normals->header.frame_id = "map"; 
        sensor_msgs::PointCloud2 normals_msg;
        pcl::toROSMsg(*normals, normals_msg);
        normal_publisher.publish (normals_msg);
        //返り値
        // std::cout<<"normal_vector"<<std::endl;
        // for(int i=0;i<3;i++){
        //     std::cout<<normal_vector[i]<<" , ";
        // }
        // std::cout<<std::endl;

        // cvcallHello();
        rate.sleep();
    }
    return 0;
}