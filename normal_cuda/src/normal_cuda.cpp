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

    //デバッグ用
    // N=36;

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
        
        //デバッグ用
        // std::vector<std::vector<float>> neighbor_points = {
        //     {-0.125271, -0.075227, 0.391254},
        //     {-0.125300, -0.125161, 0.389752},
        //     {-0.075238, -0.025616, 0.395123},
        //     {-0.125316, 0.024117, 0.394987},
        //     {-0.025357, -0.075260, 0.395241},
        //     {-0.224845, -0.025579, 0.387149},
        //     {-0.075150, 0.024371, 0.396786},
        //     {-0.033952, 0.023055, 0.398692},
        //     {-0.224909, 0.024262, 0.389172},
        //     {-0.260687, -0.117740, 0.383412},
        //     {0.023270, -0.077907, 0.398169},
        //     {-0.011237, 0.026251, 0.400479},
        //     {-0.075386, 0.074074, 0.397057},
        //     {0.024520, -0.125290, 0.397083},
        //     {-0.041617, 0.066924, 0.398884},
        //     {-0.219205, 0.071003, 0.390864},
        //     {0.024475, -0.175013, 0.394711},
        //     {-0.020334, 0.076554, 0.400942},
        //     {-0.125117, 0.124003, 0.396700},
        //     {0.069579, -0.128150, 0.398390},
        //     {0.078003, -0.072593, 0.401183},
        //     {0.074208, -0.025597, 0.402705},
        //     {0.024586, 0.074312, 0.403370},
        //     {0.074258, -0.175122, 0.397047},
        //     {-0.024854, 0.124338, 0.401608},
        //     {-0.169145, -0.306355, 0.380836},
        //     {-0.125640, 0.173722, 0.398257},
        //     {0.024450, 0.124005, 0.404232},
        //     {0.124104, -0.075305, 0.403443},
        //     {-0.003000, -0.301000, 0.389000},
        //     {-0.161000, 0.180500, 0.400000},
        //     {-0.025388, 0.174106, 0.403322},
        //     {-0.133889, 0.206463, 0.398890},
        //     {-0.157786, 0.208808, 0.398591},
        //     {-0.154583, 0.210667, 0.400000},
        //     {0.124200, 0.074224, 0.408172},
        // };
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