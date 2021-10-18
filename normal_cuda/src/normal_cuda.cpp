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
    N=25;

	pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
    normals->points.clear();
	normals->points.resize(N);

    while (nh.ok())
    {

        nh.getParam("/normal/normal_cuda/axis", axis);

        
        
        // std::vector<std::vector<float>> neighbor_points(N, std::vector<float>(3, 0));
        // std::random_device rnd;     // 非決定的な乱数生成器を生成
        // std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
        // std::uniform_int_distribution<> rand5(0, 500);

        // std::random_device rnd2;     // 非決定的な乱数生成器を生成
        // std::mt19937 mt2(rnd2());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
        // std::uniform_int_distribution<> rand1(0, 100);
        
        //デバッグ用
        std::vector<std::vector<float>> neighbor_points = {
            {-0.9282366037368774,-4.60659646987915,4.179419040679932},
            {-0.6398676633834839,-4.345795631408691,3.8638782501220703},
            {-0.3469808101654053,-4.115365028381348,3.5088584423065186},
            {-0.05463826656341553,-3.881274938583374,3.1585941314697266},
            {0.2328449785709381,-3.614520311355591,2.850792646408081},
            {-1.0035791397094727,-3.9636778831481934,3.8052620887756348},
            {-0.7170133590698242,-3.690755844116211,3.5054771900177},
            {-0.4283676743507385,-3.4318137168884277,3.1875195503234863},
            {-0.13146033883094788,-3.2284107208251953,2.7973649501800537},
            {0.15122388303279877,-2.929394483566284,2.531501293182373},
            {-1.0883535146713257,-3.2573564052581787,3.513524055480957},
            {-0.7981395721435547,-3.0089566707611084,3.181861639022827},
            {-0.5119532346725464,-2.733482599258423,2.88539457321167},
            {-0.21177172660827637,-2.5520896911621094,2.466628313064575},
            {0.08094996213912964,-2.3205485343933105,2.113051414489746},
            {-1.1616857051849365,-2.6279513835906982,3.1217992305755615},
            {-0.8680680990219116,-2.402432918548584,2.7603931427001953},
            {-0.5798090696334839,-2.140892267227173,2.4458136558532715},
            {-0.2915937602519989,-1.87905752658844,2.1316158771514893},
            {0.002897977828979492,-1.6594164371490479,1.7625699043273926},
            {-1.2396777868270874,-1.967220664024353,2.7707955837249756},
            {-0.9490185976028442,-1.7218148708343506,2.43524169921875},
            {-0.6570032835006714,-1.4855259656906128,2.087836503982544},
            {-0.36362549662590027,-1.2583959102630615,1.7285257577896118},
            {-0.0764693021774292,-0.9894416332244873,1.4235832691192627},
        };

        for(int i=0;i<N;i++){
            
            // if(axis==0){
            //     neighbor_points[i][0]=rand1(mt2)/100.0f;
            //     neighbor_points[i][1]=rand5(mt)/100.0f;
            //     neighbor_points[i][2]=rand5(mt)/100.0f;
            // }
            // if(axis==1){
            //     neighbor_points[i][0]=rand5(mt)/100.0f;
            //     neighbor_points[i][1]=rand1(mt2)/100.0f;
            //     neighbor_points[i][2]=rand5(mt)/100.0f;
            // }
            // if(axis==2){
            //     neighbor_points[i][0]=rand5(mt)/100.0f;
            //     neighbor_points[i][1]=rand5(mt)/100.0f;
            //     neighbor_points[i][2]=rand1(mt2)/100.0f;
            // }
            
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