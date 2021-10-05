#include <ros/ros.h>
#include "normal_cuda/normal_cuda.hpp"
#include <random>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "normal_cuda");
    ros::NodeHandle nh;

    ros::Rate rate(10.f);


    while (nh.ok())
    {
        //近傍点群の定義(20点くらい)(3*N)
        int N=20;
        std::vector<std::vector<float>> neighbor_points(N, std::vector<float>(3, 0));

        std::random_device rnd;     // 非決定的な乱数生成器を生成
        std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
        std::uniform_int_distribution<> rand5(0, 500);

        std::random_device rnd2;     // 非決定的な乱数生成器を生成
        std::mt19937 mt2(rnd2());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
        std::uniform_int_distribution<> rand1(0, 100);
        for(int i=0;i<N;i++){
            for(int j=0;j<3;j++){
                if(j<2){//x,y
                    neighbor_points[i][j]=rand5(mt)/100.0f;
                    // std::cout << neighbor_points[i][j] <<",";
                }
                else{//z
                    neighbor_points[i][j]=rand1(mt2)/100.0f;
                    // std::cout << neighbor_points[i][j];
                }
            }
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
        //返り値
        std::cout<<"normal_vector"<<std::endl;
        for(int i=0;i<3;i++){
            std::cout<<normal_vector[i]<<" , ";
        }
        std::cout<<std::endl;

        // cvcallHello();
        rate.sleep();
    }
    return 0;
}