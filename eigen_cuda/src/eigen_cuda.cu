#include <stdio.h>
#include "eigen_cuda/eigen_cuda.hpp"
#include <vector>

__global__ void covariance(float* neighbor_points,float* matrix,int point_size) {
    //平均計算
    float x_average=0,y_average=0,z_average=0;
    for(int i=0;i<point_size*3;i+=3){
        x_average+=neighbor_points[i];
        y_average+=neighbor_points[i+1];
        z_average+=neighbor_points[i+2];
    }
    x_average/=point_size;
    y_average/=point_size;
    z_average/=point_size;

    //要素計算
    float sxx=0,sxy=0,sxz=0,syy=0,syz=0,szz=0;
    for(int i=0;i<point_size*3;i+=3){
        sxx+=(neighbor_points[i]-x_average)*(neighbor_points[i]-x_average);
        syy+=(neighbor_points[i+1]-y_average)*(neighbor_points[i+1]-y_average);
        szz+=(neighbor_points[i+2]-z_average)*(neighbor_points[i+2]-z_average);

        sxy+=(neighbor_points[i]-x_average)*(neighbor_points[i+1]-y_average);
        sxz+=(neighbor_points[i]-x_average)*(neighbor_points[i+2]-z_average);
        syz+=(neighbor_points[i+1]-y_average)*(neighbor_points[i+2]-z_average);
    }
    sxx/=point_size;
    syy/=point_size;
    szz/=point_size;
    sxy/=point_size;
    sxz/=point_size;
    syz/=point_size;
    //出力
    matrix[0]=sxx;matrix[1]=sxy;matrix[2]=sxz;
    matrix[3]=sxy;matrix[4]=syy;matrix[5]=syz;
    matrix[6]=sxz;matrix[7]=syz;matrix[8]=szz;
    
}

extern void covarianceMatrix(std::vector<std::vector<float>> neighbor_points,float Matrix[3][3]){
    //変数宣言
    std::vector<float> h_neighbor_points(neighbor_points.size() * 3);
    std::vector<float> h_matrix(3 * 3);
    float *d_neighbor_points, *d_matrix;

    //メモリ確保
    cudaMalloc((void **)&d_neighbor_points, neighbor_points.size() * 3 * sizeof(float));
    cudaMalloc((void **)&d_matrix, 3 * 3 * sizeof(float));

    //配列化
    int k=0;
    for(int i=0;i<neighbor_points.size();i++){
        for(int j=0;j<3;j++){
            h_neighbor_points[k]=neighbor_points[i][j];
            k++;
        }
    }
    
    //コピー
    cudaMemcpy(d_neighbor_points, &h_neighbor_points[0], neighbor_points.size() * 3 * sizeof(float), cudaMemcpyHostToDevice);

    covariance<<<1, 1>>>(d_neighbor_points,d_matrix,neighbor_points.size());

    //配列にコピー
    cudaMemcpy(&h_matrix[0], d_matrix, 3 * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    //行列化
    k=0;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            Matrix[i][j]=h_matrix[k];
            k++;
        }
    }

    //メモリバラシ
    cudaFree(d_neighbor_points);
    cudaFree(d_matrix);
    
}