#include <stdio.h>
#include "normal_cuda/normal_cuda.hpp"
#include <vector>
#include <iostream>

__device__ int eigenJacobiMethod(float *a, float *v, int n, float eps = 1e-8, int iter_max = 100)
{
    float *bim, *bjm;
    float bii, bij, bjj, bji;
 
    bim = new float[n];
    bjm = new float[n];
 
    for(int i = 0; i < n; ++i){
        for(int j = 0; j < n; ++j){
            v[i*n+j] = (i == j) ? 1.0 : 0.0;
        }
    }
 
    int cnt = 0;
    for(;;){
        int i, j;
 
        float x = 0.0;
        for(int ia = 0; ia < n; ++ia){
            for(int ja = 0; ja < n; ++ja){
                int idx = ia*n+ja;
                if(ia != ja && fabs(a[idx]) > x){
                    i = ia;
                    j = ja;
                    x = fabs(a[idx]);
                }
            }
        }
 
        float aii = a[i*n+i];
        float ajj = a[j*n+j];
        float aij = a[i*n+j];
 
        float alpha, beta;
        alpha = (aii-ajj)/2.0;
        beta  = sqrt(alpha*alpha+aij*aij);
 
        float st, ct;
        ct = sqrt((1.0+fabs(alpha)/beta)/2.0);    // sinθ
        st = (((aii-ajj) >= 0.0) ? 1.0 : -1.0)*aij/(2.0*beta*ct);    // cosθ
 
        // A = PAPの計算
        for(int m = 0; m < n; ++m){
            if(m == i || m == j) continue;
 
            float aim = a[i*n+m];
            float ajm = a[j*n+m];
 
            bim[m] =  aim*ct+ajm*st;
            bjm[m] = -aim*st+ajm*ct;
        }
 
        bii = aii*ct*ct+2.0*aij*ct*st+ajj*st*st;
        bij = 0.0;
 
        bjj = aii*st*st-2.0*aij*ct*st+ajj*ct*ct;
        bji = 0.0;
 
        for(int m = 0; m < n; ++m){
            a[i*n+m] = a[m*n+i] = bim[m];
            a[j*n+m] = a[m*n+j] = bjm[m];
        }
        a[i*n+i] = bii;
        a[i*n+j] = bij;
        a[j*n+j] = bjj;
        a[j*n+i] = bji;
 
        // V = PVの計算
        for(int m = 0; m < n; ++m){
            float vmi = v[m*n+i];
            float vmj = v[m*n+j];
 
            bim[m] =  vmi*ct+vmj*st;
            bjm[m] = -vmi*st+vmj*ct;
        }
        for(int m = 0; m < n; ++m){
            v[m*n+i] = bim[m];
            v[m*n+j] = bjm[m];
        }
 
        float e = 0.0;
        for(int ja = 0; ja < n; ++ja){
            for(int ia = 0; ia < n; ++ia){
                if(ia != ja){
                    e += fabs(a[ja*n+ia]);
                }
            }
        }
        if(e < eps) break;
 
        cnt++;
        if(cnt > iter_max) break;
    }
 
    delete [] bim;
    delete [] bjm;
 
    return cnt;
} 


__global__ void covarianceGPU(float* neighbor_points,float* matrix,int point_size) {
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

__global__ void eigenGPU(float* neighbor_points,float* eigen_vector,float* eigen_value,int point_size) {
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
    //共分散行列
    float a[3*3]={
        sxx,sxy,sxz,
        sxy,syy,syz,
        sxz,syz,szz,
    };
    //固有値計算
    eigenJacobiMethod(a, eigen_vector, 3);
    eigen_value[0]=a[0];
    eigen_value[1]=a[4];
    eigen_value[2]=a[8];
    
}

__global__ void normalGPU(float* neighbor_points,float* normal_vecotr,int point_size) {
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
    //共分散行列
    float a[3*3]={
        sxx,sxy,sxz,
        sxy,syy,syz,
        sxz,syz,szz,
    };
    // printf("                          %f ,%f ,%f \ncovariance matrix     =   %f ,%f ,%f \n                          %f ,%f ,%f \n\n",sxx,sxy,sxz,sxy,syy,syz,sxz,syz,szz);
    //固有値計算
    float eigen_vector[3 * 3];
    eigenJacobiMethod(a, eigen_vector, 3);
    float eigen_value[3];
    eigen_value[0]=a[0];
    eigen_value[1]=a[4];
    eigen_value[2]=a[8];

    int min_eigen_axis=0;
    float min_eigen_value=eigen_value[0];
    for(int i=1;i<3;i++){
        if(eigen_value[i]<min_eigen_value){
            min_eigen_value=eigen_value[i];
            min_eigen_axis=i;
        }
    }
    normal_vecotr[0]=eigen_vector[min_eigen_axis*3+0];
    normal_vecotr[1]=eigen_vector[min_eigen_axis*3+1];
    normal_vecotr[2]=eigen_vector[min_eigen_axis*3+2];
    
    // printf("normals = %f, %f, %f\n\n\n\n",normal_vecotr[0],normal_vecotr[1],normal_vecotr[2]);
    
}


extern void covariance(std::vector<std::vector<float>> neighbor_points,float matrix[3][3]){
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

    covarianceGPU<<<1, 1>>>(d_neighbor_points,d_matrix,neighbor_points.size());

    //配列にコピー
    cudaMemcpy(&h_matrix[0], d_matrix, 3 * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    //行列化
    k=0;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            matrix[i][j]=h_matrix[k];
            k++;
        }
    }

    //メモリバラシ
    cudaFree(d_neighbor_points);
    cudaFree(d_matrix);
    
}

extern void eigen(std::vector<std::vector<float>> neighbor_points,float eigen_vector[3][3],float eigen_value[3]){
    //変数宣言
    std::vector<float> h_neighbor_points(neighbor_points.size() * 3);
    std::vector<float> h_eigen_vector(3 * 3);
    std::vector<float> h_eigen_value(3);
    float *d_neighbor_points, *d_eigen_vector, *d_eigen_value;

    //メモリ確保
    cudaMalloc((void **)&d_neighbor_points, neighbor_points.size() * 3 * sizeof(float));
    cudaMalloc((void **)&d_eigen_vector, 3 * 3 * sizeof(float));
    cudaMalloc((void **)&d_eigen_value, 3 * sizeof(float));

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

    eigenGPU<<<1, 1>>>(d_neighbor_points,d_eigen_vector,d_eigen_value,neighbor_points.size());

    //配列にコピー
    cudaMemcpy(&h_eigen_vector[0], d_eigen_vector, 3 * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_eigen_value[0], d_eigen_value, 3 * sizeof(float), cudaMemcpyDeviceToHost);

    //行列化
    k=0;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            eigen_vector[i][j]=h_eigen_vector[k];
            eigen_value[i]=h_eigen_value[i];
            k++;
        }
    }

    //メモリバラシ
    cudaFree(d_neighbor_points);
    cudaFree(d_eigen_vector);
    cudaFree(d_eigen_value);
    
}


extern void normal(std::vector<std::vector<float>> neighbor_points,float normal_vecotr[3]){
    //変数宣言
    std::vector<float> h_neighbor_points(neighbor_points.size() * 3);
    std::vector<float> h_normal_vector(3);
    float *d_neighbor_points, *d_normal_vecotr;

    //メモリ確保
    cudaMalloc((void **)&d_neighbor_points, neighbor_points.size() * 3 * sizeof(float));
    cudaMalloc((void **)&d_normal_vecotr, 3 * sizeof(float));

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

    normalGPU<<<1, 1>>>(d_neighbor_points,d_normal_vecotr,neighbor_points.size());

    //配列にコピー
    cudaMemcpy(&h_normal_vector[0], d_normal_vecotr, 3 * sizeof(float), cudaMemcpyDeviceToHost);

    //行列化
    for(int i=0;i<3;i++){
        normal_vecotr[i]=h_normal_vector[i];
    }

    //メモリバラシ
    cudaFree(d_neighbor_points);
    cudaFree(d_normal_vecotr);
    
}