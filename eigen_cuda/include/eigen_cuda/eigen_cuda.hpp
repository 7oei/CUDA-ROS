#ifndef EIGEN_CUDA_HPP
#define EIGEN_CUDA_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"



extern void covariance(std::vector<std::vector<float>> neighbor_points,float matrix[3][3]);

extern void eigen(std::vector<std::vector<float>> neighbor_points,float eigen_vector[3][3],float eigen_value[3]);

#endif