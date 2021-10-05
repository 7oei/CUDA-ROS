#ifndef NORMAL_CUDA_HPP
#define NORMAL_CUDA_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

extern void covariance(std::vector<std::vector<float>> neighbor_points,float matrix[3][3]);

extern void eigen(std::vector<std::vector<float>> neighbor_points,float eigen_vector[3][3],float eigen_value[3]);

extern void normal(std::vector<std::vector<float>> neighbor_points,float normal_vector[3]);

#endif