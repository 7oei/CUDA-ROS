#ifndef EIGEN_CUDA_HPP
#define EIGEN_CUDA_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"



extern void cvcallHello(void);

extern void covarianceMatrix(std::vector<std::vector<float>> neighbor_points,float Matrix[3][3]);

#endif