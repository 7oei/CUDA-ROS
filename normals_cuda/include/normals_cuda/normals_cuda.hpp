#ifndef NORMALS_CUDA_HPP
#define NORMALS_CUDA_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

// std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius);
// void CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
extern void compute_normals(std::vector<std::vector<float>> points_array,std::vector<std::vector<int>> neighbor_points_indices,std::vector<int> neighbor_start_indices,int neighbor_points_count,std::vector<std::vector<float>>& normals_array,std::vector<float>& curvatures_array,std::vector<long long int>& covariance_compute_time,std::vector<long long int>& eigen_compute_time);


#endif