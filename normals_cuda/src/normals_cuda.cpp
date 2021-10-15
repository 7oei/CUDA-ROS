#include <ros/ros.h>
#include "normals_cuda/normals_cuda.hpp"
#include <random>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

ros::Publisher normals_publisher;
ros::Subscriber cloud_subscriber;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius)
{
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if(kdtree.radiusSearch(searchpoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
	return pointIdxRadiusSearch; 
}


void CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// std::cout<<"1"<<std::endl;
	// Container for original & filtered data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
	pcl::PointCloud<pcl::PointXYZ>::Ptr remove_NaN_cloud {new pcl::PointCloud<pcl::PointXYZ>};
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals {new pcl::PointCloud<pcl::PointXYZINormal>};
	// Convert to PCL data type
	pcl::fromROSMsg(*cloud_msg, *cloud);

	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud, *remove_NaN_cloud, mapping);

	kdtree.setInputCloud(remove_NaN_cloud);
	normals->points.clear();
	normals->points.resize(remove_NaN_cloud->points.size());
    std::vector<std::vector<float>> points_array(remove_NaN_cloud->points.size(), std::vector<float>(3, 0));
    std::vector<std::vector<int>> neighbor_points_indices(remove_NaN_cloud->points.size());
    std::vector<int> neighbor_start_indices(remove_NaN_cloud->points.size());
    std::vector<std::vector<float>> normals_array(remove_NaN_cloud->points.size(), std::vector<float>(3));
	std::vector<float> curvatures_array(remove_NaN_cloud->points.size());
	int neighbor_points_count=0;
	// std::cout<<std::endl;
	// std::cout<<std::endl;
	// std::cout<<"2"<<std::endl;
	for(size_t i=0;i<remove_NaN_cloud->points.size();i++){//近傍点のindex取得のみ行う
		// std::cout<<"2.1"<<std::endl;
        /*Input point cloud conversion*/
        points_array[i][0]=remove_NaN_cloud->points[i].x;
        points_array[i][1]=remove_NaN_cloud->points[i].y;
        points_array[i][2]=remove_NaN_cloud->points[i].z;

		// std::cout<<"2.2"<<std::endl;
		/*search neighbor points*/
		double search_radius = 0.15;
		std::vector<int> indices = KdtreeSearch(remove_NaN_cloud->points[i], search_radius);
		// std::cout<<"2.3"<<std::endl;
        neighbor_start_indices[i]=neighbor_points_count;
        neighbor_points_indices[i].resize(indices.size());
		// std::cout<<"2.4"<<std::endl;
        for(size_t j=0;j<indices.size();j++){
            neighbor_points_indices[i][j]=indices[j];
            neighbor_points_count++;
        }

		//デバッグ用
        // points_array[i][0]=i*10+0;
        // points_array[i][1]=i*10+1;
        // points_array[i][2]=i*10+2;
		// neighbor_start_indices[i]=neighbor_points_count;
		// neighbor_points_indices[i].resize(20);
		// for(size_t j=0;j<20;j++){
        //     neighbor_points_indices[i][j]=i*100+j;
        //     neighbor_points_count++;
        // }

    }
	// std::cout<<"3"<<std::endl;
    compute_normals(points_array,neighbor_points_indices,neighbor_start_indices,neighbor_points_count,normals_array,curvatures_array);
	// std::cout<<"3.1"<<std::endl;
    for(size_t i=0;i<remove_NaN_cloud->points.size();i++){
		// if(i==0||i==10||i==20)std::cout<<"output normal ("<<i<<") = "<<normals_array[i][0]<<","<<normals_array[i][1]<<","<<normals_array[i][2]<<std::endl;
		normals->points[i].x = remove_NaN_cloud->points[i].x;
		normals->points[i].y = remove_NaN_cloud->points[i].y;
		normals->points[i].z = remove_NaN_cloud->points[i].z;
        normals->points[i].normal_x = normals_array[i][0];
		normals->points[i].normal_y = normals_array[i][1];
		normals->points[i].normal_z = normals_array[i][2];
		normals->points[i].curvature = curvatures_array[i];
		normals->points[i].intensity = 0;
		flipNormalTowardsViewpoint(remove_NaN_cloud->points[i], 0.0, 0.0, 0.0, normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);

		// if(normals->points[i].normal_x==0||normals->points[i].normal_y==0||normals->points[i].normal_z==0) std::cout<<normals_array[i][0]<<","<<normals_array[i][1]<<","<<normals_array[i][2];
    }
	// std::cout<<"cpp_normals : "<<normals_array[0][0]<<","<<normals_array[0][1]<<","<<normals_array[0][2]<<std::endl;
	// std::cout<<"4"<<std::endl;

	for(size_t i=0;i<normals->points.size();){
		if(std::isnan(normals->points[i].normal_x) || std::isnan(normals->points[i].normal_y) || std::isnan(normals->points[i].normal_z)){
			std::cout << "deleted NAN normal" << std::endl;
			normals->points.erase(normals->points.begin() + i);
		}
		else if(normals->points[i].normal_x==0 && normals->points[i].normal_y==0 && normals->points[i].normal_z==0){
			// std::cout << "deleted 0 normal" << std::endl;
			normals->points.erase(normals->points.begin() + i);
		}
		else	i++;
	}

	// Convert to ROS data type
	normals->header.stamp = cloud->header.stamp;
	normals->header.frame_id = cloud->header.frame_id;
	sensor_msgs::PointCloud2 normals_msg;
	pcl::toROSMsg(*normals, normals_msg);
	// std::cout<<"5"<<std::endl;
	// Publish the data
	normals_publisher.publish (normals_msg);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "normals_cuda");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  cloud_subscriber = nh.subscribe ("filtered_cloud", 1, CloudCallback);

  // Create a ROS publisher for the output point cloud
  normals_publisher = nh.advertise<sensor_msgs::PointCloud2> ("normals_cloud", 1);

  // Spin
  ros::spin ();
}
