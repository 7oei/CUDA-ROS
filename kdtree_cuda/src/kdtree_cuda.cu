

#include <string.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "kdtree_cuda/kdtree_cuda.hpp"
#include <vector>
#include <iterator>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>

int sort_axis=0;
int frames=0;
typedef struct
{
	int	id;
	float pos[3];
} point_with_id;

typedef struct
{
	int	parent_id;
	int left_id;
	int right_id;
	int axis;
} node;

typedef struct
{
	int i;
	bool ready;
} int_with_ready;

typedef struct
{
	bool ready;
	bool node_is_right;
	int parent_id;
	int left_id;
	int right_id;
	int depth;
	int axis;
	int middle;
	int group_size;
	int *x_sort_ids;
	int *y_sort_ids;
	int *z_sort_ids;
} detailed_node;

bool first=true;

//	年齢(昇順)
__host__ int AxisSort(const void * n1, const void * n2)
{
	if (((point_with_id *)n1)->pos[sort_axis] > ((point_with_id *)n2)->pos[sort_axis])
	{
		return 1;
	}
	else if (((point_with_id *)n1)->pos[sort_axis] < ((point_with_id *)n2)->pos[sort_axis])
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

__host__ int CreateTree(int* root_id,std::vector <node>& nodes, std::vector<std::vector<float>> points,std::vector<int> group_indices,int parent_id,bool node_is_right)
{
	//入力データ初期化
	int group_size = group_indices.size();
	// std::cout<<"group_size"<<group_size<<std::endl;
	point_with_id point_with_ids[group_size];
	std::vector<std::vector<int>> axis_sort_ids(3, std::vector<int>(group_size));
	// std::cout<<"oppai 1 "<<std::endl;
	for(int i=0;i<group_size;i++){/////////////////////////////////////////////////////////////////////////////////////////////////3*points
		point_with_ids[i].id=group_indices[i];
		point_with_ids[i].pos[0]=points[group_indices[i]][0];
		point_with_ids[i].pos[1]=points[group_indices[i]][1];
		point_with_ids[i].pos[2]=points[group_indices[i]][2];
	}
	// std::cout<<"oppai 2 "<<std::endl;

	//ソート
	float max[3],min[3],median[3],length[3];
	int axis_median_id[3];
	int median_id;
	for(sort_axis=0; sort_axis<3; sort_axis++){//x,y,zそれぞれにソート
		// std::cout<<"sort_axis = "<<sort_axis<<std::endl;
		qsort(point_with_ids, group_size, sizeof(point_with_id), AxisSort);
		for (int i=0 ; i < group_size ; i++){///////////////////////////////////////////////////////////////////////////////////////////3*points
			axis_sort_ids[sort_axis][i]=point_with_ids[i].id;
			// printf("%d, %f, %f, %f \n", point_with_ids[i].id, point_with_ids[i].pos[0], point_with_ids[i].pos[1], point_with_ids[i].pos[2]);
		}
		// std::cout<<std::endl;
		//max,min,median,axis_median_id取得
		max[sort_axis]=point_with_ids[group_size-1].pos[sort_axis];//minとmaxいらんかも
		min[sort_axis]=point_with_ids[0].pos[sort_axis];
		length[sort_axis]=max[sort_axis]-min[sort_axis];
		median[sort_axis]=point_with_ids[(group_size-1)/2].pos[sort_axis];//偶数なら小さい方(-1)消せば大きい方
		axis_median_id[sort_axis]=point_with_ids[(group_size-1)/2].id;
	}
	// std::cout<<"oppai 3 "<<std::endl;
	// std::cout<<"x_length = "<< length[0] <<", x_median["<<axis_median_id[0]<<"] = "<<median[0]<<std::endl;
	// std::cout<<"y_length = "<< length[1] <<", y_median["<<axis_median_id[1]<<"] = "<<median[1]<<std::endl;
	// std::cout<<"z_length = "<< length[2] <<", z_median["<<axis_median_id[2]<<"] = "<<median[2]<<std::endl;
	// std::cout<<std::endl;
	// for(int i=0;i<group_size;i++){
	// 	std::cout<<"point_id["<<i<<"] = "<<point_with_ids[i].id<<std::endl;
	// }
	

	//中央値id設定、長軸設定
	if(length[0]>=length[1]&&length[0]>=length[2]){
		median_id=axis_median_id[0];
		nodes[median_id].axis=0;
	}
	if(length[1]>=length[0]&&length[1]>=length[2]){
		median_id=axis_median_id[1];
		nodes[median_id].axis=1;
	}
	if(length[2]>=length[0]&&length[2]>=length[1]){
		median_id=axis_median_id[2];
		nodes[median_id].axis=2;
	}
	// std::cout<<"oppai 4 "<<std::endl;

	for(int i=0;i<group_size;i++){/////////////////////////////////////////////////////////////////////////////////////////////////points
		group_indices[i]=axis_sort_ids[nodes[median_id].axis][i];
	}
	// std::cout<<"oppai 5 "<<std::endl;
	// memcpy(&group_indices[0], axis_sort_ids[nodes[median_id].axis], group_size*sizeof(int));
	// std::vector<int> group_indices2;
	// group_indices.resize(0);
	// copy(axis_sort_ids[nodes[median_id].axis].begin(), axis_sort_ids[nodes[median_id].axis].end(), back_inserter(group_indices) );



	//node初期化
	nodes[median_id].left_id=-1;
	nodes[median_id].right_id=-1;

	//親設定、親の左右設定
	nodes[median_id].parent_id=parent_id;
	if(parent_id>=0){//親あり
		if(!node_is_right) nodes[parent_id].left_id=median_id;
		if(node_is_right) nodes[parent_id].right_id=median_id;
	}
	else{//親なし
		*root_id=median_id;
	}
	// std::cout<<"oppai 6 "<<std::endl;

	// std::vector<int> right_group(group_size);
	// std::vector<int> left_group(group_size);
	// int right_count=0;
	// int left_count=0;

	// for(int i=0;i<=((group_size-1)/2)-1;i++){////////////////////////////////////////////////////////////////////////////////////points
	// 	left_group[left_count] = axis_sort_ids[nodes[median_id].axis][i];
	// 	left_count++;
	// }
	// left_group.resize(left_count);///////////////////////////////////////////////////////////////////////////////////////////////points
	// for(int i=((group_size-1)/2)+1;i<group_size;i++){
	// 	right_group[right_count] = axis_sort_ids[nodes[median_id].axis][i];
	// 	right_count++;
	// }
	// right_group.resize(right_count);

	// std::vector<int> v0{1,2,3,4};
	// std::vector<int> v9(v0.begin(),v0.end());  // [1,2,3,4]
	// std::vector<int> v10(v0.begin(),v0.end());  

	// group_indices.resize(8);
	// for(int i=0;i<8;i++){
	// 	group_indices[i]=i;
	// }
	// size_t middle = ((8-1)/2);

	size_t middle = ((group_size-1)/2);
	std::vector<int>::iterator middleIter(group_indices.begin());
	std::advance(middleIter, middle);
	// std::cout<<"advance end"<<std::endl;

	std::vector<int> left_group(group_indices.begin(), middleIter);
	++middleIter;
	std::vector<int> right_group(middleIter, group_indices.end());
	// std::cout<<"oppai 7 "<<std::endl;
	// std::cout<<"group end"<<std::endl;
	// std::cout<<"left group is [";
	// for(int i=0;i<left_group.size();i++){
	// 	if(i<10) std::cout<<left_group[i]<<",";
	// }
	// std::cout<<"]"<<std::endl;

	// std::cout<<"right group is [";
	// for(int i=0;i<right_group.size();i++){
	// 	if(i<10) std::cout<<right_group[i]<<",";
	// }
	// std::cout<<"]"<<std::endl;

	//


	// std::cout<<"median_id"<<median_id<<std::endl;
	// std::cout<<"parent_id"<<parent_id<<std::endl;
	// std::cout<<"left_id"<<nodes[median_id].left_id<<std::endl;
	// std::cout<<"right_id"<<nodes[median_id].right_id<<std::endl;
	// std::cout<<"axis"<<nodes[median_id].axis<<std::endl;

	//right,left group表示
	// std::cout<<"left_group is (";
	// for(int i=0;i<left_group.size();i++){
	// 	std::cout<<left_group[i]<<",";
	// }
	// std::cout<<")"<<std::endl;
	// std::cout<<"right_group is (";
	// for(int i=0;i<right_group.size();i++){
	// 	std::cout<<right_group[i]<<",";
	// }
	// std::cout<<")"<<std::endl;
	// std::cout<<std::endl;
	// std::cout<<std::endl;
	// std::cout<<std::endl;
	// std::cout<<"--------------------------------------------------------------------------------"<<std::endl;
	bool left=false;
	bool right=false;
	if(group_size>1){//子がいる
		if(left_group.size()>0){//左に子がいる
			left= CreateTree(root_id,nodes,points,left_group,median_id,false);
		}
		else left=true;
		if(right_group.size()>0){//右に子がいる
			right= CreateTree(root_id,nodes,points,right_group,median_id,true);
		}
		else right=true;
		// std::cout<<"oppai 8 "<<std::endl;
		if(right&&left) return 1;
	}
	else return 1;//子がいない
}

__host__ int CreateNode(int* root_id,int point_size,std::vector <node>& nodes, std::vector<std::vector<int>> axis_sort_ids,int depth,int parent_id,bool node_is_right)
{

	// std::cout << "oppai 0" << std::endl;
	// std::cout << std::endl;
	int group_size = axis_sort_ids[0].size();
	int axis = depth % 3;
	size_t middle = ((group_size-1)/2);
	int median_id = axis_sort_ids[axis][middle];
	nodes[median_id].axis = axis;
	nodes[median_id].parent_id = parent_id;
	nodes[median_id].left_id = -1;
	nodes[median_id].right_id = -1;
	if(parent_id >= 0){ // 親あり
		if(!node_is_right) nodes[parent_id].left_id = median_id;
		if(node_is_right) nodes[parent_id].right_id = median_id;
	}
	else{ // 親なし
		*root_id = median_id;
	}
	// std::cout << "oppai 4" << std::endl;
	// std::cout<<std::endl;
	// std::cout<<std::endl;
	// std::cout<<std::endl;
	// std::cout << "axis_sort_ids ="<<std::endl;
	// for(int j = 0; j < 3; j++){
	// 	if(j==0) std::cout << "x =";
	// 	if(j==1) std::cout << "y =";
	// 	if(j==2) std::cout << "z =";
	// 	for(int i = 0; i < group_size; i++){
	// 		std::cout << axis_sort_ids[j][i] << ",";
	// 	}
	// 	std::cout<<std::endl;
	// }

	if(group_size > 1){ // 子あり
		std::vector<int>::iterator middle_iter(axis_sort_ids[axis].begin());
		std::advance(middle_iter,middle);
		std::vector<int> left_group(axis_sort_ids[axis].begin(),middle_iter);
		++middle_iter;
		std::vector<int> right_group(middle_iter,axis_sort_ids[axis].end());

		// std::cout<<"median_id"<<median_id<<std::endl;
		// std::cout<<"middle"<<middle<<std::endl;
		// std::cout<<"axis"<<nodes[median_id].axis<<std::endl;
		// std::cout<<"group is (";
		// for(int i=0;i<group_size;i++){
		// 	std::cout<<axis_sort_ids[axis][i]<<",";
		// }
		// std::cout<<")"<<std::endl;
		// std::cout<<"left_group is (";
		// for(int i=0;i<left_group.size();i++){
		// 	std::cout<<left_group[i]<<",";
		// }
		// std::cout<<")"<<std::endl;
		// std::cout<<"right_group is (";
		// for(int i=0;i<right_group.size();i++){
		// 	std::cout<<right_group[i]<<",";
		// }
		// std::cout<<")"<<std::endl;

		// std::cout << "oppai 1" << std::endl;

		std::vector<std::vector<int>> left_axis_sort_ids(3,std::vector<int>(left_group.size()));
		std::vector<std::vector<int>> right_axis_sort_ids(3,std::vector<int>(right_group.size()));

		std::vector<int> next_group(point_size,0);/////////////これどうにかしたい
		std::vector<int> left_axis_count(3,0);
		std::vector<int> right_axis_count(3,0);
		// std::cout << "oppai 1.5" << std::endl;
		// std::cout << "next_group.size()" << next_group.size() <<std::endl;
		// std::cout << "left_group.size()" << left_group.size() <<std::endl;
		// std::cout << "right_group.size()" << right_group.size() <<std::endl;
		for(int i = 0; i < left_group.size(); i++){
			// std::cout << "oppai 1.51" << std::endl;
			left_axis_sort_ids[axis][i] = left_group[i];
			// std::cout << "oppai 1.52" << std::endl;
			// std::cout << "left_group[i]" << left_group[i] <<std::endl;
			next_group[left_group[i]] = -1;//これで死んでそう//left_group[i]がnext_groupのレンジを超えている//この式の参照indexおかしい//1段目では正しく作用
		}
		// std::cout << "oppai 1.6" << std::endl;
		for(int i = 0; i < right_group.size(); i++){
			right_axis_sort_ids[axis][i] = right_group[i];
			// std::cout << "right_group[i]" << right_group[i] <<std::endl;
			next_group[right_group[i]] = 1;
		}
		// std::cout << "oppai 2" << std::endl;
		for(int i = 0; i < group_size; i++){
			for(int j = 0; j < 3; j++){
				if(j==axis) continue;
				if(next_group[axis_sort_ids[j][i]] == -1){
					left_axis_sort_ids[j][left_axis_count[j]] = axis_sort_ids[j][i];
					left_axis_count[j]++;
					// std::cout << "left_axis_count["<<j<<"] = "<<left_axis_count[j]<<std::endl;
				}
				else if(next_group[axis_sort_ids[j][i]] == 1){
					right_axis_sort_ids[j][right_axis_count[j]] = axis_sort_ids[j][i];
					right_axis_count[j]++;
					// std::cout << "right_axis_count["<<j<<"] = "<<right_axis_count[j]<<std::endl;
				}
			}
		}

		bool left = false;
		bool right = false;
		if(left_group.size() > 0) left = CreateNode(root_id,point_size,nodes,left_axis_sort_ids,depth+1,median_id,false);
		else left = true;

		if(right_group.size() > 0) right = CreateNode(root_id,point_size,nodes,right_axis_sort_ids,depth+1,median_id,true);
		else right = true;

		if(right&&left) return 1;
	}
	else return 1;
}

__host__ int CreateNode2(int* root_id,int point_size,std::vector <node>& nodes, std::vector<std::vector<int>> axis_sort_ids,int depth,int parent_id,bool node_is_right)
{

	// std::cout << "oppai 0" << std::endl;
	// std::cout << std::endl;
	int group_size = axis_sort_ids[0].size();
	int axis = depth % 3;
	size_t middle = ((group_size-1)/2);
	int median_id = axis_sort_ids[axis][middle];
	// std::cout<<"layer["<<depth<<"] median_id is"<<median_id<<std::endl;
	// std::ofstream ofs1("/home/adachi/cpu_tree.csv",std::ios::app);
    // ofs1 << depth << "," << median_id << ","<<std::endl;
	//depth,median_id
	nodes[median_id].axis = axis;
	nodes[median_id].parent_id = parent_id;
	nodes[median_id].left_id = -1;
	nodes[median_id].right_id = -1;
	if(parent_id >= 0){ // 親あり
		if(!node_is_right) nodes[parent_id].left_id = median_id;
		if(node_is_right) nodes[parent_id].right_id = median_id;
	}
	else{ // 親なし
		*root_id = median_id;
	}
	// std::cout << "oppai 4" << std::endl;
	// std::cout<<std::endl;
	// std::cout<<std::endl;
	// std::cout<<std::endl;
	// std::cout << "axis_sort_ids ="<<std::endl;
	// for(int j = 0; j < 3; j++){
	// 	if(j==0) std::cout << "x =";
	// 	if(j==1) std::cout << "y =";
	// 	if(j==2) std::cout << "z =";
	// 	for(int i = 0; i < group_size; i++){
	// 		std::cout << axis_sort_ids[j][i] << ",";
	// 	}
	// 	std::cout<<std::endl;
	// }

	if(group_size > 1){ // 子あり
		std::vector<int>::iterator middle_iter(axis_sort_ids[axis].begin());
		std::advance(middle_iter,middle);
		std::vector<int> left_group(axis_sort_ids[axis].begin(),middle_iter);
		++middle_iter;
		std::vector<int> right_group(middle_iter,axis_sort_ids[axis].end());

		// std::cout<<"median_id"<<median_id<<std::endl;
		// std::cout<<"middle"<<middle<<std::endl;
		// std::cout<<"axis"<<nodes[median_id].axis<<std::endl;
		// std::cout<<"group is (";
		// for(int i=0;i<group_size;i++){
		// 	std::cout<<axis_sort_ids[axis][i]<<",";
		// }
		// std::cout<<")"<<std::endl;
		// std::cout<<"left_group is (";
		// for(int i=0;i<left_group.size();i++){
		// 	std::cout<<left_group[i]<<",";
		// }
		// std::cout<<")"<<std::endl;
		// std::cout<<"right_group is (";
		// for(int i=0;i<right_group.size();i++){
		// 	std::cout<<right_group[i]<<",";
		// }
		// std::cout<<")"<<std::endl;

		// std::cout << "oppai 1" << std::endl;

		std::vector<std::vector<int>> left_axis_sort_ids(3,std::vector<int>(left_group.size()));
		std::vector<std::vector<int>> right_axis_sort_ids(3,std::vector<int>(right_group.size()));

		std::vector<int> next_group(point_size,0);/////////////これどうにかしたい
		std::vector<int> left_axis_count(3,0);
		std::vector<int> right_axis_count(3,0);
		// std::cout << "oppai 1.5" << std::endl;
		// std::cout << "next_group.size()" << next_group.size() <<std::endl;
		// std::cout << "left_group.size()" << left_group.size() <<std::endl;
		// std::cout << "right_group.size()" << right_group.size() <<std::endl;
		for(int i = 0; i < left_group.size(); i++){
			// std::cout << "oppai 1.51" << std::endl;
			left_axis_sort_ids[axis][i] = left_group[i];
			// std::cout << "oppai 1.52" << std::endl;
			// std::cout << "left_group[i]" << left_group[i] <<std::endl;
			next_group[left_group[i]] = -1;//これで死んでそう//left_group[i]がnext_groupのレンジを超えている//この式の参照indexおかしい//1段目では正しく作用
		}
		// std::cout << "oppai 1.6" << std::endl;
		for(int i = 0; i < right_group.size(); i++){
			right_axis_sort_ids[axis][i] = right_group[i];
			// std::cout << "right_group[i]" << right_group[i] <<std::endl;
			next_group[right_group[i]] = 1;
		}
		// std::cout << "oppai 2" << std::endl;
		for(int i = 0; i < group_size; i++){
			for(int j = 0; j < 3; j++){
				if(j==axis) continue;
				if(next_group[axis_sort_ids[j][i]] == -1){
					left_axis_sort_ids[j][left_axis_count[j]] = axis_sort_ids[j][i];
					left_axis_count[j]++;
					// std::cout << "left_axis_count["<<j<<"] = "<<left_axis_count[j]<<std::endl;
				}
				else if(next_group[axis_sort_ids[j][i]] == 1){
					right_axis_sort_ids[j][right_axis_count[j]] = axis_sort_ids[j][i];
					right_axis_count[j]++;
					// std::cout << "right_axis_count["<<j<<"] = "<<right_axis_count[j]<<std::endl;
				}
			}
		}

		bool left = false;
		bool right = false;
		if(left_group.size() > 0) left = CreateNode2(root_id,point_size,nodes,left_axis_sort_ids,depth+1,median_id,false);
		else left = true;

		if(right_group.size() > 0) right = CreateNode2(root_id,point_size,nodes,right_axis_sort_ids,depth+1,median_id,true);
		else right = true;

		if(right&&left) return 1;
	}
	else return 1;
}

__host__ void TreeOutCsv(std::vector<detailed_node> nodes,int depth,int median_id)
{
	std::ofstream ofs1("/home/adachi/gpu_low_warp_divergence_tree.csv",std::ios::app);
    ofs1 << depth << "," << median_id << ","<<std::endl;
	if(nodes[median_id].left_id>=0) TreeOutCsv(nodes,depth+1,nodes[median_id].left_id);
	if(nodes[median_id].right_id>=0) TreeOutCsv(nodes,depth+1,nodes[median_id].right_id);
}

__global__ void d_CreateNode(int point_size,int group_size,int depth,int parent_id,bool node_is_right,int *x_sort_ids,int *y_sort_ids,int *z_sort_ids,int *root_id,node* nodes)
{

	// printf("create node open\n");
	unsigned int ix = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int idx = ix;
	// printf("idx = %d, ",idx);
	// printf("group size = %d\n",group_size);
	int axis = depth % 3;
	size_t middle = ((group_size-1)/2);
	int median_id;
	if(axis==0) median_id = x_sort_ids[middle];
	if(axis==1) median_id = y_sort_ids[middle];
	if(axis==2) median_id = z_sort_ids[middle];

	// if(median_id==1||median_id==13||median_id==14||median_id==15||median_id==19||median_id==36||median_id==45){//ここでは正しい
	// 	printf("1 x_sort_ids[] = ");
	// 	for(int i=0;i<group_size;i++){
	// 		printf("%d,",x_sort_ids[i]);
	// 	}
	// 	printf("\n");
	// }

	int *copy_x_sort_ids,*copy_y_sort_ids,*copy_z_sort_ids;
	copy_x_sort_ids = (int *)malloc(group_size * sizeof(int));
	copy_y_sort_ids = (int *)malloc(group_size * sizeof(int));
	copy_z_sort_ids = (int *)malloc(group_size * sizeof(int));
	memcpy(copy_x_sort_ids, x_sort_ids, group_size * sizeof(int));
	memcpy(copy_y_sort_ids, y_sort_ids, group_size * sizeof(int));
	memcpy(copy_z_sort_ids, z_sort_ids, group_size * sizeof(int));

	nodes[median_id].axis = axis;
	nodes[median_id].parent_id = parent_id;
	nodes[median_id].left_id = -1;
	nodes[median_id].right_id = -1;
	// if(median_id==1||median_id==13||median_id==14||median_id==15||median_id==19||median_id==36||median_id==45){//ここでは正しい
	// 	printf("2 copy_x_sort_ids[] = ");
	// 	for(int i=0;i<group_size;i++){
	// 		printf("%d,",copy_x_sort_ids[i]);
	// 	}
	// 	printf("\n");
	// }

	// printf("1");
	if(parent_id >= 0){ // 親あり
		free(x_sort_ids);
		free(y_sort_ids);
		free(z_sort_ids);
		if(!node_is_right) nodes[parent_id].left_id = median_id;
		if(node_is_right) nodes[parent_id].right_id = median_id;
	}
	else{ // 親なし
		printf("root update\n");
		*root_id = median_id;
	}
	// printf("2");
	if(group_size > 1){ // 子あり
		int left_group_size = 0;
		int right_group_size = 0;
		int *left_x_sort_ids,*left_y_sort_ids,*left_z_sort_ids;
		int *right_x_sort_ids,*right_y_sort_ids,*right_z_sort_ids;
		int *next_group;
		next_group = (int *)malloc(point_size * sizeof(int));
		left_x_sort_ids = (int *)malloc(middle * sizeof(int));
		left_y_sort_ids = (int *)malloc(middle * sizeof(int));
		left_z_sort_ids = (int *)malloc(middle * sizeof(int));
		right_x_sort_ids = (int *)malloc((group_size - (middle + 1)) * sizeof(int));
		right_y_sort_ids = (int *)malloc((group_size - (middle + 1)) * sizeof(int));
		right_z_sort_ids = (int *)malloc((group_size - (middle + 1)) * sizeof(int));
		int left_axis_count[3]={0,0,0};
		int right_axis_count[3]={0,0,0};
		// printf("\n\n\n");
		// printf("median_id = %d\n",median_id);
		// if(!node_is_right) printf("node is left\n");
		// else printf("node is right\n");
		// printf("parent_id = %d\n",parent_id);
		// printf("middle = %d\n",middle);
		// printf("axis = %d\n",nodes[median_id].axis);

		// printf("3");
		// printf("axis = %d",axis);
		// if(median_id==1||median_id==14||median_id==19||median_id==36||median_id==45){//ここでは正しい
		// 	printf("3 copy_x_sort_ids[] = ");
		// 	for(int i=0;i<group_size;i++){
		// 		printf("%d,",copy_x_sort_ids[i]);
		// 	}
		// 	printf("\n");
		// }


		if(axis==0){
			// printf("for start");
			for(int i = 0; i < group_size; i++){
				if(point_size<copy_x_sort_ids[i]) printf("out of range copy_x_sort_ids[%d] = %d \n",i,copy_x_sort_ids[i]);
				if(point_size<copy_y_sort_ids[i]) printf("out of range copy_y_sort_ids[%d] = %d \n",i,copy_y_sort_ids[i]);
				if(point_size<copy_z_sort_ids[i]) printf("out of range copy_z_sort_ids[%d] = %d \n",i,copy_z_sort_ids[i]);
				// printf("i = %d ",i);
				// printf("middle = %d ",middle);
				if(i<middle){
					left_x_sort_ids[left_axis_count[0]] = copy_x_sort_ids[i];
					// printf("3.01 ");
					left_axis_count[0]++;
					// printf("3.02 ");
					// printf("parent_id = %d",parent_id);
					// printf("copy_x_sort_ids[%d] = %d ",i,copy_x_sort_ids[i]);
					next_group[copy_x_sort_ids[i]] = -1;
					// printf("next_group[%d] = %d ",copy_x_sort_ids[i],next_group[copy_x_sort_ids[i]]);
					// printf("3.1 ");
				}
				else if(i>middle){
					right_x_sort_ids[right_axis_count[0]] = copy_x_sort_ids[i];
					right_axis_count[0]++;
					next_group[copy_x_sort_ids[i]] = 1;
					// printf("3.3");
				}
				else{
					next_group[copy_x_sort_ids[i]] = 0;
					// printf("3.2");
				}
			}
		}
		else if(axis==1){
			// printf("for start");
			for(int i = 0; i < group_size; i++){
				if(point_size<copy_x_sort_ids[i]) printf("out of range copy_x_sort_ids[%d] = %d \n",i,copy_x_sort_ids[i]);
				if(point_size<copy_y_sort_ids[i]) printf("out of range copy_y_sort_ids[%d] = %d \n",i,copy_y_sort_ids[i]);
				if(point_size<copy_z_sort_ids[i]) printf("out of range copy_z_sort_ids[%d] = %d \n",i,copy_z_sort_ids[i]);
				// printf("i = %d ",i);
				// printf("middle = %d ",middle);
				if(i<middle){
					left_y_sort_ids[left_axis_count[1]] = copy_y_sort_ids[i];
					// printf("3.01 ");
					left_axis_count[1]++;
					// printf("3.02 ");
					// printf("copy_y_sort_ids[%d] = %d ",i,copy_y_sort_ids[i]);
					next_group[copy_y_sort_ids[i]] = -1;
					// printf("next_group[%d] = %d ",copy_y_sort_ids[i],next_group[copy_y_sort_ids[i]]);
					// printf("3.1 ");
				}
				else if(i>middle){
					right_y_sort_ids[right_axis_count[1]] = copy_y_sort_ids[i];
					right_axis_count[1]++;
					next_group[copy_y_sort_ids[i]] = 1;
					// printf("3.3");
				}
				else{
					next_group[copy_y_sort_ids[i]] = 0;
					// printf("3.2");
				}
			}

		}
		else if(axis==2){
			// printf("for start");
			for(int i = 0; i < group_size; i++){
				if(point_size<copy_x_sort_ids[i]) printf("out of range copy_x_sort_ids[%d] = %d \n",i,copy_x_sort_ids[i]);
				if(point_size<copy_y_sort_ids[i]) printf("out of range copy_y_sort_ids[%d] = %d \n",i,copy_y_sort_ids[i]);
				if(point_size<copy_z_sort_ids[i]) printf("out of range copy_z_sort_ids[%d] = %d \n",i,copy_z_sort_ids[i]);
				// printf("i = %d ",i);
				// printf("middle = %d ",middle);
				if(i<middle){
					left_z_sort_ids[left_axis_count[2]] = copy_z_sort_ids[i];
					// printf("3.01 ");
					left_axis_count[2]++;
					// printf("3.02 ");
					// printf("copy_z_sort_ids[%d] = %d ",i,copy_z_sort_ids[i]);
					next_group[copy_z_sort_ids[i]] = -1;
					// printf("next_group[%d] = %d ",copy_z_sort_ids[i],next_group[copy_z_sort_ids[i]]);
					// printf("3.1 ");
				}
				else if(i>middle){
					right_z_sort_ids[right_axis_count[2]] = copy_z_sort_ids[i];
					right_axis_count[2]++;
					next_group[copy_z_sort_ids[i]] = 1;
					// printf("3.3");
				}
				else{
					next_group[copy_z_sort_ids[i]] = 0;
					// printf("3.2");
				}
			}
		}
		// printf("\n");
		// printf("4");
		left_group_size = left_axis_count[axis];
		right_group_size = right_axis_count[axis];

		// printf("group is (");
		// for(int i=0;i<group_size;i++){
		// 	if(axis==0) printf("%d,",copy_x_sort_ids[i]);
		// 	if(axis==1) printf("%d,",copy_y_sort_ids[i]);
		// 	if(axis==2) printf("%d,",copy_z_sort_ids[i]);
		// }
		// printf(")");
		// printf("left_group is (");
		// for(int i=0;i<left_group_size;i++){
		// 	if(axis==0) printf("%d,",left_x_sort_ids[i]);
		// 	if(axis==1) printf("%d,",left_y_sort_ids[i]);
		// 	if(axis==2) printf("%d,",left_z_sort_ids[i]);
		// }
		// printf(")");
		// printf("right_group is (");
		// for(int i=0;i<right_group_size;i++){
		// 	if(axis==0) printf("%d,",right_x_sort_ids[i]);
		// 	if(axis==1) printf("%d,",right_y_sort_ids[i]);
		// 	if(axis==2) printf("%d,",right_z_sort_ids[i]);
		// }
		// printf(")");
		// printf("\n\n\n");

		for(int i = 0; i < group_size; i++){
			for(int j = 0; j < 3; j++){
				if(j==axis) continue;
				if(j==0){//x実装
					if(next_group[copy_x_sort_ids[i]] == -1){
						left_x_sort_ids[left_axis_count[j]] = copy_x_sort_ids[i];
						left_axis_count[j]++;
					}
					else if(next_group[copy_x_sort_ids[i]] == 1){
						right_x_sort_ids[right_axis_count[j]] = copy_x_sort_ids[i];
						right_axis_count[j]++;
					}
				}
				if(j==1){//y実装
					if(next_group[copy_y_sort_ids[i]] == -1){
						left_y_sort_ids[left_axis_count[j]] = copy_y_sort_ids[i];
						left_axis_count[j]++;
					}
					else if(next_group[copy_y_sort_ids[i]] == 1){
						right_y_sort_ids[right_axis_count[j]] = copy_y_sort_ids[i];
						right_axis_count[j]++;
					}
				}
				if(j==2){//z実装
					if(next_group[copy_z_sort_ids[i]] == -1){
						left_z_sort_ids[left_axis_count[j]] = copy_z_sort_ids[i];
						left_axis_count[j]++;
					}
					else if(next_group[copy_z_sort_ids[i]] == 1){
						right_z_sort_ids[right_axis_count[j]] = copy_z_sort_ids[i];
						right_axis_count[j]++;
					}
				}
			}
		}
		free(copy_x_sort_ids);
		free(copy_y_sort_ids);
		free(copy_z_sort_ids);
		free(next_group);
		// printf("5");
		// if(median_id==18) printf("Hit!!!!!!!!!!!!!!! left_x_sort_ids[16] = %d\n\n\n",left_x_sort_ids[16]);
		// if(median_id==33) printf("Hit!!!!!!!!!!!!!!! right_x_sort_ids[16] = %d\n\n\n",right_x_sort_ids[16]);
		// if(median_id==3145) printf("Hit!!!!!!!!!!!!!!! left_x_sort_ids[156] = %d\n\n\n",left_x_sort_ids[156]);
		// if(median_id==3877) printf("Hit!!!!!!!!!!!!!!! left_y_sort_ids[1] = %d\n\n\n",left_x_sort_ids[1]);
		// if(median_id==3888) printf("Hit!!!!!!!!!!!!!!! left_y_sort_ids[1] = %d\n\n\n",left_y_sort_ids[1]);
		// cudaDeviceSynchronize();
		if(left_group_size > 0) d_CreateNode<<<1, 1>>>(point_size,left_group_size,depth+1,median_id,false,left_x_sort_ids,left_y_sort_ids,left_z_sort_ids,root_id,nodes);
		else {
			free(left_x_sort_ids);
			free(left_y_sort_ids);
			free(left_z_sort_ids);
		}
		if(right_group_size > 0) d_CreateNode<<<1, 1>>>(point_size,right_group_size,depth+1,median_id,true,right_x_sort_ids,right_y_sort_ids,right_z_sort_ids,root_id,nodes);
		else {
			free(right_x_sort_ids);
			free(right_y_sort_ids);
			free(right_z_sort_ids);
		}
		// cudaDeviceSynchronize();
		// free(left_x_sort_ids);
		// free(left_y_sort_ids);
		// free(left_z_sort_ids);
		// free(right_x_sort_ids);
		// free(right_y_sort_ids);
		// free(right_z_sort_ids);
		
	}
}

__global__ void d_DepthCreateNode(int point_size,detailed_node* nodes,int* end_list)
{
    unsigned int ix = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int idx = ix;
	// printf("idx = %d\n",idx);
	if(idx < point_size){//計算領域
		// printf("idx = %d\n",idx);
		// bool limit = true;
		// if(nodes[idx].depth == 9 && !nodes[idx].node_is_right) limit = false;//left or right limit
		// int id_th = 1895;//2464o,1000x,1732x,2098o,1915o,1823x,1869x,1892x,1903o,1897o,1894x,1895x,1896o
		// if(nodes[idx].depth == 9 && (idx<id_th)) limit = false;//left or right limit
		// if(nodes[idx].depth == 9 && limit) printf("layer9 limit in\n");
		// if(nodes[idx].depth == 9 && limit && (nodes[idx].ready && (0 > end_list[idx]))) printf("layer9 new in\n");
		// if(nodes[idx].depth == 9 && limit && nodes[idx].ready) printf("layer9 limit in and ready\n");
		// if(nodes[idx].depth == 9 && limit && (0 > end_list[idx])) printf("layer9 limit in and not end\n");
		// if(nodes[idx].depth == 9 && nodes[idx].ready) printf("layer9 ready\n");
		if(/*limit && */(nodes[idx].ready && (0 == end_list[idx]))){//該当ノード
			// if(nodes[idx].depth == 9) printf("in\n");
			// printf("median_id = %d \n",idx);
			// printf("\n\n\n");
			// printf("\nmedian_id = %d\n",idx);特異
			// if(!nodes[idx].node_is_right) printf("node is left\n");
			// else printf("node is right\n");
			// printf("parent_id = %d\n",nodes[idx].parent_id);
			// printf("middle = %d\n",nodes[idx].middle);
			// printf("axis = %d\n",nodes[idx].axis);
			// if(nodes[idx].node_is_right) printf("device depth = %d\n",nodes[idx].depth);//間引きのため右のみ
			// if(nodes[idx].depth==9) printf("start idx = %d\n",idx);
			// if(idx==3040) printf("start idx = %d\n",idx);
			// printf("0");
			nodes[idx].left_id = -1;
			nodes[idx].right_id = -1;
			if(nodes[idx].group_size>1){//子あり
				// printf("\nmedian_id = %d\n",idx);
				int left_group_size = 0;
				int right_group_size = 0;
				int *left_x_sort_ids,*left_y_sort_ids,*left_z_sort_ids;
				int *right_x_sort_ids,*right_y_sort_ids,*right_z_sort_ids;
				int *next_group;
				next_group = (int *)malloc(point_size * sizeof(int));
				left_x_sort_ids = (int *)malloc(nodes[idx].middle * sizeof(int));
				left_y_sort_ids = (int *)malloc(nodes[idx].middle * sizeof(int));
				left_z_sort_ids = (int *)malloc(nodes[idx].middle * sizeof(int));
				right_x_sort_ids = (int *)malloc((nodes[idx].group_size - (nodes[idx].middle + 1)) * sizeof(int));
				right_y_sort_ids = (int *)malloc((nodes[idx].group_size - (nodes[idx].middle + 1)) * sizeof(int));
				right_z_sort_ids = (int *)malloc((nodes[idx].group_size - (nodes[idx].middle + 1)) * sizeof(int));
				int left_axis_count[3]={0,0,0};
				int right_axis_count[3]={0,0,0};

				if(nodes[idx].axis==0){
					for(int i = 0; i < nodes[idx].group_size; i++){
						if(point_size<nodes[idx].x_sort_ids[i]) printf("out of range nodes[idx].x_sort_ids[%d] = %d \n",i,nodes[idx].x_sort_ids[i]);
						if(point_size<nodes[idx].y_sort_ids[i]) printf("out of range nodes[idx].y_sort_ids[%d] = %d \n",i,nodes[idx].y_sort_ids[i]);
						if(point_size<nodes[idx].z_sort_ids[i]) printf("out of range nodes[idx].z_sort_ids[%d] = %d \n",i,nodes[idx].z_sort_ids[i]);
						if(i<nodes[idx].middle){
							left_x_sort_ids[left_axis_count[0]] = nodes[idx].x_sort_ids[i];
							left_axis_count[0]++;
							next_group[nodes[idx].x_sort_ids[i]] = -1;
						}
						else if(i>nodes[idx].middle){
							right_x_sort_ids[right_axis_count[0]] = nodes[idx].x_sort_ids[i];
							right_axis_count[0]++;
							next_group[nodes[idx].x_sort_ids[i]] = 1;
						}
						else{
							next_group[nodes[idx].x_sort_ids[i]] = 0;
						}
					}
				}
				else if(nodes[idx].axis==1){
					for(int i = 0; i < nodes[idx].group_size; i++){
						if(point_size<nodes[idx].x_sort_ids[i]) printf("out of range nodes[idx].x_sort_ids[%d] = %d \n",i,nodes[idx].x_sort_ids[i]);
						if(point_size<nodes[idx].y_sort_ids[i]) printf("out of range nodes[idx].y_sort_ids[%d] = %d \n",i,nodes[idx].y_sort_ids[i]);
						if(point_size<nodes[idx].z_sort_ids[i]) printf("out of range nodes[idx].z_sort_ids[%d] = %d \n",i,nodes[idx].z_sort_ids[i]);
						if(i<nodes[idx].middle){
							left_y_sort_ids[left_axis_count[1]] = nodes[idx].y_sort_ids[i];
							left_axis_count[1]++;
							next_group[nodes[idx].y_sort_ids[i]] = -1;
						}
						else if(i>nodes[idx].middle){
							right_y_sort_ids[right_axis_count[1]] = nodes[idx].y_sort_ids[i];
							right_axis_count[1]++;
							next_group[nodes[idx].y_sort_ids[i]] = 1;
						}
						else{
							next_group[nodes[idx].y_sort_ids[i]] = 0;
						}
					}

				}
				else if(nodes[idx].axis==2){
					for(int i = 0; i < nodes[idx].group_size; i++){
						if(point_size<nodes[idx].x_sort_ids[i]) printf("out of range nodes[idx].x_sort_ids[%d] = %d \n",i,nodes[idx].x_sort_ids[i]);
						if(point_size<nodes[idx].y_sort_ids[i]) printf("out of range nodes[idx].y_sort_ids[%d] = %d \n",i,nodes[idx].y_sort_ids[i]);
						if(point_size<nodes[idx].z_sort_ids[i]) printf("out of range nodes[idx].z_sort_ids[%d] = %d \n",i,nodes[idx].z_sort_ids[i]);
						if(i<nodes[idx].middle){
							left_z_sort_ids[left_axis_count[2]] = nodes[idx].z_sort_ids[i];
							left_axis_count[2]++;
							next_group[nodes[idx].z_sort_ids[i]] = -1;
						}
						else if(i>nodes[idx].middle){
							right_z_sort_ids[right_axis_count[2]] = nodes[idx].z_sort_ids[i];
							right_axis_count[2]++;
							next_group[nodes[idx].z_sort_ids[i]] = 1;
						}
						else{
							next_group[nodes[idx].z_sort_ids[i]] = 0;
						}
					}
				}
				// cudaDeviceSynchronize();
				// printf("depth = %d\n",nodes[idx].depth);
				// printf("median_id = %d \n",idx);
				// cudaDeviceSynchronize();
				// if(nodes[idx].depth==9) printf("section1 idx = %d\n",idx);
				// if(idx==3040) printf("section1 idx = %d\n",idx);
				// printf("1 ");
				left_group_size = left_axis_count[nodes[idx].axis];
				right_group_size = right_axis_count[nodes[idx].axis];

				for(int i = 0; i < nodes[idx].group_size; i++){
					for(int j = 0; j < 3; j++){
						if(j==nodes[idx].axis) continue;
						if(j==0){//x実装
							if(next_group[nodes[idx].x_sort_ids[i]] == -1){
								left_x_sort_ids[left_axis_count[j]] = nodes[idx].x_sort_ids[i];
								left_axis_count[j]++;
							}
							else if(next_group[nodes[idx].x_sort_ids[i]] == 1){
								right_x_sort_ids[right_axis_count[j]] = nodes[idx].x_sort_ids[i];
								right_axis_count[j]++;
							}
						}
						if(j==1){//y実装
							if(next_group[nodes[idx].y_sort_ids[i]] == -1){
								left_y_sort_ids[left_axis_count[j]] = nodes[idx].y_sort_ids[i];
								left_axis_count[j]++;
							}
							else if(next_group[nodes[idx].y_sort_ids[i]] == 1){
								right_y_sort_ids[right_axis_count[j]] = nodes[idx].y_sort_ids[i];
								right_axis_count[j]++;
							}
						}
						if(j==2){//z実装
							if(next_group[nodes[idx].z_sort_ids[i]] == -1){
								left_z_sort_ids[left_axis_count[j]] = nodes[idx].z_sort_ids[i];
								left_axis_count[j]++;
							}
							else if(next_group[nodes[idx].z_sort_ids[i]] == 1){
								right_z_sort_ids[right_axis_count[j]] = nodes[idx].z_sort_ids[i];
								right_axis_count[j]++;
							}
						}
					}
				}
				// cudaDeviceSynchronize();
				// if(nodes[idx].depth==9) printf("section2 idx = %d\n",idx);
				// if(idx==3040) printf("section2 idx = %d\n",idx);
				// printf("2 ");
				free(next_group);
				int next_axis = (nodes[idx].depth + 1) % 3;
				if(left_group_size > 0){
					size_t left_middle = ((left_group_size - 1) / 2);
					int left_median_id;
					if(next_axis == 0) left_median_id = left_x_sort_ids[left_middle];
					if(next_axis == 1) left_median_id = left_y_sort_ids[left_middle];
					if(next_axis == 2) left_median_id = left_z_sort_ids[left_middle];

					nodes[idx].left_id = left_median_id;

					nodes[left_median_id].ready = true;
					nodes[left_median_id].node_is_right = false;
					nodes[left_median_id].parent_id = idx;
					nodes[left_median_id].depth = nodes[idx].depth + 1;
					nodes[left_median_id].axis = next_axis;
					nodes[left_median_id].middle = left_middle;
					nodes[left_median_id].group_size = left_group_size;
					nodes[left_median_id].x_sort_ids = left_x_sort_ids;
					nodes[left_median_id].y_sort_ids = left_y_sort_ids;
					nodes[left_median_id].z_sort_ids = left_z_sort_ids;
				}
				if(right_group_size > 0){
					size_t right_middle = ((right_group_size - 1) / 2);
					int right_median_id;
					if(next_axis == 0) right_median_id = right_x_sort_ids[right_middle];
					if(next_axis == 1) right_median_id = right_y_sort_ids[right_middle];
					if(next_axis == 2) right_median_id = right_z_sort_ids[right_middle];

					nodes[idx].right_id = right_median_id;

					nodes[right_median_id].ready = true;
					nodes[right_median_id].node_is_right = true;
					nodes[right_median_id].parent_id = idx;
					nodes[right_median_id].depth = nodes[idx].depth + 1;
					nodes[right_median_id].axis = next_axis;
					nodes[right_median_id].middle = right_middle;
					nodes[right_median_id].group_size = right_group_size;
					nodes[right_median_id].x_sort_ids = right_x_sort_ids;
					nodes[right_median_id].y_sort_ids = right_y_sort_ids;
					nodes[right_median_id].z_sort_ids = right_z_sort_ids;
				}
				// cudaDeviceSynchronize();
				// if(nodes[idx].depth==9) printf("section3 idx = %d\n",idx);
				// if(idx==3040) printf("section3 idx = %d\n",idx);
				// printf("3 ");
			}
			if(nodes[idx].parent_id >= 0){//親あり
				free(nodes[idx].x_sort_ids);
				free(nodes[idx].y_sort_ids);
				free(nodes[idx].z_sort_ids);
			}
			// cudaDeviceSynchronize();
			// if(nodes[idx].depth==9) printf("section4 idx = %d\n",idx);
			// if(idx==3040) printf("section4 idx = %d\n",idx);
			// printf("4 ");
			end_list[idx] = 1;
			// cudaDeviceSynchronize();
			// printf("5 ");
			// if(idx==3040) printf("section5 idx = %d\n",idx);
			// if(nodes[idx].depth==9) printf("end idx = %d\n",idx);
		}
	}
}

__global__ void d_DepthCreateNodeLWD(int point_size,int *next_start_thread_id,int *next_layer_run,detailed_node* nodes,int* end_list)
{
    unsigned int ix = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int idx = ix;
	if(idx < point_size){//計算領域
		int layer_index = idx - *next_start_thread_id;
		int median_id = -1;
		int layer_size = 0;
		if(layer_index >= 0){
			for(int i=0;i<point_size;i++){
				if(next_layer_run[i]==1){
					if(layer_size == layer_index) median_id = i;
					layer_size++;
				}
			}
		}
		if(median_id >= 0) next_layer_run[median_id] = 0;
		if((median_id >= 0) && (*next_start_thread_id <= idx) && (idx < (*next_start_thread_id + layer_size))){//該当ノード
			printf("median_id = %d \n",median_id);
			nodes[median_id].left_id = -1;
			nodes[median_id].right_id = -1;
			if(nodes[median_id].group_size>1){//子あり
				int left_group_size = 0;
				int right_group_size = 0;
				int *left_x_sort_ids,*left_y_sort_ids,*left_z_sort_ids;
				int *right_x_sort_ids,*right_y_sort_ids,*right_z_sort_ids;
				int *next_group;
				next_group = (int *)malloc(point_size * sizeof(int));
				left_x_sort_ids = (int *)malloc(nodes[median_id].middle * sizeof(int));
				left_y_sort_ids = (int *)malloc(nodes[median_id].middle * sizeof(int));
				left_z_sort_ids = (int *)malloc(nodes[median_id].middle * sizeof(int));
				right_x_sort_ids = (int *)malloc((nodes[median_id].group_size - (nodes[median_id].middle + 1)) * sizeof(int));
				right_y_sort_ids = (int *)malloc((nodes[median_id].group_size - (nodes[median_id].middle + 1)) * sizeof(int));
				right_z_sort_ids = (int *)malloc((nodes[median_id].group_size - (nodes[median_id].middle + 1)) * sizeof(int));
				int left_axis_count[3]={0,0,0};
				int right_axis_count[3]={0,0,0};

				if(nodes[median_id].axis==0){
					for(int i = 0; i < nodes[median_id].group_size; i++){
						if(point_size<nodes[median_id].x_sort_ids[i]) printf("out of range nodes[median_id].x_sort_ids[%d] = %d \n",i,nodes[median_id].x_sort_ids[i]);
						if(point_size<nodes[median_id].y_sort_ids[i]) printf("out of range nodes[median_id].y_sort_ids[%d] = %d \n",i,nodes[median_id].y_sort_ids[i]);
						if(point_size<nodes[median_id].z_sort_ids[i]) printf("out of range nodes[median_id].z_sort_ids[%d] = %d \n",i,nodes[median_id].z_sort_ids[i]);
						if(i<nodes[median_id].middle){
							left_x_sort_ids[left_axis_count[0]] = nodes[median_id].x_sort_ids[i];
							left_axis_count[0]++;
							next_group[nodes[median_id].x_sort_ids[i]] = -1;
						}
						else if(i>nodes[median_id].middle){
							right_x_sort_ids[right_axis_count[0]] = nodes[median_id].x_sort_ids[i];
							right_axis_count[0]++;
							next_group[nodes[median_id].x_sort_ids[i]] = 1;
						}
						else{
							next_group[nodes[median_id].x_sort_ids[i]] = 0;
						}
					}
				}
				else if(nodes[median_id].axis==1){
					for(int i = 0; i < nodes[median_id].group_size; i++){
						if(point_size<nodes[median_id].x_sort_ids[i]) printf("out of range nodes[median_id].x_sort_ids[%d] = %d \n",i,nodes[median_id].x_sort_ids[i]);
						if(point_size<nodes[median_id].y_sort_ids[i]) printf("out of range nodes[median_id].y_sort_ids[%d] = %d \n",i,nodes[median_id].y_sort_ids[i]);
						if(point_size<nodes[median_id].z_sort_ids[i]) printf("out of range nodes[median_id].z_sort_ids[%d] = %d \n",i,nodes[median_id].z_sort_ids[i]);
						if(i<nodes[median_id].middle){
							left_y_sort_ids[left_axis_count[1]] = nodes[median_id].y_sort_ids[i];
							left_axis_count[1]++;
							next_group[nodes[median_id].y_sort_ids[i]] = -1;
						}
						else if(i>nodes[median_id].middle){
							right_y_sort_ids[right_axis_count[1]] = nodes[median_id].y_sort_ids[i];
							right_axis_count[1]++;
							next_group[nodes[median_id].y_sort_ids[i]] = 1;
						}
						else{
							next_group[nodes[median_id].y_sort_ids[i]] = 0;
						}
					}

				}
				else if(nodes[median_id].axis==2){
					for(int i = 0; i < nodes[median_id].group_size; i++){
						if(point_size<nodes[median_id].x_sort_ids[i]) printf("out of range nodes[median_id].x_sort_ids[%d] = %d \n",i,nodes[median_id].x_sort_ids[i]);
						if(point_size<nodes[median_id].y_sort_ids[i]) printf("out of range nodes[median_id].y_sort_ids[%d] = %d \n",i,nodes[median_id].y_sort_ids[i]);
						if(point_size<nodes[median_id].z_sort_ids[i]) printf("out of range nodes[median_id].z_sort_ids[%d] = %d \n",i,nodes[median_id].z_sort_ids[i]);
						if(i<nodes[median_id].middle){
							left_z_sort_ids[left_axis_count[2]] = nodes[median_id].z_sort_ids[i];
							left_axis_count[2]++;
							next_group[nodes[median_id].z_sort_ids[i]] = -1;
						}
						else if(i>nodes[median_id].middle){
							right_z_sort_ids[right_axis_count[2]] = nodes[median_id].z_sort_ids[i];
							right_axis_count[2]++;
							next_group[nodes[median_id].z_sort_ids[i]] = 1;
						}
						else{
							next_group[nodes[median_id].z_sort_ids[i]] = 0;
						}
					}
				}
				left_group_size = left_axis_count[nodes[median_id].axis];
				right_group_size = right_axis_count[nodes[median_id].axis];

				for(int i = 0; i < nodes[median_id].group_size; i++){
					for(int j = 0; j < 3; j++){
						if(j==nodes[median_id].axis) continue;
						if(j==0){//x実装
							if(next_group[nodes[median_id].x_sort_ids[i]] == -1){
								left_x_sort_ids[left_axis_count[j]] = nodes[median_id].x_sort_ids[i];
								left_axis_count[j]++;
							}
							else if(next_group[nodes[median_id].x_sort_ids[i]] == 1){
								right_x_sort_ids[right_axis_count[j]] = nodes[median_id].x_sort_ids[i];
								right_axis_count[j]++;
							}
						}
						if(j==1){//y実装
							if(next_group[nodes[median_id].y_sort_ids[i]] == -1){
								left_y_sort_ids[left_axis_count[j]] = nodes[median_id].y_sort_ids[i];
								left_axis_count[j]++;
							}
							else if(next_group[nodes[median_id].y_sort_ids[i]] == 1){
								right_y_sort_ids[right_axis_count[j]] = nodes[median_id].y_sort_ids[i];
								right_axis_count[j]++;
							}
						}
						if(j==2){//z実装
							if(next_group[nodes[median_id].z_sort_ids[i]] == -1){
								left_z_sort_ids[left_axis_count[j]] = nodes[median_id].z_sort_ids[i];
								left_axis_count[j]++;
							}
							else if(next_group[nodes[median_id].z_sort_ids[i]] == 1){
								right_z_sort_ids[right_axis_count[j]] = nodes[median_id].z_sort_ids[i];
								right_axis_count[j]++;
							}
						}
					}
				}
				free(next_group);
				int next_axis = (nodes[median_id].depth + 1) % 3;
				if(left_group_size > 0){
					size_t left_middle = ((left_group_size - 1) / 2);
					int left_median_id;
					if(next_axis == 0) left_median_id = left_x_sort_ids[left_middle];
					if(next_axis == 1) left_median_id = left_y_sort_ids[left_middle];
					if(next_axis == 2) left_median_id = left_z_sort_ids[left_middle];

					next_layer_run[left_median_id] = 1;

					nodes[median_id].left_id = left_median_id;

					nodes[left_median_id].ready = true;
					nodes[left_median_id].node_is_right = false;
					nodes[left_median_id].parent_id = median_id;
					nodes[left_median_id].depth = nodes[median_id].depth + 1;
					nodes[left_median_id].axis = next_axis;
					nodes[left_median_id].middle = left_middle;
					nodes[left_median_id].group_size = left_group_size;
					nodes[left_median_id].x_sort_ids = left_x_sort_ids;
					nodes[left_median_id].y_sort_ids = left_y_sort_ids;
					nodes[left_median_id].z_sort_ids = left_z_sort_ids;
				}
				if(right_group_size > 0){
					size_t right_middle = ((right_group_size - 1) / 2);
					int right_median_id;
					if(next_axis == 0) right_median_id = right_x_sort_ids[right_middle];
					if(next_axis == 1) right_median_id = right_y_sort_ids[right_middle];
					if(next_axis == 2) right_median_id = right_z_sort_ids[right_middle];

					next_layer_run[right_median_id] = 1;

					nodes[median_id].right_id = right_median_id;

					nodes[right_median_id].ready = true;
					nodes[right_median_id].node_is_right = true;
					nodes[right_median_id].parent_id = median_id;
					nodes[right_median_id].depth = nodes[median_id].depth + 1;
					nodes[right_median_id].axis = next_axis;
					nodes[right_median_id].middle = right_middle;
					nodes[right_median_id].group_size = right_group_size;
					nodes[right_median_id].x_sort_ids = right_x_sort_ids;
					nodes[right_median_id].y_sort_ids = right_y_sort_ids;
					nodes[right_median_id].z_sort_ids = right_z_sort_ids;
				}
			}
			if(nodes[median_id].parent_id >= 0){//親あり
				free(nodes[median_id].x_sort_ids);
				free(nodes[median_id].y_sort_ids);
				free(nodes[median_id].z_sort_ids);
			}
			end_list[median_id] = 1;
			if(idx==*next_start_thread_id){
				*next_start_thread_id += layer_size;
			}
		}
	}
}

__device__ void d_PointRangeCheckAndAdd(int *range_indices_size,int *range_indices,int head_id,float* points,float* search_point,float range_sq)
{
	float dist_sq = powf(points[head_id*3+0]-search_point[0],2)+powf(points[head_id*3+1]-search_point[1],2)+powf(points[head_id*3+2]-search_point[2],2);
	if(dist_sq<range_sq){
		// printf("device inside");
		range_indices[*range_indices_size] = head_id;
		*range_indices_size+=1;
	} 
}

__device__ int d_SearchSubTree(int *range_indices_size,int *range_indices,int root_id,node* nodes,float* points,float* search_point,float range_sq)
{
	int head_id = root_id;
	bool cross,next_is_right;
	//潜り
	while(1){
		if(search_point[nodes[head_id].axis]>points[head_id*3+nodes[head_id].axis]) next_is_right = true;
		else next_is_right = false;

		if(nodes[head_id].right_id>=0&&nodes[head_id].left_id<0) head_id = nodes[head_id].right_id;//一人っ子
		else if(nodes[head_id].left_id>=0&&nodes[head_id].right_id<0) head_id = nodes[head_id].left_id;//一人っ子
		else if(nodes[head_id].left_id>=0&&nodes[head_id].right_id>=0){//双子
			if(next_is_right){//right
				head_id = nodes[head_id].right_id;
			}
			else{//left
				head_id = nodes[head_id].left_id;
			}
		}
		else break;
	}
	//rootが底
	if(head_id==root_id) {
		d_PointRangeCheckAndAdd(range_indices_size,range_indices,head_id,points,search_point,range_sq);
		return 1;
	}
	//昇り
	int last_id;
	while(1){
		cross = false;
		d_PointRangeCheckAndAdd(range_indices_size,range_indices,head_id,points,search_point,range_sq);
		//昇る
		last_id = head_id;
		head_id = nodes[head_id].parent_id;
		//交差判定　1軸のみ低効率
		float axis_diff_sq = powf(points[head_id*3+nodes[head_id].axis] - search_point[nodes[head_id].axis],2);
		if(axis_diff_sq < range_sq) cross = true;
		int sub_tree=0;
		if(cross){
			if(last_id==nodes[head_id].right_id&&nodes[head_id].left_id>0) sub_tree = d_SearchSubTree(range_indices_size,range_indices,nodes[head_id].left_id,nodes,points,search_point,range_sq);//右から上がってきた
			if(last_id==nodes[head_id].left_id&&nodes[head_id].right_id>0) sub_tree = d_SearchSubTree(range_indices_size,range_indices,nodes[head_id].right_id,nodes,points,search_point,range_sq);//左から上がってきた
		}
		if(head_id==root_id){
			d_PointRangeCheckAndAdd(range_indices_size,range_indices,head_id,points,search_point,range_sq);
			break;
		}
	}

	return 1;
}

__device__ int d_SearchSubTree2(int *range_indices_size,int *range_indices,int root_id,detailed_node* nodes,float* points,float* search_point,float range_sq)
{
	int head_id = root_id;
	bool cross,next_is_right;
	//潜り
	while(1){
		if(search_point[nodes[head_id].axis]>points[head_id*3+nodes[head_id].axis]) next_is_right = true;
		else next_is_right = false;

		if(nodes[head_id].right_id>=0&&nodes[head_id].left_id<0) head_id = nodes[head_id].right_id;//一人っ子
		else if(nodes[head_id].left_id>=0&&nodes[head_id].right_id<0) head_id = nodes[head_id].left_id;//一人っ子
		else if(nodes[head_id].left_id>=0&&nodes[head_id].right_id>=0){//双子
			if(next_is_right){//right
				head_id = nodes[head_id].right_id;
			}
			else{//left
				head_id = nodes[head_id].left_id;
			}
		}
		else break;
	}
	//rootが底
	if(head_id==root_id) {
		d_PointRangeCheckAndAdd(range_indices_size,range_indices,head_id,points,search_point,range_sq);
		return 1;
	}
	//昇り
	int last_id;
	while(1){
		cross = false;
		d_PointRangeCheckAndAdd(range_indices_size,range_indices,head_id,points,search_point,range_sq);
		//昇る
		last_id = head_id;
		head_id = nodes[head_id].parent_id;
		//交差判定　1軸のみ低効率
		float axis_diff_sq = powf(points[head_id*3+nodes[head_id].axis] - search_point[nodes[head_id].axis],2);
		if(axis_diff_sq < range_sq) cross = true;
		int sub_tree=0;
		if(cross){
			if(last_id==nodes[head_id].right_id&&nodes[head_id].left_id>0) sub_tree = d_SearchSubTree2(range_indices_size,range_indices,nodes[head_id].left_id,nodes,points,search_point,range_sq);//右から上がってきた
			if(last_id==nodes[head_id].left_id&&nodes[head_id].right_id>0) sub_tree = d_SearchSubTree2(range_indices_size,range_indices,nodes[head_id].right_id,nodes,points,search_point,range_sq);//左から上がってきた
		}
		if(head_id==root_id){
			d_PointRangeCheckAndAdd(range_indices_size,range_indices,head_id,points,search_point,range_sq);
			break;
		}
	}

	return 1;
}

__device__ int EigenJacobiMethod(float *a, float *v, int n, float eps = 1e-8, int iter_max = 100)
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

__global__ void NormalsGPU(/*detailed_node* detailed_nodes,*/long long int* neighbor_time,int *point_neighbor_size,int* point_neighbor,int* d_parent_ids,int* d_left_ids,int* d_right_ids,int* d_axes,int root_id,float* points,int point_size,int* neighbor_points_indices,int* neighbor_start_indices,int neighbor_points_count,float* normals,float* curvatures,long long int* covariance_time,long long int* eigen_time)
{
    // printf("normalsGPU");
    //インデックス取得
    unsigned int ix = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int idx = ix;
    unsigned int output_id=50;
    // printf("idx = %d ,", idx);
	if(idx==output_id){
		long long int neighbor_start, neighbor_stop;
		asm volatile("mov.u64  %0, %globaltimer;" : "=l"(neighbor_start));
		////////////////////////////////////////////////////////////////
		node *nodes = (node*)malloc(sizeof(node) * point_size);

		for(int i=0;i<point_size;i++){
			nodes[i].parent_id=d_parent_ids[i];
			nodes[i].left_id=d_left_ids[i];
			nodes[i].right_id=d_right_ids[i];
			nodes[i].axis=d_axes[i];
		}
		float search_point[3];
		search_point[0]=points[idx*3+0];
		search_point[1]=points[idx*3+1];
		search_point[2]=points[idx*3+2];
		
		int *range_indices = (int*)malloc(sizeof(int) * point_size);
		int range_indices_size = 0;

		float range_sq = 0.15*0.15;

		//探索関数の実行
		int range_search = d_SearchSubTree(&range_indices_size,range_indices,root_id,nodes,points,search_point,range_sq);
		// int range_search = d_SearchSubTree2(&range_indices_size,range_indices,root_id,detailed_nodes,points,search_point,range_sq);
		// std::cout<<"range_indices.size()"<<range_indices.size()<<std::endl;
		if(range_search==1) {
			// printf("device range_indices size is =%d",range_indices_size);
			// printf("device range_indices is [");
			for(int i=0;i<range_indices_size;i++){
				// printf("%d,",range_indices[i]);
				point_neighbor[i]=range_indices[i];
			}
			// printf("]\n");
			point_neighbor_size[0]=range_indices_size;
			// printf("device size %d\n",range_indices_size);
		}
		
		free (nodes);
		free (range_indices);
		asm volatile("mov.u64  %0, %globaltimer;" : "=l"(neighbor_stop));
		neighbor_time[idx]=neighbor_stop - neighbor_start;
		////////////////////////////////////////////////////////////////////
	}


    if(idx<point_size){//対象スレッド内のみ計算
        //デバッグ用
        // if(idx==0||idx==10||idx==20) printf("points(%d) = %f,%f,%f\n",idx,points[idx*3+0],points[idx*3+1],points[idx*3+2]);
        
        // printf("idx<point_size");
        //近傍点終点インデックスの定義
        int end_indices;
        if(idx==(point_size-1)) end_indices = neighbor_points_count;
        else end_indices = neighbor_start_indices[idx+1]-1;

        int neighbor_size=(end_indices-neighbor_start_indices[idx]+1)/3;
        // if(idx==0||idx==10||idx==20) printf("neighbor(%d) start = %d, end = %d,size = %d\n",idx,neighbor_start_indices[idx],end_indices,neighbor_size);
        // printf("neighbor_size = %d\n", neighbor_size);
        if(neighbor_size>=3){//近傍点数3以上
            long long int covariance_start, covariance_stop;
            long long int eigen_start,eigen_stop;

            asm volatile("mov.u64  %0, %globaltimer;" : "=l"(covariance_start));
            // printf("neighbor_size>=3");
            //平均計算
            float x_average=0,y_average=0,z_average=0;
            // if(idx==output_id) printf("neighbor_size = %d\n",neighbor_size);

            // if(idx==output_id) printf("neighbor_points(%d) = {\n",idx);
            for(int i=neighbor_start_indices[idx];i<=end_indices;i+=3){//近傍点
                // デバッグ用
                // if((idx==0||idx==10||idx==20)&&(i==(neighbor_start_indices[idx]))) printf("neighbor_points_indices(%d) = %d\n",idx,neighbor_points_indices[i]);
                // if((idx==0||idx==10||idx==20)&&i==neighbor_start_indices[idx]) printf("neighbor_points(%d) = %f,%f,%f\n",idx,points[neighbor_points_indices[i]*3+0],points[neighbor_points_indices[i]*3+1],points[neighbor_points_indices[i]*3+2]);
                // if(idx==output_id) printf("{%f, %f, %f},\n",points[neighbor_points_indices[i]*3+0],points[neighbor_points_indices[i]*3+1],points[neighbor_points_indices[i]*3+2]);
                x_average+=points[neighbor_points_indices[i]*3+0];
                y_average+=points[neighbor_points_indices[i]*3+1];
                z_average+=points[neighbor_points_indices[i]*3+2];
            }
            // if(idx==output_id) printf("};\n");
            x_average/=neighbor_size;
            y_average/=neighbor_size;
            z_average/=neighbor_size;

            // //要素計算
            float sxx=0,sxy=0,sxz=0,syy=0,syz=0,szz=0;
            for(int i=neighbor_start_indices[idx];i<=end_indices;i+=3){//近傍点
                sxx+=(points[neighbor_points_indices[i]*3+0]-x_average)*(points[neighbor_points_indices[i]*3+0]-x_average);
                syy+=(points[neighbor_points_indices[i]*3+1]-y_average)*(points[neighbor_points_indices[i]*3+1]-y_average);
                szz+=(points[neighbor_points_indices[i]*3+2]-z_average)*(points[neighbor_points_indices[i]*3+2]-z_average);

                sxy+=(points[neighbor_points_indices[i]*3+0]-x_average)*(points[neighbor_points_indices[i]*3+1]-y_average);
                sxz+=(points[neighbor_points_indices[i]*3+0]-x_average)*(points[neighbor_points_indices[i]*3+2]-z_average);
                syz+=(points[neighbor_points_indices[i]*3+1]-y_average)*(points[neighbor_points_indices[i]*3+2]-z_average);
            }

            sxx/=neighbor_size;
            syy/=neighbor_size;
            szz/=neighbor_size;
            sxy/=neighbor_size;
            sxz/=neighbor_size;
            syz/=neighbor_size;

            //共分散行列
            float a[3*3]={
                sxx,sxy,sxz,
                sxy,syy,syz,
                sxz,syz,szz,
            };

            asm volatile("mov.u64  %0, %globaltimer;" : "=l"(covariance_stop));
            covariance_time[idx]=covariance_stop - covariance_start;

            asm volatile("mov.u64  %0, %globaltimer;" : "=l"(eigen_start));
            
            // __syncthreads();
            // if(idx==output_id){
            //     printf("                          %f ,%f ,%f \ncovariance matrix(%d)=   %f ,%f ,%f \n                          %f ,%f ,%f \n\n",sxx,sxy,sxz,idx,sxy,syy,syz,sxz,syz,szz);
            // }

            //固有値計算
            float eigen_vector[3 * 3];
            EigenJacobiMethod(a, eigen_vector, 3);

            // __syncthreads();
            // if(neighbor_size<3){
            //     printf("               %f ,%f ,%f \neigen_value=   %f ,%f ,%f \n               %f ,%f ,%f \n\n",a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8]);
            // }

            float eigen_value[3];
            eigen_value[0]=a[0];
            eigen_value[1]=a[4];
            eigen_value[2]=a[8];

            int min_eigen_axis=0;
            float min_eigen_value=eigen_value[0];
            float eigen_sum=0;
            for(int i=1;i<3;i++){//x,y,z
                if(eigen_value[i]<min_eigen_value){
                    min_eigen_value=eigen_value[i];
                    min_eigen_axis=i;
                }
                //正規化用にnorm計算しておく
                eigen_sum += eigen_value[i];
            }

            asm volatile("mov.u64  %0, %globaltimer;" : "=l"(eigen_stop));
            eigen_time[idx]=eigen_stop - eigen_start;

            normals[idx*3+0]=eigen_vector[min_eigen_axis+0];
            normals[idx*3+1]=eigen_vector[min_eigen_axis+3];
            normals[idx*3+2]=eigen_vector[min_eigen_axis+6];

            curvatures[idx]=min_eigen_value/eigen_sum;

            // if(idx==output_id){
            //     printf("normals(%d) = %f, %f, %f\n\n\n\n",idx,normals[idx*3+0],normals[idx*3+1],normals[idx*3+2]);
            //     printf("curvature(%d) = %f\n",idx,curvatures[idx]);
            // }

            //デバッグ用
            // normals[idx*3+0]=idx*10+0;
            // normals[idx*3+1]=idx*10+1;
            // normals[idx*3+2]=idx*10+2;
            // printf("normal_x = %f ,normal_y = %f ,normal_z = %f \n", normals[idx*3+1],normals[idx*3+2],normals[idx*3+3]);
        }
        else{
            normals[idx*3+0]=0;
            normals[idx*3+1]=0;
            normals[idx*3+2]=0;
            curvatures[idx]=0;
            covariance_time[idx]=0;
            eigen_time[idx]=0;
        }
    }
    
}

// __global__ void ChildKernel(void* data)
// {

// 	printf("child : %d, %d\n", blockIdx.x, threadIdx.x);

// }

// __global__ void ParentKernel(void* data)
// {

// 	printf("parent: %d, %d\n", blockIdx.x, threadIdx.x);

// 	ChildKernel<<<1, 2>>>(data);
// 	cudaDeviceSynchronize();

// }

// __global__ void KernelFunctionArgumentTypeCheck(node* nodes)
// {
// 	unsigned int ix = threadIdx.x + blockIdx.x * blockDim.x;
//     unsigned int idx = ix;
// 	printf("KernelFunctionArgumentTypeCheck: %d, %d\n", blockIdx.x, threadIdx.x);
// 	for(int i=0;i<3;i++){
// 		nodes[i].parent_id = i; nodes[i].left_id = i; nodes[i].right_id = i; nodes[i].axis = i;
// 	}
// }

// __global__ void MyKernel(float* devPtr, size_t pitch, int width, int height)
// {
//   for (int r = 0; r < height; ++r) {
//     float* row = (float*)((char*)devPtr + r * pitch);
//     for (int c = 0; c < width; ++c) {
//       float element = row[c];
//     }
//   }
// }

// __global__ void d_ParallelRecursionTest(int data_size,int_with_ready* data)
// {
// 	unsigned int ix = threadIdx.x + blockIdx.x * blockDim.x;
//     unsigned int idx = ix;
// 	if(idx<data_size){
// 		if(data[idx].ready){
// 			if(idx!=0) data[idx].i = data[idx-1].i + 1;
// 			data[idx+1].ready = true;
// 		}
// 	}
// }

extern void ComputeNormals(std::vector<long long int>& neighbor_time,std::vector<int>& point_neighbor,std::vector<std::vector<float>> points_array,std::vector<std::vector<int>> neighbor_points_indices,std::vector<int> neighbor_start_indices,int neighbor_points_count,std::vector<std::vector<float>>& normals_array,std::vector<float>& curvatures_array,std::vector<long long int>& covariance_compute_time,std::vector<long long int>& eigen_compute_time)
{


	// points_array.clear();
	// points_array.resize(8);
	// points_array = {{6, 0, 0}, 
	// 				{5, 3, 0},
	// 				{3, 4, 0},
	// 				{4, 6, 0},
	// 				{2, 5, 0},
	// 				{1, 2, 0},
	// 				{0, 1, 0},
	// 				{-3.21161e+38,4.57384e-41,-3.21161e+38}};
	// // points_array.resize(7);

	// int test_size = 684;
	// if(points_array.size()>test_size) points_array.resize(test_size);

	clock_t build_start,build_end;
	// build_start = clock();
	// if(frames==43) std::cout<<"dead point is ("<<points_array[77][0]<<","<<points_array[77][0]<<","<<points_array[77][0]<<")"<<std::endl;

	
	/////////////////////////////////////////////////////////////////////////////////////////
	int root_id=-1;
	std::vector <node> nodes;
	nodes.resize(points_array.size());
	std::vector<std::vector<int>> axis_sort_ids(3,std::vector<int>(points_array.size()));
	point_with_id point_with_ids[points_array.size()];
	for(int i=0;i<points_array.size();i++){
		point_with_ids[i].id = i;
		point_with_ids[i].pos[0] = points_array[i][0];
		point_with_ids[i].pos[1] = points_array[i][1];
		point_with_ids[i].pos[2] = points_array[i][2];
	}
	for(sort_axis=0; sort_axis<3; sort_axis++){
		qsort(point_with_ids, points_array.size(), sizeof(point_with_id), AxisSort);
		for (int i=0 ; i < points_array.size() ; i++){
			axis_sort_ids[sort_axis][i]=point_with_ids[i].id;
		}
	}
	int create_end = CreateNode(&root_id,points_array.size(),nodes,axis_sort_ids,0,-1,false);
	/////////////////////////////////////////////////////////////////////////////////////////


	
	/////////////////////////////////////////////////////////////////////////////////////////施工

	// if(frames==0){
	// 	std::cout<<"size = "<<points_array.size()<<std::endl;
	// 	std::cout<<"points_array = {";
	// 	for(int i=0;i<points_array.size();i++){
	// 		std::cout<<"{";
	// 		for(int j=0;j<3;j++){
	// 			std::cout<<points_array[i][j];
	// 			if(j!=(3-1)) std::cout<<",";
	// 		}
	// 		std::cout<<"}";
	// 		if(i!=(points_array.size()-1)) std::cout<<",";
	// 		std::cout<<std::endl;
	// 	}
	// 	std::cout<<"};"<<std::endl;
	// }


	if(first){
		int test_size = 4928;
		std::vector<std::vector<float>> test_points(test_size);
        test_points = {{-0.80979,-0.259579,1.09037},
                        {-0.810456,-0.223266,1.07764},
                        {-0.798821,-0.210071,1.09375},
                        {-0.810498,-0.175017,1.07209},
                        {-0.798762,-0.181714,1.0944},
                        {-0.81284,-0.12623,1.07858},
                        {-0.818543,-0.0772444,1.08231},
                        {-0.829074,-0.0359512,1.08867},
                        {-0.795893,-0.359357,1.14161},
                        {-0.802727,-0.307455,1.10882},
                        {-0.790968,-0.321059,1.12994},
                        {-0.806671,-0.273842,1.1159},
                        {-0.79007,-0.27846,1.13649},
                        {-0.801596,-0.232404,1.1267},
                        {-0.797686,-0.224305,1.1255},
                        {-0.80175,-0.175938,1.14394},
                        {-0.797603,-0.174341,1.11838},
                        {-0.803066,-0.120312,1.11829},
                        {-0.799264,-0.135588,1.12279},
                        {-0.814617,-0.0760617,1.1217},
                        {-0.828954,-0.026245,1.12214},
                        {-0.795769,-0.00492308,1.14708},
                        {-0.83725,0.00508333,1.13942},
                        {-0.796286,0.00357143,1.14771},
                        {-0.803,-0.3888,1.1688},
                        {-0.795865,-0.366058,1.16177},
                        {-0.783607,-0.323902,1.1657},
                        {-0.775632,-0.272917,1.16869},
                        {-0.801727,-0.210182,1.15291},
                        {-0.781183,-0.224118,1.16931},
                        {-0.803528,-0.173124,1.16389},
                        {-0.783135,-0.176116,1.18127},
                        {-0.808733,-0.122988,1.17395},
                        {-0.789413,-0.128232,1.18932},
                        {-0.817457,-0.0744204,1.17681},
                        {-0.790119,-0.0686069,1.19057},
                        {-0.855423,-0.00626923,1.17196},
                        {-0.825442,-0.0284047,1.16463},
                        {-0.776136,-0.023892,1.17429},
                        {-0.861123,0.01,1.18231},
                        {-0.814041,0.0287805,1.15916},
                        {-0.775774,0.0255172,1.17574},
                        {-0.743886,0.0419143,1.19634},
                        {-0.809876,0.0721856,1.16998},
                        {-0.776519,0.0739015,1.18541},
                        {-0.741184,0.0683908,1.19636},
                        {-0.804071,0.105214,1.19057},
                        {-0.769435,0.121478,1.1862},
                        {-0.727952,0.138602,1.18956},
                        {-0.699,0.1465,1.1985},
                        {-0.753167,0.154833,1.17075},
                        {-0.719431,0.172695,1.17962},
                        {-0.696906,0.178156,1.19609},
                        {-0.70591,0.21891,1.17887},
                        {-0.69772,0.21896,1.1939},
                        {-0.703,0.253714,1.195},
                        {-0.690667,0.263,1.19467},
                        {-0.784,-0.2577,1.2246},
                        {-0.777834,-0.220926,1.22815},
                        {-0.768879,-0.173295,1.22892},
                        {-0.770431,-0.125043,1.22843},
                        {-0.74546,-0.117243,1.247},
                        {-0.7704,-0.0782196,1.22124},
                        {-0.735737,-0.0728718,1.24211},
                        {-0.755975,-0.0281709,1.21573},
                        {-0.729069,-0.023931,1.23613},
                        {-0.866786,0.0137857,1.2095},
                        {-0.754484,0.00925806,1.20406},
                        {-0.727878,0.0236853,1.22106},
                        {-0.7525,0.098,1.2},
                        {-0.717597,0.0753229,1.21103},
                        {-0.692703,0.0898649,1.22162},
                        {-0.754643,0.105643,1.20021},
                        {-0.721269,0.115058,1.20705},
                        {-0.690109,0.12643,1.21699},
                        {-0.690012,0.173555,1.21252},
                        {-0.7,0.244,1.2},
                        {-0.694012,0.221456,1.20916},
                        {-0.702308,0.261308,1.20454},
                        {-0.696,0.255813,1.20594},
                        {-0.792625,-0.25475,1.2575},
                        {-0.77311,-0.2216,1.25904},
                        {-0.756028,-0.182407,1.26107},
                        {-0.744931,-0.163778,1.26749},
                        {-0.7515,-0.148,1.2505},
                        {-0.73692,-0.124739,1.26949},
                        {-0.719446,-0.0751554,1.27076},
                        {-0.705435,-0.0295442,1.26471},
                        {-0.694065,-0.0144516,1.2811},
                        {-0.708627,0.0291466,1.26389},
                        {-0.690028,0.0210112,1.28401},
                        {-0.711514,0.0620561,1.27724},
                        {-0.693731,0.0648077,1.29458},
                        {-0.754667,-0.178333,1.30433},
                        {-0.738545,-0.129,1.30418},
                        {-0.716222,-0.0731667,1.30417},
                        {-0.703,-0.047,1.30067},
                        {-0.679311,0.0379556,1.30336},
                        {-0.686465,0.0576512,1.30428},
                        {0.5896,-1.3208,1.6446},
                        {0.635229,-1.31503,1.63454},
                        {0.661533,-1.30703,1.6251},
                        {0.467,-1.3383,1.6944},
                        {0.52475,-1.3268,1.6842},
                        {0.570125,-1.31965,1.67983},
                        {0.633214,-1.31236,1.66336},
                        {0.657,-1.30614,1.66471},
                        {0.126143,0.842929,1.69725},
                        {0.0875,0.8825,1.699},
                        {0.124333,0.872343,1.69769},
                        {0.336067,-1.3616,1.74433},
                        {0.374667,-1.35442,1.73709},
                        {0.392333,-1.35,1.74467},
                        {0.436594,-1.34266,1.72944},
                        {0.472481,-1.33568,1.72558},
                        {0.52496,-1.32544,1.72428},
                        {0.5835,-1.3153,1.72608},
                        {0.617857,-1.30986,1.73286},
                        {0.654889,-1.30356,1.73278},
                        {0.6778,-1.29713,1.74013},
                        {0.730833,-1.28683,1.74267},
                        {0.772478,-1.28339,1.71813},
                        {-0.159444,0.823,1.74844},
                        {-0.121958,0.83038,1.74631},
                        {-0.0756915,0.835266,1.74222},
                        {-0.0392258,0.837581,1.74019},
                        {0.0821549,0.841296,1.71552},
                        {0.120355,0.833882,1.71233},
                        {-0.117682,0.8925,1.74486},
                        {-0.0665428,0.869686,1.74509},
                        {-0.043375,0.866875,1.74342},
                        {0.0476957,0.879826,1.72635},
                        {0.0734939,0.87342,1.70762},
                        {0.109405,0.877548,1.7011},
                        {-0.214,0.919947,1.74326},
                        {-0.1805,0.922455,1.74609},
                        {-0.117226,0.911094,1.74215},
                        {-0.0733571,0.911229,1.74449},
                        {-0.048,0.923,1.749},
                        {0.0429167,0.905667,1.72867},
                        {0.0584286,0.901286,1.70957},
                        {-0.684143,0.960572,1.74214},
                        {0.190563,-1.38475,1.792},
                        {0.227208,-1.38063,1.78792},
                        {0.276364,-1.37252,1.77865},
                        {0.324797,-1.36273,1.77446},
                        {0.36552,-1.35592,1.77412},
                        {0.414,-1.35117,1.78817},
                        {0.388156,-1.34909,1.77716},
                        {0.423268,-1.34496,1.77513},
                        {0.473338,-1.33363,1.77417},
                        {0.522764,-1.3272,1.77556},
                        {0.577529,-1.31804,1.76955},
                        {0.622214,-1.30707,1.77575},
                        {0.665,-1.3035,1.77475},
                        {0.684889,-1.29772,1.76567},
                        {0.727094,-1.29069,1.77259},
                        {0.759429,-1.28664,1.77714},
                        {0.8015,-1.2815,1.7915},
                        {1.249,-0.279,1.79867},
                        {1.241,-0.23275,1.79875},
                        {1.23088,-0.172923,1.79577},
                        {1.22103,-0.126412,1.79368},
                        {1.2125,-0.0790909,1.79259},
                        {1.198,-0.018,1.79406},
                        {1.20361,-0.0285714,1.78975},
                        {1.19279,0.0277907,1.7897},
                        {1.2025,0.00966667,1.78667},
                        {1.18575,0.0713,1.7894},
                        {1.201,0.051,1.788},
                        {1.17944,0.12604,1.7934},
                        {1.16826,0.172,1.7867},
                        {1.16407,0.211,1.78971},
                        {-0.111579,0.787553,1.796},
                        {-0.0744382,0.78618,1.78767},
                        {-0.0465,0.791875,1.79313},
                        {-0.20835,0.825625,1.77618},
                        {-0.175417,0.820262,1.76561},
                        {-0.124992,0.816561,1.77455},
                        {-0.0742248,0.815364,1.77342},
                        {-0.0403421,0.827369,1.77716},
                        {0.0896667,0.845333,1.762},
                        {0.1286,0.845,1.7606},
                        {-0.213606,0.878563,1.77721},
                        {-0.17747,0.876747,1.76694},
                        {-0.127434,0.875944,1.75906},
                        {-0.0837887,0.88138,1.75256},
                        {-0.03575,0.879625,1.76438},
                        {0.0423333,0.895,1.75433},
                        {0.0733333,0.854333,1.77083},
                        {0.1325,0.8555,1.7875},
                        {-0.719333,0.946333,1.78867},
                        {-0.271187,0.924219,1.79816},
                        {-0.222508,0.917763,1.77149},
                        {-0.176932,0.911712,1.75505},
                        {-0.14271,0.912613,1.752},
                        {-0.081875,0.903875,1.75013},
                        {-0.0411333,0.921133,1.77257},
                        {0.038,0.92,1.762},
                        {-0.707455,0.953364,1.788},
                        {-0.675273,0.966832,1.77774},
                        {-0.63065,0.97635,1.78975},
                        {-0.596,0.983,1.798},
                        {-0.2758,0.9546,1.7978},
                        {-0.249,0.954,1.796},
                        {-0.03,0.95,1.799},
                        {-0.0065,-1.42675,1.847},
                        {0.0279697,-1.41882,1.84233},
                        {0.07668,-1.40946,1.83552},
                        {0.114065,-1.40197,1.82452},
                        {0.154,-1.401,1.808},
                        {0.132837,-1.3963,1.8293},
                        {0.173073,-1.38961,1.82344},
                        {0.224567,-1.3786,1.82342},
                        {0.273041,-1.37201,1.82248},
                        {0.322118,-1.36385,1.82162},
                        {0.361242,-1.35585,1.82252},
                        {0.3842,-1.3488,1.8044},
                        {0.427629,-1.34191,1.82121},
                        {0.47578,-1.3339,1.82106},
                        {0.521057,-1.328,1.81903},
                        {0.579953,-1.31642,1.8193},
                        {0.6106,-1.3118,1.8157},
                        {0.694,-1.301,1.83525},
                        {0.693,-1.2985,1.8335},
                        {0.7038,-1.298,1.8212},
                        {0.807,-1.283,1.823},
                        {1.26478,-0.355556,1.83067},
                        {1.24775,-0.30475,1.81575},
                        {1.25641,-0.327148,1.81859},
                        {1.24404,-0.275149,1.81102},
                        {1.25238,-0.2755,1.82844},
                        {1.23998,-0.223574,1.81562},
                        {1.25,-0.249,1.842},
                        {1.22906,-0.17732,1.81638},
                        {1.22009,-0.126,1.81791},
                        {1.21289,-0.0749143,1.81994},
                        {1.19829,-0.0191429,1.80357},
                        {1.207,-0.02628,1.82548},
                        {1.19235,0.025,1.80847},
                        {1.20492,0.0199167,1.83575},
                        {1.18983,0.0705556,1.815},
                        {1.2035,0.079,1.847},
                        {1.18389,0.121,1.81256},
                        {1.201,0.119,1.849},
                        {1.1811,0.1608,1.8226},
                        {1.16829,0.215143,1.81043},
                        {-0.244778,0.592222,1.84733},
                        {-0.2532,0.6171,1.8462},
                        {-0.227326,0.627058,1.8417},
                        {-0.178932,0.640695,1.83761},
                        {-0.145429,0.647,1.83743},
                        {-0.523,0.66,1.84789},
                        {-0.210917,0.662695,1.84522},
                        {-0.172895,0.6635,1.83932},
                        {-0.12404,0.666525,1.83618},
                        {-0.0858491,0.677245,1.83379},
                        {-0.225,0.71,1.849},
                        {-0.167581,0.741512,1.84309},
                        {-0.125687,0.741194,1.83685},
                        {-0.0767536,0.742145,1.82949},
                        {-0.354,0.799,1.847},
                        {-0.348,0.799,1.849},
                        {-0.260391,0.790957,1.84652},
                        {-0.221087,0.779039,1.83784},
                        {-0.174837,0.77362,1.82281},
                        {-0.12791,0.770026,1.81565},
                        {-0.0748716,0.764606,1.81282},
                        {-0.0445526,0.777947,1.82574},
                        {-0.363116,0.827928,1.84332},
                        {-0.32524,0.825027,1.83517},
                        {-0.274877,0.82395,1.83008},
                        {-0.23125,0.82265,1.83341},
                        {-0.176192,0.805489,1.8087},
                        {-0.138933,0.804533,1.80113},
                        {-0.0361463,0.825439,1.82054},
                        {-0.56675,0.89725,1.8485},
                        {-0.518875,0.89025,1.84825},
                        {-0.48431,0.884429,1.84683},
                        {-0.371137,0.875893,1.83098},
                        {-0.325559,0.875013,1.82166},
                        {-0.275442,0.87449,1.81206},
                        {-0.238042,0.875945,1.82786},
                        {-0.026,0.881,1.807},
                        {0.074,0.868,1.806},
                        {-0.7135,0.9482,1.82855},
                        {-0.671243,0.942135,1.84587},
                        {-0.620411,0.930878,1.8454},
                        {-0.574408,0.924192,1.84301},
                        {-0.526511,0.926496,1.84124},
                        {-0.482989,0.927337,1.8429},
                        {-0.403889,0.9305,1.83667},
                        {-0.374276,0.924935,1.82208},
                        {-0.325694,0.923782,1.81193},
                        {-0.276637,0.923062,1.80396},
                        {-0.243659,0.919886,1.81568},
                        {-0.0312609,0.929522,1.82296},
                        {-0.707389,0.952111,1.81872},
                        {-0.67299,0.95954,1.82459},
                        {-0.624345,0.965673,1.82533},
                        {-0.5762,0.969926,1.82456},
                        {-0.526033,0.969124,1.82764},
                        {-0.478861,0.969593,1.83448},
                        {-0.41025,0.96775,1.82984},
                        {-0.374958,0.96157,1.81861},
                        {-0.327388,0.95798,1.80996},
                        {-0.274103,0.954828,1.80493},
                        {-0.2495,0.952,1.8055},
                        {-0.0268333,0.955333,1.83233},
                        {-0.269,-1.47137,1.89431},
                        {-0.223778,-1.46419,1.89137},
                        {-0.179967,-1.45557,1.88823},
                        {-0.142857,-1.45257,1.87857},
                        {-0.1569,-1.4485,1.8787},
                        {-0.123682,-1.44643,1.88439},
                        {-0.0743521,-1.43706,1.87728},
                        {-0.0248072,-1.42836,1.8759},
                        {0.0246,-1.41975,1.87474},
                        {0.0743548,-1.409,1.874},
                        {0.1048,-1.4019,1.875},
                        {0.128552,-1.39667,1.87252},
                        {0.174261,-1.39001,1.87478},
                        {0.221938,-1.38008,1.87475},
                        {0.275281,-1.37236,1.8727},
                        {0.325323,-1.36288,1.87441},
                        {0.3635,-1.3575,1.87582},
                        {0.405833,-1.35167,1.87717},
                        {0.423875,-1.34552,1.87418},
                        {0.477286,-1.33521,1.88436},
                        {0.502333,-1.33533,1.883},
                        {0.575364,-1.31941,1.87518},
                        {0.626,-1.313,1.8625},
                        {0.8305,-1.28121,1.88479},
                        {0.866077,-1.27346,1.89038},
                        {1.28213,-0.367533,1.88107},
                        {1.27167,-0.325424,1.87558},
                        {1.26596,-0.276792,1.87542},
                        {1.2475,-0.2205,1.853},
                        {1.25997,-0.225581,1.88084},
                        {1.24473,-0.169591,1.87018},
                        {1.25383,-0.182444,1.89072},
                        {1.24021,-0.126116,1.87788},
                        {1.23239,-0.0756327,1.87794},
                        {1.22504,-0.0236808,1.87815},
                        {1.21984,0.0249737,1.88189},
                        {1.21633,0.0777222,1.88683},
                        {1.20919,0.123375,1.88969},
                        {1.18952,0.178719,1.88875},
                        {1.2015,0.16375,1.88725},
                        {1.17583,0.22298,1.89071},
                        {1.18526,0.264316,1.89489},
                        {-0.569182,0.586636,1.88445},
                        {-0.531293,0.590805,1.88551},
                        {-0.25225,0.59325,1.85675},
                        {-0.23975,0.58655,1.8603},
                        {-0.5685,0.606125,1.89112},
                        {-0.523578,0.636978,1.87691},
                        {-0.490577,0.643539,1.88146},
                        {-0.406,0.635,1.89367},
                        {-0.375073,0.636463,1.88039},
                        {-0.325349,0.641395,1.86693},
                        {-0.267947,0.637316,1.86187},
                        {-0.231025,0.627425,1.8623},
                        {-0.181,0.635917,1.85725},
                        {-0.141,0.649,1.854},
                        {-0.562205,0.679077,1.88926},
                        {-0.521605,0.673686,1.85923},
                        {-0.478411,0.675071,1.88194},
                        {-0.3545,0.6857,1.8925},
                        {-0.32405,0.676562,1.87563},
                        {-0.275922,0.67413,1.86381},
                        {-0.231147,0.678156,1.85473},
                        {-0.177424,0.688848,1.86194},
                        {-0.131087,0.68387,1.85861},
                        {-0.077875,0.690125,1.85638},
                        {-0.562654,0.713923,1.89131},
                        {-0.521162,0.721629,1.8664},
                        {-0.477665,0.726174,1.88325},
                        {-0.446571,0.743714,1.89686},
                        {-0.357,0.729636,1.88327},
                        {-0.325129,0.725285,1.87011},
                        {-0.276382,0.725278,1.86449},
                        {-0.22402,0.725607,1.85924},
                        {-0.177333,0.724423,1.87772},
                        {-0.126083,0.723909,1.87346},
                        {-0.07564,0.724424,1.86633},
                        {-0.0297077,0.734169,1.88403},
                        {0.00566667,0.745667,1.89267},
                        {-0.62175,0.7985,1.899},
                        {-0.567614,0.788955,1.89405},
                        {-0.525957,0.778925,1.88888},
                        {-0.472419,0.772114,1.87383},
                        {-0.438317,0.776024,1.88846},
                        {-0.36309,0.775806,1.86936},
                        {-0.325147,0.773853,1.85928},
                        {-0.277392,0.771815,1.85995},
                        {-0.231948,0.760689,1.85333},
                        {-0.195667,0.752333,1.852},
                        {-0.054,0.75,1.87833},
                        {-0.0407097,0.767161,1.87794},
                        {0.00842857,0.753429,1.89429},
                        {-0.665138,0.827888,1.88913},
                        {-0.625136,0.824686,1.88708},
                        {-0.576406,0.824834,1.88134},
                        {-0.524975,0.822779,1.87491},
                        {-0.47537,0.825792,1.86917},
                        {-0.440982,0.822737,1.88014},
                        {-0.380756,0.82161,1.86185},
                        {-0.321348,0.802478,1.85404},
                        {-0.28925,0.801875,1.85363},
                        {-0.223667,0.827,1.853},
                        {-0.0301538,0.823154,1.87192},
                        {-0.7041,0.8854,1.8826},
                        {-0.672971,0.874459,1.86873},
                        {-0.624754,0.874404,1.86441},
                        {-0.574951,0.873077,1.85909},
                        {-0.527125,0.872684,1.85834},
                        {-0.470429,0.870581,1.85721},
                        {-0.442,0.87362,1.88276},
                        {-0.3924,0.8627,1.8548},
                        {-0.0216667,0.854333,1.87867},
                        {0.142,0.893,1.895},
                        {-0.71288,0.92768,1.8696},
                        {-0.675989,0.918989,1.85362},
                        {-0.632214,0.908929,1.85105},
                        {-0.596333,0.904333,1.85033},
                        {-0.521111,0.903556,1.85128},
                        {-0.460757,0.920743,1.85767},
                        {-0.441691,0.924452,1.88},
                        {-0.024,0.938438,1.87194},
                        {-0.728455,0.952,1.86573},
                        {-0.455233,0.974837,1.86716},
                        {-0.438062,0.982167,1.874},
                        {-0.0195,0.970292,1.88092},
                        {-0.445,1.00033,1.86033},
                        {-0.918143,-1.58957,1.93829},
                        {-0.877731,-1.58665,1.93838},
                        {-0.820385,-1.57096,1.93458},
                        {-0.77525,-1.5622,1.93657},
                        {-0.726035,-1.55703,1.93721},
                        {-0.69,-1.5548,1.9405},
                        {-0.709667,-1.549,1.939},
                        {-0.665947,-1.54205,1.93453},
                        {-0.626775,-1.53658,1.93438},
                        {-0.572694,-1.52953,1.9355},
                        {-0.525025,-1.51915,1.93248},
                        {-0.474429,-1.50851,1.92871},
                        {-0.431629,-1.50311,1.9274},
                        {-0.413059,-1.49794,1.93129},
                        {-0.374123,-1.49206,1.9272},
                        {-0.325437,-1.48263,1.92399},
                        {-0.275871,-1.47087,1.9233},
                        {-0.225859,-1.46433,1.92491},
                        {-0.175741,-1.45412,1.92379},
                        {-0.133,-1.451,1.914},
                        {-0.167,-1.44971,1.92286},
                        {-0.123243,-1.44537,1.92616},
                        {-0.0748,-1.43672,1.92471},
                        {-0.0260351,-1.42728,1.92516},
                        {0.0235278,-1.42114,1.92408},
                        {0.0741268,-1.41268,1.92465},
                        {0.113074,-1.40674,1.9307},
                        {0.133788,-1.39673,1.92076},
                        {0.174895,-1.39258,1.92905},
                        {0.224,-1.38262,1.92819},
                        {0.275167,-1.37353,1.92685},
                        {0.324215,-1.36557,1.92395},
                        {0.373105,-1.35795,1.92311},
                        {0.4102,-1.35267,1.9288},
                        {0.399,-1.35,1.941},
                        {0.432417,-1.34817,1.9145},
                        {0.476467,-1.34083,1.92107},
                        {0.52025,-1.332,1.94725},
                        {0.580703,-1.32346,1.9277},
                        {0.638348,-1.31278,1.92717},
                        {0.674455,-1.30627,1.93745},
                        {0.703,-1.31,1.939},
                        {0.7875,-1.29007,1.93471},
                        {0.829,-1.28417,1.932},
                        {0.8785,-1.27432,1.91932},
                        {1.32445,-0.511636,1.94336},
                        {1.319,-0.475238,1.94329},
                        {1.2936,-0.415,1.94633},
                        {1.3064,-0.423721,1.93333},
                        {1.28788,-0.375116,1.93312},
                        {1.3005,-0.397,1.906},
                        {1.27995,-0.323427,1.93213},
                        {1.26984,-0.275185,1.92964},
                        {1.24669,-0.213125,1.9425},
                        {1.26073,-0.227741,1.92574},
                        {1.24238,-0.170741,1.93064},
                        {1.25476,-0.182176,1.91697},
                        {1.23717,-0.125069,1.92668},
                        {1.25,-0.148,1.915},
                        {1.22792,-0.0747053,1.92375},
                        {1.21854,-0.0255238,1.92093},
                        {1.1985,0.041625,1.93138},
                        {1.2106,0.0239082,1.91766},
                        {1.19425,0.0757705,1.92672},
                        {1.21027,0.0735637,1.90845},
                        {1.18693,0.126202,1.91963},
                        {1.2062,0.117933,1.90687},
                        {1.17867,0.170818,1.91839},
                        {1.16588,0.224854,1.92735},
                        {1.1708,0.276913,1.91606},
                        {1.14833,0.343111,1.91944},
                        {1.16384,0.321268,1.91766},
                        {1.14432,0.374254,1.92921},
                        {1.1565,0.360643,1.91093},
                        {1.13429,0.421265,1.93968},
                        {1.099,0.48875,1.9475},
                        {1.11564,0.465091,1.94573},
                        {-0.605667,0.592667,1.935},
                        {-0.587857,0.580714,1.909},
                        {-0.529077,0.589462,1.90892},
                        {-0.606471,0.628118,1.94071},
                        {-0.586788,0.628636,1.925},
                        {-0.526667,0.623417,1.91017},
                        {-0.466478,0.641435,1.94465},
                        {-0.4295,0.641462,1.92985},
                        {-0.391,0.648,1.901},
                        {-0.235385,0.626,1.925},
                        {1.01264,0.622,1.948},
                        {-0.65175,0.69575,1.9445},
                        {-0.620741,0.677899,1.93739},
                        {-0.585053,0.671211,1.91803},
                        {-0.53,0.654667,1.91783},
                        {-0.461037,0.662815,1.92315},
                        {-0.432758,0.676468,1.92903},
                        {-0.366714,0.676429,1.91386},
                        {-0.347,0.667,1.9},
                        {-0.284,0.656,1.913},
                        {0.942778,0.685333,1.94678},
                        {0.960809,0.687429,1.94586},
                        {-0.66063,0.726522,1.9397},
                        {-0.62568,0.725296,1.9282},
                        {-0.582683,0.730127,1.91067},
                        {-0.453923,0.716385,1.90308},
                        {-0.427837,0.724612,1.92},
                        {-0.3666,0.7232,1.9184},
                        {-0.058,0.720667,1.92183},
                        {-0.0308333,0.7335,1.91817},
                        {0.005,0.746333,1.90867},
                        {0.882445,0.743778,1.948},
                        {0.934539,0.730865,1.94613},
                        {0.956591,0.715364,1.94473},
                        {-0.664382,0.774416,1.91871},
                        {-0.624588,0.774573,1.91086},
                        {-0.577874,0.767421,1.90578},
                        {-0.5464,0.7665,1.9024},
                        {-0.421038,0.76949,1.91626},
                        {-0.377,0.758,1.907},
                        {-0.028375,0.771344,1.92269},
                        {0.0162941,0.761588,1.91994},
                        {0.826,0.79625,1.949},
                        {0.886389,0.769695,1.94717},
                        {0.916806,0.772194,1.94615},
                        {-0.705333,0.842333,1.92733},
                        {-0.687259,0.823185,1.90889},
                        {-0.625833,0.801833,1.9005},
                        {-0.428744,0.825628,1.91974},
                        {-0.0234828,0.825034,1.92783},
                        {0.82687,0.82713,1.94594},
                        {0.874711,0.825639,1.94382},
                        {0.906072,0.812214,1.94371},
                        {-0.705714,0.863,1.90943},
                        {-0.434091,0.87203,1.91755},
                        {-0.0156316,0.861895,1.92753},
                        {0.0804444,0.896278,1.93478},
                        {0.129077,0.897538,1.93054},
                        {0.153,0.8975,1.9035},
                        {0.778,0.877473,1.94736},
                        {0.825418,0.875628,1.94358},
                        {0.865017,0.871441,1.94207},
                        {-0.7275,0.91725,1.912},
                        {-0.44005,0.92195,1.917},
                        {-0.015,0.948,1.914},
                        {0.096,0.902,1.945},
                        {0.134692,0.902308,1.938},
                        {0.160857,0.903714,1.93007},
                        {0.696833,0.940167,1.94733},
                        {0.729567,0.932917,1.94775},
                        {0.774736,0.923972,1.94491},
                        {0.821481,0.923481,1.93809},
                        {0.854125,0.90725,1.93825},
                        {-0.444032,0.978194,1.91765},
                        {-0.0108182,0.980182,1.91914},
                        {0.639,0.996,1.9485},
                        {0.685593,0.979333,1.94709},
                        {0.725766,0.975477,1.94379},
                        {0.774216,0.974646,1.93946},
                        {0.812818,0.971091,1.94004},
                        {-0.007,1.00567,1.92489},
                        {0.026,1.022,1.941},
                        {0.64775,1.028,1.948},
                        {0.674387,1.02357,1.94483},
                        {0.724028,1.02481,1.94267},
                        {0.771812,1.02247,1.9339},
                        {0.804,1.00433,1.93167},
                        {0.635769,1.05877,1.94669},
                        {0.676738,1.05769,1.94221},
                        {0.727471,1.06033,1.93875},
                        {0.765367,1.06093,1.93407},
                        {-0.969,-1.601,1.987},
                        {-0.962,-1.59875,1.98575},
                        {-0.924528,-1.58881,1.97617},
                        {-0.87619,-1.58286,1.97281},
                        {-0.8263,-1.57298,1.97572},
                        {-0.773896,-1.56113,1.97554},
                        {-0.724793,-1.55603,1.97743},
                        {-0.6912,-1.5519,1.9752},
                        {-0.722,-1.549,1.999},
                        {-0.669,-1.5452,1.9757},
                        {-0.622196,-1.53737,1.97484},
                        {-0.575604,-1.52834,1.97426},
                        {-0.525468,-1.51697,1.9725},
                        {-0.476196,-1.51102,1.97561},
                        {-0.431071,-1.50243,1.97357},
                        {-0.4075,-1.49743,1.97064},
                        {-0.381415,-1.49249,1.9719},
                        {-0.320565,-1.48187,1.97502},
                        {-0.275368,-1.47318,1.97619},
                        {-0.223889,-1.46497,1.97524},
                        {-0.177105,-1.45677,1.97553},
                        {-0.14875,-1.452,1.98225},
                        {-0.155,-1.449,1.951},
                        {-0.125352,-1.44463,1.97302},
                        {-0.07228,-1.43688,1.97224},
                        {-0.0266269,-1.42854,1.97355},
                        {0.0240323,-1.42039,1.97356},
                        {0.075058,-1.41202,1.9748},
                        {0.120211,-1.4037,1.97435},
                        {0.152,-1.4,1.993},
                        {0.143333,-1.3985,1.97083},
                        {0.175868,-1.39118,1.97462},
                        {0.223817,-1.38407,1.97378},
                        {0.274049,-1.3757,1.97516},
                        {0.324652,-1.36654,1.97386},
                        {0.373057,-1.35715,1.97453},
                        {0.412167,-1.35306,1.97028},
                        {0.44575,-1.34475,1.99175},
                        {0.4742,-1.3405,1.98088},
                        {0.528023,-1.33161,1.97709},
                        {0.577667,-1.3234,1.97864},
                        {0.623522,-1.31622,1.97915},
                        {0.671592,-1.30643,1.97671},
                        {0.722529,-1.304,1.98965},
                        {0.7455,-1.2965,1.991},
                        {0.780727,-1.29048,1.97825},
                        {0.82517,-1.28298,1.97881},
                        {0.873442,-1.27591,1.98028},
                        {0.9286,-1.26949,1.97854},
                        {0.972364,-1.26355,1.988},
                        {1.34833,-0.707667,1.99633},
                        {1.3566,-0.7056,1.9962},
                        {1.34214,-0.670179,1.99025},
                        {1.35629,-0.674647,1.98918},
                        {1.33309,-0.624795,1.97909},
                        {1.3555,-0.6405,1.9835},
                        {1.32424,-0.572926,1.97069},
                        {1.31421,-0.525869,1.9675},
                        {1.29444,-0.468047,1.97274},
                        {1.30908,-0.4808,1.96068},
                        {1.28677,-0.426849,1.9685},
                        {1.30367,-0.438667,1.951},
                        {1.27744,-0.374426,1.97354},
                        {1.26915,-0.326194,1.97206},
                        {1.259,-0.275981,1.97465},
                        {1.24715,-0.214077,1.97223},
                        {1.2521,-0.236033,1.97877},
                        {1.24027,-0.175651,1.97614},
                        {1.22982,-0.124597,1.97623},
                        {1.21887,-0.0749873,1.97913},
                        {1.19048,-0.01224,1.99664},
                        {1.21042,-0.0276344,1.97997},
                        {1.14486,0.0388571,1.99843},
                        {1.1807,0.0293023,1.99437},
                        {1.20325,0.0216964,1.97154},
                        {1.09033,0.0966667,1.999},
                        {1.13288,0.0834833,1.99707},
                        {1.17941,0.0735865,1.98414},
                        {1.0452,0.1383,1.9983},
                        {1.0774,0.129286,1.99746},
                        {1.1253,0.124314,1.9937},
                        {1.17401,0.123876,1.97795},
                        {0.987125,0.19175,1.999},
                        {1.02445,0.17701,1.99627},
                        {1.07496,0.174009,1.99374},
                        {1.1247,0.173936,1.98872},
                        {1.16731,0.173798,1.97468},
                        {0.941389,0.234889,1.99878},
                        {0.975991,0.225746,1.99648},
                        {1.02477,0.223445,1.99246},
                        {1.07543,0.225117,1.98837},
                        {1.12431,0.2241,1.98276},
                        {1.16173,0.223506,1.96889},
                        {0.888214,0.283,1.99857},
                        {0.92796,0.277871,1.99541},
                        {0.974365,0.274198,1.99132},
                        {1.02455,0.27481,1.9869},
                        {1.07479,0.274736,1.98309},
                        {1.12446,0.274132,1.97834},
                        {1.15646,0.272712,1.96469},
                        {0.838214,0.342643,1.99829},
                        {0.877811,0.328874,1.99688},
                        {0.92476,0.324209,1.99215},
                        {0.975016,0.324752,1.98635},
                        {1.0244,0.325357,1.98087},
                        {1.07529,0.324508,1.97892},
                        {1.12641,0.32524,1.9705},
                        {1.15188,0.312588,1.95706},
                        {0.788083,0.390417,1.99804},
                        {0.82917,0.38017,1.99628},
                        {0.874078,0.373703,1.991},
                        {0.924515,0.374735,1.98643},
                        {0.973641,0.374778,1.9805},
                        {1.02388,0.374265,1.97651},
                        {1.07512,0.375115,1.97498},
                        {1.12526,0.373732,1.96741},
                        {0.693133,0.435733,1.99807},
                        {0.726977,0.43108,1.9959},
                        {0.775391,0.425376,1.99039},
                        {0.825163,0.425497,1.98551},
                        {0.875052,0.423821,1.98196},
                        {0.92496,0.424798,1.97817},
                        {0.974594,0.423903,1.97337},
                        {1.02413,0.425424,1.97111},
                        {1.07559,0.423926,1.96952},
                        {1.11837,0.422322,1.9611},
                        {0.697667,0.493333,1.999},
                        {0.727607,0.475473,1.9967},
                        {0.7739,0.474392,1.99027},
                        {0.824758,0.474395,1.98248},
                        {0.874288,0.475352,1.97613},
                        {0.924429,0.47404,1.96876},
                        {0.975268,0.47389,1.96356},
                        {1.02501,0.473823,1.95953},
                        {1.07529,0.473861,1.95913},
                        {1.10843,0.4626,1.95573},
                        {0.640333,0.544,1.99856},
                        {0.679112,0.530918,1.9969},
                        {0.724646,0.525354,1.9933},
                        {0.775234,0.523347,1.98719},
                        {0.824814,0.524109,1.97891},
                        {0.875101,0.524597,1.97407},
                        {0.924234,0.523281,1.96854},
                        {0.975,0.525367,1.96492},
                        {1.02414,0.524403,1.96024},
                        {1.0702,0.521341,1.95612},
                        {0.641846,0.573308,1.99792},
                        {0.674302,0.574119,1.99426},
                        {0.724458,0.575046,1.98856},
                        {0.773601,0.57482,1.98147},
                        {0.824405,0.574135,1.97565},
                        {0.873181,0.574055,1.96965},
                        {0.925254,0.574413,1.96576},
                        {0.974257,0.574805,1.9623},
                        {1.02415,0.574114,1.95958},
                        {1.05679,0.562947,1.95684},
                        {-0.652,0.6445,1.98875},
                        {-0.62888,0.6311,1.96694},
                        {-0.5686,0.6417,1.9757},
                        {-0.512444,0.626778,1.97433},
                        {-0.473084,0.626737,1.96535},
                        {-0.441722,0.635167,1.97269},
                        {-0.232233,0.629767,1.97237},
                        {0.579,0.638936,1.9981},
                        {0.625741,0.624344,1.99477},
                        {0.674843,0.623955,1.98943},
                        {0.723798,0.624544,1.98207},
                        {0.775582,0.624783,1.97695},
                        {0.825182,0.623929,1.97107},
                        {0.874983,0.623727,1.96388},
                        {0.924208,0.62436,1.95877},
                        {0.975036,0.623712,1.95604},
                        {1.01459,0.618633,1.95533},
                        {-0.661132,0.678711,1.96892},
                        {-0.636677,0.660871,1.95713},
                        {-0.574304,0.665609,1.98035},
                        {-0.541556,0.665111,1.98056},
                        {-0.4765,0.6545,1.9505},
                        {-0.413,0.68656,1.96828},
                        {-0.304,0.67,1.965},
                        {-0.229,0.652,1.997},
                        {0.496,0.699,1.999},
                        {0.533097,0.689645,1.998},
                        {0.577232,0.67552,1.99506},
                        {0.625086,0.674597,1.98995},
                        {0.674656,0.674304,1.98365},
                        {0.724921,0.67526,1.97843},
                        {0.7741,0.674408,1.97173},
                        {0.825416,0.674054,1.96472},
                        {0.874533,0.67432,1.95872},
                        {0.923514,0.672972,1.95445},
                        {0.974778,0.667792,1.95468},
                        {1.003,0.660375,1.95788},
                        {-0.677429,0.724,1.95819},
                        {-0.408345,0.720586,1.97014},
                        {-0.0555,0.7345,1.97217},
                        {-0.04525,0.74475,1.97825},
                        {0.486932,0.733864,1.99798},
                        {0.52535,0.724967,1.99536},
                        {0.5742,0.724392,1.99058},
                        {0.624344,0.72487,1.9848},
                        {0.674788,0.724409,1.97895},
                        {0.724254,0.723912,1.97194},
                        {0.77476,0.725108,1.96548},
                        {0.824496,0.723711,1.95839},
                        {0.873832,0.723028,1.95538},
                        {0.918539,0.719526,1.95288},
                        {0.97025,0.70675,1.95125},
                        {-0.691,0.761714,1.96086},
                        {-0.414462,0.772154,1.96692},
                        {-0.0216667,0.782583,1.97108},
                        {0.0144444,0.774833,1.97494},
                        {0.436333,0.769833,1.999},
                        {0.475436,0.774479,1.99619},
                        {0.524492,0.775616,1.99129},
                        {0.574175,0.77411,1.98638},
                        {0.62441,0.774524,1.97975},
                        {0.673682,0.773907,1.97235},
                        {0.724661,0.774162,1.96654},
                        {0.774383,0.775008,1.96085},
                        {0.823254,0.774008,1.95534},
                        {0.868247,0.77626,1.95283},
                        {0.916167,0.77,1.95075},
                        {-0.424182,0.817182,1.96864},
                        {-0.0148621,0.824569,1.97595},
                        {0.008375,0.820125,1.99538},
                        {0.431352,0.832019,1.99706},
                        {0.474089,0.824,1.99239},
                        {0.524893,0.824923,1.98583},
                        {0.574836,0.824156,1.98041},
                        {0.624297,0.823273,1.9733},
                        {0.674921,0.825159,1.96775},
                        {0.723734,0.823782,1.96144},
                        {0.773893,0.824728,1.9554},
                        {0.813481,0.817037,1.95122},
                        {0.8736,0.806067,1.95147},
                        {-0.431,0.866667,1.95667},
                        {-0.00604762,0.868953,1.97152},
                        {0.001,0.868875,1.99463},
                        {0.07855,0.89795,1.971},
                        {0.138571,0.898571,1.96629},
                        {0.378483,0.879862,1.99769},
                        {0.427563,0.877393,1.99383},
                        {0.474445,0.874222,1.98706},
                        {0.524484,0.875135,1.98073},
                        {0.574961,0.873651,1.97447},
                        {0.624822,0.874718,1.96927},
                        {0.673179,0.874431,1.9611},
                        {0.723849,0.874908,1.95681},
                        {0.76387,0.870087,1.95178},
                        {0.8098,0.8769,1.9501},
                        {0.893,0.85,1.95},
                        {-0.442833,0.921833,1.96633},
                        {-0.0095,0.9225,1.9755},
                        {0.0799445,0.902722,1.98194},
                        {0.122717,0.902804,1.9763},
                        {0.172083,0.911617,1.9786},
                        {0.32175,0.946167,1.99883},
                        {0.37875,0.924182,1.9954},
                        {0.425619,0.925064,1.98998},
                        {0.474566,0.925566,1.98245},
                        {0.524306,0.924395,1.97665},
                        {0.574463,0.924314,1.96941},
                        {0.623789,0.923838,1.9623},
                        {0.673579,0.924675,1.95743},
                        {0.7201,0.916234,1.9523},
                        {0.756333,0.913,1.95},
                        {-0.0073125,0.975063,1.977},
                        {0.027,0.993875,1.98612},
                        {0.29575,0.9935,1.998},
                        {0.334771,0.978984,1.99626},
                        {0.373706,0.973059,1.9901},
                        {0.424083,0.974934,1.98534},
                        {0.474333,0.973778,1.97768},
                        {0.523614,0.975,1.9726},
                        {0.573583,0.973475,1.96431},
                        {0.623935,0.974813,1.95749},
                        {0.666338,0.969103,1.95194},
                        {0.716,0.962571,1.95029},
                        {-0.003,1,1.971},
                        {0.0151129,1.02177,1.97858},
                        {0.247,1.02,1.999},
                        {0.279289,1.03008,1.99792},
                        {0.326648,1.02527,1.99453},
                        {0.374108,1.02343,1.98805},
                        {0.424274,1.02281,1.98062},
                        {0.474894,1.0248,1.97372},
                        {0.525094,1.02385,1.96827},
                        {0.574558,1.02448,1.96046},
                        {0.624413,1.02389,1.95399},
                        {0.65875,1.03025,1.95025},
                        {0.246,1.06,1.999},
                        {0.27924,1.05628,1.99632},
                        {0.32329,1.05458,1.98871},
                        {0.374813,1.05634,1.98731},
                        {0.424083,1.05642,1.98125},
                        {0.472839,1.05526,1.97019},
                        {0.526406,1.05603,1.96397},
                        {0.574394,1.05782,1.9587},
                        {0.617742,1.056,1.95374},
                        {0.652333,1.064,1.95133},
                        {-1.0055,-1.6065,2.047},
                        {-0.9863,-1.6035,2.0344},
                        {-0.961177,-1.59453,2.02571},
                        {-0.92644,-1.59194,2.02662},
                        {-0.875122,-1.58153,2.02576},
                        {-0.8255,-1.57374,2.02514},
                        {-0.773833,-1.56039,2.02361},
                        {-0.723691,-1.55407,2.02517},
                        {-0.6926,-1.552,2.021},
                        {-0.72,-1.5485,2.0165},
                        {-0.669163,-1.54388,2.02437},
                        {-0.624127,-1.53618,2.02333},
                        {-0.576378,-1.5288,2.02322},
                        {-0.524566,-1.51721,2.02358},
                        {-0.475828,-1.51026,2.02386},
                        {-0.432825,-1.50458,2.0241},
                        {-0.411895,-1.49721,2.03084},
                        {-0.376138,-1.49333,2.02536},
                        {-0.322864,-1.48205,2.02407},
                        {-0.27561,-1.47391,2.02517},
                        {-0.227362,-1.4644,2.02295},
                        {-0.175508,-1.45544,2.02174},
                        {-0.142222,-1.45356,2.037},
                        {-0.120935,-1.44646,2.02146},
                        {-0.0773846,-1.43831,2.02072},
                        {-0.0251579,-1.42997,2.02444},
                        {0.0240597,-1.42237,2.02633},
                        {0.0741379,-1.41216,2.02379},
                        {0.119457,-1.40396,2.02141},
                        {0.142211,-1.39679,2.02479},
                        {0.174322,-1.39139,2.02439},
                        {0.222968,-1.38498,2.02279},
                        {0.277088,-1.3763,2.02386},
                        {0.323776,-1.3665,2.02374},
                        {0.372021,-1.35804,2.02375},
                        {0.420111,-1.35215,2.03074},
                        {0.45175,-1.351,2.03025},
                        {0.440364,-1.34927,2.02327},
                        {0.475617,-1.34215,2.02615},
                        {0.525321,-1.33357,2.02709},
                        {0.574339,-1.32439,2.02478},
                        {0.62485,-1.31535,2.02332},
                        {0.674949,-1.307,2.02581},
                        {0.72234,-1.30332,2.02466},
                        {0.755667,-1.30033,2.035},
                        {0.697,-1.299,2.046},
                        {0.737445,-1.29833,2.032},
                        {0.775554,-1.29341,2.02457},
                        {0.824468,-1.28197,2.02423},
                        {0.8739,-1.27193,2.0233},
                        {0.925083,-1.26635,2.02429},
                        {0.974031,-1.25823,2.02352},
                        {1.02234,-1.25434,2.02437},
                        {1.06273,-1.25467,2.02947},
                        {1.11063,-1.2545,2.02775},
                        {1.02067,-1.24913,2.04207},
                        {1.06796,-1.24284,2.041},
                        {1.12106,-1.24718,2.04018},
                        {1.35806,-0.76075,2.04131},
                        {1.34503,-0.713226,2.03058},
                        {1.35463,-0.733593,2.02674},
                        {-1.377,-0.655,2.042},
                        {1.33373,-0.672962,2.03055},
                        {1.243,-0.6065,2.049},
                        {1.29382,-0.623765,2.04777},
                        {1.32034,-0.625019,2.02997},
                        {1.24325,-0.5675,2.0485},
                        {1.27661,-0.572167,2.04596},
                        {1.31438,-0.577299,2.02891},
                        {1.1945,-0.5175,2.04875},
                        {1.22738,-0.523282,2.04708},
                        {1.27563,-0.52466,2.04011},
                        {1.30629,-0.529918,2.02231},
                        {1.131,-0.481,2.049},
                        {1.18053,-0.470952,2.04731},
                        {1.22338,-0.475313,2.04398},
                        {1.27763,-0.47558,2.03066},
                        {1.30075,-0.49475,2.00475},
                        {1.08411,-0.415,2.04879},
                        {1.12593,-0.418603,2.04685},
                        {1.17293,-0.42575,2.04356},
                        {1.22447,-0.425757,2.03955},
                        {1.27296,-0.425468,2.02513},
                        {1.0488,-0.364,2.0486},
                        {1.07917,-0.369522,2.04793},
                        {1.1236,-0.373056,2.04285},
                        {1.17365,-0.37601,2.03975},
                        {1.22452,-0.374449,2.034},
                        {1.26741,-0.3759,2.02039},
                        {0.997,-0.305,2.0485},
                        {1.02913,-0.316971,2.04737},
                        {1.07371,-0.325637,2.04424},
                        {1.12295,-0.324881,2.03814},
                        {1.17334,-0.324206,2.03349},
                        {1.22364,-0.325097,2.02761},
                        {1.26274,-0.326548,2.01618},
                        {0.986235,-0.262912,2.04782},
                        {1.02569,-0.273682,2.04445},
                        {1.07441,-0.27567,2.03869},
                        {1.12382,-0.276682,2.03317},
                        {1.17434,-0.274056,2.0305},
                        {1.22437,-0.275487,2.02195},
                        {1.257,-0.279342,2.01105},
                        {0.933692,-0.213718,2.04708},
                        {0.976229,-0.223524,2.04487},
                        {1.02432,-0.225779,2.04002},
                        {1.07475,-0.226664,2.03611},
                        {1.12441,-0.224974,2.03061},
                        {1.17469,-0.224657,2.0253},
                        {1.22611,-0.22537,2.01617},
                        {1.25175,-0.24125,2.00438},
                        {0.890385,-0.169769,2.04838},
                        {0.923948,-0.174491,2.04555},
                        {0.973974,-0.175191,2.03972},
                        {1.02454,-0.176779,2.03608},
                        {1.07424,-0.175311,2.03142},
                        {1.12369,-0.175147,2.02536},
                        {1.17398,-0.174269,2.01943},
                        {1.2208,-0.177167,2.01187},
                        {0.799,-0.104,2.049},
                        {0.8319,-0.1191,2.04863},
                        {0.874151,-0.124975,2.04556},
                        {0.924512,-0.124611,2.03988},
                        {0.973951,-0.125521,2.03454},
                        {1.02413,-0.125408,2.03132},
                        {1.0742,-0.12445,2.02609},
                        {1.1244,-0.125421,2.01969},
                        {1.17474,-0.125324,2.0143},
                        {1.21515,-0.128487,2.00723},
                        {0.793429,-0.0715714,2.048},
                        {0.825858,-0.0732566,2.046},
                        {0.874476,-0.0763125,2.04087},
                        {0.925085,-0.0745471,2.03553},
                        {0.974328,-0.0758793,2.02893},
                        {1.02478,-0.0757182,2.02641},
                        {1.07398,-0.075693,2.0204},
                        {1.12341,-0.0766297,2.01422},
                        {1.17398,-0.0763893,2.00867},
                        {1.20816,-0.0849375,2.00359},
                        {0.738353,-0.00782353,2.04765},
                        {0.779737,-0.0202842,2.046},
                        {0.824131,-0.0255154,2.04159},
                        {0.87397,-0.0255344,2.03681},
                        {0.924515,-0.0254621,2.0312},
                        {0.974192,-0.024896,2.02454},
                        {1.02433,-0.02588,2.02017},
                        {1.07426,-0.0261885,2.01479},
                        {1.12438,-0.0259334,2.00992},
                        {1.17053,-0.02869,2.00359},
                        {0.729011,0.0310833,2.04569},
                        {0.774964,0.025554,2.04115},
                        {0.825557,0.0258931,2.0359},
                        {0.875423,0.0262,2.03129},
                        {0.92505,0.0245868,2.02677},
                        {0.97412,0.0234051,2.01915},
                        {1.02368,0.0245664,2.01357},
                        {1.07396,0.0244298,2.00882},
                        {1.12314,0.0234608,2.00499},
                        {1.16274,0.0130882,2.00226},
                        {0.640429,0.0941429,2.04821},
                        {0.679165,0.0802913,2.04618},
                        {0.724662,0.0755899,2.04135},
                        {0.774138,0.074626,2.03542},
                        {0.823661,0.0745041,2.0299},
                        {0.874149,0.0748595,2.02581},
                        {0.925047,0.0735892,2.02013},
                        {0.973836,0.0736803,2.01393},
                        {1.02381,0.0733231,2.00875},
                        {1.07415,0.0734872,2.00406},
                        {1.11708,0.0668889,2.00103},
                        {0.595059,0.138059,2.04829},
                        {0.627113,0.128304,2.04657},
                        {0.674861,0.126195,2.04255},
                        {0.724485,0.125828,2.03682},
                        {0.774246,0.123425,2.03025},
                        {0.824794,0.123213,2.02526},
                        {0.874792,0.123415,2.01928},
                        {0.92373,0.123651,2.01364},
                        {0.974356,0.124644,2.00901},
                        {1.02254,0.124416,2.003},
                        {1.06554,0.114162,2.00197},
                        {0.580719,0.18049,2.04721},
                        {0.624197,0.175902,2.04246},
                        {0.673722,0.176285,2.03659},
                        {0.724528,0.173944,2.03166},
                        {0.774577,0.173111,2.02559},
                        {0.825795,0.173682,2.01954},
                        {0.87542,0.174359,2.01397},
                        {0.924787,0.175622,2.00842},
                        {0.97269,0.173638,2.00352},
                        {1.01617,0.156917,2.00025},
                        {0.528378,0.2307,2.04704},
                        {0.575551,0.226217,2.04299},
                        {0.623894,0.22594,2.03814},
                        {0.673554,0.225705,2.03288},
                        {0.72382,0.22285,2.02714},
                        {0.773909,0.223448,2.01999},
                        {0.824545,0.224091,2.01303},
                        {0.874705,0.22503,2.00839},
                        {0.921802,0.223865,2.00309},
                        {0.9555,0.205833,2},
                        {0.481583,0.28225,2.04775},
                        {0.524489,0.276014,2.04385},
                        {0.574097,0.275306,2.03719},
                        {0.624979,0.275542,2.03149},
                        {0.674271,0.274714,2.02638},
                        {0.724036,0.273421,2.01979},
                        {0.774588,0.273265,2.01418},
                        {0.824129,0.274629,2.00785},
                        {0.871713,0.274183,2.0031},
                        {0.910154,0.262423,2.00212},
                        {0.382045,0.342909,2.04809},
                        {0.431148,0.335492,2.04647},
                        {0.474204,0.324613,2.04457},
                        {0.523292,0.324778,2.03838},
                        {0.573951,0.325201,2.03186},
                        {0.624929,0.325865,2.02669},
                        {0.674339,0.32418,2.02092},
                        {0.72491,0.323375,2.01545},
                        {0.775182,0.324212,2.00851},
                        {0.822119,0.323102,2.00247},
                        {0.863,0.312394,2.00064},
                        {0.3419,0.377,2.04805},
                        {0.377504,0.373075,2.04462},
                        {0.425743,0.373118,2.03857},
                        {0.474949,0.373942,2.03401},
                        {0.524042,0.374,2.02849},
                        {0.573928,0.374237,2.02283},
                        {0.624333,0.375222,2.01755},
                        {0.674543,0.375314,2.01367},
                        {0.724317,0.373352,2.00897},
                        {0.771973,0.371682,2.00411},
                        {0.811861,0.363861,2.00103},
                        {0.343333,0.444333,2.0475},
                        {0.37478,0.425614,2.04569},
                        {0.423939,0.424007,2.04033},
                        {0.474306,0.424243,2.0321},
                        {0.524597,0.424597,2.02524},
                        {0.573928,0.425137,2.01745},
                        {0.62335,0.42411,2.01009},
                        {0.672496,0.423226,2.00486},
                        {0.720572,0.407612,2.00206},
                        {0.757667,0.404,2.00133},
                        {0.2934,0.4828,2.049},
                        {0.3264,0.477591,2.04739},
                        {0.373819,0.475021,2.04243},
                        {0.423889,0.475069,2.03623},
                        {0.473868,0.475056,2.02882},
                        {0.524326,0.475584,2.02265},
                        {0.575404,0.475277,2.01615},
                        {0.62457,0.474437,2.01095},
                        {0.673425,0.473546,2.00538},
                        {0.706304,0.469,2.00074},
                        {0.290698,0.526279,2.04833},
                        {0.324667,0.525036,2.04345},
                        {0.37415,0.52495,2.03686},
                        {0.42458,0.525553,2.03057},
                        {0.474518,0.525057,2.02415},
                        {0.524871,0.52545,2.01656},
                        {0.575158,0.524619,2.01032},
                        {0.623689,0.5225,2.00439},
                        {0.664848,0.5115,2.00206},
                        {-0.619667,0.589,2.03933},
                        {0.239929,0.579643,2.04839},
                        {0.280657,0.575796,2.04557},
                        {0.324993,0.574124,2.03851},
                        {0.375182,0.574102,2.03185},
                        {0.424609,0.573993,2.02378},
                        {0.47542,0.574094,2.01857},
                        {0.52586,0.57439,2.01133},
                        {0.574758,0.574461,2.00486},
                        {0.619405,0.574315,2.00207},
                        {-0.6585,0.633083,2.02925},
                        {-0.626875,0.611875,2.0355},
                        {-0.571737,0.613789,2.03805},
                        {-0.52635,0.6278,2.02385},
                        {-0.469727,0.619273,2.02191},
                        {-0.440943,0.634286,2.02429},
                        {-0.267917,0.6155,2.04067},
                        {-0.231917,0.622583,2.02679},
                        {0.228829,0.626385,2.04491},
                        {0.275653,0.62475,2.03992},
                        {0.325392,0.62407,2.03218},
                        {0.375692,0.62393,2.02588},
                        {0.425286,0.623379,2.01982},
                        {0.474852,0.62419,2.01282},
                        {0.524283,0.625137,2.00684},
                        {0.572346,0.621093,2.00263},
                        {0.61825,0.607,2},
                        {-0.6739,0.6646,2.0234},
                        {-0.572022,0.665222,2.01071},
                        {-0.545,0.664,2},
                        {-0.428077,0.657231,2.02954},
                        {-0.311,0.677,2.01},
                        {-0.215333,0.668,2.03938},
                        {-0.176909,0.684939,2.03915},
                        {-0.127107,0.695107,2.04025},
                        {-0.0964286,0.697571,2.03414},
                        {0.1464,0.6962,2.0484},
                        {0.182615,0.679934,2.04675},
                        {0.225725,0.674681,2.04259},
                        {0.276386,0.67481,2.03508},
                        {0.32512,0.673774,2.0261},
                        {0.374992,0.673576,2.01993},
                        {0.424163,0.675985,2.01456},
                        {0.473301,0.674316,2.00863},
                        {0.521116,0.670456,2.0026},
                        {0.556417,0.660833,2.00208},
                        {-0.694,0.702,2.039},
                        {-0.410556,0.719445,2.02389},
                        {-0.117609,0.703783,2.03022},
                        {-0.0744902,0.720196,2.03239},
                        {-0.0352,0.746333,2.0408},
                        {0.139033,0.728328,2.04734},
                        {0.174927,0.725236,2.04498},
                        {0.223386,0.72381,2.03682},
                        {0.273709,0.723269,2.02979},
                        {0.324014,0.725522,2.02313},
                        {0.373521,0.724493,2.01647},
                        {0.423578,0.725171,2.00796},
                        {0.46899,0.720082,2.00343},
                        {0.518,0.713462,2.00046},
                        {-0.418667,0.772,2.03067},
                        {-0.0205538,0.758646,2.03072},
                        {0.014625,0.771304,2.03307},
                        {0.0905714,0.785714,2.04886},
                        {0.128061,0.777616,2.04542},
                        {0.172879,0.774144,2.03899},
                        {0.223712,0.774475,2.0329},
                        {0.273901,0.774663,2.0259},
                        {0.324091,0.77503,2.01671},
                        {0.374788,0.774977,2.01071},
                        {0.424384,0.774416,2.00267},
                        {0.454857,0.765857,2},
                        {-0.4265,0.8115,2.02875},
                        {-0.00707692,0.818923,2.00608},
                        {0.0194355,0.829371,2.02811},
                        {0.0892941,0.82651,2.04798},
                        {0.124782,0.825311,2.04192},
                        {0.173326,0.823182,2.03267},
                        {0.224534,0.824626,2.0265},
                        {0.274697,0.825272,2.0187},
                        {0.325515,0.824894,2.01281},
                        {0.375352,0.823773,2.00543},
                        {0.418718,0.819901,2.00184},
                        {-0.442,0.876,2.04},
                        {0.0257551,0.876847,2.03788},
                        {0.0779007,0.879071,2.03774},
                        {0.123803,0.874816,2.03014},
                        {0.173386,0.876379,2.02476},
                        {0.224548,0.873817,2.02268},
                        {0.27527,0.87481,2.01495},
                        {0.324898,0.874425,2.00677},
                        {0.370143,0.870086,2.00134},
                        {0.408,0.86075,2.0013},
                        {-0.45125,0.939,2.04575},
                        {-0.447429,0.935714,2.02871},
                        {-0.00433333,0.93075,2.01125},
                        {0.0259244,0.924807,2.03992},
                        {0.0643429,0.907743,2.02177},
                        {0.120708,0.901583,2.00833},
                        {0.186091,0.912091,2.00655},
                        {0.224431,0.92365,2.01846},
                        {0.273864,0.924448,2.01051},
                        {0.323864,0.923551,2.00351},
                        {0.359143,0.924857,2.00076},
                        {-0.828833,0.969,2.043},
                        {-0.78,0.956,2.045},
                        {-0.455308,0.976231,2.03408},
                        {-0.4472,0.984,2.0444},
                        {-0.0045,0.970875,2.01313},
                        {0.0204318,0.9745,2.02627},
                        {0.193889,0.974222,2.01204},
                        {0.224496,0.975529,2.01299},
                        {0.273583,0.97275,2.00612},
                        {0.315847,0.970729,2.00124},
                        {0.01624,1.00988,2.00488},
                        {0.190361,1.02525,2.01486},
                        {0.224381,1.02394,2.00903},
                        {0.271451,1.01944,2.00316},
                        {0.306125,1.02994,2.00194},
                        {0.186478,1.05748,2.0117},
                        {0.222674,1.05688,2.00847},
                        {0.25875,1.05463,2.0015},
                        {-1.01909,-1.60982,2.07864},
                        {-0.988333,-1.60439,2.072},
                        {-0.962883,-1.59724,2.07753},
                        {-0.922087,-1.58824,2.0743},
                        {-0.87405,-1.5824,2.07393},
                        {-0.825304,-1.57324,2.07291},
                        {-0.775046,-1.56282,2.07493},
                        {-0.72456,-1.55516,2.07388},
                        {-0.6926,-1.5512,2.073},
                        {-0.673429,-1.54486,2.07229},
                        {-0.621351,-1.5383,2.07597},
                        {-0.574216,-1.52782,2.07653},
                        {-0.52644,-1.5184,2.07442},
                        {-0.475796,-1.51078,2.07426},
                        {-0.436821,-1.50371,2.07682},
                        {-0.411455,-1.49709,2.07127},
                        {-0.374481,-1.49369,2.07379},
                        {-0.32564,-1.48348,2.07418},
                        {-0.275179,-1.47386,2.0755},
                        {-0.224526,-1.46554,2.07461},
                        {-0.174667,-1.45735,2.07421},
                        {-0.14087,-1.45396,2.07539},
                        {-0.115545,-1.44682,2.07115},
                        {-0.0784074,-1.44341,2.0717},
                        {-0.0227551,-1.42982,2.07157},
                        {0.0255254,-1.41969,2.07327},
                        {0.074875,-1.41188,2.0735},
                        {0.1195,-1.40412,2.07448},
                        {0.157,-1.4,2.089},
                        {0.1394,-1.3978,2.0681},
                        {0.172317,-1.39295,2.07239},
                        {0.224759,-1.38372,2.07579},
                        {0.274569,-1.377,2.07569},
                        {0.324296,-1.36915,2.07424},
                        {0.37429,-1.36039,2.0756},
                        {0.421895,-1.35321,2.07132},
                        {0.431364,-1.34841,2.08095},
                        {0.473984,-1.34147,2.07444},
                        {0.524656,-1.33326,2.07495},
                        {0.575785,-1.32278,2.07477},
                        {0.624591,-1.31311,2.07232},
                        {0.6713,-1.30374,2.0696},
                        {0.723125,-1.30038,2.0575},
                        {0.759,-1.3,2.053},
                        {0.685467,-1.29827,2.08693},
                        {0.725279,-1.2958,2.07702},
                        {0.773913,-1.28932,2.07448},
                        {0.824582,-1.27887,2.07525},
                        {0.875642,-1.26869,2.07507},
                        {0.923334,-1.25835,2.07328},
                        {0.968269,-1.25435,2.0605},
                        {1.0025,-1.2515,2.057},
                        {0.9384,-1.2496,2.095},
                        {0.976093,-1.24672,2.08126},
                        {1.02531,-1.2411,2.07466},
                        {1.07289,-1.22968,2.07486},
                        {1.12575,-1.22675,2.0749},
                        {1.16944,-1.21565,2.0849},
                        {1.1795,-1.1995,2.097},
                        {1.2025,-1.1955,2.0985},
                        {1.19025,-1.01017,2.09792},
                        {1.22347,-1.01719,2.09784},
                        {1.26629,-1.02114,2.09757},
                        {1.131,-0.9565,2.099},
                        {1.18258,-0.97398,2.09648},
                        {1.22383,-0.973788,2.0923},
                        {1.27542,-0.972678,2.09181},
                        {1.303,-0.9565,2.0855},
                        {-1.337,-0.904,2.097},
                        {1.09056,-0.908111,2.09844},
                        {1.12372,-0.912278,2.09644},
                        {1.17466,-0.926355,2.09451},
                        {1.22289,-0.9241,2.0887},
                        {1.27318,-0.926671,2.08682},
                        {1.31239,-0.920182,2.08718},
                        {-1.35467,-0.858167,2.094},
                        {-1.34323,-0.879154,2.09423},
                        {1.04625,-0.85925,2.098},
                        {1.07699,-0.872437,2.09617},
                        {1.12466,-0.87533,2.09307},
                        {1.17359,-0.874941,2.08346},
                        {1.22503,-0.874364,2.08078},
                        {1.27353,-0.87524,2.07813},
                        {1.32173,-0.872841,2.07925},
                        {1.351,-0.851,2.077},
                        {-1.36649,-0.822282,2.0929},
                        {0.98875,-0.806,2.09875},
                        {1.02858,-0.8188,2.09703},
                        {1.07474,-0.826478,2.09307},
                        {1.12441,-0.824727,2.08608},
                        {1.17293,-0.825573,2.07857},
                        {1.22476,-0.825962,2.0772},
                        {1.27413,-0.8255,2.07343},
                        {1.32442,-0.825055,2.07103},
                        {1.35536,-0.823286,2.06871},
                        {-1.4052,-0.756,2.0848},
                        {-1.3773,-0.77415,2.08737},
                        {0.948,-0.751,2.099},
                        {0.981139,-0.770846,2.097},
                        {1.02571,-0.77474,2.09285},
                        {1.07423,-0.775473,2.08677},
                        {1.12432,-0.774854,2.08118},
                        {1.17494,-0.775204,2.07583},
                        {1.22433,-0.775477,2.07019},
                        {1.27523,-0.774762,2.06726},
                        {1.32499,-0.774292,2.06087},
                        {1.35795,-0.784952,2.05676},
                        {-1.40978,-0.722519,2.08004},
                        {-1.38281,-0.725433,2.08597},
                        {0.941103,-0.717483,2.09748},
                        {0.974323,-0.724444,2.09327},
                        {1.02451,-0.723939,2.08841},
                        {1.0737,-0.724563,2.08136},
                        {1.12316,-0.723903,2.07526},
                        {1.17467,-0.723787,2.07047},
                        {1.22401,-0.724876,2.06458},
                        {1.27328,-0.724546,2.06026},
                        {1.32246,-0.725295,2.05599},
                        {-1.401,-0.6945,2.0675},
                        {-1.38684,-0.677286,2.08216},
                        {0.880721,-0.663558,2.09737},
                        {0.925202,-0.673289,2.09314},
                        {0.974716,-0.673716,2.08671},
                        {1.02527,-0.674,2.08127},
                        {1.0744,-0.675229,2.07565},
                        {1.12408,-0.673177,2.06946},
                        {1.17389,-0.673716,2.06505},
                        {1.22356,-0.674455,2.06242},
                        {1.27307,-0.675587,2.05701},
                        {1.3126,-0.6797,2.05293},
                        {-1.38811,-0.642,2.092},
                        {0.835865,-0.614568,2.09816},
                        {0.875771,-0.623729,2.09456},
                        {0.924962,-0.623868,2.08883},
                        {0.974768,-0.62497,2.08236},
                        {1.02318,-0.62547,2.0748},
                        {1.0749,-0.62548,2.07147},
                        {1.12541,-0.624031,2.06386},
                        {1.1746,-0.625104,2.06103},
                        {1.22226,-0.624773,2.05638},
                        {1.27145,-0.62596,2.05336},
                        {0.791682,-0.565409,2.09732},
                        {0.824724,-0.574609,2.09531},
                        {0.874569,-0.576229,2.08813},
                        {0.924429,-0.574247,2.08367},
                        {0.975556,-0.575917,2.07718},
                        {1.02455,-0.575248,2.07122},
                        {1.07556,-0.575774,2.06688},
                        {1.12457,-0.575454,2.06164},
                        {1.17347,-0.574691,2.05514},
                        {1.22307,-0.577791,2.05367},
                        {1.26194,-0.589437,2.05075},
                        {0.739174,-0.516478,2.09835},
                        {0.777807,-0.523011,2.0945},
                        {0.825105,-0.524687,2.08909},
                        {0.873983,-0.525798,2.0827},
                        {0.924934,-0.524972,2.07784},
                        {0.974259,-0.525339,2.07325},
                        {1.02524,-0.526148,2.06733},
                        {1.07405,-0.524721,2.06194},
                        {1.1251,-0.525752,2.05725},
                        {1.17287,-0.526129,2.05298},
                        {1.21527,-0.532682,2.05082},
                        {0.681625,-0.4661,2.09855},
                        {0.726322,-0.468954,2.09602},
                        {0.774008,-0.476558,2.09089},
                        {0.824739,-0.475461,2.0841},
                        {0.874439,-0.474921,2.07795},
                        {0.923179,-0.475774,2.07424},
                        {0.974409,-0.476445,2.06864},
                        {1.02409,-0.475288,2.06166},
                        {1.07502,-0.475752,2.05651},
                        {1.12392,-0.47505,2.05302},
                        {1.16519,-0.484444,2.05092},
                        {0.640818,-0.407,2.09809},
                        {0.675853,-0.423395,2.09734},
                        {0.724182,-0.425008,2.09195},
                        {0.774033,-0.426017,2.08548},
                        {0.82463,-0.425933,2.08081},
                        {0.874373,-0.424449,2.07414},
                        {0.92413,-0.425765,2.0677},
                        {0.973798,-0.425983,2.06222},
                        {1.02492,-0.424453,2.05731},
                        {1.07198,-0.428542,2.05164},
                        {1.12042,-0.44029,2.05113},
                        {0.592889,-0.357889,2.09822},
                        {0.627268,-0.370852,2.097},
                        {0.674726,-0.374339,2.09304},
                        {0.724557,-0.375131,2.08616},
                        {0.774808,-0.376033,2.08184},
                        {0.823402,-0.374893,2.07588},
                        {0.873725,-0.374058,2.07023},
                        {0.924181,-0.375664,2.06443},
                        {0.974439,-0.37486,2.05918},
                        {1.02371,-0.374773,2.05321},
                        {1.06988,-0.38745,2.05105},
                        {0.547,-0.3035,2.09875},
                        {0.584177,-0.31729,2.09785},
                        {0.624504,-0.32474,2.09329},
                        {0.674309,-0.324919,2.0884},
                        {0.724244,-0.325829,2.08258},
                        {0.774382,-0.325805,2.07716},
                        {0.823683,-0.323602,2.07141},
                        {0.874381,-0.325153,2.06578},
                        {0.924086,-0.325257,2.05896},
                        {0.973927,-0.326,2.05396},
                        {1.01771,-0.334156,2.05058},
                        {1.054,-0.348,2.05},
                        {0.53566,-0.269415,2.09777},
                        {0.575085,-0.275686,2.09546},
                        {0.624156,-0.275672,2.09013},
                        {0.674954,-0.27631,2.08328},
                        {0.724015,-0.275045,2.07702},
                        {0.773719,-0.2745,2.07248},
                        {0.824286,-0.27495,2.06657},
                        {0.874067,-0.275292,2.06012},
                        {0.925186,-0.276059,2.05557},
                        {0.969341,-0.280071,2.05141},
                        {1.00314,-0.289714,2.05014},
                        {0.479216,-0.207054,2.09805},
                        {0.526461,-0.223742,2.09409},
                        {0.574645,-0.225833,2.08921},
                        {0.62418,-0.226128,2.08475},
                        {0.673659,-0.225163,2.0793},
                        {0.72448,-0.224886,2.07268},
                        {0.774585,-0.225675,2.06681},
                        {0.82458,-0.22579,2.06143},
                        {0.87512,-0.22576,2.05566},
                        {0.920621,-0.230563,2.05199},
                        {0.955444,-0.237444,2.05011},
                        {0.444,-0.164,2.098},
                        {0.47629,-0.174658,2.0954},
                        {0.524273,-0.175477,2.08949},
                        {0.574228,-0.175423,2.08387},
                        {0.624343,-0.175588,2.07909},
                        {0.674659,-0.175174,2.07457},
                        {0.7234,-0.174348,2.0673},
                        {0.773963,-0.175193,2.06235},
                        {0.824169,-0.176008,2.05739},
                        {0.868561,-0.177551,2.05312},
                        {0.915,-0.196,2.05025},
                        {0.43051,-0.121922,2.09669},
                        {0.474906,-0.125754,2.0917},
                        {0.52379,-0.126978,2.08558},
                        {0.574535,-0.126639,2.08055},
                        {0.624511,-0.125782,2.07398},
                        {0.674172,-0.125539,2.06809},
                        {0.724586,-0.125634,2.06289},
                        {0.774041,-0.125317,2.05613},
                        {0.821068,-0.128875,2.05127},
                        {0.877,-0.1474,2.05},
                        {0.3854,-0.06365,2.09712},
                        {0.424352,-0.0753239,2.09399},
                        {0.474653,-0.0755139,2.08748},
                        {0.525114,-0.0751,2.08187},
                        {0.573993,-0.0761471,2.07517},
                        {0.62434,-0.0757639,2.06926},
                        {0.674546,-0.0759697,2.06381},
                        {0.723797,-0.0755869,2.0575},
                        {0.770164,-0.0782091,2.05309},
                        {0.821647,-0.0961176,2.05076},
                        {0.337555,-0.0144889,2.09764},
                        {0.375336,-0.0241679,2.09396},
                        {0.424021,-0.0251,2.08789},
                        {0.4735,-0.0252113,2.08166},
                        {0.524021,-0.0249167,2.07537},
                        {0.574653,-0.0245461,2.06936},
                        {0.624037,-0.0254148,2.06353},
                        {0.674352,-0.0247606,2.05842},
                        {0.723068,-0.0279145,2.05419},
                        {0.762625,-0.035575,2.05085},
                        {0.292926,0.0354445,2.09767},
                        {0.326664,0.0258175,2.09574},
                        {0.375472,0.0253333,2.0881},
                        {0.425979,0.0253333,2.08326},
                        {0.475252,0.0258561,2.07563},
                        {0.523657,0.0248248,2.06933},
                        {0.573792,0.0254306,2.06427},
                        {0.624653,0.0258723,2.05808},
                        {0.674845,0.0251556,2.05358},
                        {0.713463,0.013561,2.05222},
                        {-1.495,0.051,2.073},
                        {0.241,0.0943333,2.09808},
                        {0.278034,0.0783866,2.09534},
                        {0.325014,0.0754653,2.09097},
                        {0.374743,0.075382,2.08333},
                        {0.42484,0.0753959,2.07678},
                        {0.475417,0.0754722,2.07162},
                        {0.525538,0.0755245,2.06427},
                        {0.574226,0.075609,2.05796},
                        {0.621946,0.0738,2.05275},
                        {0.660941,0.0648235,2.05077},
                        {0.233112,0.128719,2.09587},
                        {0.274493,0.125451,2.09162},
                        {0.324437,0.125389,2.08542},
                        {0.374271,0.12534,2.07902},
                        {0.424201,0.125292,2.07185},
                        {0.474458,0.125396,2.06597},
                        {0.524903,0.125472,2.05926},
                        {0.572565,0.124371,2.05393},
                        {0.6082,0.10905,2.05135},
                        {0.188111,0.187694,2.0975},
                        {0.225268,0.176413,2.09337},
                        {0.274181,0.175347,2.08653},
                        {0.323875,0.175174,2.08016},
                        {0.373681,0.175028,2.07313},
                        {0.423854,0.175181,2.06792},
                        {0.473736,0.17516,2.06019},
                        {0.524236,0.17534,2.05428},
                        {0.564167,0.165792,2.05142},
                        {0.141077,0.241769,2.09854},
                        {0.177039,0.227228,2.0943},
                        {0.224354,0.225465,2.08869},
                        {0.273833,0.225125,2.08096},
                        {0.323479,0.224798,2.07381},
                        {0.373194,0.224722,2.06728},
                        {0.423181,0.22475,2.06151},
                        {0.473424,0.22493,2.05553},
                        {0.515815,0.215852,2.05187},
                        {0.553333,0.208333,2.05033},
                        {0.0454,0.2958,2.0972},
                        {0.0774025,0.285854,2.09648},
                        {0.125894,0.277538,2.09364},
                        {0.174243,0.275653,2.08834},
                        {0.224076,0.275243,2.08222},
                        {0.273618,0.274854,2.07547},
                        {0.323437,0.274701,2.06942},
                        {0.372979,0.274528,2.06211},
                        {0.422951,0.274562,2.05632},
                        {0.464445,0.266986,2.05182},
                        {0.505,0.255,2.05},
                        {0.0325949,0.322912,2.09665},
                        {0.0747059,0.324713,2.09019},
                        {0.123958,0.325132,2.08238},
                        {0.173708,0.324597,2.07715},
                        {0.223215,0.324042,2.07005},
                        {0.273042,0.324049,2.06619},
                        {0.3235,0.324378,2.06067},
                        {0.372508,0.3205,2.05627},
                        {0.419083,0.315702,2.05224},
                        {0.45,0.304,2.0505},
                        {0.03065,0.380988,2.0969},
                        {0.0739357,0.373971,2.09187},
                        {0.124538,0.374538,2.0857},
                        {0.174329,0.375336,2.07656},
                        {0.223771,0.374819,2.06939},
                        {0.273993,0.373517,2.061},
                        {0.322156,0.372946,2.05314},
                        {0.356214,0.377929,2.05093},
                        {-0.0124694,0.427837,2.0978},
                        {0.0244126,0.424944,2.09257},
                        {0.074507,0.423736,2.08648},
                        {0.124549,0.422896,2.0805},
                        {0.174695,0.425355,2.07344},
                        {0.224076,0.42541,2.0664},
                        {0.273896,0.424896,2.0601},
                        {0.32263,0.423688,2.05361},
                        {0.355583,0.409583,2.051},
                        {-0.0610286,0.483286,2.09734},
                        {-0.023886,0.47772,2.09502},
                        {0.0247971,0.475015,2.08883},
                        {0.0746875,0.474958,2.08321},
                        {0.124722,0.473674,2.07586},
                        {0.174626,0.474712,2.06914},
                        {0.224429,0.475229,2.061},
                        {0.273468,0.475101,2.05515},
                        {0.313897,0.46469,2.051},
                        {-0.101,0.545,2.099},
                        {-0.0694138,0.52746,2.09607},
                        {-0.0262448,0.524985,2.09237},
                        {0.0237319,0.52463,2.08615},
                        {0.0753478,0.52521,2.07858},
                        {0.124924,0.524763,2.07098},
                        {0.174362,0.523886,2.06303},
                        {0.224537,0.523868,2.05619},
                        {0.266933,0.523359,2.05202},
                        {0.302,0.52,2.05},
                        {-0.654,0.592,2.099},
                        {-0.619228,0.585614,2.07647},
                        {-0.575921,0.589381,2.08065},
                        {-0.5435,0.5975,2.09317},
                        {-0.307143,0.595286,2.09329},
                        {-0.280756,0.587707,2.09019},
                        {-0.11675,0.584659,2.09755},
                        {-0.074832,0.57452,2.09477},
                        {-0.0266767,0.574759,2.08783},
                        {0.0248,0.574443,2.08066},
                        {0.0744058,0.574862,2.07477},
                        {0.125574,0.574868,2.06624},
                        {0.175104,0.574764,2.05824},
                        {0.221509,0.572362,2.05222},
                        {0.259028,0.567111,2.05053},
                        {-0.669304,0.627435,2.07074},
                        {-0.607895,0.606316,2.06442},
                        {-0.573784,0.608098,2.06526},
                        {-0.528043,0.621495,2.08455},
                        {-0.475978,0.620283,2.07761},
                        {-0.440667,0.628472,2.07389},
                        {-0.30675,0.63675,2.09363},
                        {-0.2761,0.617867,2.07768},
                        {-0.220462,0.625692,2.0784},
                        {-0.176798,0.626365,2.0878},
                        {-0.120473,0.623813,2.09747},
                        {-0.0757438,0.624132,2.09199},
                        {-0.0260143,0.62475,2.08426},
                        {0.0239632,0.624698,2.07676},
                        {0.0753582,0.623859,2.06934},
                        {0.124907,0.624571,2.06154},
                        {0.175682,0.624833,2.05495},
                        {0.210714,0.612572,2.05071},
                        {-0.701,0.687,2.094},
                        {-0.69,0.6666,2.0758},
                        {-0.422044,0.669696,2.0793},
                        {-0.382765,0.681059,2.09271},
                        {-0.31793,0.676491,2.09063},
                        {-0.295357,0.665286,2.08571},
                        {-0.212415,0.659585,2.07242},
                        {-0.173843,0.669835,2.08089},
                        {-0.123765,0.675644,2.08376},
                        {-0.077,0.676022,2.08622},
                        {-0.0270662,0.675228,2.07885},
                        {0.0249259,0.674637,2.07293},
                        {0.0749859,0.674521,2.06568},
                        {0.125466,0.674038,2.05685},
                        {0.164679,0.665509,2.0524},
                        {-0.7105,0.707,2.089},
                        {-0.410267,0.7376,2.0882},
                        {-0.395444,0.736444,2.08222},
                        {-0.0678235,0.710039,2.06569},
                        {-0.0262345,0.7248,2.06636},
                        {0.0239296,0.725225,2.06621},
                        {0.0754697,0.724667,2.05948},
                        {0.115513,0.721698,2.05345},
                        {-0.734,0.787,2.083},
                        {-0.414981,0.777346,2.08825},
                        {-0.3978,0.7654,2.071},
                        {0.034875,0.770347,2.05779},
                        {0.074864,0.773168,2.05483},
                        {0.10772,0.76628,2.05124},
                        {-0.424077,0.824634,2.07983},
                        {0.0440667,0.827067,2.05313},
                        {0.0664359,0.821911,2.05279},
                        {-0.434953,0.875628,2.07926},
                        {0.0395625,0.861438,2.05069},
                        {0.0588,0.8576,2.051},
                        {-0.7844,0.9452,2.0844},
                        {-0.451,0.911,2.0765},
                        {-0.44496,0.91648,2.06812},
                        {-0.915438,0.958188,2.09056},
                        {-0.876553,0.963638,2.08234},
                        {-0.827911,0.964947,2.07764},
                        {-0.793417,0.96025,2.0725},
                        {-0.4539,0.9718,2.0543},
                        {-0.447,0.965429,2.05414},
                        {-1.0586,-1.6144,2.1228},
                        {-1.02153,-1.608,2.12384},
                        {-0.992182,-1.60382,2.12573},
                        {-0.96625,-1.59629,2.12289},
                        {-0.925739,-1.58607,2.12333},
                        {-0.876633,-1.58308,2.1221},
                        {-0.825721,-1.57291,2.12316},
                        {-0.774326,-1.56379,2.12226},
                        {-0.72625,-1.55656,2.12796},
                        {-0.691571,-1.55157,2.109},
                        {-0.7205,-1.549,2.146},
                        {-0.673913,-1.54372,2.12546},
                        {-0.625,-1.53833,2.12404},
                        {-0.5746,-1.52606,2.12464},
                        {-0.52528,-1.5189,2.12518},
                        {-0.475558,-1.50988,2.1244},
                        {-0.435733,-1.50247,2.12263},
                        {-0.390083,-1.50058,2.1255},
                        {-0.412,-1.49816,2.12412},
                        {-0.369634,-1.49388,2.12398},
                        {-0.325426,-1.48411,2.12478},
                        {-0.275102,-1.47418,2.12365},
                        {-0.224793,-1.46629,2.12719},
                        {-0.175983,-1.45674,2.12407},
                        {-0.135941,-1.45303,2.12691},
                        {-0.159,-1.45,2.145},
                        {-0.11205,-1.44895,2.12485},
                        {-0.0749815,-1.44413,2.12726},
                        {-0.0228043,-1.432,2.12561},
                        {0.0246875,-1.41984,2.1243},
                        {0.0749454,-1.40951,2.12402},
                        {0.123983,-1.40379,2.12372},
                        {0.155,-1.401,2.10633},
                        {0.1465,-1.39725,2.13725},
                        {0.175662,-1.39085,2.12517},
                        {0.223271,-1.38278,2.12347},
                        {0.275183,-1.3765,2.1258},
                        {0.324094,-1.36752,2.12295},
                        {0.373823,-1.35723,2.12373},
                        {0.406733,-1.35353,2.11513},
                        {0.3965,-1.3485,2.1455},
                        {0.429365,-1.34685,2.12723},
                        {0.47475,-1.33801,2.1246},
                        {0.524853,-1.32909,2.12575},
                        {0.574029,-1.31856,2.12531},
                        {0.624925,-1.3086,2.12419},
                        {0.659333,-1.30294,2.11578},
                        {0.647,-1.298,2.146},
                        {0.67984,-1.29476,2.12888},
                        {0.723675,-1.28574,2.12404},
                        {0.774921,-1.27761,2.12508},
                        {0.825099,-1.26738,2.12516},
                        {0.872429,-1.25889,2.11967},
                        {0.909462,-1.25531,2.10708},
                        {0.849,-1.247,2.147},
                        {0.879468,-1.23983,2.14236},
                        {0.925844,-1.23231,2.13145},
                        {0.973081,-1.22559,2.12652},
                        {1.02194,-1.22105,2.11834},
                        {1.07182,-1.21184,2.11214},
                        {1.11464,-1.21043,2.10682},
                        {1.16567,-1.2025,2.10167},
                        {1.203,-1.203,2.102},
                        {0.9829,-1.18146,2.14542},
                        {1.02484,-1.17755,2.13829},
                        {1.07379,-1.17479,2.12979},
                        {1.12378,-1.17654,2.11914},
                        {1.17482,-1.17481,2.11261},
                        {1.21205,-1.16711,2.10668},
                        {0.9385,-1.1065,2.14883},
                        {0.979226,-1.12373,2.14595},
                        {1.0249,-1.12505,2.13906},
                        {1.07449,-1.12421,2.13126},
                        {1.12403,-1.12464,2.12429},
                        {1.17535,-1.12592,2.12076},
                        {1.22396,-1.12586,2.11641},
                        {1.2522,-1.119,2.1148},
                        {0.894167,-1.06567,2.14833},
                        {0.927073,-1.06946,2.14523},
                        {0.973659,-1.07472,2.13889},
                        {1.02316,-1.07486,2.13283},
                        {1.07525,-1.07567,2.1261},
                        {1.12494,-1.07547,2.11939},
                        {1.17399,-1.07411,2.11244},
                        {1.22369,-1.07459,2.10837},
                        {1.2623,-1.07277,2.11183},
                        {-1.31575,-1.01437,2.13863},
                        {0.8445,-1.007,2.1485},
                        {0.881957,-1.02074,2.14696},
                        {0.924728,-1.02548,2.1426},
                        {0.973467,-1.02436,2.13261},
                        {1.02543,-1.025,2.1253},
                        {1.07327,-1.02512,2.11833},
                        {1.12372,-1.02501,2.11241},
                        {1.17155,-1.02798,2.10687},
                        {1.22221,-1.03091,2.1017},
                        {1.26876,-1.0234,2.10443},
                        {1.3,-1.013,2.112},
                        {-1.32678,-0.972963,2.13478},
                        {0.786944,-0.970278,2.14808},
                        {0.827136,-0.972227,2.14624},
                        {0.873989,-0.973856,2.14022},
                        {0.9232,-0.9756,2.13301},
                        {0.975171,-0.974898,2.12656},
                        {1.02441,-0.976093,2.11822},
                        {1.07457,-0.974917,2.11146},
                        {1.125,-0.976654,2.10644},
                        {1.16164,-0.978727,2.10197},
                        {1.238,-0.999,2.1},
                        {1.2862,-0.9954,2.1006},
                        {-1.33544,-0.922581,2.12719},
                        {0.728048,-0.911048,2.14838},
                        {0.778206,-0.921986,2.14353},
                        {0.824242,-0.925275,2.13895},
                        {0.874835,-0.924868,2.13351},
                        {0.924516,-0.925594,2.12584},
                        {0.974943,-0.924603,2.11806},
                        {1.025,-0.925607,2.11042},
                        {1.07319,-0.926186,2.10505},
                        {1.12577,-0.937042,2.1019},
                        {1.1655,-0.9375,2.1},
                        {-1.354,-0.861,2.106},
                        {-1.3415,-0.874,2.12369},
                        {0.680467,-0.872689,2.14856},
                        {0.726747,-0.87567,2.14503},
                        {0.774845,-0.875546,2.13813},
                        {0.824736,-0.876143,2.13295},
                        {0.873484,-0.8746,2.12678},
                        {0.924484,-0.876011,2.11913},
                        {0.974021,-0.875236,2.11396},
                        {1.0236,-0.875576,2.10562},
                        {1.05986,-0.889857,2.10236},
                        {-1.35423,-0.821149,2.11802},
                        {-1.34752,-0.836191,2.13176},
                        {0.638226,-0.817613,2.14636},
                        {0.675959,-0.824546,2.14312},
                        {0.724382,-0.825628,2.1378},
                        {0.77506,-0.82469,2.1324},
                        {0.823396,-0.824882,2.12521},
                        {0.874376,-0.82543,2.11942},
                        {0.923388,-0.824704,2.11243},
                        {0.973105,-0.826628,2.10593},
                        {1.01358,-0.835742,2.10274},
                        {-1.36298,-0.775097,2.12339},
                        {0.582143,-0.767821,2.14764},
                        {0.626202,-0.774464,2.14335},
                        {0.674126,-0.775689,2.13688},
                        {0.724791,-0.774238,2.13094},
                        {0.774388,-0.775112,2.1256},
                        {0.825114,-0.774371,2.11851},
                        {0.87366,-0.775679,2.11324},
                        {0.923798,-0.773529,2.1063},
                        {0.962583,-0.78425,2.10164},
                        {-1.37122,-0.724745,2.12505},
                        {0.497,-0.703333,2.14867},
                        {0.531509,-0.715836,2.14784},
                        {0.574691,-0.724618,2.14415},
                        {0.625054,-0.725427,2.13734},
                        {0.675027,-0.725226,2.13023},
                        {0.724533,-0.723038,2.12361},
                        {0.774472,-0.725217,2.11834},
                        {0.824029,-0.72465,2.11205},
                        {0.87406,-0.72492,2.10828},
                        {0.917869,-0.727159,2.10284},
                        {0.953,-0.742,2.1},
                        {-1.38095,-0.673143,2.12717},
                        {0.483344,-0.667918,2.146},
                        {0.524779,-0.673576,2.14272},
                        {0.574321,-0.675199,2.13688},
                        {0.624421,-0.67514,2.13198},
                        {0.674018,-0.673991,2.12325},
                        {0.725027,-0.674009,2.11998},
                        {0.775142,-0.674764,2.1127},
                        {0.824038,-0.674154,2.10558},
                        {0.869564,-0.681532,2.10224},
                        {0.90375,-0.69475,2.1},
                        {-1.39014,-0.626965,2.12998},
                        {0.433638,-0.614043,2.14755},
                        {0.475409,-0.624609,2.14313},
                        {0.524436,-0.624909,2.13632},
                        {0.574009,-0.625261,2.12994},
                        {0.624398,-0.62612,2.12576},
                        {0.674374,-0.624916,2.11835},
                        {0.724148,-0.624833,2.11372},
                        {0.773822,-0.625925,2.10687},
                        {0.817634,-0.630127,2.10289},
                        {0.857667,-0.640444,2.10033},
                        {-1.39376,-0.584762,2.141},
                        {0.348,-0.555,2.149},
                        {0.382836,-0.567855,2.14738},
                        {0.424672,-0.573922,2.14404},
                        {0.4744,-0.574296,2.1376},
                        {0.525126,-0.576042,2.13094},
                        {0.574443,-0.575517,2.12475},
                        {0.62424,-0.575843,2.11901},
                        {0.673623,-0.576307,2.11311},
                        {0.724614,-0.574991,2.1071},
                        {0.770413,-0.577577,2.10273},
                        {0.800333,-0.595333,2.10033},
                        {0.33929,-0.517355,2.14729},
                        {0.374521,-0.525857,2.14414},
                        {0.424808,-0.526109,2.13856},
                        {0.474063,-0.524667,2.13172},
                        {0.523711,-0.526149,2.12622},
                        {0.575288,-0.525102,2.11895},
                        {0.624851,-0.524446,2.11378},
                        {0.674306,-0.525396,2.10758},
                        {0.72,-0.527706,2.10308},
                        {0.763182,-0.5365,2.10109},
                        {-0.038,-0.456,2.149},
                        {0.28403,-0.46203,2.14794},
                        {0.325698,-0.473905,2.14441},
                        {0.374134,-0.476677,2.13925},
                        {0.424354,-0.475465,2.13277},
                        {0.474722,-0.476881,2.12725},
                        {0.524402,-0.476721,2.12182},
                        {0.573587,-0.475992,2.11521},
                        {0.624591,-0.475922,2.10842},
                        {0.671372,-0.478974,2.10203},
                        {0.719844,-0.492406,2.10106},
                        {0.244261,-0.420261,2.14826},
                        {0.2755,-0.424064,2.14575},
                        {0.323786,-0.426168,2.13988},
                        {0.374767,-0.425349,2.13322},
                        {0.424637,-0.426387,2.12793},
                        {0.474142,-0.425488,2.12124},
                        {0.524444,-0.427,2.11568},
                        {0.574472,-0.426846,2.11074},
                        {0.622317,-0.427658,2.10441},
                        {0.6615,-0.43325,2.10075},
                        {0.194667,-0.358133,2.14813},
                        {0.229906,-0.368482,2.1468},
                        {0.274785,-0.37459,2.14133},
                        {0.324391,-0.374226,2.13541},
                        {0.373937,-0.374598,2.12942},
                        {0.424701,-0.375022,2.12224},
                        {0.474662,-0.374489,2.11635},
                        {0.524528,-0.374359,2.11048},
                        {0.573323,-0.376549,2.10502},
                        {0.611227,-0.389364,2.10232},
                        {0.185403,-0.316719,2.14769},
                        {0.224529,-0.324017,2.1437},
                        {0.273583,-0.323644,2.13738},
                        {0.324917,-0.323541,2.13079},
                        {0.375189,-0.322756,2.12444},
                        {0.424508,-0.32478,2.11761},
                        {0.474758,-0.32325,2.11119},
                        {0.524345,-0.324033,2.1064},
                        {0.565742,-0.330333,2.1018},
                        {0.142316,-0.265158,2.14826},
                        {0.174794,-0.275801,2.14505},
                        {0.2254,-0.275431,2.13982},
                        {0.274315,-0.275347,2.13229},
                        {0.32371,-0.27529,2.12686},
                        {0.374568,-0.27497,2.11984},
                        {0.425062,-0.274411,2.11268},
                        {0.474202,-0.275339,2.10642},
                        {0.516747,-0.279063,2.102},
                        {0.553,-0.275667,2.1},
                        {0.096,-0.213273,2.14818},
                        {0.127346,-0.221481,2.1464},
                        {0.173958,-0.225875,2.14072},
                        {0.224826,-0.225549,2.13442},
                        {0.275121,-0.224851,2.12784},
                        {0.324958,-0.224937,2.12123},
                        {0.37419,-0.225467,2.11472},
                        {0.423931,-0.22491,2.1081},
                        {0.473029,-0.230829,2.10202},
                        {0.504636,-0.237636,2.10045},
                        {0.0842927,-0.169915,2.14711},
                        {0.125225,-0.173984,2.14341},
                        {0.17319,-0.174458,2.13651},
                        {0.224061,-0.175985,2.13038},
                        {0.274636,-0.17572,2.12339},
                        {0.325273,-0.175606,2.117},
                        {0.375454,-0.175161,2.10962},
                        {0.422259,-0.176786,2.1033},
                        {0.454385,-0.186308,2.10031},
                        {0.0330909,-0.113477,2.14761},
                        {0.0759846,-0.125015,2.14547},
                        {0.124484,-0.124893,2.13884},
                        {0.173203,-0.125369,2.13265},
                        {0.223368,-0.126701,2.12562},
                        {0.273764,-0.126528,2.11877},
                        {0.32416,-0.126403,2.11215},
                        {0.374792,-0.12643,2.10609},
                        {0.412,-0.136833,2.10069},
                        {-0.00762963,-0.0675926,2.14826},
                        {0.0248028,-0.0759789,2.14497},
                        {0.0753819,-0.0761527,2.13994},
                        {0.126097,-0.0760139,2.13442},
                        {0.175103,-0.0752942,2.1271},
                        {0.223329,-0.0762143,2.12072},
                        {0.273083,-0.0756597,2.11494},
                        {0.323208,-0.0755416,2.10735},
                        {0.369096,-0.0800577,2.10238},
                        {0.401,-0.084,2.1},
                        {-0.062,-0.0126333,2.14767},
                        {-0.0233136,-0.0220848,2.14531},
                        {0.0244583,-0.02525,2.14},
                        {0.0751806,-0.0251042,2.13532},
                        {0.125792,-0.0250833,2.1292},
                        {0.176007,-0.0250208,2.12217},
                        {0.226208,-0.0249861,2.1151},
                        {0.27621,-0.0247692,2.10845},
                        {0.319421,-0.0293263,2.10195},
                        {0.357,-0.043,2.1},
                        {-0.108545,0.0361212,2.14797},
                        {-0.0735586,0.0281892,2.14682},
                        {-0.0265,0.02575,2.14206},
                        {0.0244028,0.0256875,2.13508},
                        {0.0749931,0.0255972,2.12947},
                        {0.125333,0.0255,2.12308},
                        {0.175674,0.0255,2.11735},
                        {0.225701,0.0254375,2.11033},
                        {0.271598,0.0230769,2.10355},
                        {0.305286,0.016,2.10014},
                        {-0.119629,0.0787629,2.14632},
                        {-0.0753884,0.0746281,2.14284},
                        {-0.026447,0.074303,2.13604},
                        {0.02475,0.0755643,2.13029},
                        {0.074882,0.0760903,2.12483},
                        {0.12516,0.0759444,2.11841},
                        {0.175208,0.0757709,2.11146},
                        {0.22372,0.0739319,2.10477},
                        {0.25988,0.06176,2.1006},
                        {-0.2085,0.145833,2.14833},
                        {-0.172313,0.13405,2.14718},
                        {-0.124099,0.123908,2.14276},
                        {-0.0751894,0.123205,2.13676},
                        {-0.0263333,0.12291,2.13133},
                        {0.0238162,0.123971,2.12606},
                        {0.0752662,0.125568,2.11898},
                        {0.124965,0.126167,2.11322},
                        {0.17491,0.125875,2.10595},
                        {0.210945,0.120564,2.10218},
                        {-0.214031,0.177831,2.14665},
                        {-0.175958,0.17516,2.14351},
                        {-0.124306,0.174507,2.13766},
                        {-0.0750682,0.173848,2.13093},
                        {-0.0263333,0.173326,2.12579},
                        {0.02425,0.172896,2.12023},
                        {0.0740876,0.173642,2.11386},
                        {0.124799,0.176271,2.10781},
                        {0.170194,0.171981,2.10224},
                        {0.205333,0.155667,2.10033},
                        {-0.355,0.246,2.149},
                        {-0.322283,0.237133,2.14675},
                        {-0.275402,0.229879,2.14444},
                        {-0.224689,0.225134,2.14081},
                        {-0.175576,0.225861,2.13568},
                        {-0.124294,0.224993,2.13115},
                        {-0.0752181,0.224564,2.12556},
                        {-0.0263333,0.223896,2.1216},
                        {0.02425,0.223104,2.11398},
                        {0.0745833,0.222701,2.1093},
                        {0.123496,0.224031,2.10315},
                        {0.155294,0.214647,2.10035},
                        {-0.355,0.253,2.149},
                        {-0.317915,0.269592,2.14559},
                        {-0.275559,0.274615,2.13943},
                        {-0.225197,0.273447,2.13295},
                        {-0.174977,0.274391,2.12638},
                        {-0.125758,0.275091,2.1195},
                        {-0.0769167,0.274347,2.11558},
                        {-0.0261944,0.273583,2.11075},
                        {0.0237589,0.272695,2.10691},
                        {0.071,0.261905,2.10345},
                        {0.107813,0.256813,2.10181},
                        {-0.361667,0.341429,2.14824},
                        {-0.324142,0.327058,2.14537},
                        {-0.274953,0.323984,2.13913},
                        {-0.225389,0.325031,2.13326},
                        {-0.175706,0.323965,2.12707},
                        {-0.125103,0.324897,2.12033},
                        {-0.075927,0.324825,2.11334},
                        {-0.0261944,0.32409,2.10571},
                        {0.0128889,0.323873,2.10149},
                        {-0.411852,0.388407,2.14859},
                        {-0.373592,0.376925,2.14623},
                        {-0.324008,0.374447,2.14135},
                        {-0.274891,0.373826,2.13371},
                        {-0.225713,0.374254,2.12833},
                        {-0.175545,0.375035,2.12341},
                        {-0.124266,0.373853,2.11718},
                        {-0.0751048,0.374306,2.11084},
                        {-0.0263264,0.375382,2.10474},
                        {0.01625,0.366062,2.10083},
                        {-0.4162,0.43082,2.14716},
                        {-0.374385,0.424474,2.1429},
                        {-0.324411,0.425089,2.13605},
                        {-0.274461,0.424,2.12988},
                        {-0.225697,0.423962,2.12372},
                        {-0.175632,0.425007,2.11921},
                        {-0.124222,0.424972,2.11249},
                        {-0.0752214,0.42371,2.10656},
                        {-0.0335698,0.423116,2.10154},
                        {0.001,0.403,2.1},
                        {-0.462658,0.482927,2.147},
                        {-0.42623,0.475328,2.14504},
                        {-0.374538,0.474235,2.13925},
                        {-0.322976,0.474342,2.13178},
                        {-0.274891,0.47452,2.1263},
                        {-0.225787,0.473959,2.11921},
                        {-0.176418,0.474546,2.11358},
                        {-0.124364,0.474356,2.10804},
                        {-0.0800521,0.471406,2.10299},
                        {-0.0387778,0.457815,2.10104},
                        {-0.52175,0.535375,2.14863},
                        {-0.46722,0.526963,2.14566},
                        {-0.423901,0.523876,2.14135},
                        {-0.374097,0.524726,2.1349},
                        {-0.323661,0.523887,2.12809},
                        {-0.27432,0.524888,2.1208},
                        {-0.226489,0.523969,2.11463},
                        {-0.176007,0.524231,2.1089},
                        {-0.124901,0.523902,2.10412},
                        {-0.0894445,0.517056,2.10089},
                        {-0.662643,0.586643,2.1365},
                        {-0.623634,0.576658,2.12778},
                        {-0.572818,0.581854,2.12955},
                        {-0.524761,0.584023,2.14108},
                        {-0.476041,0.575122,2.14371},
                        {-0.424901,0.57338,2.13787},
                        {-0.374402,0.574852,2.13163},
                        {-0.325585,0.573642,2.12327},
                        {-0.272909,0.569535,2.11274},
                        {-0.225947,0.575634,2.1113},
                        {-0.175173,0.572953,2.10579},
                        {-0.128864,0.569568,2.10192},
                        {-0.097,0.5605,2.1},
                        {-0.7052,0.6424,2.142},
                        {-0.682417,0.621375,2.12913},
                        {-0.52195,0.616967,2.11803},
                        {-0.476734,0.610016,2.12597},
                        {-0.423294,0.621442,2.1231},
                        {-0.374918,0.624164,2.12522},
                        {-0.327168,0.624449,2.11422},
                        {-0.285333,0.611125,2.108},
                        {-0.223929,0.606952,2.10538},
                        {-0.174727,0.605242,2.10452},
                        {-0.136024,0.623342,2.10093},
                        {-0.7135,0.67335,2.1339},
                        {-0.697,0.658,2.1025},
                        {-0.417051,0.666339,2.11493},
                        {-0.375438,0.667743,2.11265},
                        {-0.334434,0.665962,2.10968},
                        {-0.154,0.652,2.1},
                        {-0.139444,0.654333,2.10056},
                        {-0.732074,0.724482,2.13444},
                        {-0.411333,0.747333,2.10267},
                        {-0.754667,0.783917,2.14417},
                        {-0.743,0.769583,2.12975},
                        {-0.411143,0.755714,2.10229},
                        {-0.76855,0.82795,2.1385},
                        {-0.748,0.815,2.104},
                        {-0.8735,0.899,2.149},
                        {-0.818967,0.8909,2.14607},
                        {-0.781022,0.878689,2.13822},
                        {-0.95855,0.9378,2.13915},
                        {-0.924408,0.934323,2.13732},
                        {-0.875347,0.928723,2.1397},
                        {-0.8254,0.92559,2.1371},
                        {-0.788379,0.919724,2.12234},
                        {-0.952,0.951,2.113},
                        {-0.9274,0.9532,2.1073},
                        {-0.872625,0.95425,2.11138},
                        {-0.825775,0.95515,2.11345},
                        {-0.7985,0.953,2.108},
                        {-1.105,-1.61833,2.18467},
                        {-1.07886,-1.6176,2.17974},
                        {-1.0213,-1.6097,2.18019},
                        {-0.992214,-1.60314,2.1725},
                        {-0.967364,-1.59523,2.17441},
                        {-0.923326,-1.58623,2.17521},
                        {-0.876167,-1.58023,2.17365},
                        {-0.824222,-1.57409,2.1742},
                        {-0.775673,-1.56161,2.17255},
                        {-0.732333,-1.55181,2.17819},
                        {-0.698,-1.55,2.197},
                        {-0.716727,-1.54805,2.16477},
                        {-0.672341,-1.54449,2.1741},
                        {-0.624654,-1.53617,2.17475},
                        {-0.575021,-1.52511,2.17417},
                        {-0.522891,-1.51602,2.17438},
                        {-0.4751,-1.50838,2.1745},
                        {-0.428881,-1.50095,2.1745},
                        {-0.394,-1.5012,2.164},
                        {-0.416545,-1.49809,2.17745},
                        {-0.374667,-1.49406,2.17598},
                        {-0.324522,-1.48567,2.17606},
                        {-0.275519,-1.47567,2.17526},
                        {-0.224673,-1.46247,2.17529},
                        {-0.176268,-1.4542,2.1755},
                        {-0.128357,-1.45207,2.15929},
                        {-0.167,-1.45,2.192},
                        {-0.123904,-1.44642,2.17792},
                        {-0.0743235,-1.43866,2.17603},
                        {-0.0260794,-1.42997,2.17336},
                        {0.0233966,-1.41783,2.1755},
                        {0.0751408,-1.40683,2.17561},
                        {0.11428,-1.40264,2.16648},
                        {0.13186,-1.39449,2.18067},
                        {0.174393,-1.38605,2.1763},
                        {0.224687,-1.37778,2.17339},
                        {0.274352,-1.36994,2.17486},
                        {0.32368,-1.361,2.17629},
                        {0.361448,-1.35528,2.16814},
                        {0.3824,-1.3464,2.17847},
                        {0.4253,-1.33796,2.17548},
                        {0.473663,-1.32669,2.1756},
                        {0.52336,-1.31484,2.17583},
                        {0.571952,-1.30848,2.17013},
                        {0.616238,-1.30362,2.15757},
                        {0.533867,-1.29287,2.1964},
                        {0.577042,-1.28307,2.19227},
                        {0.624919,-1.2769,2.1826},
                        {0.673682,-1.27453,2.17314},
                        {0.722619,-1.26664,2.16612},
                        {0.77152,-1.26112,2.15834},
                        {0.81505,-1.2561,2.1528},
                        {0.6352,-1.22983,2.19743},
                        {0.674833,-1.22638,2.19278},
                        {0.724326,-1.22614,2.18364},
                        {0.77337,-1.22533,2.17402},
                        {0.824283,-1.22554,2.16311},
                        {0.87219,-1.21712,2.1566},
                        {0.9195,-1.2065,2.15315},
                        {0.590571,-1.15629,2.199},
                        {0.626891,-1.17583,2.19734},
                        {0.674306,-1.17429,2.19184},
                        {0.723897,-1.17578,2.18723},
                        {0.774253,-1.1751,2.17899},
                        {0.824798,-1.17506,2.17233},
                        {0.874744,-1.17656,2.1623},
                        {0.925451,-1.176,2.15621},
                        {0.964308,-1.17213,2.15192},
                        {1.007,-1.166,2.15},
                        {-1.30441,-1.11312,2.18759},
                        {-1.2975,-1.113,2.199},
                        {0.537,-1.1065,2.199},
                        {0.583795,-1.11964,2.19752},
                        {0.624153,-1.12558,2.19292},
                        {0.673964,-1.12408,2.18692},
                        {0.724659,-1.12463,2.17982},
                        {0.775116,-1.12476,2.17429},
                        {0.824463,-1.1237,2.16566},
                        {0.875537,-1.1272,2.15984},
                        {0.923114,-1.12764,2.15426},
                        {0.957737,-1.13532,2.15147},
                        {-1.31058,-1.0716,2.18218},
                        {-1.297,-1.05,2.194},
                        {0.1952,-1.055,2.198},
                        {0.206,-1.05333,2.19767},
                        {0.492692,-1.05938,2.19785},
                        {0.52737,-1.07073,2.19607},
                        {0.573753,-1.07501,2.19245},
                        {0.624714,-1.07571,2.18646},
                        {0.675244,-1.07545,2.18101},
                        {0.724196,-1.07466,2.17367},
                        {0.77365,-1.0742,2.16681},
                        {0.824313,-1.07588,2.16067},
                        {0.872738,-1.07771,2.15462},
                        {0.915263,-1.09242,2.15205},
                        {-1.31374,-1.024,2.17629},
                        {-1.2986,-1.0422,2.1914},
                        {0.190903,-1.02045,2.19558},
                        {0.206556,-1.02311,2.19419},
                        {0.448667,-1.00933,2.198},
                        {0.478381,-1.01986,2.19543},
                        {0.524822,-1.02496,2.1902},
                        {0.574903,-1.02562,2.18394},
                        {0.624978,-1.02471,2.17963},
                        {0.674011,-1.02559,2.17357},
                        {0.724573,-1.0244,2.16576},
                        {0.775384,-1.02636,2.15988},
                        {0.823048,-1.02698,2.15412},
                        {0.867475,-1.03045,2.15118},
                        {-1.32151,-0.975206,2.17751},
                        {0.143143,-0.955286,2.19357},
                        {0.177746,-0.972702,2.19178},
                        {0.203222,-0.987556,2.19478},
                        {0.393,-0.955833,2.198},
                        {0.427344,-0.969406,2.19631},
                        {0.47434,-0.975351,2.18873},
                        {0.524617,-0.975766,2.1842},
                        {0.574505,-0.977248,2.17768},
                        {0.624628,-0.974837,2.17092},
                        {0.674967,-0.975484,2.16618},
                        {0.724183,-0.976667,2.15895},
                        {0.765569,-0.977103,2.1534},
                        {0.816727,-0.987864,2.15068},
                        {-1.33117,-0.926766,2.17642},
                        {0.0266818,-0.907591,2.19723},
                        {0.079225,-0.9129,2.19165},
                        {0.125774,-0.923376,2.19032},
                        {0.161788,-0.936333,2.18945},
                        {0.334769,-0.911385,2.19738},
                        {0.376892,-0.923952,2.1948},
                        {0.424225,-0.925296,2.18898},
                        {0.474598,-0.925289,2.18235},
                        {0.524085,-0.925479,2.17605},
                        {0.57425,-0.927292,2.17135},
                        {0.62441,-0.925095,2.16475},
                        {0.674463,-0.924968,2.15831},
                        {0.722857,-0.931029,2.15373},
                        {0.75695,-0.9347,2.1519},
                        {-1.335,-0.874952,2.1769},
                        {0.0405385,-0.893461,2.19485},
                        {0.0726177,-0.894971,2.19306},
                        {0.106,-0.8982,2.1948},
                        {0.249,-0.851,2.199},
                        {0.274035,-0.86579,2.19826},
                        {0.324866,-0.875557,2.19334},
                        {0.374206,-0.87601,2.18662},
                        {0.424545,-0.876202,2.18188},
                        {0.474639,-0.87566,2.17554},
                        {0.524556,-0.876364,2.1686},
                        {0.574444,-0.874849,2.16258},
                        {0.624604,-0.873989,2.15611},
                        {0.668417,-0.876229,2.1515},
                        {0.7055,-0.883875,2.15025},
                        {-1.35267,-0.803333,2.15433},
                        {-1.34221,-0.825164,2.17321},
                        {0.197,-0.804,2.199},
                        {0.23346,-0.822095,2.19654},
                        {0.276579,-0.823074,2.19194},
                        {0.325273,-0.825709,2.18635},
                        {0.375309,-0.825428,2.18013},
                        {0.425294,-0.825147,2.17381},
                        {0.474962,-0.825632,2.1693},
                        {0.524308,-0.826627,2.16341},
                        {0.574495,-0.825361,2.15736},
                        {0.617681,-0.82845,2.15178},
                        {0.6596,-0.8488,2.1508},
                        {-1.35757,-0.773393,2.17166},
                        {-1.34875,-0.798,2.1855},
                        {0.142,-0.754,2.199},
                        {0.1824,-0.7686,2.1964},
                        {0.2235,-0.775455,2.19265},
                        {0.273518,-0.774062,2.18579},
                        {0.324036,-0.773616,2.17969},
                        {0.374696,-0.773776,2.17411},
                        {0.424855,-0.773536,2.1674},
                        {0.474718,-0.773664,2.16089},
                        {0.524908,-0.774532,2.15568},
                        {0.566098,-0.784294,2.15216},
                        {0.608625,-0.79375,2.15037},
                        {-1.36082,-0.722829,2.1724},
                        {-1.34875,-0.7455,2.194},
                        {0.0925,-0.708,2.198},
                        {0.128338,-0.722312,2.19634},
                        {0.173661,-0.724905,2.19183},
                        {0.225122,-0.72493,2.18577},
                        {0.275303,-0.723477,2.17861},
                        {0.324755,-0.723136,2.17352},
                        {0.374045,-0.7228,2.16722},
                        {0.423757,-0.722568,2.16077},
                        {0.473908,-0.723752,2.15587},
                        {0.516887,-0.732472,2.151},
                        {0.552,-0.747,2.15},
                        {-1.36841,-0.674905,2.17249},
                        {0.044,-0.652,2.199},
                        {0.0788513,-0.668432,2.19584},
                        {0.124008,-0.675492,2.1919},
                        {0.175532,-0.67582,2.18591},
                        {0.224864,-0.675091,2.18131},
                        {0.273866,-0.673956,2.17379},
                        {0.324419,-0.673171,2.16707},
                        {0.375643,-0.673911,2.16267},
                        {0.425144,-0.673487,2.15551},
                        {0.46338,-0.6795,2.15192},
                        {-1.37726,-0.6242,2.17167},
                        {0.0321475,-0.620623,2.19656},
                        {0.0751311,-0.624598,2.19191},
                        {0.125965,-0.624833,2.18653},
                        {0.174071,-0.625797,2.17968},
                        {0.224193,-0.625482,2.17491},
                        {0.274853,-0.625586,2.16803},
                        {0.324855,-0.624791,2.16231},
                        {0.374186,-0.624106,2.15617},
                        {0.418209,-0.632224,2.15269},
                        {0.452,-0.645333,2.15033},
                        {-1.38546,-0.575444,2.17274},
                        {-0.0111613,-0.567516,2.198},
                        {0.0245614,-0.574465,2.19454},
                        {0.0739836,-0.575967,2.1878},
                        {0.12542,-0.575438,2.1811},
                        {0.174713,-0.574984,2.17425},
                        {0.224672,-0.575836,2.16826},
                        {0.273958,-0.575085,2.16129},
                        {0.323927,-0.575342,2.15643},
                        {0.367644,-0.584187,2.15134},
                        {0.40525,-0.5935,2.15025},
                        {-1.404,-0.507667,2.19433},
                        {-1.39193,-0.532833,2.18126},
                        {-0.0615152,-0.514303,2.19379},
                        {-0.0242157,-0.523402,2.18212},
                        {0.0232182,-0.525209,2.18704},
                        {0.0742923,-0.526339,2.18262},
                        {0.124339,-0.525322,2.1755},
                        {0.174565,-0.524695,2.17003},
                        {0.224322,-0.524939,2.16379},
                        {0.275218,-0.52521,2.15633},
                        {0.320244,-0.529779,2.15245},
                        {0.35525,-0.53225,2.15},
                        {-1.403,-0.494,2.19767},
                        {-0.112875,-0.460958,2.198},
                        {-0.0730313,-0.477011,2.19104},
                        {-0.024319,-0.475276,2.16964},
                        {0.0249385,-0.475946,2.18217},
                        {0.0752683,-0.475699,2.17734},
                        {0.124234,-0.473774,2.17043},
                        {0.173908,-0.475758,2.16367},
                        {0.225262,-0.475516,2.15725},
                        {0.270725,-0.48222,2.15298},
                        {0.3069,-0.4953,2.1515},
                        {-0.158083,-0.413583,2.1985},
                        {-0.124645,-0.423551,2.19655},
                        {-0.0751338,-0.423921,2.18457},
                        {-0.0255039,-0.425756,2.17132},
                        {0.0252045,-0.425841,2.17815},
                        {0.0753388,-0.42495,2.17283},
                        {0.125159,-0.423924,2.16578},
                        {0.174535,-0.425803,2.15879},
                        {0.220385,-0.428422,2.15468},
                        {0.26275,-0.445167,2.1505},
                        {-0.2085,-0.354083,2.19833},
                        {-0.170614,-0.369904,2.1964},
                        {-0.126149,-0.37505,2.19169},
                        {-0.0748686,-0.37492,2.18685},
                        {-0.025053,-0.374727,2.17919},
                        {0.0247333,-0.37537,2.17356},
                        {0.0749174,-0.375058,2.16764},
                        {0.124606,-0.374348,2.16205},
                        {0.172965,-0.3758,2.15517},
                        {0.215444,-0.387241,2.15122},
                        {-0.263,-0.319,2.199},
                        {-0.220674,-0.319185,2.19764},
                        {-0.174117,-0.325833,2.19243},
                        {-0.125628,-0.325397,2.18782},
                        {-0.0751364,-0.324546,2.18367},
                        {-0.0254322,-0.323788,2.17545},
                        {0.0253333,-0.326101,2.16878},
                        {0.074562,-0.32595,2.16334},
                        {0.124038,-0.32525,2.15757},
                        {0.167773,-0.330493,2.15201},
                        {-0.264333,-0.271772,2.19746},
                        {-0.22592,-0.274352,2.19315},
                        {-0.174977,-0.276156,2.18757},
                        {-0.125017,-0.275983,2.18174},
                        {-0.074803,-0.275189,2.17694},
                        {-0.0243548,-0.274952,2.1708},
                        {0.0247986,-0.275548,2.16319},
                        {0.0742955,-0.27497,2.15802},
                        {0.119863,-0.277701,2.1536},
                        {0.151,-0.298,2.15},
                        {-0.317979,-0.215917,2.19796},
                        {-0.274348,-0.22493,2.19463},
                        {-0.225975,-0.224669,2.18879},
                        {-0.174916,-0.225811,2.18341},
                        {-0.124748,-0.225282,2.1774},
                        {-0.0744924,-0.226591,2.17143},
                        {-0.0246364,-0.225843,2.16559},
                        {0.024697,-0.225242,2.15995},
                        {0.0731983,-0.225259,2.15433},
                        {0.115,-0.242367,2.15097},
                        {-0.357885,-0.162808,2.19842},
                        {-0.321974,-0.173209,2.19498},
                        {-0.274675,-0.176399,2.19048},
                        {-0.225289,-0.176091,2.18496},
                        {-0.176244,-0.175634,2.17853},
                        {-0.125733,-0.174534,2.17273},
                        {-0.0761654,-0.176421,2.16726},
                        {-0.0265315,-0.175434,2.16001},
                        {0.0246528,-0.175278,2.15597},
                        {0.0646129,-0.181016,2.15102},
                        {0.104,-0.194333,2.15},
                        {-0.4045,-0.11,2.19825},
                        {-0.372117,-0.121874,2.19633},
                        {-0.325174,-0.124348,2.19043},
                        {-0.275093,-0.125557,2.18596},
                        {-0.224485,-0.125561,2.17926},
                        {-0.175985,-0.125046,2.17336},
                        {-0.125875,-0.124597,2.1676},
                        {-0.0761157,-0.126339,2.16135},
                        {-0.0266515,-0.125902,2.15619},
                        {0.0202955,-0.131659,2.15152},
                        {0.0545,-0.147,2.151},
                        {-0.4576,-0.0542,2.1984},
                        {-0.419841,-0.0697101,2.19596},
                        {-0.376405,-0.0765537,2.19316},
                        {-0.326397,-0.0761405,2.18659},
                        {-0.275594,-0.0762578,2.18054},
                        {-0.224704,-0.075,2.17515},
                        {-0.175537,-0.0751736,2.16863},
                        {-0.12547,-0.0748333,2.16203},
                        {-0.0759015,-0.0767046,2.15769},
                        {-0.0309573,-0.0785812,2.15216},
                        {0.003,-0.1,2.15},
                        {-0.472869,-0.0204286,2.19796},
                        {-0.426008,-0.0261136,2.19439},
                        {-0.375492,-0.0259773,2.18848},
                        {-0.325568,-0.0258939,2.18217},
                        {-0.276379,-0.0258334,2.17613},
                        {-0.225389,-0.0255695,2.16895},
                        {-0.175015,-0.0255,2.16284},
                        {-0.125146,-0.0254236,2.15767},
                        {-0.0796765,-0.0291079,2.15334},
                        {-0.0410385,-0.0397308,2.15069},
                        {-0.514312,0.0361875,2.19816},
                        {-0.475513,0.0249076,2.19474},
                        {-0.425116,0.0243636,2.18996},
                        {-0.374595,0.0242727,2.18321},
                        {-0.324719,0.0241322,2.17621},
                        {-0.275678,0.024,2.17072},
                        {-0.22497,0.0239091,2.16505},
                        {-0.174216,0.024624,2.15949},
                        {-0.129703,0.0227748,2.15389},
                        {-0.0859048,0.0128571,2.15033},
                        {-0.5636,0.0854333,2.19863},
                        {-0.523907,0.0772093,2.19671},
                        {-0.474775,0.0746589,2.1907},
                        {-0.424295,0.0748409,2.18501},
                        {-0.373811,0.0743636,2.17791},
                        {-0.32428,0.0740303,2.17239},
                        {-0.27525,0.0736516,2.16655},
                        {-0.226081,0.0729185,2.16031},
                        {-0.176423,0.0741314,2.15352},
                        {-0.138314,0.0640857,2.15083},
                        {-0.608852,0.132593,2.19826},
                        {-0.572916,0.127516,2.19599},
                        {-0.526023,0.124311,2.19135},
                        {-0.474886,0.124984,2.18478},
                        {-0.425404,0.124702,2.17982},
                        {-0.374767,0.124721,2.17245},
                        {-0.323071,0.124659,2.16644},
                        {-0.274546,0.125182,2.15954},
                        {-0.227,0.123587,2.15385},
                        {-0.181172,0.111766,2.15148},
                        {-0.148,0.107,2.15},
                        {-0.708875,0.185,2.19875},
                        {-0.673103,0.176412,2.19652},
                        {-0.624187,0.17517,2.19214},
                        {-0.575926,0.175205,2.186},
                        {-0.52547,0.174274,2.18318},
                        {-0.474829,0.174577,2.17758},
                        {-0.426291,0.175654,2.17279},
                        {-0.374875,0.174167,2.16731},
                        {-0.324095,0.174103,2.16236},
                        {-0.274434,0.174615,2.15494},
                        {-0.23719,0.172365,2.15124},
                        {-0.707167,0.237667,2.199},
                        {-0.67178,0.225512,2.19604},
                        {-0.625941,0.2265,2.19028},
                        {-0.574404,0.223877,2.18194},
                        {-0.525295,0.223348,2.17525},
                        {-0.476405,0.224048,2.1678},
                        {-0.425661,0.224529,2.16216},
                        {-0.375709,0.224,2.15752},
                        {-0.329429,0.214029,2.15436},
                        {-0.276676,0.204757,2.1516},
                        {-0.246,0.205286,2.15},
                        {-0.758,0.262,2.199},
                        {-0.711778,0.281611,2.197},
                        {-0.672518,0.274509,2.19377},
                        {-0.622964,0.275514,2.18694},
                        {-0.573991,0.274045,2.18113},
                        {-0.525203,0.274289,2.17548},
                        {-0.475726,0.274602,2.16891},
                        {-0.426031,0.274398,2.16238},
                        {-0.377033,0.275426,2.15539},
                        {-0.338177,0.280843,2.15124},
                        {-0.8565,0.3445,2.19825},
                        {-0.839731,0.337115,2.19738},
                        {-0.763184,0.333421,2.19626},
                        {-0.724461,0.32547,2.19421},
                        {-0.673622,0.324441,2.18908},
                        {-0.624241,0.324143,2.18309},
                        {-0.574807,0.324882,2.17627},
                        {-0.524756,0.32505,2.17024},
                        {-0.475033,0.324126,2.16401},
                        {-0.427104,0.324568,2.15824},
                        {-0.378139,0.321241,2.15329},
                        {-0.339769,0.305846,2.15046},
                        {-0.859054,0.377324,2.19743},
                        {-0.837182,0.375564,2.19674},
                        {-0.771583,0.375135,2.19528},
                        {-0.723769,0.374435,2.19062},
                        {-0.673381,0.374522,2.18326},
                        {-0.62336,0.373901,2.17713},
                        {-0.575027,0.375735,2.17107},
                        {-0.524293,0.37391,2.16573},
                        {-0.475746,0.375105,2.15914},
                        {-0.429848,0.370525,2.15346},
                        {-0.389933,0.358867,2.15027},
                        {-0.865738,0.424672,2.19571},
                        {-0.825082,0.428616,2.19516},
                        {-0.77378,0.424495,2.1906},
                        {-0.7235,0.425074,2.1851},
                        {-0.673228,0.424447,2.17875},
                        {-0.622956,0.424089,2.17256},
                        {-0.5748,0.425045,2.16686},
                        {-0.525056,0.424662,2.16027},
                        {-0.475705,0.423705,2.15349},
                        {-0.435039,0.421105,2.15116},
                        {-0.870547,0.475326,2.19002},
                        {-0.825648,0.474435,2.19063},
                        {-0.774213,0.474509,2.18493},
                        {-0.72323,0.474927,2.17917},
                        {-0.672873,0.474573,2.17271},
                        {-0.6238,0.474255,2.16788},
                        {-0.575618,0.474555,2.16291},
                        {-0.525558,0.474692,2.15714},
                        {-0.481805,0.469974,2.15308},
                        {-0.45,0.45,2.15},
                        {-0.906083,0.536417,2.19175},
                        {-0.873083,0.525,2.18758},
                        {-0.824863,0.524758,2.18566},
                        {-0.775202,0.524798,2.1803},
                        {-0.723156,0.524688,2.17241},
                        {-0.673443,0.523726,2.16746},
                        {-0.624717,0.524973,2.16344},
                        {-0.576243,0.524189,2.15847},
                        {-0.526761,0.523655,2.1537},
                        {-0.491282,0.521026,2.15203},
                        {-0.910862,0.577241,2.18955},
                        {-0.875543,0.575447,2.18337},
                        {-0.825149,0.574138,2.1812},
                        {-0.775858,0.575151,2.17597},
                        {-0.724654,0.574365,2.16887},
                        {-0.677478,0.57012,2.16311},
                        {-0.625759,0.562352,2.15694},
                        {-0.574088,0.563073,2.15413},
                        {-0.53335,0.562375,2.1526},
                        {-0.915808,0.62634,2.18392},
                        {-0.87693,0.62456,2.17964},
                        {-0.825368,0.624925,2.17644},
                        {-0.773743,0.624657,2.17052},
                        {-0.72567,0.622394,2.16452},
                        {-0.692588,0.612529,2.15659},
                        {-0.920735,0.675456,2.17829},
                        {-0.876265,0.673196,2.17568},
                        {-0.825439,0.674183,2.17123},
                        {-0.774228,0.674381,2.16552},
                        {-0.735204,0.672041,2.15745},
                        {-0.9568,0.7368,2.1938},
                        {-0.9245,0.725613,2.17581},
                        {-0.87653,0.72338,2.1727},
                        {-0.826094,0.725115,2.16758},
                        {-0.775833,0.72401,2.16078},
                        {-0.742571,0.711643,2.15521},
                        {-0.960368,0.778,2.18647},
                        {-0.923763,0.774462,2.1704},
                        {-0.876077,0.774077,2.16999},
                        {-0.825089,0.774208,2.1615},
                        {-0.778309,0.772889,2.1559},
                        {-0.964606,0.826303,2.18},
                        {-0.925,0.82412,2.16609},
                        {-0.874798,0.823786,2.16447},
                        {-0.826098,0.825881,2.15946},
                        {-0.788667,0.817641,2.15521},
                        {-0.9685,0.876408,2.16906},
                        {-0.925344,0.87414,2.15962},
                        {-0.87597,0.87389,2.15747},
                        {-0.82793,0.869282,2.15455},
                        {-0.793643,0.864071,2.15186},
                        {-1.008,0.926833,2.18717},
                        {-0.976351,0.919561,2.16184},
                        {-0.927889,0.907815,2.15341},
                        {-0.883778,0.904,2.15033},
                        {-1.11857,-1.62171,2.22491},
                        {-1.07468,-1.61505,2.2235},
                        {-1.02622,-1.60788,2.2251},
                        {-0.987294,-1.602,2.21947},
                        {-0.965913,-1.594,2.22778},
                        {-0.92641,-1.58677,2.22603},
                        {-0.872977,-1.57935,2.22391},
                        {-0.826833,-1.57255,2.22433},
                        {-0.773791,-1.56016,2.22502},
                        {-0.724085,-1.55366,2.226},
                        {-0.6926,-1.55,2.2286},
                        {-0.734,-1.5475,2.2425},
                        {-0.672075,-1.5453,2.22675},
                        {-0.625231,-1.53492,2.22531},
                        {-0.573694,-1.52435,2.22531},
                        {-0.526104,-1.51329,2.22513},
                        {-0.476255,-1.50578,2.22367},
                        {-0.437647,-1.50112,2.21906},
                        {-0.456,-1.498,2.246},
                        {-0.419605,-1.49613,2.22989},
                        {-0.374661,-1.48995,2.22321},
                        {-0.324254,-1.48144,2.22475},
                        {-0.27535,-1.47012,2.22388},
                        {-0.226943,-1.4604,2.22511},
                        {-0.18995,-1.4525,2.2206},
                        {-0.206,-1.45,2.249},
                        {-0.1693,-1.44755,2.22733},
                        {-0.124625,-1.44005,2.22537},
                        {-0.0752576,-1.42761,2.22415},
                        {-0.02544,-1.41975,2.22496},
                        {0.0221364,-1.41035,2.22412},
                        {0.0608333,-1.40287,2.21629},
                        {0.0443333,-1.39833,2.24067},
                        {0.0806818,-1.3945,2.22984},
                        {0.124936,-1.38591,2.22506},
                        {0.174822,-1.37228,2.22741},
                        {0.223289,-1.36434,2.22355},
                        {0.272522,-1.35957,2.21404},
                        {0.319867,-1.35247,2.20653},
                        {0.3525,-1.354,2.2035},
                        {0.235293,-1.3381,2.24383},
                        {0.277767,-1.33193,2.2378},
                        {0.325043,-1.32696,2.22955},
                        {0.372452,-1.32438,2.21756},
                        {0.422213,-1.31532,2.21096},
                        {0.475425,-1.30852,2.20618},
                        {0.50375,-1.304,2.20225},
                        {0.291,-1.26965,2.24815},
                        {0.32519,-1.27587,2.24529},
                        {0.374,-1.27699,2.23632},
                        {0.424649,-1.27706,2.22754},
                        {0.474474,-1.27541,2.21768},
                        {0.522429,-1.27202,2.2089},
                        {0.566917,-1.26081,2.20406},
                        {0.605,-1.2525,2.2005},
                        {-1.293,-1.206,2.245},
                        {-0.00725,-1.20825,2.24575},
                        {0.0279688,-1.21272,2.24216},
                        {0.0738,-1.21466,2.2382},
                        {0.1216,-1.21533,2.23653},
                        {0.161,-1.20933,2.24483},
                        {0.282346,-1.22004,2.24706},
                        {0.324489,-1.22594,2.24361},
                        {0.374817,-1.2275,2.23533},
                        {0.424659,-1.22621,2.22899},
                        {0.474814,-1.22527,2.22163},
                        {0.524595,-1.22462,2.21436},
                        {0.575824,-1.22636,2.2067},
                        {0.619039,-1.22343,2.20118},
                        {-1.3,-1.182,2.229},
                        {-1.29356,-1.17706,2.23928},
                        {-0.0612353,-1.16029,2.24482},
                        {-0.0274407,-1.17683,2.24319},
                        {0.0237255,-1.18773,2.24318},
                        {0.078025,-1.18009,2.24042},
                        {0.12356,-1.17332,2.22855},
                        {0.171839,-1.17418,2.22553},
                        {0.225429,-1.16769,2.24552},
                        {0.277095,-1.1738,2.24353},
                        {0.325035,-1.17342,2.23633},
                        {0.375337,-1.17501,2.22741},
                        {0.424744,-1.17548,2.22492},
                        {0.474089,-1.17433,2.21777},
                        {0.525302,-1.17428,2.2123},
                        {0.572561,-1.1768,2.20346},
                        {0.61575,-1.17725,2.20045},
                        {-1.302,-1.103,2.21},
                        {-1.29544,-1.11632,2.23012},
                        {-0.1045,-1.1095,2.248},
                        {-0.0749178,-1.12397,2.24293},
                        {-0.0276364,-1.12091,2.24827},
                        {0.025,-1.11264,2.24804},
                        {0.0789178,-1.1236,2.24292},
                        {0.125326,-1.12494,2.23821},
                        {0.172089,-1.12511,2.21531},
                        {0.225229,-1.12534,2.2295},
                        {0.274407,-1.1254,2.23871},
                        {0.324776,-1.12515,2.22989},
                        {0.374012,-1.12366,2.22168},
                        {0.424585,-1.12369,2.21723},
                        {0.474711,-1.12507,2.21328},
                        {0.522811,-1.12635,2.20599},
                        {0.565561,-1.13298,2.20234},
                        {-1.30331,-1.08103,2.21879},
                        {-1.2961,-1.0797,2.2308},
                        {-0.108348,-1.0707,2.24191},
                        {-0.0834678,-1.07592,2.24085},
                        {-0.0177895,-1.07579,2.24704},
                        {0.0252099,-1.07378,2.24505},
                        {0.0754512,-1.07612,2.23974},
                        {0.125281,-1.07555,2.23363},
                        {0.174311,-1.07588,2.2175},
                        {0.225635,-1.07529,2.21748},
                        {0.274978,-1.07523,2.23023},
                        {0.324138,-1.07416,2.22433},
                        {0.37387,-1.07507,2.21698},
                        {0.425045,-1.07439,2.20953},
                        {0.472296,-1.07574,2.2052},
                        {0.512188,-1.09037,2.20131},
                        {-1.3033,-1.01876,2.2183},
                        {-1.29625,-1.03279,2.23267},
                        {-0.109031,-1.02428,2.2355},
                        {-0.0768293,-1.02384,2.23329},
                        {-0.0231,-1.02336,2.239},
                        {0.0243511,-1.02549,2.23732},
                        {0.0750833,-1.025,2.2297},
                        {0.122958,-1.02456,2.22208},
                        {0.167426,-1.02692,2.21354},
                        {0.231317,-1.0239,2.21703},
                        {0.274698,-1.02427,2.22114},
                        {0.32469,-1.02477,2.21592},
                        {0.374769,-1.02532,2.21013},
                        {0.423494,-1.02641,2.20515},
                        {0.462273,-1.0375,2.20232},
                        {-1.30802,-0.975982,2.22531},
                        {-1.297,-0.99,2.248},
                        {-0.105063,-0.982063,2.2435},
                        {-0.0777527,-0.973688,2.22174},
                        {-0.025619,-0.97606,2.22889},
                        {0.0252366,-0.976053,2.22809},
                        {0.0743,-0.976144,2.22319},
                        {0.122494,-0.978883,2.21422},
                        {0.16937,-0.977963,2.206},
                        {0.225462,-0.974043,2.2141},
                        {0.274245,-0.977043,2.21467},
                        {0.324343,-0.975757,2.20859},
                        {0.373652,-0.976685,2.20403},
                        {0.419031,-0.989938,2.20128},
                        {-1.31925,-0.927797,2.22309},
                        {-0.111111,-0.910222,2.24778},
                        {-0.0748929,-0.926206,2.23554},
                        {-0.0284035,-0.922052,2.21292},
                        {0.0228438,-0.929625,2.20866},
                        {0.0725953,-0.937691,2.20945},
                        {0.133471,-0.915235,2.20512},
                        {0.178947,-0.923307,2.21207},
                        {0.224243,-0.926525,2.21184},
                        {0.275031,-0.925949,2.20713},
                        {0.320412,-0.930632,2.20279},
                        {0.3566,-0.934133,2.20047},
                        {-1.32429,-0.875085,2.22536},
                        {-0.163731,-0.8605,2.24731},
                        {-0.124365,-0.874448,2.24519},
                        {-0.0756044,-0.875307,2.24038},
                        {-0.0253182,-0.877409,2.23116},
                        {0.0228099,-0.875496,2.2195},
                        {0.0750891,-0.872822,2.21499},
                        {0.125634,-0.876366,2.21237},
                        {0.175429,-0.875552,2.21079},
                        {0.224278,-0.875426,2.2056},
                        {0.27351,-0.885939,2.20153},
                        {0.301667,-0.893667,2.20067},
                        {-1.33191,-0.826152,2.22385},
                        {-0.217241,-0.810173,2.247},
                        {-0.172547,-0.823093,2.24459},
                        {-0.124346,-0.825855,2.23998},
                        {-0.0751263,-0.825611,2.23405},
                        {-0.0266789,-0.825908,2.22912},
                        {0.0241919,-0.82598,2.2231},
                        {0.0752243,-0.825636,2.21636},
                        {0.124807,-0.825129,2.21109},
                        {0.175061,-0.826602,2.20614},
                        {0.210722,-0.829195,2.20211},
                        {0.2605,-0.844333,2.20017},
                        {-1.35253,-0.774733,2.20767},
                        {-1.34411,-0.779094,2.22743},
                        {-0.263343,-0.768343,2.24737},
                        {-0.225324,-0.773714,2.24444},
                        {-0.174278,-0.775805,2.2406},
                        {-0.125784,-0.775165,2.23467},
                        {-0.0766789,-0.775716,2.22895},
                        {-0.026486,-0.774664,2.22297},
                        {0.0235487,-0.775902,2.21762},
                        {0.0741546,-0.774436,2.21061},
                        {0.1234,-0.774314,2.20454},
                        {0.165273,-0.784382,2.20098},
                        {-1.352,-0.709214,2.21636},
                        {-1.34702,-0.728614,2.22586},
                        {-0.315482,-0.710667,2.24785},
                        {-0.27495,-0.723624,2.2452},
                        {-0.224604,-0.724415,2.24018},
                        {-0.175238,-0.725,2.23461},
                        {-0.126618,-0.723945,2.22863},
                        {-0.0763091,-0.724727,2.22321},
                        {-0.0261441,-0.724036,2.21705},
                        {0.0243551,-0.724832,2.21127},
                        {0.0726,-0.724438,2.20515},
                        {0.108458,-0.734917,2.201},
                        {-1.3578,-0.671833,2.22083},
                        {-1.34878,-0.690889,2.243},
                        {-0.364556,-0.665611,2.24753},
                        {-0.323848,-0.672808,2.24412},
                        {-0.275179,-0.675226,2.23964},
                        {-0.226,-0.674019,2.23363},
                        {-0.176312,-0.673872,2.22816},
                        {-0.125962,-0.674509,2.22323},
                        {-0.0758091,-0.674491,2.21733},
                        {-0.0255446,-0.673241,2.21112},
                        {0.0233519,-0.675731,2.20545},
                        {0.0611389,-0.68475,2.20217},
                        {-1.36688,-0.625091,2.22424},
                        {-0.414333,-0.6035,2.24817},
                        {-0.369268,-0.620887,2.2451},
                        {-0.325422,-0.625137,2.23956},
                        {-0.276114,-0.624295,2.23464},
                        {-0.225843,-0.624889,2.2273},
                        {-0.175336,-0.624354,2.22225},
                        {-0.125153,-0.625378,2.21635},
                        {-0.0753091,-0.624754,2.21056},
                        {-0.0258899,-0.623872,2.20583},
                        {0.0129184,-0.632735,2.20231},
                        {-1.37494,-0.57542,2.22451},
                        {-0.457083,-0.569167,2.24858},
                        {-0.423869,-0.572778,2.24606},
                        {-0.374748,-0.576225,2.24103},
                        {-0.324305,-0.574657,2.23411},
                        {-0.275482,-0.575682,2.22885},
                        {-0.224509,-0.575276,2.22266},
                        {-0.174364,-0.575727,2.21756},
                        {-0.123889,-0.575017,2.21103},
                        {-0.0749211,-0.575649,2.20622},
                        {-0.0306173,-0.578457,2.20193},
                        {0.00514286,-0.595714,2.20043},
                        {-1.4015,-0.5055,2.2025},
                        {-1.38396,-0.5242,2.22359},
                        {-0.472628,-0.522077,2.24765},
                        {-0.425698,-0.525906,2.24255},
                        {-0.375873,-0.525282,2.23696},
                        {-0.324784,-0.525306,2.22961},
                        {-0.274887,-0.526121,2.22364},
                        {-0.224514,-0.525752,2.21777},
                        {-0.174886,-0.525081,2.21136},
                        {-0.124526,-0.524956,2.2061},
                        {-0.0799756,-0.53061,2.20332},
                        {-0.0433333,-0.5475,2.2015},
                        {-1.40186,-0.481,2.22286},
                        {-1.39221,-0.483714,2.22807},
                        {-0.7226,-0.4638,2.2464},
                        {-0.519971,-0.470841,2.24726},
                        {-0.475101,-0.47497,2.24325},
                        {-0.425412,-0.47436,2.2373},
                        {-0.374345,-0.475965,2.23204},
                        {-0.324964,-0.476036,2.2249},
                        {-0.275569,-0.475294,2.21855},
                        {-0.226653,-0.47538,2.21267},
                        {-0.177377,-0.475,2.2074},
                        {-0.128408,-0.480156,2.20268},
                        {-0.08232,-0.46804,2.20116},
                        {-0.725,-0.449,2.242},
                        {-0.560542,-0.42075,2.24771},
                        {-0.523033,-0.423911,2.2444},
                        {-0.475116,-0.424741,2.23805},
                        {-0.425963,-0.426047,2.23383},
                        {-0.374883,-0.42575,2.22733},
                        {-0.325154,-0.424682,2.21986},
                        {-0.275554,-0.425884,2.21371},
                        {-0.22565,-0.424333,2.20842},
                        {-0.177429,-0.427686,2.20313},
                        {-0.143071,-0.439857,2.20014},
                        {-0.0836,-0.4478,2.2012},
                        {-0.6062,-0.3682,2.249},
                        {-0.571384,-0.372302,2.24592},
                        {-0.525385,-0.375743,2.24038},
                        {-0.47594,-0.374543,2.23453},
                        {-0.42533,-0.375447,2.22739},
                        {-0.375438,-0.374025,2.22074},
                        {-0.324529,-0.375639,2.21584},
                        {-0.274033,-0.374802,2.20784},
                        {-0.225855,-0.3757,2.20347},
                        {-0.18545,-0.38875,2.20078},
                        {-0.709533,-0.309933,2.2486},
                        {-0.682333,-0.311667,2.249},
                        {-0.620297,-0.318734,2.24684},
                        {-0.575764,-0.325359,2.24185},
                        {-0.525579,-0.324614,2.23484},
                        {-0.475088,-0.324716,2.22888},
                        {-0.425339,-0.32514,2.22285},
                        {-0.374377,-0.324676,2.21604},
                        {-0.324609,-0.326164,2.2101},
                        {-0.276352,-0.324926,2.20422},
                        {-0.236353,-0.334059,2.20088},
                        {-0.721608,-0.27277,2.24763},
                        {-0.66988,-0.26464,2.24784},
                        {-0.624353,-0.275636,2.24377},
                        {-0.575204,-0.274496,2.23685},
                        {-0.525962,-0.275179,2.23083},
                        {-0.47538,-0.276223,2.22384},
                        {-0.425204,-0.275151,2.21811},
                        {-0.374925,-0.276467,2.21136},
                        {-0.325371,-0.27569,2.20595},
                        {-0.287156,-0.277953,2.20253},
                        {-0.25,-0.297,2.2},
                        {-0.75525,-0.213833,2.24742},
                        {-0.724576,-0.224404,2.24543},
                        {-0.67176,-0.225302,2.24493},
                        {-0.625337,-0.225019,2.23863},
                        {-0.576132,-0.224684,2.23177},
                        {-0.524679,-0.225857,2.22563},
                        {-0.47564,-0.226108,2.21902},
                        {-0.426233,-0.22445,2.21303},
                        {-0.37546,-0.22596,2.20603},
                        {-0.329589,-0.23374,2.20158},
                        {-0.293,-0.2405,2.20017},
                        {-0.76125,-0.1723,2.24478},
                        {-0.724424,-0.176405,2.24277},
                        {-0.674255,-0.175973,2.23935},
                        {-0.624239,-0.174963,2.23246},
                        {-0.5751,-0.173582,2.22564},
                        {-0.525699,-0.177522,2.21983},
                        {-0.476144,-0.175627,2.21509},
                        {-0.424942,-0.175025,2.20802},
                        {-0.377636,-0.178051,2.20237},
                        {-0.338688,-0.19275,2.20044},
                        {-0.769427,-0.123973,2.23939},
                        {-0.725638,-0.126612,2.23858},
                        {-0.675075,-0.124972,2.23416},
                        {-0.626407,-0.123991,2.22751},
                        {-0.575413,-0.123934,2.22146},
                        {-0.524991,-0.127072,2.21525},
                        {-0.475132,-0.12657,2.21011},
                        {-0.425667,-0.127054,2.20413},
                        {-0.387429,-0.140476,2.20176},
                        {-0.801,-0.053,2.231},
                        {-0.774055,-0.074633,2.23318},
                        {-0.725455,-0.0752819,2.23525},
                        {-0.6748,-0.0747909,2.22896},
                        {-0.624764,-0.0742909,2.22165},
                        {-0.575696,-0.0740536,2.21702},
                        {-0.525163,-0.0752713,2.2111},
                        {-0.475867,-0.0769666,2.20588},
                        {-0.434544,-0.0864348,2.20246},
                        {-0.806333,-0.0206667,2.23738},
                        {-0.773218,-0.0249818,2.23115},
                        {-0.723709,-0.0249091,2.23059},
                        {-0.6731,-0.0246909,2.22407},
                        {-0.624217,-0.0249717,2.2177},
                        {-0.575518,-0.0237983,2.21184},
                        {-0.52592,-0.0246903,2.20552},
                        {-0.4836,-0.0318571,2.20106},
                        {-0.812769,0.026359,2.23582},
                        {-0.77433,0.0255218,2.22599},
                        {-0.723604,0.0253513,2.22542},
                        {-0.674559,0.0258039,2.21866},
                        {-0.6267,0.0252091,2.21267},
                        {-0.575967,0.0248512,2.20774},
                        {-0.530077,0.0200641,2.20215},
                        {-0.5,0.005,2.2005},
                        {-0.815822,0.0775536,2.22946},
                        {-0.775414,0.0757748,2.2232},
                        {-0.725624,0.0757248,2.22111},
                        {-0.675154,0.0749364,2.2144},
                        {-0.625773,0.0744637,2.20858},
                        {-0.579621,0.0692874,2.20348},
                        {-0.531107,0.0643929,2.20025},
                        {-0.819931,0.124861,2.22272},
                        {-0.775148,0.126687,2.21671},
                        {-0.7238,0.125273,2.2136},
                        {-0.6737,0.124436,2.20797},
                        {-0.629494,0.120687,2.20359},
                        {-0.587572,0.109571,2.20048},
                        {-0.851,0.179,2.227},
                        {-0.824891,0.174826,2.2165},
                        {-0.775562,0.175162,2.20993},
                        {-0.726703,0.173287,2.20363},
                        {-0.679211,0.157211,2.20126},
                        {-0.63725,0.15075,2.20025},
                        {-0.856333,0.230333,2.22158},
                        {-0.824064,0.223083,2.21191},
                        {-0.774145,0.225291,2.21069},
                        {-0.725516,0.224165,2.20459},
                        {-0.689385,0.217308,2.20092},
                        {-0.860576,0.278727,2.21661},
                        {-0.823655,0.273746,2.2075},
                        {-0.773461,0.274471,2.20539},
                        {-0.729083,0.270847,2.20192},
                        {-0.696,0.258,2.201},
                        {-0.867083,0.324229,2.2106},
                        {-0.820844,0.321389,2.20436},
                        {-0.781924,0.318348,2.20386},
                        {-0.745,0.303,2.2},
                        {-0.9032,0.3796,2.238},
                        {-0.881974,0.373079,2.21111},
                        {-0.813793,0.374483,2.20228},
                        {-0.796778,0.370778,2.20133},
                        {-0.91025,0.430063,2.23238},
                        {-0.8884,0.42256,2.20628},
                        {-0.824053,0.409526,2.20121},
                        {-0.912131,0.475522,2.22339},
                        {-0.895167,0.467167,2.20217},
                        {-0.919955,0.522318,2.21714},
                        {-0.9341,0.5745,2.21975},
                        {-0.957625,0.630875,2.23863},
                        {-0.939571,0.623429,2.20986},
                        {-0.960294,0.679353,2.22335},
                        {-0.970118,0.721941,2.22435},
                        {-0.981071,0.773143,2.22307},
                        {-1.0078,0.8322,2.2452},
                        {-0.991667,0.823917,2.21992},
                        {-1.01443,0.879048,2.23267},
                        {-0.9954,0.873,2.2052},
                        {-1.053,0.92475,2.24325},
                        {-1.029,0.918963,2.22811},
                        {-1.159,-1.62587,2.28625},
                        {-1.12295,-1.62092,2.27558},
                        {-1.07576,-1.61279,2.27321},
                        {-1.02424,-1.60454,2.27356},
                        {-0.9956,-1.6036,2.2578},
                        {-1.011,-1.6,2.295},
                        {-0.972616,-1.59182,2.2782},
                        {-0.925362,-1.58296,2.27438},
                        {-0.876302,-1.57751,2.27367},
                        {-0.824783,-1.56933,2.27539},
                        {-0.775409,-1.55723,2.27366},
                        {-0.734778,-1.55067,2.26122},
                        {-0.75,-1.547,2.297},
                        {-0.720514,-1.54626,2.27374},
                        {-0.67474,-1.54062,2.27396},
                        {-0.625237,-1.53229,2.27812},
                        {-0.5746,-1.51811,2.27524},
                        {-0.524547,-1.50809,2.27487},
                        {-0.486567,-1.50273,2.271},
                        {-0.4635,-1.49475,2.28075},
                        {-0.425322,-1.49003,2.27441},
                        {-0.375288,-1.48376,2.27699},
                        {-0.324939,-1.47064,2.27615},
                        {-0.275431,-1.46085,2.27522},
                        {-0.236063,-1.45531,2.26809},
                        {-0.266,-1.45,2.296},
                        {-0.217209,-1.44384,2.28244},
                        {-0.173103,-1.43377,2.27787},
                        {-0.126946,-1.42117,2.27717},
                        {-0.0795417,-1.41444,2.27187},
                        {-0.0303044,-1.40787,2.26565},
                        {0.0143333,-1.40333,2.25667},
                        {-0.111273,-1.39227,2.29564},
                        {-0.0719152,-1.38349,2.29103},
                        {-0.0258421,-1.37515,2.28495},
                        {0.0238113,-1.37591,2.27331},
                        {0.0715326,-1.37279,2.26529},
                        {0.120153,-1.36564,2.25802},
                        {0.163625,-1.35731,2.253},
                        {-0.063,-1.3065,2.2985},
                        {-0.0269762,-1.32563,2.29533},
                        {0.0235119,-1.32585,2.28791},
                        {0.0743256,-1.32637,2.27959},
                        {0.123141,-1.32652,2.27123},
                        {0.173354,-1.32697,2.26144},
                        {0.219274,-1.31925,2.25663},
                        {0.268667,-1.30737,2.25359},
                        {-0.108583,-1.26317,2.29792},
                        {-0.071,-1.26987,2.29687},
                        {-0.0258919,-1.27569,2.29247},
                        {0.0233529,-1.27778,2.28453},
                        {0.0733286,-1.27714,2.27613},
                        {0.123853,-1.27637,2.27157},
                        {0.173952,-1.27615,2.2661},
                        {0.225217,-1.275,2.25953},
                        {0.269281,-1.27605,2.25351},
                        {0.30375,-1.27975,2.25},
                        {-1.288,-1.20567,2.27833},
                        {-0.183714,-1.208,2.29871},
                        {-0.119429,-1.22202,2.29666},
                        {-0.0765942,-1.226,2.28988},
                        {-0.0290222,-1.22867,2.27298},
                        {0.0192174,-1.23661,2.26139},
                        {0.0727857,-1.241,2.2565},
                        {0.1316,-1.2408,2.2588},
                        {0.178738,-1.22995,2.25889},
                        {0.224263,-1.22474,2.25504},
                        {0.263556,-1.23092,2.25203},
                        {-1.2842,-1.17284,2.27696},
                        {-0.263,-1.15325,2.2985},
                        {-0.215542,-1.16217,2.29717},
                        {-0.17497,-1.17135,2.29539},
                        {-0.12525,-1.17558,2.28988},
                        {-0.0793023,-1.17891,2.26798},
                        {-0.0186061,-1.16794,2.25215},
                        {0.0220566,-1.16466,2.25479},
                        {0.0617586,-1.16452,2.25252},
                        {0.227667,-1.18639,2.25112},
                        {0.255111,-1.18689,2.25022},
                        {-1.28469,-1.12344,2.27433},
                        {-0.266605,-1.1243,2.29686},
                        {-0.224447,-1.12282,2.29297},
                        {-0.176744,-1.12638,2.29074},
                        {-0.128015,-1.12525,2.27719},
                        {-0.0671579,-1.12237,2.25289},
                        {-0.0252105,-1.12691,2.254},
                        {0.0243606,-1.12879,2.2519},
                        {0.0561667,-1.13575,2.251},
                        {-1.30233,-1.08144,2.26733},
                        {-1.2898,-1.07912,2.27822},
                        {-0.354,-1.0555,2.299},
                        {-0.32607,-1.06258,2.29681},
                        {-0.273372,-1.07474,2.29169},
                        {-0.225542,-1.07376,2.28548},
                        {-0.175679,-1.07438,2.28083},
                        {-0.133172,-1.07607,2.26767},
                        {-0.0587143,-1.07577,2.25357},
                        {-0.0402,-1.07637,2.25433},
                        {0.029,-1.08791,2.25055},
                        {0.05525,-1.073,2.25125},
                        {-1.28924,-1.02589,2.27258},
                        {-0.40525,-1.01,2.298},
                        {-0.373794,-1.02153,2.29484},
                        {-0.325947,-1.02505,2.29258},
                        {-0.276775,-1.02511,2.28531},
                        {-0.225378,-1.02439,2.27877},
                        {-0.174571,-1.02512,2.27404},
                        {-0.133193,-1.02561,2.26405},
                        {-0.056,-1.04433,2.2515},
                        {-0.0428,-1.045,2.2518},
                        {-1.30239,-0.966222,2.26211},
                        {-1.29483,-0.98185,2.275},
                        {-0.62425,-0.96275,2.297},
                        {-0.459286,-0.963286,2.29771},
                        {-0.423949,-0.969661,2.29549},
                        {-0.374681,-0.974725,2.28886},
                        {-0.323773,-0.975363,2.28367},
                        {-0.274741,-0.974706,2.27805},
                        {-0.225422,-0.975518,2.27093},
                        {-0.17487,-0.97441,2.26511},
                        {-0.127812,-0.975025,2.25825},
                        {-1.31022,-0.924797,2.27397},
                        {-1.29825,-0.9435,2.29325},
                        {-0.626818,-0.933273,2.29655},
                        {-0.59,-0.918,2.299},
                        {-0.5168,-0.9141,2.29803},
                        {-0.474704,-0.921254,2.29491},
                        {-0.425966,-0.924921,2.28906},
                        {-0.375753,-0.925146,2.2822},
                        {-0.326012,-0.924494,2.27595},
                        {-0.275859,-0.925261,2.26969},
                        {-0.225726,-0.925937,2.26439},
                        {-0.176247,-0.923899,2.25837},
                        {-0.128613,-0.9294,2.25501},
                        {-0.098,-0.945,2.25},
                        {-1.31365,-0.874754,2.27544},
                        {-0.651,-0.8565,2.2945},
                        {-0.624625,-0.87225,2.29683},
                        {-0.570281,-0.866844,2.29756},
                        {-0.525321,-0.875572,2.29483},
                        {-0.475851,-0.874862,2.28755},
                        {-0.425495,-0.874818,2.28221},
                        {-0.374308,-0.875418,2.27541},
                        {-0.323826,-0.874956,2.26969},
                        {-0.275211,-0.875622,2.26378},
                        {-0.224951,-0.875874,2.25825},
                        {-0.179,-0.879803,2.25395},
                        {-0.1452,-0.8972,2.2506},
                        {-1.32143,-0.824556,2.27397},
                        {-0.655517,-0.822586,2.29186},
                        {-0.623872,-0.825047,2.29256},
                        {-0.575321,-0.82369,2.2945},
                        {-0.525102,-0.826421,2.28776},
                        {-0.475374,-0.82611,2.28288},
                        {-0.425132,-0.825242,2.27656},
                        {-0.374547,-0.825852,2.26952},
                        {-0.326204,-0.826065,2.26402},
                        {-0.275559,-0.825422,2.25742},
                        {-0.228968,-0.831603,2.25278},
                        {-0.195,-0.8386,2.2518},
                        {-1.33679,-0.77629,2.27476},
                        {-0.660417,-0.774445,2.29372},
                        {-0.624505,-0.774907,2.28995},
                        {-0.575659,-0.773901,2.28867},
                        {-0.525219,-0.775323,2.28123},
                        {-0.475798,-0.774838,2.27703},
                        {-0.426306,-0.774653,2.27003},
                        {-0.375476,-0.77501,2.26245},
                        {-0.324089,-0.774554,2.25671},
                        {-0.2825,-0.778339,2.25239},
                        {-0.247,-0.79275,2.2505},
                        {-1.33986,-0.724351,2.27233},
                        {-0.66437,-0.7235,2.28883},
                        {-0.624852,-0.724659,2.28501},
                        {-0.575764,-0.724473,2.2842},
                        {-0.525978,-0.723945,2.27527},
                        {-0.475646,-0.724177,2.26984},
                        {-0.425227,-0.724608,2.26472},
                        {-0.375299,-0.723732,2.25727},
                        {-0.330222,-0.729667,2.25225},
                        {-0.2865,-0.74575,2.25},
                        {-1.35265,-0.660824,2.26512},
                        {-1.34551,-0.681385,2.27713},
                        {-0.669364,-0.672954,2.28282},
                        {-0.62502,-0.67491,2.28061},
                        {-0.575255,-0.674213,2.27668},
                        {-0.525424,-0.674273,2.27064},
                        {-0.474569,-0.673245,2.26324},
                        {-0.424343,-0.674091,2.2562},
                        {-0.379389,-0.678903,2.25258},
                        {-0.346,-0.6945,2.25},
                        {-1.35788,-0.62612,2.27114},
                        {-0.7046,-0.6118,2.2928},
                        {-0.673095,-0.624417,2.27768},
                        {-0.626351,-0.624948,2.27632},
                        {-0.574916,-0.624684,2.27228},
                        {-0.525116,-0.625263,2.26615},
                        {-0.475872,-0.62483,2.25824},
                        {-0.426454,-0.626752,2.25351},
                        {-0.387429,-0.634286,2.25107},
                        {-1.36786,-0.575902,2.27294},
                        {-0.706,-0.572118,2.27976},
                        {-0.673394,-0.575638,2.27098},
                        {-0.626273,-0.575111,2.27407},
                        {-0.576152,-0.575212,2.26654},
                        {-0.5262,-0.57627,2.26065},
                        {-0.478631,-0.576609,2.25343},
                        {-0.441667,-0.594,2.25056},
                        {-1.37657,-0.525123,2.27212},
                        {-0.709279,-0.524489,2.26793},
                        {-0.67358,-0.52622,2.26774},
                        {-0.624804,-0.525072,2.26806},
                        {-0.575374,-0.525747,2.26133},
                        {-0.525409,-0.526057,2.25605},
                        {-0.479962,-0.533962,2.25088},
                        {-0.45,-0.545,2.25},
                        {-1.38702,-0.475296,2.27282},
                        {-0.712315,-0.475019,2.26315},
                        {-0.67502,-0.476941,2.26398},
                        {-0.625273,-0.474697,2.26218},
                        {-0.576029,-0.476039,2.25661},
                        {-0.532975,-0.483875,2.25105},
                        {-0.49675,-0.493,2.25025},
                        {-1.401,-0.443,2.293},
                        {-1.3948,-0.4478,2.2812},
                        {-0.717454,-0.42497,2.25942},
                        {-0.675053,-0.426149,2.26041},
                        {-0.625722,-0.425773,2.25771},
                        {-0.578919,-0.426756,2.25215},
                        {-0.54225,-0.43625,2.2505},
                        {-0.755,-0.35925,2.286},
                        {-0.720901,-0.374472,2.25773},
                        {-0.675204,-0.373805,2.25609},
                        {-0.625155,-0.37599,2.25345},
                        {-0.5915,-0.38425,2.25055},
                        {-0.755929,-0.322643,2.274},
                        {-0.726083,-0.327431,2.25526},
                        {-0.674821,-0.325453,2.25415},
                        {-0.635838,-0.333027,2.25192},
                        {-0.760645,-0.273903,2.26416},
                        {-0.726429,-0.279229,2.25183},
                        {-0.679691,-0.284782,2.25142},
                        {-0.629,-0.296,2.25},
                        {-0.7682,-0.2273,2.26155},
                        {-0.703333,-0.242333,2.251},
                        {-0.6935,-0.242,2.2505},
                        {-0.805,-0.163,2.298},
                        {-0.780815,-0.177963,2.25804},
                        {-0.804,-0.139,2.28},
                        {-0.789857,-0.139143,2.25514},
                        {-0.823,-0.002,2.251},
                        {-0.8388,0.033,2.275},
                        {-0.842333,0.0638333,2.26883},
                        {-0.845,0.106,2.254},
                        {-1.00567,0.396333,2.29833},
                        {-0.9612,0.3918,2.2973},
                        {-0.929556,0.383037,2.27689},
                        {-1.00869,0.422,2.29831},
                        {-0.968881,0.425424,2.29451},
                        {-0.936172,0.421862,2.27555},
                        {-0.965968,0.475387,2.29561},
                        {-0.941445,0.473,2.27422},
                        {-1.0025,0.5445,2.29775},
                        {-0.972661,0.526018,2.29071},
                        {-0.943333,0.524444,2.25889},
                        {-1.01895,0.577527,2.29733},
                        {-0.978684,0.573368,2.28284},
                        {-0.95,0.57,2.25},
                        {-1.11157,0.641571,2.29843},
                        {-1.07151,0.629231,2.29754},
                        {-1.02401,0.625123,2.29425},
                        {-0.984279,0.622558,2.27907},
                        {-1.11241,0.679706,2.29724},
                        {-1.07743,0.676273,2.29536},
                        {-1.02668,0.674543,2.28989},
                        {-0.989214,0.672107,2.27175},
                        {-1.1597,0.732304,2.29678},
                        {-1.12641,0.732439,2.29585},
                        {-1.07494,0.725171,2.29268},
                        {-1.02532,0.723915,2.28172},
                        {-0.993,0.720533,2.26253},
                        {-1.1643,0.770696,2.29678},
                        {-1.12414,0.773658,2.29256},
                        {-1.07513,0.773013,2.28765},
                        {-1.02709,0.774076,2.27626},
                        {-0.992667,0.758667,2.25067},
                        {-1.15425,0.805,2.29475},
                        {-1.12157,0.821309,2.28663},
                        {-1.07538,0.824729,2.28488},
                        {-1.03156,0.82266,2.274},
                        {-1.10809,0.858909,2.27627},
                        {-1.07266,0.872466,2.26897},
                        {-1.03572,0.870969,2.26163},
                        {-1.06406,0.909063,2.25644},
                        {-1.04586,0.908572,2.25329},
                        {-1.20988,-1.63563,2.33562},
                        {-1.171,-1.62636,2.32736},
                        {-1.12778,-1.617,2.32565},
                        {-1.07541,-1.60691,2.32341},
                        {-1.03946,-1.60285,2.31815},
                        {-1.053,-1.597,2.347},
                        {-1.01758,-1.59629,2.32819},
                        {-0.974743,-1.58697,2.32391},
                        {-0.924275,-1.57967,2.32568},
                        {-0.874286,-1.57324,2.32653},
                        {-0.825455,-1.56307,2.32596},
                        {-0.781667,-1.55277,2.3185},
                        {-0.7345,-1.55025,2.3075},
                        {-0.76165,-1.54645,2.33235},
                        {-0.723096,-1.54088,2.32617},
                        {-0.673897,-1.5309,2.3249},
                        {-0.625298,-1.5216,2.32623},
                        {-0.57676,-1.51022,2.3265},
                        {-0.5334,-1.50372,2.31412},
                        {-0.489,-1.5,2.3},
                        {-0.566083,-1.49458,2.34533},
                        {-0.52118,-1.49012,2.33576},
                        {-0.475088,-1.47913,2.32959},
                        {-0.426647,-1.47133,2.32524},
                        {-0.375197,-1.46492,2.31703},
                        {-0.328688,-1.45781,2.31009},
                        {-0.291667,-1.45367,2.30333},
                        {-0.4605,-1.449,2.349},
                        {-0.420091,-1.43543,2.34418},
                        {-0.373084,-1.42741,2.33983},
                        {-0.324761,-1.42576,2.3279},
                        {-0.275107,-1.42494,2.31783},
                        {-0.227437,-1.42117,2.30952},
                        {-0.180811,-1.41141,2.30395},
                        {-0.146,-1.403,2.3},
                        {-0.41452,-1.3724,2.3484},
                        {-0.373692,-1.37389,2.34535},
                        {-0.324333,-1.37575,2.33765},
                        {-0.27525,-1.37644,2.3312},
                        {-0.225247,-1.37558,2.32235},
                        {-0.174629,-1.37691,2.31413},
                        {-0.127803,-1.37417,2.30749},
                        {-0.0832432,-1.36516,2.3043},
                        {-0.46105,-1.3203,2.34775},
                        {-0.425458,-1.32374,2.34468},
                        {-0.375343,-1.32369,2.34048},
                        {-0.325658,-1.32508,2.3329},
                        {-0.276117,-1.32409,2.32692},
                        {-0.225089,-1.32457,2.32196},
                        {-0.174458,-1.3254,2.31276},
                        {-0.126613,-1.32464,2.30859},
                        {-0.0792917,-1.32796,2.30404},
                        {-0.049,-1.315,2.3},
                        {-0.519562,-1.25906,2.34619},
                        {-0.471978,-1.27071,2.34542},
                        {-0.425382,-1.27375,2.33925},
                        {-0.374667,-1.27394,2.33332},
                        {-0.32497,-1.27476,2.32749},
                        {-0.274662,-1.27472,2.32199},
                        {-0.224557,-1.27414,2.31561},
                        {-0.176105,-1.27463,2.3086},
                        {-0.129985,-1.27686,2.30431},
                        {-0.0887727,-1.28477,2.30123},
                        {-1.2798,-1.2081,2.3209},
                        {-0.611,-1.2085,2.349},
                        {-0.568654,-1.21404,2.34715},
                        {-0.525183,-1.22617,2.34263},
                        {-0.475278,-1.2265,2.33733},
                        {-0.425778,-1.22378,2.33114},
                        {-0.375573,-1.22511,2.3272},
                        {-0.324974,-1.22595,2.32022},
                        {-0.276,-1.22374,2.31308},
                        {-0.226453,-1.22532,2.30853},
                        {-0.1745,-1.22669,2.30328},
                        {-0.139217,-1.23413,2.30204},
                        {-1.26738,-1.1731,2.32364},
                        {-0.720565,-1.16757,2.34839},
                        {-0.674935,-1.1652,2.34293},
                        {-0.622926,-1.17382,2.34211},
                        {-0.575574,-1.1765,2.33965},
                        {-0.526013,-1.17504,2.33668},
                        {-0.47559,-1.17445,2.32926},
                        {-0.425269,-1.17556,2.32656},
                        {-0.374467,-1.17579,2.32005},
                        {-0.324301,-1.17452,2.31292},
                        {-0.275297,-1.17703,2.30655},
                        {-0.228464,-1.18209,2.303},
                        {-0.169714,-1.19279,2.30136},
                        {-1.27203,-1.12615,2.32578},
                        {-0.709111,-1.13206,2.34794},
                        {-0.670864,-1.1278,2.34306},
                        {-0.624459,-1.12505,2.33372},
                        {-0.575395,-1.12462,2.32499},
                        {-0.526459,-1.1245,2.32895},
                        {-0.476093,-1.12435,2.32351},
                        {-0.42559,-1.12423,2.31733},
                        {-0.374342,-1.12567,2.31172},
                        {-0.325078,-1.12592,2.30595},
                        {-0.286974,-1.12659,2.3018},
                        {-0.243429,-1.14371,2.30086},
                        {-1.27847,-1.07743,2.32492},
                        {-0.618125,-1.08195,2.3364},
                        {-0.575405,-1.07525,2.3217},
                        {-0.526259,-1.07604,2.32332},
                        {-0.474716,-1.07484,2.3162},
                        {-0.425225,-1.07346,2.31065},
                        {-0.375953,-1.07638,2.30602},
                        {-0.322167,-1.08738,2.30119},
                        {-0.296,-1.08975,2.30025},
                        {-1.28266,-1.02317,2.32362},
                        {-0.6118,-1.02573,2.3198},
                        {-0.574733,-1.02446,2.31547},
                        {-0.524691,-1.02421,2.31457},
                        {-0.475481,-1.02488,2.30847},
                        {-0.427753,-1.026,2.30505},
                        {-0.379182,-1.04436,2.30055},
                        {-1.29282,-0.976235,2.32651},
                        {-0.612463,-0.974342,2.30712},
                        {-0.575591,-0.975012,2.31016},
                        {-0.525765,-0.974272,2.30769},
                        {-0.478431,-0.977389,2.30294},
                        {-0.428963,-0.990778,2.30119},
                        {-1.30284,-0.9128,2.32076},
                        {-1.2957,-0.932939,2.33106},
                        {-0.619068,-0.921881,2.30166},
                        {-0.575047,-0.925279,2.30364},
                        {-0.530415,-0.930811,2.30236},
                        {-0.485846,-0.943154,2.30069},
                        {-1.30444,-0.874375,2.32423},
                        {-1.298,-0.8955,2.3475},
                        {-0.62525,-0.8865,2.30042},
                        {-0.579263,-0.879754,2.30172},
                        {-0.543,-0.897,2.3},
                        {-1.30978,-0.824517,2.32285},
                        {-0.586,-0.843833,2.30033},
                        {-1.32251,-0.775532,2.32537},
                        {-0.669,-0.794,2.3},
                        {-1.33033,-0.724507,2.32828},
                        {-1.35267,-0.653167,2.31017},
                        {-1.33944,-0.6755,2.3274},
                        {-1.35719,-0.622555,2.31658},
                        {-1.34506,-0.626909,2.33642},
                        {-1.36168,-0.573905,2.32257},
                        {-1.34836,-0.584,2.34182},
                        {-1.3681,-0.524767,2.32321},
                        {-1.3752,-0.475689,2.32234},
                        {-1.40386,-0.431143,2.31243},
                        {-1.38471,-0.431776,2.32426},
                        {-0.7748,-0.3686,2.3324},
                        {-0.778,-0.329143,2.32271},
                        {-0.783,-0.268,2.3},
                        {-0.807,-0.214,2.331},
                        {-0.7915,-0.24,2.3055},
                        {-0.802,-0.187,2.303},
                        {-0.8605,0.0245,2.3315},
                        {-0.849,0.017,2.308},
                        {-0.869,0.066,2.333},
                        {-1.26719,0.337563,2.34706},
                        {-1.22081,0.340143,2.34333},
                        {-1.17665,0.340903,2.33655},
                        {-1.13013,0.345563,2.335},
                        {-1.26893,0.372546,2.34205},
                        {-1.22561,0.375642,2.33347},
                        {-1.17499,0.374048,2.32968},
                        {-1.12318,0.373622,2.32172},
                        {-1.07442,0.3751,2.31976},
                        {-1.0256,0.380581,2.31429},
                        {-0.98068,0.38264,2.31386},
                        {-0.9442,0.3712,2.315},
                        {-1.30586,0.436143,2.34729},
                        {-1.27305,0.4265,2.34459},
                        {-1.22597,0.424771,2.33897},
                        {-1.1756,0.424123,2.32938},
                        {-1.12419,0.424519,2.3207},
                        {-1.07336,0.424364,2.31187},
                        {-1.02769,0.423824,2.30509},
                        {-0.990654,0.420039,2.30281},
                        {-1.30927,0.483867,2.3456},
                        {-1.27549,0.473985,2.34165},
                        {-1.22636,0.473663,2.33325},
                        {-1.17532,0.475812,2.32868},
                        {-1.12365,0.47445,2.3232},
                        {-1.07409,0.473987,2.31532},
                        {-1.02577,0.474413,2.30786},
                        {-0.985875,0.474021,2.30181},
                        {-1.31814,0.530657,2.34263},
                        {-1.276,0.524847,2.33411},
                        {-1.22691,0.524014,2.32891},
                        {-1.17473,0.524,2.32135},
                        {-1.12383,0.52458,2.31644},
                        {-1.07482,0.52375,2.31215},
                        {-1.02517,0.52406,2.30514},
                        {-0.991769,0.514538,2.30138},
                        {-1.31861,0.569544,2.33891},
                        {-1.27507,0.574366,2.33048},
                        {-1.2264,0.574307,2.32557},
                        {-1.17544,0.574875,2.3183},
                        {-1.12465,0.57459,2.3109},
                        {-1.07586,0.57458,2.30578},
                        {-1.03445,0.566065,2.30074},
                        {-1.303,0.6045,2.328},
                        {-1.27333,0.620315,2.32593},
                        {-1.22519,0.624679,2.31855},
                        {-1.17524,0.624778,2.3134},
                        {-1.12811,0.622718,2.30663},
                        {-1.07846,0.61908,2.30294},
                        {-1.0438,0.6082,2.3014},
                        {-1.25942,0.661083,2.32358},
                        {-1.22206,0.674029,2.31218},
                        {-1.17404,0.674582,2.30766},
                        {-1.13004,0.672577,2.30335},
                        {-1.0707,0.6542,2.3013},
                        {-1.21054,0.715917,2.30871},
                        {-1.18017,0.721463,2.304},
                        {-1.12581,0.715722,2.30233},
                        {-1.1,0.712,2.3},
                        {-1.16887,0.772391,2.30157},
                        {-1.256,-1.63767,2.392},
                        {-1.22447,-1.63472,2.37564},
                        {-1.17572,-1.62291,2.37678},
                        {-1.12544,-1.61051,2.3749},
                        {-1.08258,-1.60274,2.36611},
                        {-1.103,-1.6,2.384},
                        {-1.06732,-1.59628,2.38284},
                        {-1.0254,-1.58975,2.37503},
                        {-0.974962,-1.57998,2.37451},
                        {-0.925541,-1.56926,2.37766},
                        {-0.876231,-1.56102,2.37631},
                        {-0.830559,-1.555,2.36765},
                        {-0.794,-1.552,2.35525},
                        {-0.868056,-1.5385,2.39694},
                        {-0.821193,-1.52902,2.39002},
                        {-0.775165,-1.5252,2.37943},
                        {-0.725826,-1.52025,2.3732},
                        {-0.676608,-1.51324,2.36527},
                        {-0.625914,-1.51014,2.35803},
                        {-0.597,-1.506,2.3515},
                        {-0.811,-1.495,2.399},
                        {-0.766,-1.48354,2.39689},
                        {-0.725203,-1.47542,2.39177},
                        {-0.674195,-1.47753,2.38358},
                        {-0.623648,-1.47447,2.37158},
                        {-0.57612,-1.4734,2.36304},
                        {-0.526146,-1.46669,2.35696},
                        {-0.489889,-1.45656,2.35256},
                        {-0.807,-1.401,2.399},
                        {-0.768333,-1.4202,2.39687},
                        {-0.723295,-1.4241,2.38869},
                        {-0.673839,-1.42556,2.38334},
                        {-0.624984,-1.42608,2.37811},
                        {-0.574972,-1.42489,2.37185},
                        {-0.524931,-1.42669,2.36447},
                        {-0.475143,-1.425,2.35877},
                        {-0.42675,-1.41608,2.35227},
                        {-0.394,-1.40475,2.35025},
                        {-0.870857,-1.36071,2.398},
                        {-0.822651,-1.36849,2.39374},
                        {-0.774525,-1.37405,2.39095},
                        {-0.72461,-1.37368,2.38305},
                        {-0.675615,-1.37545,2.37895},
                        {-0.624375,-1.37536,2.37316},
                        {-0.575295,-1.37405,2.36728},
                        {-0.525662,-1.3758,2.36194},
                        {-0.475141,-1.37364,2.35683},
                        {-0.432,-1.37656,2.35214},
                        {-0.386286,-1.38686,2.35043},
                        {-0.953,-1.3065,2.399},
                        {-0.9165,-1.31038,2.3975},
                        {-0.873119,-1.32405,2.39558},
                        {-0.824967,-1.32508,2.38946},
                        {-0.774969,-1.32513,2.386},
                        {-0.725381,-1.32452,2.37838},
                        {-0.675281,-1.32547,2.37441},
                        {-0.62606,-1.32531,2.36861},
                        {-0.574944,-1.32389,2.36256},
                        {-0.524486,-1.32474,2.35665},
                        {-0.4816,-1.32648,2.35366},
                        {-0.45,-1.347,2.351},
                        {-1.307,-1.255,2.393},
                        {-0.954571,-1.27214,2.39786},
                        {-0.924827,-1.27517,2.39529},
                        {-0.876295,-1.27443,2.38995},
                        {-0.825688,-1.27594,2.38564},
                        {-0.774841,-1.2751,2.37997},
                        {-0.72359,-1.27479,2.37282},
                        {-0.674254,-1.27484,2.36618},
                        {-0.624656,-1.27546,2.36085},
                        {-0.575569,-1.27494,2.35777},
                        {-0.526702,-1.2827,2.35468},
                        {-0.483421,-1.28921,2.35163},
                        {-1.27656,-1.21981,2.37719},
                        {-1.05713,-1.21775,2.39788},
                        {-1.02429,-1.22235,2.39504},
                        {-0.975177,-1.22327,2.38998},
                        {-0.924562,-1.22747,2.38653},
                        {-0.875105,-1.22649,2.38286},
                        {-0.825638,-1.22647,2.3781},
                        {-0.774877,-1.22608,2.37143},
                        {-0.724079,-1.22676,2.3669},
                        {-0.674612,-1.22522,2.36082},
                        {-0.6258,-1.22758,2.35483},
                        {-0.580023,-1.23335,2.35153},
                        {-0.549,-1.231,2.351},
                        {-1.25544,-1.17218,2.36795},
                        {-1.24933,-1.16783,2.39283},
                        {-1.05433,-1.19267,2.399},
                        {-1.021,-1.19086,2.39486},
                        {-0.970107,-1.18027,2.38913},
                        {-0.923842,-1.17416,2.38171},
                        {-0.87487,-1.17423,2.37155},
                        {-0.825493,-1.17358,2.36535},
                        {-0.775719,-1.17436,2.35777},
                        {-0.728283,-1.17598,2.3533},
                        {-0.6753,-1.19245,2.3546},
                        {-0.6384,-1.1919,2.3513},
                        {-1.25777,-1.12406,2.37467},
                        {-1.2475,-1.1415,2.397},
                        {-0.9085,-1.14583,2.39417},
                        {-0.870829,-1.14029,2.38323},
                        {-0.824328,-1.13365,2.37753},
                        {-0.773758,-1.12778,2.36888},
                        {-0.726659,-1.11916,2.36437},
                        {-0.682682,-1.10655,2.35636},
                        {-1.26687,-1.07472,2.37512},
                        {-0.671314,-1.09469,2.37511},
                        {-0.638,-1.07994,2.36178},
                        {-1.27052,-1.02588,2.37497},
                        {-1.27754,-0.975667,2.3707},
                        {-1.28271,-0.925629,2.37313},
                        {-1.301,-0.857,2.3505},
                        {-1.29044,-0.876952,2.37371},
                        {-1.30285,-0.822154,2.36288},
                        {-1.29666,-0.830207,2.384},
                        {-1.30696,-0.773714,2.37439},
                        {-1.299,-0.799,2.397},
                        {-1.31662,-0.724309,2.3764},
                        {-1.32571,-0.673948,2.37302},
                        {-1.33571,-0.624051,2.37281},
                        {-1.35336,-0.556727,2.36555},
                        {-1.34476,-0.579217,2.37693},
                        {-1.35643,-0.525204,2.37408},
                        {-1.34875,-0.5395,2.39525},
                        {-1.36226,-0.474828,2.37319},
                        {-1.423,-0.406,2.395},
                        {-1.37407,-0.423768,2.37368},
                        {-1.406,-0.3955,2.384},
                        {-1.38548,-0.39181,2.38019},
                        {-0.803,-0.375667,2.39433},
                        {-0.7948,-0.3656,2.3742},
                        {-0.8056,-0.338,2.387},
                        {-0.792667,-0.325,2.356},
                        {-0.82975,-0.224,2.388},
                        {-0.879,0.0225,2.3755},
                        {-0.887833,0.0616667,2.377},
                        {-1.30467,0.3235,2.3815},
                        {-1.27386,0.326091,2.36723},
                        {-1.2268,0.322961,2.37365},
                        {-1.17326,0.319397,2.38101},
                        {-1.12685,0.329219,2.38452},
                        {-1.3115,0.384167,2.369},
                        {-1.29094,0.374056,2.353},
                        {-1.247,0.35,2.35},
                        {-1.10367,0.350333,2.37267},
                        {-1.07439,0.358333,2.37228},
                        {-1.02464,0.371455,2.37045},
                        {-0.98375,0.375,2.3785},
                        {-1.31713,0.424167,2.35879},
                        {-1.296,0.4068,2.3512},
                        {-1.32059,0.469552,2.35669},
                        {-1.293,0.474667,2.35033},
                        {-1.35875,0.540125,2.36325},
                        {-1.33568,0.519,2.35442},
                        {-1.30467,-1.63267,2.44733},
                        {-1.27203,-1.63436,2.42955},
                        {-1.22264,-1.62336,2.42777},
                        {-1.17664,-1.61268,2.4238},
                        {-1.13811,-1.60611,2.41133},
                        {-1.21,-1.59833,2.44733},
                        {-1.16833,-1.58541,2.44415},
                        {-1.1239,-1.57973,2.43479},
                        {-1.0759,-1.57471,2.42395},
                        {-1.02602,-1.57069,2.41739},
                        {-0.977727,-1.56127,2.40945},
                        {-0.92575,-1.55358,2.40283},
                        {-0.895,-1.5525,2.4015},
                        {-1.15733,-1.50867,2.44767},
                        {-1.11603,-1.52483,2.4459},
                        {-1.07304,-1.52676,2.43993},
                        {-1.0242,-1.52511,2.43266},
                        {-0.973708,-1.52546,2.42286},
                        {-0.925698,-1.52506,2.41424},
                        {-0.878745,-1.51845,2.40613},
                        {-0.841125,-1.51175,2.40281},
                        {-1.16268,-1.47159,2.447},
                        {-1.12463,-1.47388,2.44404},
                        {-1.07602,-1.47506,2.43842},
                        {-1.0254,-1.47508,2.43224},
                        {-0.974922,-1.47502,2.4238},
                        {-0.92488,-1.47502,2.41848},
                        {-0.8755,-1.47586,2.41148},
                        {-0.825719,-1.47567,2.40539},
                        {-0.783103,-1.47269,2.40277},
                        {-1.21828,-1.41608,2.44552},
                        {-1.17323,-1.42394,2.44498},
                        {-1.12513,-1.42487,2.43885},
                        {-1.0751,-1.4235,2.43221},
                        {-1.024,-1.4252,2.42648},
                        {-0.973909,-1.42424,2.41836},
                        {-0.924073,-1.42567,2.41125},
                        {-0.875614,-1.42461,2.40733},
                        {-0.825923,-1.42417,2.40195},
                        {-0.783258,-1.43077,2.40119},
                        {-1.27272,-1.36572,2.44717},
                        {-1.22205,-1.37312,2.44438},
                        {-1.17649,-1.37429,2.43696},
                        {-1.12372,-1.37356,2.43078},
                        {-1.07437,-1.374,2.42552},
                        {-1.02487,-1.37615,2.4185},
                        {-0.975423,-1.37444,2.4126},
                        {-0.924947,-1.37486,2.40517},
                        {-0.878167,-1.38297,2.4025},
                        {-0.832364,-1.39173,2.40109},
                        {-1.357,-1.3145,2.443},
                        {-1.32035,-1.31326,2.44757},
                        {-1.27258,-1.32214,2.44467},
                        {-1.22604,-1.32448,2.43879},
                        {-1.1752,-1.32467,2.43165},
                        {-1.12532,-1.32413,2.42417},
                        {-1.07562,-1.32475,2.41883},
                        {-1.02447,-1.32621,2.41153},
                        {-0.976931,-1.32555,2.40578},
                        {-0.929467,-1.3296,2.40218},
                        {-0.899,-1.339,2.401},
                        {-1.31419,-1.28024,2.43076},
                        {-1.27333,-1.27343,2.42993},
                        {-1.22707,-1.27455,2.42793},
                        {-1.17508,-1.27486,2.42098},
                        {-1.12622,-1.27618,2.41988},
                        {-1.07497,-1.27567,2.41239},
                        {-1.02433,-1.27544,2.40512},
                        {-0.977392,-1.27569,2.40257},
                        {-0.937333,-1.287,2.4},
                        {-1.26594,-1.23113,2.41432},
                        {-1.22621,-1.22349,2.42365},
                        {-1.17513,-1.22431,2.41574},
                        {-1.12582,-1.2249,2.41033},
                        {-1.07967,-1.22512,2.4041},
                        {-1.03057,-1.24557,2.40143},
                        {-0.9775,-1.24825,2.40025},
                        {-1.251,-1.194,2.403},
                        {-1.23409,-1.17886,2.42849},
                        {-1.17142,-1.19024,2.43473},
                        {-1.12362,-1.18451,2.42734},
                        {-1.07506,-1.17907,2.42052},
                        {-1.02554,-1.16793,2.41776},
                        {-0.981282,-1.15792,2.41849},
                        {-0.937167,-1.15117,2.4035},
                        {-1.252,-1.11333,2.40167},
                        {-1.24058,-1.12482,2.42584},
                        {-0.956,-1.14775,2.42575},
                        {-0.925926,-1.14326,2.41985},
                        {-0.87775,-1.13475,2.42368},
                        {-0.826783,-1.12404,2.42043},
                        {-0.77481,-1.11524,2.42243},
                        {-0.724533,-1.10613,2.42127},
                        {-0.685667,-1.1025,2.43667},
                        {-1.25406,-1.0685,2.4075},
                        {-1.24626,-1.07795,2.43013},
                        {-0.7055,-1.1,2.407},
                        {-0.673364,-1.09418,2.41227},
                        {-1.25679,-1.02247,2.42315},
                        {-1.249,-1.03486,2.44157},
                        {-1.26334,-0.976492,2.42658},
                        {-1.26939,-0.92663,2.42365},
                        {-1.27854,-0.87525,2.42348},
                        {-1.28821,-0.826038,2.42092},
                        {-1.30127,-0.77,2.40536},
                        {-1.29385,-0.777727,2.42082},
                        {-1.30686,-0.7224,2.41257},
                        {-1.29688,-0.7365,2.42163},
                        {-1.31744,-0.676293,2.41712},
                        {-1.32689,-0.626027,2.41476},
                        {-1.33957,-0.573676,2.41673},
                        {-1.35318,-0.5145,2.42073},
                        {-1.34893,-0.5376,2.4182},
                        {-1.35807,-0.4769,2.4188},
                        {-1.36748,-0.427174,2.4157},
                        {-0.823,-0.402,2.441},
                        {-1.404,-0.371,2.426},
                        {-1.38262,-0.379448,2.41366},
                        {-0.868555,-0.365489,2.43051},
                        {-0.830625,-0.370521,2.42325},
                        {-0.906,-0.344,2.448},
                        {-0.87225,-0.335975,2.4339},
                        {-0.831829,-0.332257,2.42243},
                        {-0.8405,-0.275929,2.43621},
                        {-0.86225,-0.2338,2.4397},
                        {-0.842875,-0.23325,2.41875},
                        {-0.8665,-0.1633,2.4372},
                        {-0.953,0.0403333,2.44467},
                        {-0.922921,0.0366842,2.43274},
                        {-0.8925,0.033,2.4045},
                        {-1.16126,0.0788518,2.43833},
                        {-1.12412,0.0765658,2.43537},
                        {-1.09563,0.0778125,2.44512},
                        {-0.953,0.054,2.441},
                        {-0.92304,0.06144,2.422},
                        {-0.899,0.0675,2.4},
                        {-1.15973,0.109636,2.44182},
                        {-1.132,0.105167,2.43772},
                        {-1.099,0.101,2.445},
                        {-1.25892,0.232385,2.44492},
                        {-1.23222,0.23825,2.43997},
                        {-1.16569,0.243308,2.44038},
                        {-1.13821,0.241929,2.43921},
                        {-1.31552,0.28719,2.43762},
                        {-1.27351,0.275197,2.42533},
                        {-1.22458,0.275704,2.41285},
                        {-1.17551,0.274477,2.41545},
                        {-1.12471,0.275814,2.42648},
                        {-1.09775,0.2905,2.446},
                        {-1.31888,0.308625,2.41775},
                        {-1.27719,0.306333,2.40876},
                        {-1.23033,0.306733,2.4018},
                        {-1.175,0.3015,2.4005},
                        {-1.11197,0.313054,2.41059},
                        {-1.08926,0.327807,2.42895},
                        {-0.970333,0.335333,2.44067},
                        {-1.07871,0.361167,2.42596},
                        {-1.03085,0.374154,2.42262},
                        {-0.98925,0.368,2.4345},
                        {-1.31931,-1.61721,2.47424},
                        {-1.27687,-1.61781,2.46081},
                        {-1.238,-1.61054,2.45369},
                        {-1.35773,-1.568,2.48573},
                        {-1.32514,-1.57531,2.48039},
                        {-1.27351,-1.5759,2.46902},
                        {-1.22518,-1.57582,2.45924},
                        {-1.18019,-1.56569,2.45391},
                        {-1.1475,-1.5605,2.451},
                        {-1.36554,-1.52071,2.48193},
                        {-1.32524,-1.52364,2.47878},
                        {-1.27451,-1.52593,2.46893},
                        {-1.22423,-1.52557,2.4612},
                        {-1.17573,-1.52538,2.45523},
                        {-1.13832,-1.52584,2.45195},
                        {-1.401,-1.458,2.479},
                        {-1.37405,-1.47538,2.48015},
                        {-1.32522,-1.47541,2.47468},
                        {-1.2747,-1.47589,2.46525},
                        {-1.22476,-1.47494,2.45622},
                        {-1.18525,-1.47958,2.45379},
                        {-1.40771,-1.41936,2.47393},
                        {-1.37483,-1.42468,2.47155},
                        {-1.32602,-1.4256,2.46723},
                        {-1.27478,-1.42493,2.45907},
                        {-1.23263,-1.43505,2.45274},
                        {-1.18733,-1.44367,2.45033},
                        {-1.41224,-1.37981,2.47033},
                        {-1.37546,-1.374,2.46677},
                        {-1.32476,-1.37422,2.45836},
                        {-1.27493,-1.38072,2.45193},
                        {-1.24429,-1.38214,2.45057},
                        {-1.3692,-1.33933,2.4616},
                        {-1.32763,-1.33363,2.45241},
                        {-1.28814,-1.34229,2.45186},
                        {-1.22097,-1.1739,2.46152},
                        {-1.17653,-1.18047,2.45933},
                        {-1.122,-1.1696,2.4528},
                        {-1.07867,-1.164,2.45611},
                        {-1.02775,-1.16183,2.46475},
                        {-0.974545,-1.15609,2.47218},
                        {-0.927833,-1.15317,2.489},
                        {-1.22753,-1.128,2.45573},
                        {-0.9234,-1.1455,2.4587},
                        {-0.874875,-1.14163,2.4745},
                        {-0.8306,-1.1292,2.4556},
                        {-0.7782,-1.125,2.4654},
                        {-0.73775,-1.11725,2.461},
                        {-1.23816,-1.07705,2.45479},
                        {-1.24589,-1.03056,2.45189},
                        {-1.246,-0.997,2.45},
                        {-1.35733,-0.518667,2.45267},
                        {-0.864125,-0.406625,2.48025},
                        {-0.843,-0.406,2.466},
                        {-1.4165,-0.363,2.462},
                        {-0.867111,-0.393111,2.46167},
                        {-0.843667,-0.397667,2.45333},
                        {-0.911,-0.31625,2.4795},
                        {-0.877881,-0.315381,2.46967},
                        {-0.844857,-0.308286,2.45614},
                        {-0.958167,-0.259167,2.4905},
                        {-0.919026,-0.269744,2.48077},
                        {-0.877817,-0.276042,2.46777},
                        {-0.848667,-0.280667,2.45167},
                        {-0.953,-0.246,2.492},
                        {-0.918227,-0.232432,2.47907},
                        {-0.884718,-0.22241,2.46915},
                        {-1.10925,-0.15625,2.497},
                        {-1.07264,-0.1592,2.4946},
                        {-1.029,-0.1593,2.4962},
                        {-0.957583,-0.182917,2.49383},
                        {-0.923452,-0.179226,2.485},
                        {-0.886081,-0.177946,2.46713},
                        {-1.15913,-0.127533,2.49573},
                        {-1.12584,-0.138316,2.4945},
                        {-1.07388,-0.13466,2.49604},
                        {-1.02724,-0.11403,2.49446},
                        {-0.999,-0.101,2.498},
                        {-0.891,-0.148,2.4725},
                        {-1.02129,-0.0938571,2.49521},
                        {-0.999,-0.096,2.498},
                        {-1.16939,-0.00872222,2.4895},
                        {-1.16492,0.0240462,2.47294},
                        {-1.123,0.0293396,2.48192},
                        {-1.09506,0.0342353,2.481},
                        {-0.965,0.0284706,2.47776},
                        {-0.9355,0.0185556,2.46739},
                        {-1.2105,0.095,2.48333},
                        {-1.17985,0.06665,2.46315},
                        {-1.12093,0.0521429,2.45521},
                        {-1.08929,0.0706,2.46509},
                        {-1.03047,0.0764706,2.49635},
                        {-0.963867,0.0732,2.48673},
                        {-0.93975,0.0855833,2.47442},
                        {-1.22056,0.126558,2.47754},
                        {-1.17493,0.128421,2.46111},
                        {-1.12939,0.123314,2.47571},
                        {-1.09507,0.108333,2.47393},
                        {-0.973353,0.126588,2.49057},
                        {-0.943053,0.123684,2.482},
                        {-1.3618,0.18464,2.49124},
                        {-1.347,0.19,2.4965},
                        {-1.21562,0.180094,2.48119},
                        {-1.17391,0.175047,2.47444},
                        {-0.964,0.157909,2.49036},
                        {-0.9455,0.1545,2.475},
                        {-1.35927,0.225423,2.494},
                        {-1.32662,0.22708,2.48522},
                        {-1.27828,0.222372,2.46649},
                        {-1.21942,0.220036,2.45686},
                        {-1.17568,0.219921,2.46551},
                        {-1.1337,0.234162,2.47914},
                        {-1.06924,0.239541,2.48465},
                        {-1.0451,0.241,2.4949},
                        {-1.35722,0.262222,2.49278},
                        {-1.32875,0.267536,2.46436},
                        {-1.293,0.254,2.45133},
                        {-1.1085,0.256063,2.46738},
                        {-1.07367,0.275952,2.47046},
                        {-1.02559,0.280459,2.49061},
                        {-0.9888,0.2933,2.4921},
                        {-1.06826,0.323,2.47582},
                        {-1.02372,0.318924,2.4827},
                        {-0.988658,0.322079,2.46718},
                        {-1.06725,0.35915,2.47635},
                        {-1.02744,0.364976,2.47649},
                        {-0.993125,0.361125,2.4575},
                        {-1.001,-1.172,2.514},
                        {-0.99,-1.165,2.501},
                        {-0.729,-1.128,2.502},
                        {-0.907,-0.429667,2.54567},
                        {-0.8843,-0.4208,2.5259},
                        {-1.455,-0.365,2.536},
                        {-0.957667,-0.309667,2.544},
                        {-0.938929,-0.318857,2.52393},
                        {-1.07023,-0.265125,2.54697},
                        {-0.972409,-0.275136,2.53086},
                        {-0.937222,-0.283889,2.51683},
                        {-1.1145,-0.206125,2.54363},
                        {-1.0716,-0.226645,2.54308},
                        {-1.03384,-0.21773,2.54138},
                        {-0.967267,-0.2188,2.52653},
                        {-0.933231,-0.216462,2.50623},
                        {-1.16795,-0.176159,2.53927},
                        {-1.12537,-0.178904,2.52692},
                        {-1.075,-0.18498,2.52176},
                        {-1.02191,-0.180776,2.51762},
                        {-0.98069,-0.171,2.51828},
                        {-0.939375,-0.15675,2.51425},
                        {-1.207,-0.126429,2.53743},
                        {-1.17551,-0.121615,2.5118},
                        {-1.12414,-0.114619,2.50526},
                        {-1.07554,-0.114892,2.50197},
                        {-1.02125,-0.132367,2.50723},
                        {-0.981736,-0.127218,2.53077},
                        {-0.9336,-0.1494,2.5354},
                        {-1.321,-0.0955,2.549},
                        {-1.202,-0.099,2.539},
                        {-1.17221,-0.093,2.53071},
                        {-1.12448,-0.0931429,2.52471},
                        {-1.07403,-0.0898966,2.51721},
                        {-1.02148,-0.0818,2.51556},
                        {-0.984,-0.0788136,2.52849},
                        {-1.205,-0.02,2.53733},
                        {-1.16573,-0.0282745,2.52273},
                        {-1.12823,-0.0146731,2.53023},
                        {-1.1,-0.002,2.536},
                        {-1.005,-0.02075,2.53988},
                        {-0.982519,-0.0215385,2.53531},
                        {-0.941,-0.0199091,2.53855},
                        {-1.217,0.0435,2.5195},
                        {-1.176,0.0055,2.5035},
                        {-1.12713,0.0115652,2.50978},
                        {-1.09695,0.01865,2.52095},
                        {-1.01621,0.029303,2.5327},
                        {-0.980963,0.0125926,2.51419},
                        {-0.944,0.00466667,2.50433},
                        {-1.22433,0.0764444,2.518},
                        {-1.10114,0.0634286,2.54},
                        {-1.07741,0.0805397,2.5223},
                        {-1.02102,0.0739231,2.51015},
                        {-0.986436,0.0800257,2.50936},
                        {-1.26408,0.125667,2.527},
                        {-1.241,0.105,2.504},
                        {-1.12119,0.139437,2.51994},
                        {-1.07653,0.1253,2.52488},
                        {-1.02461,0.122148,2.5108},
                        {-0.9839,0.1088,2.5024},
                        {-1.36017,0.168667,2.50783},
                        {-1.33064,0.17912,2.52668},
                        {-1.2685,0.182955,2.53118},
                        {-1.23853,0.167467,2.51553},
                        {-1.15907,0.175805,2.52627},
                        {-1.12291,0.165565,2.54096},
                        {-1.074,0.163696,2.54022},
                        {-1.02322,0.176859,2.52667},
                        {-0.983606,0.177697,2.51527},
                        {-1.32125,0.205917,2.508},
                        {-1.28967,0.203,2.50867},
                        {-1.15133,0.201667,2.509},
                        {-1.12807,0.221131,2.52175},
                        {-1.0761,0.217125,2.52305},
                        {-1.02422,0.223915,2.52077},
                        {-0.990957,0.226043,2.5213},
                        {-1.371,0.254,2.502},
                        {-1.01888,0.258485,2.50724},
                        {-0.991583,0.268833,2.50887},
                        {-1.05167,0.338667,2.50367},
                        {-1.04229,0.341286,2.50429},
                        {-1.039,0.35075,2.50225},
                        {-1.16685,-0.663,2.59207},
                        {-1.13718,-0.658091,2.59355},
                        {-1.21511,-0.617222,2.59083},
                        {-1.17141,-0.630628,2.59318},
                        {-1.13133,-0.626804,2.59165},
                        {-1.205,-0.586,2.5985},
                        {-1.187,-0.5871,2.5973},
                        {-1.19,-0.542,2.599},
                        {-0.926056,-0.509778,2.58761},
                        {-0.888,-0.506333,2.57983},
                        {-1.20988,-0.45525,2.59425},
                        {-0.912189,-0.476892,2.56951},
                        {-0.884482,-0.482537,2.5768},
                        {-0.85,-0.486,2.563},
                        {-1.21743,-0.422524,2.59136},
                        {-0.908429,-0.442429,2.557},
                        {-0.891357,-0.439929,2.57},
                        {-1.484,-0.368,2.599},
                        {-1.257,-0.365833,2.5965},
                        {-1.227,-0.373113,2.58549},
                        {-1.11392,-0.358462,2.59485},
                        {-1.07333,-0.362455,2.58697},
                        {-1.02771,-0.354833,2.59446},
                        {-0.975,-0.356,2.59667},
                        {-1.26459,-0.322963,2.58196},
                        {-1.23151,-0.323909,2.5794},
                        {-1.16423,-0.318727,2.58018},
                        {-1.12206,-0.325524,2.56995},
                        {-1.07322,-0.32546,2.57582},
                        {-1.02596,-0.323772,2.58671},
                        {-0.979465,-0.326977,2.5756},
                        {-0.95,-0.332,2.554},
                        {-1.26319,-0.274571,2.58071},
                        {-1.22724,-0.275346,2.58217},
                        {-1.17322,-0.275391,2.58404},
                        {-1.12312,-0.275742,2.55714},
                        {-1.07929,-0.288588,2.55621},
                        {-1.02708,-0.274753,2.56416},
                        {-0.986538,-0.291308,2.56308},
                        {-1.31375,-0.20575,2.59413},
                        {-1.26803,-0.221345,2.57403},
                        {-1.2319,-0.226598,2.58167},
                        {-1.17398,-0.224017,2.56655},
                        {-1.12738,-0.228621,2.55879},
                        {-1.0925,-0.231,2.55138},
                        {-1.02284,-0.231388,2.57196},
                        {-0.987572,-0.226357,2.57664},
                        {-1.35625,-0.161,2.5765},
                        {-1.3215,-0.172412,2.57184},
                        {-1.27268,-0.173818,2.57374},
                        {-1.23115,-0.176083,2.57803},
                        {-1.19291,-0.179182,2.55382},
                        {-1.36331,-0.124385,2.57923},
                        {-1.32312,-0.123377,2.55923},
                        {-1.27299,-0.123125,2.56456},
                        {-1.23486,-0.125611,2.56761},
                        {-0.9612,-0.123125,2.5736},
                        {-0.942833,-0.1445,2.57367},
                        {-1.35667,-0.0825,2.56783},
                        {-1.32275,-0.0732462,2.55589},
                        {-1.27893,-0.07528,2.57001},
                        {-1.21954,-0.0852308,2.57908},
                        {-1.17407,-0.0612593,2.56874},
                        {-1.1284,-0.0826,2.5748},
                        {-0.968773,-0.075,2.5758},
                        {-0.95,-0.056,2.582},
                        {-1.31983,-0.0269815,2.5617},
                        {-1.28492,-0.0246792,2.56913},
                        {-1.22567,-0.025,2.5785},
                        {-1.154,-0.0453333,2.55567},
                        {-1.12391,-0.0276471,2.573},
                        {-1.025,-0.0137143,2.57},
                        {-0.968932,-0.0330227,2.5598},
                        {-0.944429,-0.0401429,2.55657},
                        {-1.30904,0.0231667,2.5665},
                        {-1.2884,0.0232326,2.5784},
                        {-1.246,0.003,2.593},
                        {-1.10209,0.033,2.56091},
                        {-1.08276,0.0258235,2.58227},
                        {-1.03979,0.0197857,2.56421},
                        {-1.30704,0.0770435,2.58683},
                        {-1.2845,0.08035,2.58435},
                        {-1.2465,0.0795,2.56},
                        {-1.1025,0.0515,2.556},
                        {-1.08943,0.0575,2.55807},
                        {-1.31354,0.128286,2.57274},
                        {-1.28962,0.120769,2.56438},
                        {-1.31215,0.171256,2.56544},
                        {-1.28663,0.172842,2.56105},
                        {-1.15473,0.176,2.55473},
                        {-1.1264,0.182755,2.56308},
                        {-1.08193,0.176707,2.5642},
                        {-1.1218,0.206667,2.5574},
                        {-1.0898,0.2028,2.5566},
                        {-1.16925,-0.7525,2.64525},
                        {-1.17169,-0.730063,2.64156},
                        {-1.13511,-0.715667,2.64644},
                        {-1.17182,-0.687235,2.61565},
                        {-1.12036,-0.676379,2.62726},
                        {-1.08251,-0.666641,2.64133},
                        {-1.44,-0.601,2.642},
                        {-1.21606,-0.607625,2.60206},
                        {-1.18563,-0.618474,2.60158},
                        {-1.11516,-0.621689,2.61842},
                        {-1.08735,-0.636522,2.63913},
                        {-1.03244,-0.631111,2.64711},
                        {-0.993,-0.6035,2.6455},
                        {-1.44024,-0.575235,2.63965},
                        {-1.368,-0.553,2.649},
                        {-1.32174,-0.565032,2.63774},
                        {-1.27239,-0.568488,2.61868},
                        {-1.2291,-0.575133,2.60922},
                        {-1.16956,-0.573658,2.61404},
                        {-1.1356,-0.577967,2.6312},
                        {-1.087,-0.551,2.64275},
                        {-0.974587,-0.5735,2.643},
                        {-0.934455,-0.561636,2.64114},
                        {-1.458,-0.52,2.649},
                        {-1.4369,-0.5388,2.6453},
                        {-1.36207,-0.5356,2.64707},
                        {-1.32454,-0.526357,2.6405},
                        {-1.27513,-0.523533,2.6308},
                        {-1.22807,-0.525845,2.61614},
                        {-1.17953,-0.529649,2.61678},
                        {-1.12592,-0.54036,2.63108},
                        {-1.07443,-0.535071,2.62875},
                        {-1.02379,-0.529571,2.62076},
                        {-0.975627,-0.527902,2.61775},
                        {-0.926579,-0.535421,2.62645},
                        {-0.8885,-0.518,2.61938},
                        {-1.475,-0.473,2.648},
                        {-1.31713,-0.476933,2.64309},
                        {-1.27554,-0.482744,2.64231},
                        {-1.221,-0.479029,2.61483},
                        {-1.19521,-0.476263,2.62353},
                        {-0.885087,-0.468739,2.61543},
                        {-1.31263,-0.429368,2.643},
                        {-1.27944,-0.424792,2.63354},
                        {-1.2145,-0.421861,2.62139},
                        {-1.19862,-0.444375,2.62313},
                        {-1.107,-0.407,2.648},
                        {-1.0775,-0.405714,2.63879},
                        {-1.03289,-0.406444,2.638},
                        {-0.887,-0.4475,2.602},
                        {-1.341,-0.355,2.637},
                        {-1.27248,-0.383714,2.61171},
                        {-1.21264,-0.373533,2.62838},
                        {-1.16967,-0.362944,2.63028},
                        {-1.12512,-0.37476,2.61504},
                        {-1.07344,-0.386259,2.61456},
                        {-1.02395,-0.381877,2.61526},
                        {-0.99025,-0.370417,2.61488},
                        {-1.285,-0.344,2.606},
                        {-1.21487,-0.32425,2.6175},
                        {-1.18717,-0.331111,2.61494},
                        {-1.1,-0.342,2.6},
                        {-1.305,-0.256,2.626},
                        {-1.29009,-0.275091,2.61255},
                        {-1.20836,-0.273636,2.60355},
                        {-1.195,-0.2756,2.604},
                        {-1.30833,-0.225444,2.614},
                        {-1.2924,-0.2332,2.6044},
                        {-1.012,-0.238571,2.60721},
                        {-0.995667,-0.235333,2.604},
                        {-1.307,-0.1975,2.6},
                        {-1.411,-0.122667,2.644},
                        {-1.3895,-0.129667,2.616},
                        {-0.955533,-0.1184,2.61213},
                        {-0.947,-0.1345,2.6055},
                        {-1.26277,-0.0685897,2.61977},
                        {-1.23275,-0.0741875,2.62747},
                        {-1.16521,-0.0668214,2.62661},
                        {-1.14514,-0.0568571,2.62743},
                        {-0.961882,-0.0831765,2.61182},
                        {-1.26993,-0.0249818,2.61705},
                        {-1.2395,-0.0393333,2.61233},
                        {-1.1207,-0.0240333,2.62727},
                        {-1.07863,-0.006375,2.63656},
                        {-1.04167,-0.0186667,2.60533},
                        {-1.27707,0.0249535,2.60965},
                        {-1.08183,0.00591667,2.60658},
                        {-1.31133,0.0833333,2.603},
                        {-1.28513,0.067375,2.60938},
                        {-1.1555,-0.751,2.655},
                        {-1.1425,-0.751,2.67},
                        {-1.20233,-0.723333,2.66633},
                        {-1.17656,-0.725051,2.66236},
                        {-1.12926,-0.725489,2.66745},
                        {-1.096,-0.70225,2.67325},
                        {-1.07257,-0.673095,2.66862},
                        {-1.04425,-0.65525,2.669},
                        {-1.42138,-0.615517,2.66666},
                        {-1.387,-0.609,2.6628},
                        {-1.11105,-0.607381,2.66743},
                        {-1.07883,-0.627463,2.67074},
                        {-1.0261,-0.623781,2.66361},
                        {-0.988,-0.608,2.6604},
                        {-0.886,-0.604,2.697},
                        {-1.42069,-0.574625,2.65525},
                        {-1.378,-0.574804,2.65608},
                        {-1.3475,-0.576,2.6525},
                        {-1.151,-0.57,2.651},
                        {-1.12338,-0.581138,2.66653},
                        {-1.07709,-0.568857,2.67226},
                        {-1.02293,-0.571655,2.67138},
                        {-0.977,-0.582467,2.65607},
                        {-0.915519,-0.574385,2.66963},
                        {-0.891571,-0.585286,2.68757},
                        {-1.46375,-0.5144,2.67595},
                        {-1.41998,-0.526177,2.65922},
                        {-1.37958,-0.520833,2.65414},
                        {-1.34667,-0.504,2.65067},
                        {-1.17332,-0.522162,2.67478},
                        {-1.12509,-0.534,2.67927},
                        {-1.069,-0.533333,2.659},
                        {-1.038,-0.55,2.652},
                        {-0.904875,-0.5435,2.6655},
                        {-0.8962,-0.5378,2.6812},
                        {-1.47691,-0.471543,2.6836},
                        {-1.42195,-0.472132,2.6785},
                        {-1.3758,-0.476382,2.6614},
                        {-1.33928,-0.469778,2.65361},
                        {-1.28088,-0.468875,2.655},
                        {-1.191,-0.476857,2.67481},
                        {-1.50291,-0.416091,2.68809},
                        {-1.47498,-0.426732,2.69327},
                        {-1.42455,-0.424745,2.68382},
                        {-1.37532,-0.426089,2.67154},
                        {-1.33687,-0.430174,2.65848},
                        {-1.20313,-0.4198,2.66033},
                        {-1.19644,-0.426406,2.68113},
                        {-1.1158,-0.4187,2.6748},
                        {-1.07993,-0.427185,2.67685},
                        {-1.03233,-0.414444,2.66889},
                        {-1.51486,-0.3735,2.68214},
                        {-1.47211,-0.391667,2.69778},
                        {-1.42309,-0.391391,2.69148},
                        {-1.37405,-0.38825,2.6853},
                        {-1.348,-0.3955,2.661},
                        {-1.20517,-0.38325,2.65733},
                        {-1.18621,-0.386667,2.67075},
                        {-1.1355,-0.393,2.658},
                        {-1.5215,-0.342,2.6935},
                        {-1.4705,-0.204,2.6935},
                        {-1.4615,-0.193,2.697},
                        {-1.423,-0.153,2.6715},
                        {-1.4286,-0.1424,2.6706},
                        {-1.224,-0.07825,2.657},
                        {-1.167,-0.0686667,2.66633},
                        {-1.1429,-0.0559,2.6792},
                        {-1.11784,-0.0352162,2.67005},
                        {-1.088,-0.0217778,2.6695},
                        {-1.12,-0.75,2.734},
                        {-1.11325,-0.731688,2.72375},
                        {-1.095,-0.705667,2.73267},
                        {-1.477,-0.654,2.745},
                        {-1.06933,-0.683,2.72967},
                        {-1.04175,-0.658,2.72125},
                        {-1.437,-0.64,2.704},
                        {-1.07771,-0.6122,2.72354},
                        {-1.03783,-0.619,2.716},
                        {-0.907,-0.61075,2.71275},
                        {-0.893333,-0.609667,2.70533},
                        {-1.07544,-0.592625,2.71938},
                        {-1.03617,-0.588833,2.71333},
                        {-0.902286,-0.567857,2.72229},
                        {-0.898667,-0.566333,2.70967},
                        {-1.46175,-0.507,2.7065},
                        {-1.444,-0.504,2.703},
                        {-1.16991,-0.518677,2.72512},
                        {-1.13514,-0.534,2.72086},
                        {-0.899,-0.549,2.708},
                        {-1.47054,-0.481769,2.70646},
                        {-1.445,-0.491667,2.70233},
                        {-1.18585,-0.476088,2.72488},
                        {-1.107,-0.456,2.728},
                        {-1.0885,-0.45425,2.73},
                        {-1.5005,-0.413,2.7},
                        {-1.49433,-0.418,2.70167},
                        {-1.18317,-0.425468,2.72332},
                        {-1.12183,-0.442167,2.71633},
                        {-1.07138,-0.441385,2.71708},
                        {-1.045,-0.427,2.701},
                        {-1.51182,-0.370605,2.71568},
                        {-1.47372,-0.371196,2.71513},
                        {-1.42645,-0.372214,2.72038},
                        {-1.38871,-0.377143,2.70557},
                        {-1.51924,-0.327,2.72682},
                        {-1.517,-0.2776,2.73515},
                        {-1.50633,-0.236444,2.73333},
                        {-1.482,-0.212583,2.72717},
                        {-1.46363,-0.181263,2.72784},
                        {-1.4475,-0.16975,2.71325},
                        {-1.151,-0.065,2.705},
                        {-1.13583,-0.0565,2.7225},
                        {-1.11333,-0.045,2.72233},
                        {-1.093,-0.036,2.703},
                        {-1.485,-0.78,2.796},
                        {-1.12029,-0.756714,2.77986},
                        {-1.4907,-0.7169,2.791},
                        {-1.11127,-0.7318,2.77573},
                        {-1.089,-0.7066,2.7862},
                        {-1.5,-0.683,2.785},
                        {-1.4858,-0.6792,2.78473},
                        {-1.06533,-0.683667,2.78344},
                        {-1.03925,-0.6605,2.7745},
                        {-1.072,-0.60675,2.754},
                        {-1.16378,-0.516037,2.77263},
                        {-1.13037,-0.529947,2.77516},
                        {-1.17559,-0.474196,2.77512},
                        {-1.12657,-0.467143,2.779},
                        {-1.09075,-0.46,2.768},
                        {-1.17614,-0.440357,2.76171},
                        {-1.149,-0.444,2.754},
                        {-1.061,-0.4405,2.775},
                        {-1.505,-0.35,2.753},
                        {-1.47357,-0.352857,2.76636},
                        {-1.4428,-0.356,2.7594},
                        {-1.50898,-0.323841,2.77141},
                        {-1.48637,-0.337125,2.79219},
                        {-1.445,-0.345,2.776},
                        {-1.50872,-0.2789,2.771},
                        {-1.49419,-0.2715,2.79138},
                        {-1.50338,-0.241769,2.76446},
                        {-1.48904,-0.225926,2.78141},
                        {-1.47,-0.1995,2.7585},
                        {-1.12955,-0.0579091,2.77273},
                        {-1.115,-0.0483333,2.757},
                        {-1.3855,-1.3125,2.84775},
                        {-1.39183,-1.27533,2.847},
                        {-1.40229,-1.21243,2.83914},
                        {-1.39433,-1.23611,2.83944},
                        {-1.40942,-1.17574,2.84068},
                        {-1.41819,-1.12148,2.83729},
                        {-1.4274,-1.07395,2.83415},
                        {-1.43593,-1.02543,2.83236},
                        {-1.453,-0.9624,2.8242},
                        {-1.44417,-0.978565,2.8347},
                        {-1.45579,-0.919792,2.82558},
                        {-1.44933,-0.9415,2.83117},
                        {-1.46297,-0.875727,2.82618},
                        {-1.47367,-0.824278,2.82653},
                        {-1.48222,-0.773512,2.82244},
                        {-1.10888,-0.754625,2.826},
                        {-1.098,-0.751,2.848},
                        {-1.48643,-0.724,2.82362},
                        {-1.10753,-0.738733,2.81987},
                        {-1.07542,-0.722021,2.83669},
                        {-1.03744,-0.711375,2.84063},
                        {-1.48263,-0.689,2.81788},
                        {-1.06,-0.690769,2.82923},
                        {-1.02573,-0.679489,2.84018},
                        {-0.989429,-0.667357,2.84236},
                        {-1.15619,-0.510438,2.81475},
                        {-1.13094,-0.518452,2.83048},
                        {-1.07745,-0.516318,2.828},
                        {-1.046,-0.51,2.84},
                        {-1.1605,-0.4871,2.8115},
                        {-1.13021,-0.486,2.83068},
                        {-1.08433,-0.459,2.83267},
                        {-1.07,-0.446667,2.83367},
                        {-1.502,-0.301,2.805},
                        {-1.48232,-0.320936,2.81558},
                        {-1.48295,-0.277523,2.8228},
                        {-1.48691,-0.238909,2.81718},
                        {-1.37479,-1.32073,2.87621},
                        {-1.38306,-1.27394,2.87244},
                        {-1.39174,-1.22666,2.8746},
                        {-1.40386,-1.16595,2.86838},
                        {-1.39667,-1.18587,2.8816},
                        {-1.41006,-1.12472,2.87464},
                        {-1.42241,-1.07515,2.87497},
                        {-1.43093,-1.02428,2.87369},
                        {-1.43871,-0.973294,2.87285},
                        {-1.4525,-0.91475,2.8695},
                        {-1.44615,-0.9337,2.8815},
                        {-1.45785,-0.8745,2.87368},
                        {-1.45,-0.8905,2.8975},
                        {-1.46631,-0.824722,2.87531},
                        {-1.47428,-0.775,2.87303},
                        {-1.093,-0.752,2.852},
                        {-1.48217,-0.7256,2.8749},
                        {-1.01833,-0.703667,2.85967},
                        {-1.48433,-0.691778,2.87311},
                        {-1.011,-0.685333,2.85367},
                        {-0.98875,-0.67975,2.85575},
                        {-1.11833,-0.508133,2.85673},
                        {-1.076,-0.50725,2.85625},
                        {-1.047,-0.503,2.854},
                        {-1.11267,-0.490667,2.85627},
                        {-1.07433,-0.479709,2.86062},
                        {-1.0447,-0.4791,2.8562},
                        {-1.0625,-0.45,2.8565},
                        {-1.4878,-0.2666,2.8612},
                        {-1.36757,-1.36129,2.92871},
                        {-1.36671,-1.32186,2.92496},
                        {-1.37447,-1.27417,2.92423},
                        {-1.38349,-1.22597,2.92417},
                        {-0.86275,-1.20325,2.933},
                        {-0.832667,-1.20167,2.92267},
                        {-1.39271,-1.17412,2.92456},
                        {-0.868467,-1.17384,2.93564},
                        {-0.844,-1.18875,2.941},
                        {-1.40373,-1.12377,2.9248},
                        {-1.398,-1.149,2.936},
                        {-0.873141,-1.12317,2.93},
                        {-1.4144,-1.07589,2.92351},
                        {-0.904222,-1.07189,2.932},
                        {-0.878525,-1.07818,2.93098},
                        {-1.42526,-1.02603,2.92484},
                        {-1.43363,-0.976367,2.92537},
                        {-1.44015,-0.924471,2.92165},
                        {-1.4532,-0.8585,2.9169},
                        {-1.44717,-0.880917,2.92558},
                        {-1.45818,-0.826394,2.92315},
                        {-1.4662,-0.77515,2.92348},
                        {-1.47657,-0.724595,2.92408},
                        {-1.48324,-0.687824,2.92606},
                        {-1.35672,-1.36864,2.97508},
                        {-1.35963,-1.32537,2.97237},
                        {-1.35,-1.347,2.997},
                        {-1.36704,-1.27621,2.97532},
                        {-1.37609,-1.22545,2.97518},
                        {-1.38603,-1.175,2.97525},
                        {-0.854,-1.16,2.95},
                        {-0.8454,-1.1682,2.9544},
                        {-1.40171,-1.10643,2.96957},
                        {-1.396,-1.13054,2.9755},
                        {-0.8485,-1.146,2.958},
                        {-1.40803,-1.07484,2.97463},
                        {-0.911167,-1.05725,2.97525},
                        {-0.8861,-1.0578,2.9592},
                        {-1.41844,-1.02452,2.97296},
                        {-0.9534,-1.0122,2.9924},
                        {-0.923455,-1.02864,2.99293},
                        {-0.892882,-1.028,2.98953},
                        {-1.42927,-0.9773,2.97397},
                        {-0.9584,-0.97195,2.9903},
                        {-0.9203,-0.975967,2.99713},
                        {-0.896143,-0.974286,2.99486},
                        {-1.43448,-0.924419,2.9739},
                        {-1.064,-0.9315,2.996},
                        {-0.968024,-0.924952,2.98971},
                        {-0.927,-0.918043,2.99665},
                        {-1.44459,-0.875931,2.97517},
                        {-0.968552,-0.882724,2.99269},
                        {-0.932632,-0.877605,2.99392},
                        {-1.45317,-0.821167,2.97033},
                        {-1.448,-0.843333,2.98233},
                        {-1.46022,-0.773813,2.97566},
                        {-1.46944,-0.725438,2.97491},
                        {-1.47725,-0.689417,2.96967},
                        {-1.362,-1.406,3.009},
                        {-1.3444,-1.4044,3.0354},
                        {-1.352,-1.397,3.008},
                        {-1.34429,-1.37364,3.02557},
                        {-1.35542,-1.3195,3.02271},
                        {-1.34817,-1.34117,3.03583},
                        {-1.36032,-1.27396,3.02389},
                        {-1.36889,-1.226,3.0255},
                        {-1.37897,-1.17497,3.02377},
                        {-1.401,-1.101,3.006},
                        {-1.39148,-1.12768,3.02665},
                        {-1.40442,-1.06888,3.02304},
                        {-1.39729,-1.09143,3.035},
                        {-1.41194,-1.02564,3.02418},
                        {-0.92875,-1.01233,3.00233},
                        {-1.42274,-0.975176,3.02306},
                        {-1.174,-0.955833,3.046},
                        {-1.13525,-0.95525,3.03675},
                        {-0.951,-1,3.001},
                        {-0.934036,-0.974964,3.00218},
                        {-1.42977,-0.92471,3.02616},
                        {-1.20738,-0.921625,3.04325},
                        {-1.17453,-0.924245,3.03167},
                        {-1.12462,-0.922778,3.02264},
                        {-1.07251,-0.917872,3.01241},
                        {-1.0251,-0.918128,3.01244},
                        {-0.9944,-0.916,3.0044},
                        {-0.931609,-0.932522,3.00235},
                        {-1.43844,-0.875719,3.02456},
                        {-1.201,-0.893,3.045},
                        {-1.17064,-0.891,3.03595},
                        {-1.12352,-0.884897,3.02921},
                        {-1.07404,-0.881896,3.024},
                        {-1.02421,-0.874189,3.02508},
                        {-0.982484,-0.865065,3.01713},
                        {-0.93925,-0.85525,3.00725},
                        {-1.45233,-0.805,3.00933},
                        {-1.44578,-0.827778,3.02404},
                        {-1.018,-0.849,3.049},
                        {-0.963643,-0.840143,3.03771},
                        {-0.9439,-0.8374,3.0229},
                        {-1.45484,-0.77332,3.02012},
                        {-1.4498,-0.7924,3.041},
                        {-0.954,-0.799,3.049},
                        {-1.46474,-0.730895,3.02053},
                        {-1.33373,-1.4146,3.08067},
                        {-1.3363,-1.37437,3.07467},
                        {-1.3524,-1.308,3.0608},
                        {-1.34457,-1.32861,3.07517},
                        {-1.35571,-1.27346,3.07488},
                        {-1.349,-1.299,3.086},
                        {-1.36224,-1.22355,3.07503},
                        {-1.37183,-1.1756,3.07247},
                        {-1.38468,-1.12506,3.07587},
                        {-1.40129,-1.05871,3.06714},
                        {-1.39305,-1.08295,3.07509},
                        {-1.40629,-1.02271,3.07686},
                        {-1.399,-1.047,3.086},
                        {-1.41637,-0.9741,3.07443},
                        {-1.255,-0.953,3.098},
                        {-1.22241,-0.968059,3.08129},
                        {-1.18333,-0.971,3.068},
                        {-1.143,-0.971,3.066},
                        {-1.42257,-0.924857,3.07418},
                        {-1.25167,-0.933333,3.09433},
                        {-1.23358,-0.923316,3.07463},
                        {-1.43269,-0.878846,3.076},
                        {-1.22304,-0.885875,3.08467},
                        {-1.17429,-0.877625,3.07675},
                        {-1.12435,-0.868304,3.07296},
                        {-1.07629,-0.859708,3.07088},
                        {-1.03183,-0.854,3.05633},
                        {-1.44103,-0.826035,3.07062},
                        {-1.0555,-0.848,3.091},
                        {-1.01843,-0.840524,3.07533},
                        {-0.982162,-0.823432,3.07019},
                        {-1.45471,-0.766929,3.0685},
                        {-1.44722,-0.791444,3.07178},
                        {-0.961,-0.792667,3.067},
                        {-1.4645,-0.741,3.0605},
                        {-1.32244,-1.42636,3.12848},
                        {-1.33182,-1.37595,3.12609},
                        {-1.351,-1.3,3.109},
                        {-1.34089,-1.32467,3.12281},
                        {-1.35313,-1.27031,3.122},
                        {-1.34656,-1.28956,3.136},
                        {-1.35428,-1.22544,3.12232},
                        {-1.36678,-1.17741,3.12452},
                        {-1.37447,-1.12357,3.1218},
                        {-1.4,-1.052,3.104},
                        {-1.37881,-1.0744,3.13274},
                        {-1.3235,-1.06947,3.14178},
                        {-1.27534,-1.07143,3.14289},
                        {-1.23739,-1.06826,3.13896},
                        {-1.403,-1.00933,3.10267},
                        {-1.38076,-1.02673,3.12773},
                        {-1.32545,-1.02643,3.13358},
                        {-1.27557,-1.02495,3.13008},
                        {-1.2297,-1.02503,3.12879},
                        {-1.40679,-0.974105,3.10842},
                        {-1.37489,-0.974617,3.11734},
                        {-1.32398,-0.976659,3.11636},
                        {-1.2767,-0.974541,3.11019},
                        {-1.229,-0.9928,3.1075},
                        {-1.193,-0.9945,3.1105},
                        {-1.414,-0.926923,3.117},
                        {-1.37354,-0.932892,3.12481},
                        {-1.32156,-0.927077,3.11233},
                        {-1.27861,-0.926139,3.10736},
                        {-1.43047,-0.878467,3.1148},
                        {-1.309,-0.897,3.1075},
                        {-1.27144,-0.892875,3.10475},
                        {-1.2158,-0.8728,3.11533},
                        {-1.17643,-0.863929,3.13046},
                        {-1.12495,-0.857955,3.12891},
                        {-1.087,-0.852571,3.12},
                        {-1.44214,-0.824571,3.10843},
                        {-1.126,-0.846,3.1475},
                        {-1.06753,-0.839632,3.13568},
                        {-1.03004,-0.826435,3.12057},
                        {-0.9945,-0.807,3.1145},
                        {-1.447,-0.794,3.104},
                        {-1.051,-0.75,3.106},
                        {-1.045,-0.751,3.109},
                        {-1.304,-1.456,3.1695},
                        {-1.29475,-1.46725,3.18925},
                        {-1.31475,-1.4255,3.17493},
                        {-1.3286,-1.37736,3.17424},
                        {-1.33528,-1.32352,3.17348},
                        {-1.352,-1.2625,3.1685},
                        {-1.34476,-1.28035,3.17729},
                        {-1.3545,-1.21888,3.17594},
                        {-1.34871,-1.24,3.18229},
                        {-1.363,-1.17342,3.17404},
                        {-1.36248,-1.12835,3.16652},
                        {-1.32836,-1.11273,3.17236},
                        {-1.2902,-1.1096,3.1698},
                        {-1.3595,-1.0995,3.1525},
                        {-1.33163,-1.09488,3.15313},
                        {-1.28433,-1.09533,3.15167},
                        {-1.22192,-1.06838,3.16931},
                        {-1.365,-1.048,3.15},
                        {-1.21354,-1.03362,3.17238},
                        {-1.181,-0.8615,3.1535},
                        {-1.1395,-0.8545,3.15},
                        {-1.1585,-0.8465,3.15525},
                        {-1.12843,-0.842714,3.15743},
                        {-1.0936,-0.8392,3.1552},
                        {-1.30357,-1.46,3.22671},
                        {-1.2915,-1.47688,3.22338},
                        {-0.931,-1.454,3.245},
                        {-1.31426,-1.42568,3.22553},
                        {-1.32355,-1.37425,3.2205},
                        {-1.33259,-1.32486,3.22418},
                        {-1.34,-1.27308,3.22092},
                        {-1.35336,-1.21418,3.22345},
                        {-1.34627,-1.23873,3.22864},
                        {-1.35563,-1.17763,3.22104},
                        {-1.347,-1.155,3.23633},
                        {-1.35433,-1.145,3.21033},
                        {-1.334,-1.13614,3.215},
                        {-1.218,-1.0595,3.222},
                        {-1.212,-1.04433,3.22533},
                        {-1.2,-1.033,3.216},
                        {-1.08,-0.833,3.247},
                        {-1.271,-1.508,3.298},
                        {-1.243,-1.509,3.287},
                        {-1.303,-1.4585,3.27275},
                        {-1.28737,-1.48495,3.27058},
                        {-0.99,-1.4745,3.2825},
                        {-1.31338,-1.42743,3.272},
                        {-1.32267,-1.37419,3.27343},
                        {-1.328,-1.32542,3.27442},
                        {-1.33738,-1.2759,3.27605},
                        {-1.351,-1.214,3.252},
                        {-1.34523,-1.2255,3.27605},
                        {-1.351,-1.2,3.256},
                        {-1.3444,-1.1822,3.272},
                        {-1.21725,-1.06525,3.26425},
                        {-1.156,-0.849,3.297},
                        {-1.11,-0.8385,3.2885},
                        {-1.088,-0.835,3.284},
                        {-1.272,-1.51025,3.32483},
                        {-1.23,-1.5235,3.326},
                        {-1.08833,-1.50767,3.34033},
                        {-1.301,-1.45633,3.31733},
                        {-1.2945,-1.47639,3.3235},
                        {-1.0265,-1.492,3.32125},
                        {-0.96,-1.482,3.325},
                        {-0.941,-1.474,3.309},
                        {-1.3087,-1.42274,3.32119},
                        {-1.2985,-1.446,3.34},
                        {-1.31546,-1.37529,3.32388},
                        {-1.32167,-1.32613,3.32488},
                        {-1.33386,-1.27518,3.32486},
                        {-1.33933,-1.22424,3.32295},
                        {-1.3364,-1.192,3.3158},
                        {-1.227,-1.082,3.349},
                        {-1.105,-0.833,3.304},
                        {-1.0805,-0.8325,3.3075},
                        {-1.26905,-1.51673,3.37427},
                        {-1.22892,-1.53442,3.3805},
                        {-1.16943,-1.53,3.384},
                        {-1.12057,-1.521,3.38157},
                        {-1.06989,-1.51256,3.38178},
                        {-1.02625,-1.50283,3.38408},
                        {-0.996,-1.5015,3.384},
                        {-1.2896,-1.48008,3.37648},
                        {-1.0105,-1.4975,3.352},
                        {-0.97325,-1.49188,3.376},
                        {-0.9392,-1.486,3.3652},
                        {-1.3019,-1.4127,3.3683},
                        {-1.29742,-1.43533,3.37883},
                        {-1.30837,-1.37511,3.37195},
                        {-1.31652,-1.32224,3.37628},
                        {-1.32407,-1.27578,3.373},
                        {-1.33104,-1.22913,3.37442},
                        {-1.254,-1.128,3.395},
                        {-1.238,-1.105,3.399},
                        {-1.231,-1.09167,3.38133},
                        {-1.26055,-1.51559,3.42245},
                        {-1.22656,-1.53116,3.42844},
                        {-1.17377,-1.52573,3.43023},
                        {-1.1248,-1.51413,3.4292},
                        {-1.0769,-1.50938,3.41995},
                        {-1.035,-1.5028,3.4104},
                        {-1.27941,-1.47467,3.42418},
                        {-1.11556,-1.49111,3.44756},
                        {-1.0735,-1.48069,3.44216},
                        {-1.02113,-1.48,3.43128},
                        {-0.976649,-1.47632,3.42673},
                        {-0.945667,-1.4855,3.41667},
                        {-1.3,-1.405,3.414},
                        {-1.2895,-1.42725,3.42821},
                        {-1.30372,-1.37139,3.4145},
                        {-1.29129,-1.37641,3.43924},
                        {-1.30565,-1.32335,3.42235},
                        {-1.29236,-1.32855,3.44427},
                        {-1.151,-1.303,3.448},
                        {-1.12606,-1.31283,3.44617},
                        {-1.0955,-1.3105,3.4465},
                        {-1.31083,-1.27621,3.42583},
                        {-1.27506,-1.26365,3.44641},
                        {-1.22252,-1.27595,3.44762},
                        {-1.17403,-1.27235,3.44339},
                        {-1.12514,-1.27678,3.44243},
                        {-1.098,-1.27325,3.4425},
                        {-1.31337,-1.22734,3.42544},
                        {-1.28229,-1.23017,3.44608},
                        {-1.22444,-1.22267,3.44626},
                        {-1.17568,-1.22647,3.44105},
                        {-1.12663,-1.22471,3.4348},
                        {-1.09578,-1.22944,3.43022},
                        {-1.305,-1.19433,3.42733},
                        {-1.27076,-1.17659,3.435},
                        {-1.22571,-1.1748,3.43889},
                        {-1.17542,-1.17474,3.43163},
                        {-1.13968,-1.17495,3.43611},
                        {-1.25767,-1.14317,3.42133},
                        {-1.22411,-1.12549,3.42754},
                        {-1.17478,-1.12581,3.42694},
                        {-1.14212,-1.12771,3.43124},
                        {-1.219,-1.09363,3.42325},
                        {-1.17762,-1.09492,3.42554},
                        {-1.257,-1.503,3.451},
                        {-1.22404,-1.51313,3.4575},
                        {-1.18214,-1.51029,3.45429},
                        {-1.26239,-1.47578,3.46257},
                        {-1.22389,-1.47651,3.46795},
                        {-1.17337,-1.47487,3.46366},
                        {-1.12597,-1.47223,3.45477},
                        {-1.067,-1.45667,3.45808},
                        {-1.03193,-1.4586,3.46627},
                        {-0.977071,-1.46007,3.46386},
                        {-1.26771,-1.42468,3.46239},
                        {-1.22703,-1.42731,3.46814},
                        {-1.17654,-1.42532,3.46559},
                        {-1.12471,-1.42524,3.459},
                        {-1.08044,-1.42707,3.46488},
                        {-1.26865,-1.3765,3.45881},
                        {-1.22492,-1.37582,3.46289},
                        {-1.17429,-1.37411,3.4608},
                        {-1.12333,-1.37558,3.46737},
                        {-1.09443,-1.39143,3.47886},
                        {-1.27117,-1.32397,3.45678},
                        {-1.2246,-1.32471,3.4564},
                        {-1.17547,-1.32524,3.4545},
                        {-1.12311,-1.339,3.45828},
                        {-1.09767,-1.32733,3.46667},
                        {-1.27519,-1.28563,3.45338},
                        {-1.22838,-1.26931,3.45231},
                        {-1.177,-1.29825,3.45025},
                        {-1.27033,-1.21917,3.45117},
                        {-1.2343,-1.234,3.4506},
                        {-1.1235,-1.2055,3.459},
                        {-1.12933,-1.19267,3.456},
                        {-1.134,-1.146,3.456},
                        {-1.079,-1.415,3.5},
                        {-1.102,-1.377,3.505},
                        {-1.094,-1.391,3.501},
                        {-4.291,-1.786,6.534},
                        {-4.291,-1.703,6.579},
                        {-7.135,-2.666,11.018},
                        {-7.109,-2.407,11.168},
                        {-7.069,-2.318,11.27}};

		// std::vector<std::vector<float>> test_points;
		// std::copy(points_array.begin(), points_array.end(), std::back_inserter(test_points));
		//CPU
		// int cpu_root_id=-1;
		// std::vector <node> cpu_nodes;
		// cpu_nodes.resize(test_points.size());
		// std::vector<std::vector<int>> cpu_axis_sort_ids(3,std::vector<int>(test_points.size()));
		// point_with_id cpu_point_with_ids[test_points.size()];
		// for(int i=0;i<test_points.size();i++){
		// 	cpu_point_with_ids[i].id = i;
		// 	cpu_point_with_ids[i].pos[0] = test_points[i][0];
		// 	cpu_point_with_ids[i].pos[1] = test_points[i][1];
		// 	cpu_point_with_ids[i].pos[2] = test_points[i][2];
		// }
		// for(sort_axis=0; sort_axis<3; sort_axis++){
		// 	qsort(cpu_point_with_ids, test_points.size(), sizeof(point_with_id), AxisSort);
		// 	for (int i=0 ; i < test_points.size() ; i++){
		// 		cpu_axis_sort_ids[sort_axis][i]=cpu_point_with_ids[i].id;
		// 	}
		// }
		// int cpu_create_end = CreateNode2(&cpu_root_id,test_points.size(),cpu_nodes,cpu_axis_sort_ids,0,-1,false);

		//GPU
		build_start = clock();
		std::vector <detailed_node> detailed_nodes;
		detailed_nodes.resize(test_points.size());
		std::vector<int> x_sort_ids(test_points.size());
		std::vector<int> y_sort_ids(test_points.size());
		std::vector<int> z_sort_ids(test_points.size());
		std::vector<int> next_layer_run(test_points.size());
		std::vector<int> end_list(test_points.size());
		int next_start_thread_id = 0;

		// point_with_id point_with_ids[test_points.size()];
		for(int i=0;i<test_points.size();i++){
			point_with_ids[i].id = i;
			point_with_ids[i].pos[0] = test_points[i][0];
			point_with_ids[i].pos[1] = test_points[i][1];
			point_with_ids[i].pos[2] = test_points[i][2];
			detailed_nodes[i].ready = false;
			next_layer_run[i] = 0;
			end_list[i] = 0;
		}
		for(sort_axis=0; sort_axis<3; sort_axis++){
			qsort(point_with_ids, test_points.size(), sizeof(point_with_id), AxisSort);
			for (int i=0 ; i < test_points.size(); i++){
				if(sort_axis==0){
					x_sort_ids[i]=point_with_ids[i].id;
				}
				if(sort_axis==1){
					y_sort_ids[i]=point_with_ids[i].id;
				}
				if(sort_axis==2){
					z_sort_ids[i]=point_with_ids[i].id;
				}
			}
		}
		//最初のmedian特定
		size_t root_middle = ((test_points.size() - 1) / 2);
		int root_median_id = x_sort_ids[root_middle];//最初はx

		next_layer_run[root_median_id] = 1;

		detailed_nodes[root_median_id].ready = true;
		detailed_nodes[root_median_id].node_is_right = false;
		detailed_nodes[root_median_id].parent_id = -1;
		detailed_nodes[root_median_id].depth = 0;
		detailed_nodes[root_median_id].axis = 0;
		detailed_nodes[root_median_id].middle = root_middle;
		detailed_nodes[root_median_id].group_size = test_points.size();

		int *d_end_list,*d_next_layer_run,*d_next_start_thread_id;
		detailed_node *d_detailed_nodes;
		cudaMalloc((void **)&detailed_nodes[root_median_id].x_sort_ids, test_points.size() * sizeof(int));
		cudaMalloc((void **)&detailed_nodes[root_median_id].y_sort_ids, test_points.size() * sizeof(int));
		cudaMalloc((void **)&detailed_nodes[root_median_id].z_sort_ids, test_points.size() * sizeof(int));
		cudaMalloc((void **)&d_end_list, test_points.size() * sizeof(int));
		cudaMalloc((void **)&d_detailed_nodes, test_points.size() * sizeof(detailed_node));
		cudaMalloc((void **)&d_next_layer_run, test_points.size() * sizeof(int));
		cudaMalloc((void **)&d_next_start_thread_id, sizeof(int));

		cudaMemcpy(detailed_nodes[root_median_id].x_sort_ids, &x_sort_ids[0], test_points.size() * sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(detailed_nodes[root_median_id].y_sort_ids, &y_sort_ids[0], test_points.size() * sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(detailed_nodes[root_median_id].z_sort_ids, &z_sort_ids[0], test_points.size() * sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(d_end_list, &end_list[0], test_points.size() * sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(d_detailed_nodes, &detailed_nodes[0], test_points.size() * sizeof(detailed_node), cudaMemcpyHostToDevice);
		cudaMemcpy(d_next_layer_run, &next_layer_run[0], test_points.size() * sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(d_next_start_thread_id, &next_start_thread_id, sizeof(int), cudaMemcpyHostToDevice);
		cudaDeviceSetLimit(cudaLimitMallocHeapSize, 1024*1024*64);//5000pointで16で死ぬので64が最適
		// cudaDeviceSetLimit(cudaLimitStackSize, 1024);//並列ならいらん
		int dimx_create_node = 32;//32
		dim3 block_create_node(dimx_create_node, 1);
		dim3 grid_create_node((test_points.size() + block_create_node.x - 1) / block_create_node.x, 1);
		float estimate_depth = log2((test_points.size()+1.0f)/2.0f);
		int depth_count = 0;

		// std::cout << "frames" << frames <<"------------------------------------------------------------------------------------------------------------"<< std::endl;
		// if(frames==114) std::cout<<"dead point is ("<<test_points[684][0]<<","<<test_points[684][1]<<","<<test_points[684][2]<<")"<<std::endl;
		// if(frames==114) std::cout<<"around point is ("<<test_points[683][0]<<","<<test_points[683][1]<<","<<test_points[683][2]<<")"<<std::endl;
		// if(frames==114) std::cout<<"around point is ("<<test_points[682][0]<<","<<test_points[682][1]<<","<<test_points[682][2]<<")"<<std::endl;

		// if(frames==114){
		// 	std::cout<<"test_points = {";
		// 	for(int i=0;i<test_points.size();i++){
		// 		std::cout<<"{";
		// 		for(int j=0;j<3;j++){
		// 			std::cout<<test_points[i][j];
		// 			if(j!=(3-1)) std::cout<<",";
		// 		}
		// 		std::cout<<"}";
		// 		if(i!=(test_points.size()-1)) std::cout<<",";
		// 		std::cout<<std::endl;
		// 	}
		// 	std::cout<<"};"<<std::endl;
		// }
		std::cout<<"estimate_depth = "<<estimate_depth<<std::endl;
		
		while(1){
			// if(depth_count==test_points.size()) break;
			// std::cout<<"call depth = "<< depth_count <<std::endl;
			// std::cout<<"create kernel start"<<std::endl;
			// d_DepthCreateNode<<<grid_create_node,block_create_node>>>(test_points.size(),d_detailed_nodes,d_end_list);
			d_DepthCreateNodeLWD<<<grid_create_node,block_create_node>>>(test_points.size(),d_next_start_thread_id,d_next_layer_run,d_detailed_nodes,d_end_list);
			cudaDeviceSynchronize();
			// std::cout<<"create kernel end"<<std::endl;
			if(depth_count >= (estimate_depth*2)){//*2で基本的に終わる(カバー領域は指数関数的に増えるため安心してok)
				// std::cout<<"limit termination"<<std::endl;
				break;
			} 

			depth_count++;
			
			// if(all_end) break;
		}
		
		
		//これ以降に遅くしてるやつがいる
		// 犯人はmemcpy
		
		cudaFree(detailed_nodes[root_median_id].x_sort_ids);
		cudaFree(detailed_nodes[root_median_id].y_sort_ids);
		cudaFree(detailed_nodes[root_median_id].z_sort_ids);

		cudaMemcpy(&end_list[0], d_end_list, test_points.size() * sizeof(int), cudaMemcpyDeviceToHost);
		cudaMemcpy(&detailed_nodes[0], d_detailed_nodes, test_points.size() * sizeof(detailed_node), cudaMemcpyDeviceToHost);

		cudaFree(d_end_list);
		cudaFree(d_detailed_nodes);
		cudaFree(d_next_start_thread_id);
		cudaFree(d_next_layer_run);
		
		bool all_end = std::all_of(end_list.begin(), end_list.end(), [](int end) { return 0 < end; });
		if(all_end) std::cout<<"successful termination"<<std::endl;
		// if(!all_end) std::cout<<"not all end"<<std::endl;
		
		TreeOutCsv(detailed_nodes,0,root_median_id);
		build_end = clock();
		printf("create tree time is %.5fs\n",(double)(build_end-build_start)/CLOCKS_PER_SEC);
		first=false;
	}
	// root_id=root_median_id;
	//表示用スクリプト

	/////////////////////////////////////////////////////////////////////////////////////////施工
	

	//root_id表示
	// std::cout << "root_id = " << root_id << std::endl;
	//nodes表示

	// std::cout<<"search kernel start"<<std::endl;
	// std::cout<<"3.01"<<std::endl;
    //ホスト1次配列宣言
	//kd
	std::vector<int> h_parent_ids(points_array.size());
	std::vector<int> h_left_ids(points_array.size());
	std::vector<int> h_right_ids(points_array.size());
	std::vector<int> h_axes(points_array.size());

	std::vector<int> h_point_neighbor(points_array.size());
	std::vector<int> h_point_neighbor_size(1);
	std::vector<long long int> h_neighbor_time(points_array.size());

	//normal
    std::vector<float> h_points(points_array.size() * 3);
    std::vector<int> h_neighbor_points_indices(neighbor_points_count);
    std::vector<float> h_normals(points_array.size() * 3);
    std::vector<float> h_curvatures(points_array.size());
    std::vector<long long int> h_covariance_compute_time(points_array.size());
    std::vector<long long int> h_eigen_compute_time(points_array.size());
    // std::cout<<"3.02"<<std::endl;
    //デバイス1次配列宣言
	//kd
	int *d_parent_ids,*d_left_ids,*d_right_ids,*d_axes;
	int *d_point_neighbor,*d_point_neighbor_size;
	long long int *d_neighbor_time;

	//normal
    float *d_points,*d_normals,*d_curvatures;
    int *d_neighbor_points_indices,*d_neighbor_start_indices;
    long long int *d_covariance_compute_time,*d_eigen_compute_time;


    // std::cout<<"3.03"<<std::endl;
    //メモリ確保
	//kd
	cudaMalloc((void **)&d_parent_ids, points_array.size() * sizeof(int));
	cudaMalloc((void **)&d_left_ids, points_array.size() * sizeof(int));
	cudaMalloc((void **)&d_right_ids, points_array.size() * sizeof(int));
	cudaMalloc((void **)&d_axes, points_array.size() * sizeof(int));

	cudaMalloc((void **)&d_point_neighbor, points_array.size() * sizeof(int));
	cudaMalloc((void **)&d_point_neighbor_size, sizeof(int));
	cudaMalloc((void **)&d_neighbor_time, points_array.size() * sizeof(long long int));
	//normal
    cudaMalloc((void **)&d_points, points_array.size() * 3 * sizeof(float));
    cudaMalloc((void **)&d_neighbor_points_indices, neighbor_points_count * sizeof(int));
    cudaMalloc((void **)&d_neighbor_start_indices, points_array.size() * sizeof(int));
    cudaMalloc((void **)&d_normals, points_array.size() * 3 * sizeof(float));
    cudaMalloc((void **)&d_curvatures, points_array.size() * sizeof(float));
    cudaMalloc((void **)&d_covariance_compute_time, points_array.size() * sizeof(long long int));
    cudaMalloc((void **)&d_eigen_compute_time, points_array.size() * sizeof(long long int));
    // std::cout<<"3.04"<<std::endl;
    //1次配列化
    int k=0,l=0;
    for(int i=0;i<points_array.size();i++){//点群
        for(int j=0;j<3;j++){//x,y,z
            h_points[k]=points_array[i][j];
            k++;
        }
        for(int j=0;j<neighbor_points_indices[i].size();j++){//近傍
            h_neighbor_points_indices[l]=neighbor_points_indices[i][j];
            l++;
        }
		h_parent_ids[i]=nodes[i].parent_id;
		h_left_ids[i]=nodes[i].left_id;
		h_right_ids[i]=nodes[i].right_id;
		h_axes[i]=nodes[i].axis;
    }



    // std::cout<<"3.05"<<std::endl;
    //コピー
	//kd
	cudaMemcpy(d_parent_ids, &h_parent_ids[0], points_array.size() * sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_left_ids, &h_left_ids[0], points_array.size() * sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_right_ids, &h_right_ids[0], points_array.size() * sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_axes, &h_axes[0], points_array.size() * sizeof(int), cudaMemcpyHostToDevice);
	//normal
    cudaMemcpy(d_points, &h_points[0], points_array.size() * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_neighbor_points_indices, &h_neighbor_points_indices[0], neighbor_points_count * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_neighbor_start_indices, &neighbor_start_indices[0], points_array.size() * sizeof(int), cudaMemcpyHostToDevice);
    // std::cout<<"3.06"<<std::endl;
    //並列スレッド設定
    int dimx = 32;
    dim3 block(dimx, 1);
    dim3 grid((points_array.size() + block.x - 1) / block.x, 1);
    // std::cout<<"3.07"<<std::endl;
    // std::cout<<"normalsGPUstart"<<std::endl;
    // cudaDeviceSetLimit(cudaLimitStackSize, 1024*8);
    //実行
    NormalsGPU<<<grid,block>>>(/*d_detailed_nodes,*/d_neighbor_time,d_point_neighbor_size,d_point_neighbor,d_parent_ids,d_left_ids,d_right_ids,d_axes,root_id,d_points,points_array.size(),d_neighbor_points_indices,d_neighbor_start_indices,neighbor_points_count,d_normals,d_curvatures,d_covariance_compute_time,d_eigen_compute_time);
    // std::cout<<"normalsGPUend"<<std::endl;
    // std::cout<<"3.08"<<std::endl;
    //コピー
	//kd
	cudaMemcpy(&h_point_neighbor[0], d_point_neighbor, points_array.size() * sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(&h_point_neighbor_size[0], d_point_neighbor_size, sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(&h_neighbor_time[0], d_neighbor_time, points_array.size() * sizeof(long long int), cudaMemcpyDeviceToHost);
	//normal
    cudaMemcpy(&h_normals[0], d_normals, points_array.size() * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_curvatures[0], d_curvatures, points_array.size() * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_covariance_compute_time[0], d_covariance_compute_time, points_array.size() * sizeof(long long int), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_eigen_compute_time[0], d_eigen_compute_time, points_array.size() * sizeof(long long int), cudaMemcpyDeviceToHost);
    // std::cout<<"3.09"<<std::endl;
    //2次配列化
	for(int i=0;i<h_point_neighbor_size[0];i++){
		point_neighbor[i]=h_point_neighbor[i];
	}
	
	point_neighbor.resize(h_point_neighbor_size[0]);
	// std::cout<<"host cu size "<<h_point_neighbor_size[0]<<std::endl;

    k=0;
    for(int i=0;i<points_array.size();i++){//点群
        for(int j=0;j<3;j++){//x,y,z
            normals_array[i][j]=h_normals[k];
            k++;
        }
        curvatures_array[i]=h_curvatures[i];
        covariance_compute_time[i]=h_covariance_compute_time[i];
        eigen_compute_time[i]=h_eigen_compute_time[i];
		neighbor_time[i]=h_neighbor_time[i];
    }
	// std::cout<<"search kernel end"<<std::endl;
    // std::cout<<"cu_normals : "<<normals_array[0][0]<<","<<normals_array[0][1]<<","<<normals_array[0][2]<<std::endl;
    // std::cout<<"3.10"<<std::endl;
    //メモリ解放
	//kd2

	//kd
	cudaFree(d_parent_ids);
	cudaFree(d_left_ids);
	cudaFree(d_right_ids);
	cudaFree(d_axes);

	cudaFree(d_point_neighbor);
	cudaFree(d_point_neighbor_size);
	cudaFree(d_neighbor_time);

	//normal
    cudaFree(d_points);
    cudaFree(d_neighbor_points_indices);
    cudaFree(d_neighbor_start_indices);
    cudaFree(d_normals);
    cudaFree(d_curvatures);
    cudaFree(d_covariance_compute_time);
    cudaFree(d_eigen_compute_time);


	frames++;
}