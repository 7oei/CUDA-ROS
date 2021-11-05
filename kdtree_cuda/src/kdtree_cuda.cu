

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

__global__ void d_CreateNode(int point_size,int group_size,int depth,int parent_id,bool node_is_right,int *x_sort_ids,int *y_sort_ids,int *z_sort_ids,int *root_id,node* nodes)
{

	// printf("create node open\n");
	unsigned int ix = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int idx = ix;
	printf("idx = %d, ",idx);
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

__global__ void NormalsGPU(long long int* neighbor_time,int *point_neighbor_size,int* point_neighbor,int* d_parent_ids,int* d_left_ids,int* d_right_ids,int* d_axes,int root_id,float* points,int point_size,int* neighbor_points_indices,int* neighbor_start_indices,int neighbor_points_count,float* normals,float* curvatures,long long int* covariance_time,long long int* eigen_time)
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


    if(idx<point_size-1){//対象スレッド内のみ計算
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

extern void ComputeNormals(std::vector<long long int>& neighbor_time,std::vector<int>& point_neighbor,std::vector<std::vector<float>> points_array,std::vector<std::vector<int>> neighbor_points_indices,std::vector<int> neighbor_start_indices,int neighbor_points_count,std::vector<std::vector<float>>& normals_array,std::vector<float>& curvatures_array,std::vector<long long int>& covariance_compute_time,std::vector<long long int>& eigen_compute_time)
{
	//minimum recursive function in cuda
	// int nbytes = 1024;
	// int *data_dev = 0;
	// cudaMalloc((void**)&data_dev, nbytes);
	// cudaMemset(data_dev, 255, nbytes);
	// ParentKernel<<<1, 2>>>(data_dev);

	//Kernel Function Argument Type Check
	// std::vector<node> h_nodes_test(3);
	// node *d_nodes_test;
	// cudaMalloc((void **)&d_nodes_test, 3 * sizeof(node));
	// KernelFunctionArgumentTypeCheck<<<1, 1>>>(d_nodes_test);

	// cudaMemcpy(&h_nodes_test[0], d_nodes_test, 3 * sizeof(node), cudaMemcpyDeviceToHost);
	// cudaFree(d_nodes_test);
	// for(int i=0;i<3;i++){
	// 	std::cout<<"nodes = "<<h_nodes_test[i].parent_id<<","<<h_nodes_test[i].left_id<<","<<h_nodes_test[i].right_id<<","<<h_nodes_test[i].axis<<std::endl;
	// }

	//2d確保
	// int width = 64, height = 64;
	// float* devPtr;
	// size_t pitch;
	// cudaMallocPitch(&devPtr, &pitch, width * sizeof(float), height);
	// MyKernel<<<100, 512>>>(devPtr, pitch, width, height);

	// points_array.clear();
	// points_array.resize(9);
	// points_array = {{6, 0, 0}, 
	// 				{5, 3, 0},
	// 				{3, 4, 0},
	// 				{4, 6, 0},
	// 				{2, 5, 0},
	// 				{1, 2, 0},
	// 				{0, 1, 0},
	// 				{0, 0, 0},
	// 				{0, 0, 0}};
	// points_array.resize(7);

	// points_array.resize(77);
	std::vector <node> nodes;
	nodes.resize(points_array.size());
	int root_id=-1;
	clock_t build_start,build_end;
	build_start = clock();
	// if(frames==43) std::cout<<"dead point is ("<<points_array[77][0]<<","<<points_array[77][0]<<","<<points_array[77][0]<<")"<<std::endl;


	/////////////////////////////////////////////////////////////////////////////////////////
	// std::vector<std::vector<int>> axis_sort_ids(3,std::vector<int>(points_array.size()));
	// point_with_id point_with_ids[points_array.size()];
	// for(int i=0;i<points_array.size();i++){
	// 	point_with_ids[i].id = i;
	// 	point_with_ids[i].pos[0] = points_array[i][0];
	// 	point_with_ids[i].pos[1] = points_array[i][1];
	// 	point_with_ids[i].pos[2] = points_array[i][2];
	// }
	// for(sort_axis=0; sort_axis<3; sort_axis++){
	// 	qsort(point_with_ids, points_array.size(), sizeof(point_with_id), AxisSort);
	// 	for (int i=0 ; i < points_array.size() ; i++){
	// 		axis_sort_ids[sort_axis][i]=point_with_ids[i].id;
	// 	}
	// }
	// int create_end = CreateNode(&root_id,points_array.size(),nodes,axis_sort_ids,0,-1,false);
	/////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////////////
	std::vector<int> x_sort_ids(points_array.size());
	std::vector<int> y_sort_ids(points_array.size());
	std::vector<int> z_sort_ids(points_array.size());
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
	int *d_x_sort_ids,*d_y_sort_ids,*d_z_sort_ids,*d_root_id;
	node *d_nodes;
	cudaMalloc((void **)&d_x_sort_ids, points_array.size() * sizeof(int));
	cudaMalloc((void **)&d_y_sort_ids, points_array.size() * sizeof(int));
	cudaMalloc((void **)&d_z_sort_ids, points_array.size() * sizeof(int));
	cudaMalloc((void **)&d_root_id, sizeof(int));
	cudaMalloc((void **)&d_nodes, points_array.size() * sizeof(node));
	cudaMemcpy(d_x_sort_ids, &x_sort_ids[0], points_array.size() * sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_y_sort_ids, &y_sort_ids[0], points_array.size() * sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_z_sort_ids, &z_sort_ids[0], points_array.size() * sizeof(int), cudaMemcpyHostToDevice);
	cudaDeviceSetLimit(cudaLimitStackSize, 1024*1024);
	// std::cout << "frames" << frames <<"------------------------------------------------------------------------------------------------------------"<< std::endl;
	d_CreateNode<<<1, 1>>>(points_array.size(),points_array.size(),0,-1,false,d_x_sort_ids,d_y_sort_ids,d_z_sort_ids,d_root_id,d_nodes);
	cudaMemcpy(&root_id, d_root_id, sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(&nodes[0], d_nodes, points_array.size() * sizeof(node), cudaMemcpyDeviceToHost);
	cudaFree(d_x_sort_ids);
	cudaFree(d_y_sort_ids);
	cudaFree(d_z_sort_ids);
	cudaFree(d_root_id);
	cudaFree(d_nodes);
	/////////////////////////////////////////////////////////////////////////////////////////
	
	build_end = clock();
	printf("create tree time is %.5fs\n",(double)(build_end-build_start)/CLOCKS_PER_SEC);
	//root_id表示
	std::cout << "root_id = " << root_id << std::endl;
	//nodes表示


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
    NormalsGPU<<<grid,block>>>(d_neighbor_time,d_point_neighbor_size,d_point_neighbor,d_parent_ids,d_left_ids,d_right_ids,d_axes,root_id,d_points,points_array.size(),d_neighbor_points_indices,d_neighbor_start_indices,neighbor_points_count,d_normals,d_curvatures,d_covariance_compute_time,d_eigen_compute_time);
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
    // std::cout<<"cu_normals : "<<normals_array[0][0]<<","<<normals_array[0][1]<<","<<normals_array[0][2]<<std::endl;
    // std::cout<<"3.10"<<std::endl;
    //メモリ解放
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