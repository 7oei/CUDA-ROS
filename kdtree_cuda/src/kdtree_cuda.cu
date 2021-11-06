

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

__global__ void d_DepthCreateNode(int point_size,detailed_node* nodes,int* end_list)
{
    unsigned int ix = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int idx = ix;
	// printf("idx = %d\n",idx);
	if(idx < point_size){//計算領域
		// printf("idx = %d\n",idx);
		if(nodes[idx].ready&&(0 > end_list[idx])){//該当ノード
			// printf("\n\n\n");
			// printf("\nmedian_id = %d\n",idx);
			// if(!nodes[idx].node_is_right) printf("node is left\n");
			// else printf("node is right\n");
			// printf("parent_id = %d\n",nodes[idx].parent_id);
			// printf("middle = %d\n",nodes[idx].middle);
			// printf("axis = %d\n",nodes[idx].axis);
			// if(nodes[idx].node_is_right) printf("device depth = %d\n",nodes[idx].depth);//間引きのため右のみ
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
				cudaDeviceSynchronize();
				// printf("depth = %d\n",nodes[idx].depth);
				// printf("median_id = %d \n",idx);
				cudaDeviceSynchronize();
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
				cudaDeviceSynchronize();
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
				cudaDeviceSynchronize();
				// printf("3 ");
			}
			if(nodes[idx].parent_id >= 0){//親あり
				free(nodes[idx].x_sort_ids);
				free(nodes[idx].y_sort_ids);
				free(nodes[idx].z_sort_ids);
			}
			cudaDeviceSynchronize();
			// printf("4 ");
			end_list[idx] = 1;
			cudaDeviceSynchronize();
			// printf("5 ");
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

	//Parallel Recursion Test
	// int data_size=10;
	// std::vector<int_with_ready> h_test_data(data_size);
	// for(int i=0;i<data_size;i++){
	// 	h_test_data[i].i = 0;
	// 	h_test_data[i].ready = false;
	// }
	// h_test_data[0].i = 0;
	// h_test_data[0].ready = true;
	// int_with_ready *d_test_data;
	// cudaMalloc((void **)&d_test_data, data_size * sizeof(int_with_ready));
	// cudaMemcpy(d_test_data, &h_test_data[0], data_size * sizeof(int_with_ready), cudaMemcpyHostToDevice);
	// int dimx_test = 32;
    // dim3 block_test(dimx_test, 1);
    // dim3 grid_test((data_size + block_test.x - 1) / block_test.x, 1);
	// //ここ繰り返す
	// for(int i=0;i<data_size;i++){
	// 	d_ParallelRecursionTest<<<grid_test, block_test>>>(data_size,d_test_data);
	// }
	// cudaMemcpy(&h_test_data[0], d_test_data, data_size * sizeof(int_with_ready), cudaMemcpyDeviceToHost);
	// cudaFree(d_test_data);
	// for(int i=0;i<data_size;i++){
	// 	std::cout<<"data["<<i<<"] = "<<h_test_data[i].i<<std::endl;
	// }

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

	int root_id=-1;
	/////////////////////////////////////////////////////////////////////////////////////////
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

	/////////////////////////////////////////////////////////////////////////////////////////
	// std::vector <node> nodes;
	// nodes.resize(points_array.size());
	// std::vector<int> x_sort_ids(points_array.size());
	// std::vector<int> y_sort_ids(points_array.size());
	// std::vector<int> z_sort_ids(points_array.size());
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
	// 		if(sort_axis==0){
	// 			x_sort_ids[i]=point_with_ids[i].id;
	// 		}
	// 		if(sort_axis==1){
	// 			y_sort_ids[i]=point_with_ids[i].id;
	// 		}
	// 		if(sort_axis==2){
	// 			z_sort_ids[i]=point_with_ids[i].id;
	// 		}
	// 	}
	// }
	// int *d_x_sort_ids,*d_y_sort_ids,*d_z_sort_ids,*d_root_id;
	// node *d_nodes;
	// cudaMalloc((void **)&d_x_sort_ids, points_array.size() * sizeof(int));
	// cudaMalloc((void **)&d_y_sort_ids, points_array.size() * sizeof(int));
	// cudaMalloc((void **)&d_z_sort_ids, points_array.size() * sizeof(int));
	// cudaMalloc((void **)&d_root_id, sizeof(int));
	// cudaMalloc((void **)&d_nodes, points_array.size() * sizeof(node));
	// cudaMemcpy(d_x_sort_ids, &x_sort_ids[0], points_array.size() * sizeof(int), cudaMemcpyHostToDevice);
	// cudaMemcpy(d_y_sort_ids, &y_sort_ids[0], points_array.size() * sizeof(int), cudaMemcpyHostToDevice);
	// cudaMemcpy(d_z_sort_ids, &z_sort_ids[0], points_array.size() * sizeof(int), cudaMemcpyHostToDevice);
	// cudaDeviceSetLimit(cudaLimitStackSize, 1024*1024);
	// // std::cout << "frames" << frames <<"------------------------------------------------------------------------------------------------------------"<< std::endl;
	// d_CreateNode<<<1, 1>>>(points_array.size(),points_array.size(),0,-1,false,d_x_sort_ids,d_y_sort_ids,d_z_sort_ids,d_root_id,d_nodes);
	// cudaMemcpy(&root_id, d_root_id, sizeof(int), cudaMemcpyDeviceToHost);
	// cudaMemcpy(&nodes[0], d_nodes, points_array.size() * sizeof(node), cudaMemcpyDeviceToHost);
	// cudaFree(d_x_sort_ids);
	// cudaFree(d_y_sort_ids);
	// cudaFree(d_z_sort_ids);
	// cudaFree(d_root_id);
	// cudaFree(d_nodes);
	/////////////////////////////////////////////////////////////////////////////////////////
	build_start = clock();
	/////////////////////////////////////////////////////////////////////////////////////////施工
	if(first){
		int test_size = 684;
		std::vector<std::vector<float>> test_points(test_size);
		test_points   = {{-0.366395,0.518837,0.94393},
						{-0.3442,0.522,0.948},
						{-0.407158,0.513263,0.990368},
						{-0.373634,0.517484,0.975457},
						{-0.326235,0.523437,0.976535},
						{-0.278208,0.529258,0.987033},
						{-0.237484,0.533419,0.996065},
						{-0.453,0.509,1.047},
						{-0.419682,0.510802,1.02677},
						{-0.37531,0.516848,1.02439},
						{-0.325245,0.521911,1.02402},
						{-0.275882,0.527577,1.02387},
						{-0.225303,0.53295,1.02343},
						{-0.176959,0.538552,1.02896},
						{-0.126577,0.544138,1.03702},
						{-0.0879474,0.547658,1.04142},
						{-0.075,0.551167,1.04417},
						{-0.0402,0.5534,1.0482},
						{-0.463693,0.507173,1.08027},
						{-0.425837,0.511141,1.07546},
						{-0.374025,0.515198,1.0739},
						{-0.324798,0.519641,1.0751},
						{-0.275609,0.52514,1.07413},
						{-0.225977,0.5309,1.07382},
						{-0.175565,0.537407,1.07381},
						{-0.125527,0.542495,1.07441},
						{-0.085626,0.547309,1.07574},
						{-0.0625909,0.551927,1.07175},
						{-0.0259132,0.554694,1.07533},
						{0.0238439,0.561208,1.08147},
						{0.0735357,0.565329,1.08422},
						{0.122252,0.570738,1.08775},
						{0.171655,0.57606,1.091},
						{0.224524,0.581587,1.09262},
						{0.271811,0.587622,1.0953},
						{0.335615,0.592154,1.09431},
						{0.377,0.597143,1.09557},
						{0.373,0.6,1.095},
						{0.810972,-0.313278,1.12881},
						{0.798727,-0.255727,1.119},
						{0.805384,-0.277361,1.1277},
						{0.796467,-0.223848,1.12138},
						{0.8021,-0.2327,1.1421},
						{0.792618,-0.17474,1.12536},
						{0.788477,-0.124848,1.1264},
						{0.784994,-0.0752322,1.12714},
						{0.78113,-0.0246827,1.12686},
						{0.776771,0.0251525,1.12547},
						{0.772898,0.0706549,1.12891},
						{0.764793,0.109414,1.14103},
						{-0.505143,0.501929,1.12907},
						{-0.474858,0.50516,1.12398},
						{-0.425741,0.509171,1.12531},
						{-0.375282,0.512077,1.12378},
						{-0.32462,0.51597,1.12428},
						{-0.275349,0.521452,1.12449},
						{-0.225269,0.528537,1.12363},
						{-0.175598,0.535106,1.12393},
						{-0.125695,0.540138,1.12423},
						{-0.0776875,0.545903,1.12401},
						{-0.04524,0.54884,1.13612},
						{-0.0612273,0.550046,1.11395},
						{-0.02315,0.552017,1.12214},
						{0.024327,0.55708,1.12316},
						{0.0744901,0.562247,1.12303},
						{0.125197,0.568099,1.12301},
						{0.174717,0.57432,1.12358},
						{0.223955,0.579372,1.1221},
						{0.273579,0.585704,1.12294},
						{0.32319,0.590918,1.12511},
						{0.37218,0.595685,1.12872},
						{0.4095,0.5974,1.1328},
						{0.371583,0.601,1.10942},
						{0.403333,0.601333,1.11233},
						{0.826267,-0.3588,1.18573},
						{0.817554,-0.321124,1.17598},
						{0.812171,-0.276151,1.17377},
						{0.805374,-0.225041,1.17285},
						{0.798066,-0.164738,1.16721},
						{0.801645,-0.183785,1.17857},
						{0.794746,-0.124403,1.1739},
						{0.800167,-0.144667,1.195},
						{0.78948,-0.0748767,1.17525},
						{0.783503,-0.0248965,1.1758},
						{0.77932,0.024694,1.17807},
						{0.776364,0.0740698,1.1785},
						{0.773065,0.121083,1.18056},
						{0.772833,0.157167,1.19533},
						{-0.5561,0.4893,1.1905},
						{-0.518054,0.493649,1.19222},
						{-0.490667,0.496944,1.19311},
						{-0.504308,0.501385,1.17554},
						{-0.474961,0.502315,1.17324},
						{-0.425328,0.504474,1.17573},
						{-0.374875,0.507424,1.17408},
						{-0.325989,0.511528,1.17452},
						{-0.275243,0.517389,1.17372},
						{-0.226186,0.525893,1.17353},
						{-0.175715,0.533023,1.17438},
						{-0.125583,0.538161,1.17435},
						{-0.0761044,0.544071,1.17433},
						{-0.0343077,0.548808,1.17758},
						{-0.0528571,0.550143,1.15614},
						{-0.0172234,0.550915,1.17165},
						{0.0240292,0.555445,1.17475},
						{0.0739699,0.561253,1.17449},
						{0.124118,0.566409,1.17414},
						{0.174776,0.572511,1.17313},
						{0.224104,0.578422,1.17095},
						{0.271018,0.58445,1.16871},
						{0.323835,0.589722,1.1705},
						{0.376226,0.594538,1.16799},
						{0.40787,0.596652,1.16965},
						{0.4205,0.6,1.1945},
						{0.83,-0.4065,1.24425},
						{0.826382,-0.367553,1.22418},
						{0.813475,-0.323839,1.21742},
						{0.814195,-0.275325,1.22425},
						{0.808896,-0.225645,1.22365},
						{0.804096,-0.174585,1.22486},
						{0.79766,-0.117817,1.22501},
						{0.801,-0.141453,1.22385},
						{0.794089,-0.0753526,1.22598},
						{0.788794,-0.0249533,1.22524},
						{0.784247,0.0250411,1.22649},
						{0.781012,0.0746795,1.22605},
						{0.776785,0.125288,1.22471},
						{0.776345,0.165,1.22697},
						{-0.560455,0.486091,1.20964},
						{-0.515483,0.491067,1.21488},
						{-0.477713,0.496537,1.22591},
						{-0.425786,0.497224,1.23159},
						{-0.391909,0.498409,1.239},
						{-0.4642,0.5,1.20507},
						{-0.422043,0.500575,1.20881},
						{-0.373162,0.502331,1.22094},
						{-0.325213,0.508113,1.22447},
						{-0.275106,0.512754,1.22345},
						{-0.226362,0.521324,1.22395},
						{-0.175503,0.529983,1.2245},
						{-0.124551,0.535005,1.22347},
						{-0.0750595,0.541268,1.22405},
						{-0.0300147,0.546779,1.22517},
						{-0.0109706,0.550294,1.21759},
						{0.0249639,0.553747,1.22478},
						{0.074064,0.558971,1.22454},
						{0.122851,0.564915,1.22287},
						{0.1753,0.570693,1.22293},
						{0.2186,0.576189,1.22424},
						{0.260114,0.581114,1.21446},
						{0.326227,0.588409,1.23209},
						{0.357091,0.592545,1.22809},
						{0.434105,0.597947,1.23211},
						{0.437,0.6,1.245},
						{0.839824,-0.465471,1.29259},
						{0.834353,-0.423235,1.27829},
						{0.82743,-0.381431,1.27497},
						{0.799,-0.336,1.272},
						{0.809821,-0.329615,1.26713},
						{0.813595,-0.273601,1.27463},
						{0.808752,-0.225829,1.27247},
						{0.806273,-0.175813,1.27247},
						{0.798217,-0.114022,1.2657},
						{0.80145,-0.13185,1.2765},
						{0.794799,-0.0754154,1.27154},
						{0.8,-0.088,1.299},
						{0.789886,-0.0252819,1.27299},
						{0.785136,0.0249214,1.2724},
						{0.781251,0.0750477,1.27358},
						{0.777928,0.124686,1.27433},
						{0.776337,0.167805,1.27304},
						{0.775429,0.223714,1.28836},
						{-0.472218,0.492964,1.27237},
						{-0.429344,0.492885,1.26484},
						{-0.375438,0.497212,1.2718},
						{-0.336269,0.4985,1.29004},
						{-0.355474,0.500684,1.26284},
						{-0.32208,0.503174,1.2692},
						{-0.274956,0.506813,1.27347},
						{-0.225226,0.515748,1.274},
						{-0.174609,0.525257,1.27353},
						{-0.125836,0.527765,1.27185},
						{-0.0759701,0.536755,1.27302},
						{-0.0251543,0.542789,1.27418},
						{0.0166818,0.547341,1.282},
						{0.0319878,0.550805,1.26528},
						{0.0756242,0.554287,1.27292},
						{0.120095,0.560631,1.26932},
						{0.168435,0.566696,1.26139},
						{0.222692,0.572058,1.27179},
						{0.256333,0.576667,1.257},
						{0.33475,0.58975,1.254},
						{0.839917,-0.515792,1.30973},
						{0.835397,-0.474742,1.31253},
						{0.835246,-0.436018,1.31504},
						{0.8245,-0.379611,1.30844},
						{0.817346,-0.313269,1.31135},
						{0.812466,-0.274683,1.32094},
						{0.811039,-0.225331,1.32287},
						{0.799,-0.154,1.347},
						{0.808006,-0.174513,1.32645},
						{0.797691,-0.123357,1.34331},
						{0.804326,-0.126621,1.32334},
						{0.795822,-0.0705445,1.33439},
						{0.801261,-0.0827102,1.32096},
						{0.793732,-0.024949,1.33124},
						{0.8,-0.048,1.3275},
						{0.789351,0.0248831,1.32989},
						{0.78462,0.0739467,1.32892},
						{0.780882,0.12425,1.3268},
						{0.776992,0.173644,1.32528},
						{0.773891,0.222446,1.32598},
						{0.770711,0.263,1.33497},
						{0.769,0.31,1.349},
						{-0.478018,0.487873,1.31951},
						{-0.40425,0.488,1.3455},
						{-0.371723,0.493015,1.32632},
						{-0.33586,0.498023,1.32391},
						{-0.314547,0.500812,1.32517},
						{-0.275354,0.503894,1.32296},
						{-0.224478,0.51258,1.32314},
						{-0.175854,0.519364,1.32158},
						{-0.125825,0.523349,1.3224},
						{-0.0777983,0.531378,1.31876},
						{-0.0285513,0.537692,1.31387},
						{0.0191184,0.543289,1.31403},
						{0.053,0.548,1.311},
						{0.1035,0.549,1.348},
						{0.047,0.55,1.3},
						{0.076,0.5514,1.305},
						{0.13425,0.55725,1.303},
						{0.234,0.5682,1.3032},
						{0.834,-0.427,1.3575},
						{0.813456,-0.215978,1.36548},
						{0.7982,-0.154667,1.3694},
						{0.804463,-0.180098,1.36173},
						{0.79594,-0.124168,1.37229},
						{0.792224,-0.075392,1.3736},
						{0.787264,-0.0248722,1.37359},
						{0.782591,0.024781,1.37237},
						{0.778162,0.0749118,1.37099},
						{0.775013,0.12472,1.37099},
						{0.772173,0.173899,1.37039},
						{0.769523,0.224387,1.36829},
						{0.766827,0.273776,1.37092},
						{0.765644,0.321856,1.37137},
						{0.770222,0.353333,1.379},
						{-0.552,0.471,1.387},
						{-0.4888,0.483,1.3572},
						{-0.4153,0.4868,1.366},
						{-0.365333,0.492786,1.3674},
						{-0.331487,0.497462,1.37112},
						{-0.30975,0.50035,1.36785},
						{-0.281702,0.503433,1.36928},
						{-0.226303,0.510037,1.37207},
						{-0.174892,0.51618,1.37182},
						{-0.126859,0.522484,1.36608},
						{-0.0920526,0.526105,1.36416},
						{0.845,-0.542,1.449},
						{0.834429,-0.47,1.44314},
						{0.834,-0.434667,1.44533},
						{0.813125,-0.211125,1.43313},
						{0.806069,-0.190897,1.42062},
						{0.7955,-0.12,1.41648},
						{0.792263,-0.0749649,1.42285},
						{0.787136,-0.0236562,1.42174},
						{0.782041,0.0253559,1.42387},
						{0.777634,0.0749032,1.42626},
						{0.772365,0.1206,1.42004},
						{0.7695,0.163022,1.42657},
						{0.764737,0.215263,1.42363},
						{0.763,0.253,1.44},
						{-0.514445,0.475889,1.439},
						{-0.404167,0.488167,1.42817},
						{-0.385379,0.489448,1.421},
						{-0.319,0.4975,1.42667},
						{-0.305,0.500111,1.42411},
						{-0.293308,0.501231,1.42338},
						{-0.232632,0.506026,1.42695},
						{-0.17182,0.51528,1.42236},
						{-0.126324,0.522059,1.42415},
						{-0.095,0.5215,1.426},
						{0.866,-0.75575,1.461},
						{0.859644,-0.721511,1.46038},
						{0.847333,-0.656667,1.4675},
						{0.857125,-0.6775,1.46481},
						{0.84564,-0.6206,1.47168},
						{0.853915,-0.623234,1.46681},
						{0.843661,-0.57178,1.47929},
						{0.852053,-0.579868,1.46887},
						{0.841576,-0.525085,1.4745},
						{0.85,-0.547,1.464},
						{0.836567,-0.47475,1.47717},
						{0.832297,-0.430752,1.48035},
						{0.814726,-0.218306,1.48019},
						{0.81109,-0.17706,1.48243},
						{0.798111,-0.116,1.45611},
						{0.806018,-0.126255,1.48353},
						{0.795379,-0.0727027,1.46292},
						{0.802259,-0.0780371,1.48956},
						{0.792729,-0.0238475,1.47369},
						{0.8004,-0.0434,1.4964},
						{0.786812,0.0247812,1.47205},
						{0.781043,0.0743571,1.47117},
						{0.776448,0.122155,1.47347},
						{0.771095,0.174,1.47249},
						{0.767525,0.217625,1.47795},
						{0.7634,0.26375,1.48675},
						{0.7586,0.3184,1.4956},
						{0.754333,0.369333,1.49167},
						{-0.523333,0.474833,1.4715},
						{-0.398,0.492,1.498},
						{-0.315,0.499,1.4525},
						{-0.316778,0.501,1.47678},
						{-0.2534,0.5092,1.4624},
						{-0.2341,0.5046,1.4554},
						{-0.171486,0.515568,1.47586},
						{-0.130382,0.522206,1.47165},
						{0.8245,-0.4365,1.501},
						{0.808445,-0.215622,1.50364},
						{0.798833,-0.16,1.51183},
						{0.805479,-0.175845,1.50558},
						{0.796897,-0.122138,1.51376},
						{0.804482,-0.127536,1.50707},
						{0.794948,-0.0745689,1.51812},
						{0.802582,-0.0773023,1.5074},
						{0.791178,-0.0239068,1.51893},
						{0.80025,-0.0404167,1.506},
						{0.787566,0.0245659,1.51991},
						{0.783539,0.0746538,1.52292},
						{0.779412,0.123863,1.52455},
						{0.774163,0.173489,1.52772},
						{0.770168,0.224115,1.53023},
						{0.765492,0.274523,1.53268},
						{0.762928,0.324036,1.53473},
						{0.74875,0.37275,1.54025},
						{0.758391,0.361219,1.52966},
						{-0.553,0.47,1.543},
						{-0.539333,0.473,1.516},
						{-0.414786,0.489857,1.52893},
						{-0.31572,0.50356,1.52948},
						{-0.279778,0.506778,1.53883},
						{-0.2452,0.5039,1.5286},
						{-0.166979,0.516596,1.52772},
						{-0.13731,0.522643,1.52302},
						{0.87,-0.807,1.596},
						{0.86975,-0.7724,1.59455},
						{0.866909,-0.724182,1.59427},
						{0.861667,-0.6705,1.5955},
						{0.85725,-0.625833,1.59475},
						{0.8482,-0.5568,1.5902},
						{0.851833,-0.580083,1.59242},
						{0.845529,-0.529529,1.58994},
						{0.839538,-0.483308,1.58954},
						{0.795,-0.073,1.599},
						{0.782,0.019,1.55},
						{0.767375,0.175375,1.57275},
						{0.7636,0.2216,1.5502},
						{0.758,0.2725,1.55267},
						{0.753937,0.330313,1.55634},
						{0.751833,0.357167,1.55717},
						{-0.566905,0.468333,1.57638},
						{-0.425773,0.490273,1.57382},
						{-0.384444,0.496889,1.59711},
						{-0.366667,0.501056,1.59111},
						{-0.324125,0.503453,1.57602},
						{-0.276506,0.505265,1.57604},
						{-0.241043,0.502348,1.57626},
						{-0.167418,0.517073,1.57425},
						{-0.130691,0.52225,1.57537},
						{0.7375,0.54,1.5965},
						{0.734,0.590333,1.59133},
						{0.87,-0.80375,1.60275},
						{0.867815,-0.774185,1.60678},
						{0.864591,-0.725068,1.60816},
						{0.859212,-0.673673,1.61085},
						{0.855207,-0.623431,1.61522},
						{0.847296,-0.567926,1.62556},
						{0.852439,-0.580439,1.61383},
						{0.845736,-0.525458,1.62015},
						{0.840804,-0.477639,1.62026},
						{0.837,-0.449,1.603},
						{0.814769,-0.205231,1.63254},
						{0.811631,-0.175662,1.63262},
						{0.807867,-0.126667,1.6325},
						{0.797375,-0.07275,1.60888},
						{0.803395,-0.0783023,1.63388},
						{0.797606,-0.0211212,1.63645},
						{0.8,-0.0485,1.6385},
						{0.792,0.0251538,1.63231},
						{0.787636,0.0719091,1.63527},
						{0.780514,0.127243,1.63105},
						{0.775133,0.176022,1.6282},
						{0.771458,0.224667,1.63625},
						{0.7682,0.2676,1.6433},
						{0.765,0.301,1.644},
						{-0.573,0.467,1.603},
						{-0.45525,0.48925,1.646},
						{-0.422566,0.489487,1.62042},
						{-0.376145,0.493783,1.6206},
						{-0.331744,0.497233,1.62928},
						{-0.36275,0.500625,1.60363},
						{-0.318452,0.501572,1.61369},
						{-0.275353,0.50275,1.62374},
						{-0.238048,0.502952,1.62671},
						{-0.172714,0.516,1.62583},
						{-0.129565,0.519903,1.62592},
						{0.737,0.54325,1.60725},
						{0.735,0.573742,1.62113},
						{0.813,-0.204,1.654},
						{0.808935,-0.174484,1.65648},
						{0.805226,-0.124245,1.66153},
						{0.798,-0.064875,1.67131},
						{0.802074,-0.0783704,1.66032},
						{0.796125,-0.0238281,1.66597},
						{0.8,-0.039,1.6525},
						{0.792441,0.0253676,1.66574},
						{0.787127,0.0749747,1.66847},
						{0.782869,0.124655,1.67185},
						{0.778464,0.173833,1.67367},
						{0.77407,0.224118,1.67473},
						{0.769701,0.274046,1.67897},
						{0.766628,0.312349,1.67702},
						{-0.459381,0.486857,1.67776},
						{-0.424593,0.488831,1.67607},
						{-0.374377,0.493443,1.6762},
						{-0.327534,0.496914,1.67817},
						{-0.291889,0.498667,1.68},
						{-0.2355,0.49875,1.6945},
						{-0.308636,0.500182,1.66736},
						{-0.272692,0.5024,1.67668},
						{-0.228977,0.502442,1.67526},
						{-0.174817,0.514073,1.6769},
						{-0.13029,0.5165,1.67339},
						{0.738,0.5465,1.6815},
						{0.734911,0.574929,1.67896},
						{0.765333,0.288333,1.70133},
						{0.767,0.321,1.701},
						{-0.867667,0.459333,1.74733},
						{-0.729,0.4685,1.7455},
						{-0.610444,0.477778,1.735},
						{-0.5782,0.4849,1.7439},
						{-0.521111,0.492,1.73515},
						{-0.470784,0.484922,1.73},
						{-0.425671,0.484939,1.7261},
						{-0.374598,0.488696,1.72615},
						{-0.325175,0.492612,1.72758},
						{-0.275519,0.495506,1.72577},
						{-0.238645,0.497806,1.72858},
						{-0.196,0.499,1.749},
						{-0.278333,0.5,1.70567},
						{-0.21329,0.501613,1.72597},
						{-0.175598,0.50777,1.72667},
						{-0.130273,0.509376,1.72494},
						{0.738,0.547,1.701},
						{0.73865,0.57945,1.72215},
						{0.914207,-1.02045,1.78952},
						{0.911032,-0.976548,1.79306},
						{0.898167,-0.909833,1.79567},
						{0.9046,-0.9299,1.78957},
						{0.897188,-0.875438,1.79438},
						{0.903889,-0.879333,1.79244},
						{0.892483,-0.824552,1.79255},
						{0.901,-0.8404,1.7932},
						{0.889839,-0.776097,1.79529},
						{0.887,-0.7252,1.7982},
						{0.881455,-0.686818,1.79691},
						{-0.424149,0.244,1.79123},
						{-0.377088,0.245735,1.79321},
						{-0.328191,0.246857,1.79452},
						{-0.289833,0.248,1.79583},
						{-0.154833,0.2455,1.7835},
						{-0.123776,0.242612,1.77294},
						{-0.0935294,0.243647,1.77324},
						{-0.453,0.253,1.7985},
						{-0.423225,0.257796,1.79084},
						{-0.373162,0.26105,1.78844},
						{-0.3255,0.268779,1.78909},
						{-0.284882,0.266788,1.78938},
						{-0.168648,0.275648,1.77661},
						{-0.125033,0.274446,1.7701},
						{-0.094425,0.274075,1.7706},
						{-0.452,0.348,1.798},
						{-0.438167,0.347833,1.7975},
						{-0.355063,0.329,1.79531},
						{-0.330527,0.327242,1.79569},
						{-0.2915,0.34175,1.79656},
						{-0.172,0.324826,1.77913},
						{-0.125306,0.323842,1.772},
						{-0.097,0.320125,1.77425},
						{-0.4573,0.370567,1.79163},
						{-0.425182,0.371724,1.78469},
						{-0.375143,0.374566,1.78299},
						{-0.326657,0.380709,1.78522},
						{-0.29146,0.38274,1.78776},
						{-0.174765,0.374624,1.78027},
						{-0.125702,0.37449,1.77423},
						{-0.0991538,0.374308,1.77831},
						{-0.9058,0.4432,1.7922},
						{-0.883357,0.447357,1.79143},
						{-0.460214,0.431071,1.78771},
						{-0.42441,0.425735,1.78253},
						{-0.375045,0.425688,1.78309},
						{-0.324863,0.426467,1.78372},
						{-0.278609,0.429293,1.78259},
						{-0.203,0.43375,1.79588},
						{-0.175425,0.424436,1.77903},
						{-0.126937,0.424714,1.77578},
						{-0.1,0.405,1.785},
						{-0.869408,0.45537,1.77522},
						{-0.824817,0.46055,1.78467},
						{-0.773941,0.461235,1.78378},
						{-0.724699,0.462904,1.77804},
						{-0.67315,0.46825,1.78528},
						{-0.620981,0.473463,1.77802},
						{-0.574636,0.478937,1.77624},
						{-0.524699,0.482313,1.77463},
						{-0.474011,0.473979,1.77872},
						{-0.424796,0.467705,1.77291},
						{-0.374667,0.469544,1.76976},
						{-0.324526,0.471091,1.76769},
						{-0.274794,0.473165,1.7659},
						{-0.225285,0.484021,1.77635},
						{-0.175701,0.476523,1.77492},
						{-0.126887,0.475424,1.77437},
						{-0.212,0.5,1.7515},
						{-0.1706,0.5018,1.7582},
						{-0.128069,0.502276,1.75728},
						{0.73975,0.58575,1.7675},
						{0.911667,-1.0195,1.806},
						{0.906,-0.973333,1.80511},
						{0.898,-0.907,1.809},
						{0.9021,-0.9325,1.8108},
						{0.895368,-0.875368,1.80705},
						{0.890609,-0.82387,1.81443},
						{0.885291,-0.775161,1.81194},
						{0.882302,-0.724868,1.8096},
						{0.875635,-0.671058,1.80919},
						{0.87305,-0.627067,1.815},
						{0.867017,-0.574983,1.81559},
						{0.86169,-0.523948,1.82121},
						{0.85722,-0.474373,1.82281},
						{0.858111,-0.447778,1.83189},
						{0.8425,-0.253,1.845},
						{0.834981,-0.226412,1.83837},
						{0.828612,-0.175429,1.83871},
						{0.8233,-0.126,1.83932},
						{0.818854,-0.0767708,1.84183},
						{0.817471,-0.0315294,1.84606},
						{-0.853,0.02775,1.84225},
						{-0.826731,0.0363881,1.83724},
						{-0.79025,0.0358,1.84195},
						{-0.511615,0.0274,1.84154},
						{-0.494444,0.0448889,1.84333},
						{0.8115,0.0314,1.8479},
						{-0.816091,0.0830909,1.83991},
						{-0.771115,0.0909508,1.83808},
						{-0.510167,0.0540833,1.84467},
						{-0.471904,0.0639759,1.84194},
						{-0.431917,0.0684583,1.84675},
						{-0.373412,0.0787059,1.84475},
						{-0.33995,0.0868,1.84385},
						{0.806182,0.0709091,1.84745},
						{-0.807364,0.121136,1.84186},
						{-0.779258,0.123403,1.83736},
						{0.801714,0.111143,1.84657},
						{-0.805333,0.154,1.84833},
						{-0.782567,0.161784,1.84635},
						{-0.453857,0.192143,1.84671},
						{-0.423873,0.191309,1.84224},
						{-0.372648,0.187409,1.84005},
						{-0.325336,0.18176,1.83889},
						{-0.2975,0.19625,1.836},
						{-0.457219,0.227656,1.83722},
						{-0.425473,0.22113,1.83177},
						{-0.37488,0.222596,1.82448},
						{-0.324731,0.222819,1.81699},
						{-0.291025,0.226329,1.82496},
						{-0.161,0.247,1.803},
						{-0.57451,0.282959,1.8469},
						{-0.51791,0.27966,1.84687},
						{-0.473603,0.274644,1.83448},
						{-0.42575,0.283894,1.81398},
						{-0.375475,0.287088,1.81094},
						{-0.318594,0.2915,1.80441},
						{-0.277732,0.281366,1.81202},
						{-0.186,0.262,1.813},
						{-0.611211,0.333684,1.84811},
						{-0.575629,0.324818,1.84487},
						{-0.525406,0.324055,1.84012},
						{-0.476643,0.326063,1.83306},
						{-0.424698,0.323515,1.80656},
						{-0.378531,0.322701,1.80512},
						{-0.318755,0.320786,1.80406},
						{-0.284182,0.320647,1.81184},
						{-0.197333,0.325167,1.80783},
						{-0.813154,0.394,1.84869},
						{-0.796,0.394,1.849},
						{-0.610029,0.3796,1.84826},
						{-0.56906,0.372554,1.84629},
						{-0.52549,0.374647,1.84124},
						{-0.483462,0.374846,1.83029},
						{-0.4168,0.3604,1.8002},
						{-0.368583,0.352417,1.80237},
						{-0.323105,0.358562,1.80309},
						{-0.279446,0.367631,1.81691},
						{-0.20375,0.38775,1.81025},
						{-0.199,0.3596,1.8022},
						{-0.917217,0.427145,1.83129},
						{-0.875453,0.434,1.82866},
						{-0.82624,0.436594,1.83179},
						{-0.775673,0.436627,1.83541},
						{-0.731246,0.440721,1.8403},
						{-0.6745,0.4485,1.844},
						{-0.615875,0.42225,1.84796},
						{-0.571039,0.42767,1.84658},
						{-0.52614,0.424752,1.84262},
						{-0.481033,0.419652,1.82177},
						{-0.438933,0.405333,1.80067},
						{-0.262045,0.418939,1.82315},
						{-0.217059,0.432588,1.81876},
						{-0.853,0.45,1.802},
						{-0.8272,0.4526,1.80572},
						{-0.77464,0.45248,1.8052},
						{-0.720192,0.454385,1.81581},
						{-0.677059,0.458212,1.82393},
						{-0.624052,0.46001,1.8219},
						{-0.574848,0.461276,1.8238},
						{-0.527254,0.465,1.82766},
						{-0.48793,0.462326,1.81463},
						{-0.233,0.470387,1.82106},
						{0.75,0.5875,1.837},
						{0.9475,-1.321,1.8985},
						{0.944,-1.292,1.895},
						{0.945,-1.11617,1.89417},
						{0.938429,-1.07543,1.89329},
						{0.856,-0.455,1.854},
						{1.2225,-0.4652,1.88515},
						{1.25,-0.47,1.899},
						{0.858,-0.45,1.86},
						{0.840167,-0.254333,1.871},
						{0.831739,-0.224391,1.86196},
						{0.826826,-0.17087,1.86652},
						{0.819968,-0.124806,1.86403},
						{0.8155,-0.0736471,1.86615},
						{-0.51975,-0.003,1.87025},
						{0.812982,-0.0239107,1.86407},
						{-0.861833,0.0365,1.86367},
						{-0.822421,0.0244737,1.86095},
						{-0.788167,0.0346667,1.86094},
						{-0.512947,0.0180526,1.86195},
						{-0.484667,0.0444667,1.86947},
						{0.80741,0.024623,1.86195},
						{-0.864567,0.0731667,1.88777},
						{-0.828227,0.0880455,1.87464},
						{-0.767222,0.079,1.86663},
						{-0.740286,0.0917714,1.87406},
						{-0.671543,0.0656,1.88217},
						{-0.620061,0.074449,1.87327},
						{-0.575817,0.0841936,1.86733},
						{-0.52217,0.0810982,1.85813},
						{-0.476613,0.0826,1.85488},
						{-0.424257,0.0803465,1.85621},
						{-0.378037,0.0856296,1.85972},
						{-0.338,0.0902,1.85813},
						{0.798545,0.085,1.868},
						{0.801938,0.0729167,1.86527},
						{-0.864786,0.126839,1.88961},
						{-0.833597,0.128274,1.8695},
						{-0.756817,0.128067,1.86005},
						{-0.731978,0.123,1.88284},
						{-0.611425,0.125957,1.88383},
						{-0.57607,0.125076,1.87418},
						{-0.52482,0.123756,1.86543},
						{-0.476671,0.123598,1.86983},
						{-0.425375,0.126068,1.87897},
						{-0.37165,0.1273,1.88366},
						{-0.33052,0.126415,1.88488},
						{-0.2958,0.1258,1.8976},
						{0.795538,0.124846,1.864},
						{0.8008,0.1231,1.858},
						{-0.868,0.181404,1.89549},
						{-0.827411,0.1774,1.88232},
						{-0.767217,0.173906,1.85862}};


		std::vector <detailed_node> detailed_nodes;
		detailed_nodes.resize(test_points.size());
		std::vector<int> x_sort_ids(test_points.size());
		std::vector<int> y_sort_ids(test_points.size());
		std::vector<int> z_sort_ids(test_points.size());
		std::vector<int> end_list(test_points.size());

		// point_with_id point_with_ids[test_points.size()];
		for(int i=0;i<test_points.size();i++){
			point_with_ids[i].id = i;
			point_with_ids[i].pos[0] = test_points[i][0];
			point_with_ids[i].pos[1] = test_points[i][1];
			point_with_ids[i].pos[2] = test_points[i][2];
			detailed_nodes[i].ready = false;
			end_list[i] = -1;
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

		detailed_nodes[root_median_id].ready = true;
		detailed_nodes[root_median_id].node_is_right = false;
		detailed_nodes[root_median_id].parent_id = -1;
		detailed_nodes[root_median_id].depth = 0;
		detailed_nodes[root_median_id].axis = 0;
		detailed_nodes[root_median_id].middle = root_middle;
		detailed_nodes[root_median_id].group_size = test_points.size();

		int *d_end_list;
		detailed_node *d_detailed_nodes;
		cudaMalloc((void **)&detailed_nodes[root_median_id].x_sort_ids, test_points.size() * sizeof(int));
		cudaMalloc((void **)&detailed_nodes[root_median_id].y_sort_ids, test_points.size() * sizeof(int));
		cudaMalloc((void **)&detailed_nodes[root_median_id].z_sort_ids, test_points.size() * sizeof(int));
		cudaMalloc((void **)&d_end_list, test_points.size() * sizeof(int));
		cudaMalloc((void **)&d_detailed_nodes, test_points.size() * sizeof(detailed_node));

		cudaMemcpy(detailed_nodes[root_median_id].x_sort_ids, &x_sort_ids[0], test_points.size() * sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(detailed_nodes[root_median_id].y_sort_ids, &y_sort_ids[0], test_points.size() * sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(detailed_nodes[root_median_id].z_sort_ids, &z_sort_ids[0], test_points.size() * sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(d_end_list, &end_list[0], test_points.size() * sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(d_detailed_nodes, &detailed_nodes[0], test_points.size() * sizeof(detailed_node), cudaMemcpyHostToDevice);

		cudaDeviceSetLimit(cudaLimitStackSize, 1024*1024);
		int dimx_create_node = 32;
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
			std::cout<<"call depth = "<< depth_count <<std::endl;
			// std::cout<<"create kernel start"<<std::endl;
			d_DepthCreateNode<<<grid_create_node,block_create_node>>>(test_points.size(),d_detailed_nodes,d_end_list);
			// std::cout<<"create kernel end"<<std::endl;
			if(depth_count >= estimate_depth){
				std::cout<<"limit termination"<<std::endl;
				break;
			} 
			depth_count++;
			
			// if(all_end) break;
		}
		cudaMemcpy(&end_list[0], d_end_list, test_points.size() * sizeof(int), cudaMemcpyDeviceToHost);
		cudaMemcpy(&detailed_nodes[0], d_detailed_nodes, test_points.size() * sizeof(detailed_node), cudaMemcpyDeviceToHost);

		cudaFree(detailed_nodes[root_median_id].x_sort_ids);
		cudaFree(detailed_nodes[root_median_id].y_sort_ids);
		cudaFree(detailed_nodes[root_median_id].z_sort_ids);
		cudaFree(d_end_list);
		cudaFree(d_detailed_nodes);
		bool all_end = std::all_of(end_list.begin(), end_list.end(), [](int end) { return 0 < end; });
		if(all_end) std::cout<<"successful termination"<<std::endl;


		// first=false;
	}
	// root_id=root_median_id;
	//表示用スクリプト

	/////////////////////////////////////////////////////////////////////////////////////////施工
	
	build_end = clock();
	// printf("create tree time is %.5fs\n",(double)(build_end-build_start)/CLOCKS_PER_SEC);
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