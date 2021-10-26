

#include <string.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "kdtree_cuda/kdtree_cuda.hpp"
#include <vector>
#include <iostream>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include <math.h>

struct kdhyperrect {
	int dim;
	double *min, *max;              /* minimum/maximum coords */
};

struct kdnode {
	double *pos;
	int dir;
	void *data;

	struct kdnode *left, *right;	/* negative/positive side */
};

struct res_node {
	struct kdnode *item;
	double dist_sq;
	struct res_node *next;
};

struct kdtree {
	int dim;
	struct kdnode *root;
	struct kdhyperrect *rect;
	void (*destr)(void*);
};

struct kdres {
	struct kdtree *tree;
	struct res_node *rlist, *riter;
	int size;
};

#define SQ(x)			((x) * (x))

//device
__device__ struct kdtree *d_kd_create(int k);
__device__ void d_kd_free(struct kdtree *tree);
__device__ void d_kd_clear(struct kdtree *tree);
__device__ void d_kd_data_destructor(struct kdtree *tree, void (*destr)(void*));

__device__ int d_kd_insert(struct kdtree *tree, const double *pos, void *data);
__device__ int d_kd_insertf(struct kdtree *tree, const float *pos, void *data);
__device__ int d_kd_insert3(struct kdtree *tree, double x, double y, double z, void *data);
__device__ int d_kd_insert3f(struct kdtree *tree, float x, float y, float z, void *data);

__device__ static int d_find_nearest(struct kdnode *node, const double *pos, double range, struct res_node *list, int ordered, int dim);

#if 0
__device__ static int d_find_nearest_n(struct kdnode *node, const double *pos, double range, int num, struct rheap *heap, int dim);
#endif

__device__ struct kdres *d_kd_nearest(struct kdtree *tree, const double *pos);
__device__ struct kdres *d_kd_nearestf(struct kdtree *tree, const float *pos);
__device__ struct kdres *d_kd_nearest3(struct kdtree *tree, double x, double y, double z);
__device__ struct kdres *d_kd_nearest3f(struct kdtree *tree, float x, float y, float z);

__device__ struct kdres *d_kd_nearest_range(struct kdtree *tree, const double *pos, double range);
__device__ struct kdres *d_kd_nearest_rangef(struct kdtree *tree, const float *pos, float range);
__device__ struct kdres *d_kd_nearest_range3(struct kdtree *tree, double x, double y, double z, double range);
__device__ struct kdres *d_kd_nearest_range3f(struct kdtree *tree, float x, float y, float z, float range);

__device__ void d_kd_res_free(struct kdres *set);
__device__ int d_kd_res_size(struct kdres *set);
__device__ void d_kd_res_rewind(struct kdres *set);
__device__ int d_kd_res_end(struct kdres *set);
__device__ int d_kd_res_next(struct kdres *set);

__device__ void *d_kd_res_item(struct kdres *set, double *pos);
__device__ void *d_kd_res_itemf(struct kdres *set, float *pos);
__device__ void *d_kd_res_item3(struct kdres *set, double *x, double *y, double *z);
__device__ void *d_kd_res_item3f(struct kdres *set, float *x, float *y, float *z);
__device__ void *d_kd_res_item_data(struct kdres *set);

__device__ static void d_clear_rec(struct kdnode *node, void (*destr)(void*));
__device__ static int d_insert_rec(struct kdnode **node, const double *pos, void *data, int dir, int dim);
__device__ static int d_rlist_insert(struct res_node *list, struct kdnode *item, double dist_sq);
__device__ static void d_clear_results(struct kdres *set);

__device__ static struct kdhyperrect* d_hyperrect_create(int dim, const double *min, const double *max);
__device__ static void d_hyperrect_free(struct kdhyperrect *rect);
__device__ static struct kdhyperrect* d_hyperrect_duplicate(const struct kdhyperrect *rect);
__device__ static void d_hyperrect_extend(struct kdhyperrect *rect, const double *pos);
__device__ static double d_hyperrect_dist_sq(struct kdhyperrect *rect, const double *pos);

//host
struct kdtree *h_kd_create(int k);
void h_kd_free(struct kdtree *tree);
void h_kd_clear(struct kdtree *tree);
void h_kd_data_destructor(struct kdtree *tree, void (*destr)(void*));

int h_kd_insert(struct kdtree *tree, const double *pos, void *data);
int h_kd_insertf(struct kdtree *tree, const float *pos, void *data);
int h_kd_insert3(struct kdtree *tree, double x, double y, double z, void *data);
int h_kd_insert3f(struct kdtree *tree, float x, float y, float z, void *data);

static int h_find_nearest(struct kdnode *node, const double *pos, double range, struct res_node *list, int ordered, int dim);

#if 0
static int h_find_nearest_n(struct kdnode *node, const double *pos, double range, int num, struct rheap *heap, int dim);
#endif

struct kdres *h_kd_nearest(struct kdtree *tree, const double *pos);
struct kdres *h_kd_nearestf(struct kdtree *tree, const float *pos);
struct kdres *h_kd_nearest3(struct kdtree *tree, double x, double y, double z);
struct kdres *h_kd_nearest3f(struct kdtree *tree, float x, float y, float z);

struct kdres *h_kd_nearest_range(struct kdtree *tree, const double *pos, double range);
struct kdres *h_kd_nearest_rangef(struct kdtree *tree, const float *pos, float range);
struct kdres *h_kd_nearest_range3(struct kdtree *tree, double x, double y, double z, double range);
struct kdres *h_kd_nearest_range3f(struct kdtree *tree, float x, float y, float z, float range);

void h_kd_res_free(struct kdres *set);
int h_kd_res_size(struct kdres *set);
void h_kd_res_rewind(struct kdres *set);
int h_kd_res_end(struct kdres *set);
int h_kd_res_next(struct kdres *set);

void *h_kd_res_item(struct kdres *set, double *pos);
void *h_kd_res_itemf(struct kdres *set, float *pos);
void *h_kd_res_item3(struct kdres *set, double *x, double *y, double *z);
void *h_kd_res_item3f(struct kdres *set, float *x, float *y, float *z);
void *h_kd_res_item_data(struct kdres *set);

static void h_clear_rec(struct kdnode *node, void (*destr)(void*));
static int h_insert_rec(struct kdnode **node, const double *pos, void *data, int dir, int dim);
static int h_rlist_insert(struct res_node *list, struct kdnode *item, double dist_sq);
static void h_clear_results(struct kdres *set);

static struct kdhyperrect* h_hyperrect_create(int dim, const double *min, const double *max);
static void h_hyperrect_free(struct kdhyperrect *rect);
static struct kdhyperrect* h_hyperrect_duplicate(const struct kdhyperrect *rect);
static void h_hyperrect_extend(struct kdhyperrect *rect, const double *pos);
static double h_hyperrect_dist_sq(struct kdhyperrect *rect, const double *pos);


//device
__device__ struct kdtree *d_kd_create(int k)
{
	struct kdtree *tree;

	if(!(tree = (struct kdtree *)malloc(sizeof *tree))) {
		return 0;
	}

	tree->dim = k;
	tree->root = 0;
	tree->destr = 0;
	tree->rect = 0;

	return tree;
}

__device__ void d_kd_free(struct kdtree *tree)
{
	if(tree) {
		d_kd_clear(tree);
		free(tree);
	}
}

__device__ static void d_clear_rec(struct kdnode *node, void (*destr)(void*))
{
	if(!node) return;

	d_clear_rec(node->left, destr);
	d_clear_rec(node->right, destr);
	
	if(destr) {
		destr(node->data);
	}
	free(node->pos);
	free(node);
}

__device__ void d_kd_clear(struct kdtree *tree)
{
	d_clear_rec(tree->root, tree->destr);
	tree->root = 0;

	if (tree->rect) {
		d_hyperrect_free(tree->rect);
		tree->rect = 0;
	}
}

__device__ void d_kd_data_destructor(struct kdtree *tree, void (*destr)(void*))
{
	tree->destr = destr;
}


__device__ static int d_insert_rec(struct kdnode **nptr, const double *pos, void *data, int dir, int dim)
{
	int new_dir;
	struct kdnode *node;

	if(!*nptr) {
		if(!(node = (struct kdnode *)malloc(sizeof *node))) {
			return -1;
		}
		if(!(node->pos = (double *)malloc(dim * sizeof *node->pos))) {
			free(node);
			return -1;
		}
		memcpy(node->pos, pos, dim * sizeof *node->pos);
		node->data = data;
		node->dir = dir;
		node->left = node->right = 0;
		*nptr = node;
		return 0;
	}

	node = *nptr;
	new_dir = (node->dir + 1) % dim;
	if(pos[node->dir] < node->pos[node->dir]) {
		return d_insert_rec(&(*nptr)->left, pos, data, new_dir, dim);
	}
	return d_insert_rec(&(*nptr)->right, pos, data, new_dir, dim);
}

__device__ int d_kd_insert(struct kdtree *tree, const double *pos, void *data)
{
	if (d_insert_rec(&tree->root, pos, data, 0, tree->dim)) {
		return -1;
	}

	if (tree->rect == 0) {
		tree->rect = d_hyperrect_create(tree->dim, pos, pos);
	} else {
		d_hyperrect_extend(tree->rect, pos);
	}

	return 0;
}

__device__ int d_kd_insertf(struct kdtree *tree, const float *pos, void *data)
{
	static double sbuf[16];
	double *bptr, *buf = 0;
	int res, dim = tree->dim;

	if(dim > 16) {
        if(!(bptr = buf = (double *)malloc(dim * sizeof *bptr))) {
            return -1;
        }
	} else {
		bptr = buf = sbuf;
	}

	while(dim-- > 0) {
		*bptr++ = *pos++;
	}

	res = d_kd_insert(tree, buf, data);
	if(tree->dim > 16) free(buf);
	return res;
}

__device__ int d_kd_insert3(struct kdtree *tree, double x, double y, double z, void *data)
{
	double buf[3];
	buf[0] = x;
	buf[1] = y;
	buf[2] = z;
	return d_kd_insert(tree, buf, data);
}

__device__ int d_kd_insert3f(struct kdtree *tree, float x, float y, float z, void *data)
{
	double buf[3];
	buf[0] = x;
	buf[1] = y;
	buf[2] = z;
	return d_kd_insert(tree, buf, data);
}

__device__ static int d_find_nearest(struct kdnode *node, const double *pos, double range, struct res_node *list, int ordered, int dim)
{
    printf("d_find_nearest open\n");
	double dist_sq, dx;
	int i, ret, added_res = 0;

	if(!node) return 0;
    printf("fn 1\n");
	dist_sq = 0;
	for(i=0; i<dim; i++) {
		dist_sq += SQ(node->pos[i] - pos[i]);
	}
    printf("fn 2\n");
	if(dist_sq <= SQ(range)) {
		if(d_rlist_insert(list, node, ordered ? dist_sq : -1.0) == -1) {
			return -1;
		}
		added_res = 1;
	}
    printf("fn 3\n");

	dx = pos[node->dir] - node->pos[node->dir];
    printf("fn 4\n");
	ret = d_find_nearest(dx <= 0.0 ? node->left : node->right, pos, range, list, ordered, dim);
    printf("fn 5\n");
	if(ret >= 0 && fabs(dx) < range) {
		added_res += ret;
		ret = d_find_nearest(dx <= 0.0 ? node->right : node->left, pos, range, list, ordered, dim);
	}
    printf("fn 6\n");
	if(ret == -1) {
		return -1;
	}
    printf("fn 7\n");
	added_res += ret;
    printf("fn 8\n");
	return added_res;
}

#if 0
__device__ static int d_find_nearest_n(struct kdnode *node, const double *pos, double range, int num, struct rheap *heap, int dim)
{
	double dist_sq, dx;
	int i, ret, added_res = 0;

	if(!node) return 0;
	
	/* if the photon is close enough, add it to the result heap */
	dist_sq = 0;
	for(i=0; i<dim; i++) {
		dist_sq += SQ(node->pos[i] - pos[i]);
	}
	if(dist_sq <= range_sq) {
		if(heap->size >= num) {
			/* get furthest element */
			struct res_node *maxelem = rheap_get_max(heap);

			/* and check if the new one is closer than that */
			if(maxelem->dist_sq > dist_sq) {
				rheap_remove_max(heap);

				if(rheap_insert(heap, node, dist_sq) == -1) {
					return -1;
				}
				added_res = 1;

				range_sq = dist_sq;
			}
		} else {
			if(rheap_insert(heap, node, dist_sq) == -1) {
				return =1;
			}
			added_res = 1;
		}
	}


	/* find signed distance from the splitting plane */
	dx = pos[node->dir] - node->pos[node->dir];

	ret = d_find_nearest_n(dx <= 0.0 ? node->left : node->right, pos, range, num, heap, dim);
	if(ret >= 0 && fabs(dx) < range) {
		added_res += ret;
		ret = d_find_nearest_n(dx <= 0.0 ? node->right : node->left, pos, range, num, heap, dim);
	}

}
#endif

__device__ static void d_kd_nearest_i(struct kdnode *node, const double *pos, struct kdnode **result, double *result_dist_sq, struct kdhyperrect* rect)
{
	int dir = node->dir;
	int i;
	double dummy, dist_sq;
	struct kdnode *nearer_subtree, *farther_subtree;
	double *nearer_hyperrect_coord, *farther_hyperrect_coord;

	/* Decide whether to go left or right in the tree */
	dummy = pos[dir] - node->pos[dir];
	if (dummy <= 0) {
		nearer_subtree = node->left;
		farther_subtree = node->right;
		nearer_hyperrect_coord = rect->max + dir;
		farther_hyperrect_coord = rect->min + dir;
	} else {
		nearer_subtree = node->right;
		farther_subtree = node->left;
		nearer_hyperrect_coord = rect->min + dir;
		farther_hyperrect_coord = rect->max + dir;
	}

	if (nearer_subtree) {
		/* Slice the hyperrect to get the hyperrect of the nearer subtree */
		dummy = *nearer_hyperrect_coord;
		*nearer_hyperrect_coord = node->pos[dir];
		/* Recurse down into nearer subtree */
		d_kd_nearest_i(nearer_subtree, pos, result, result_dist_sq, rect);
		/* Undo the slice */
		*nearer_hyperrect_coord = dummy;
	}

	/* Check the distance of the point at the current node, compare it
	 * with our best so far */
	dist_sq = 0;
	for(i=0; i < rect->dim; i++) {
		dist_sq += SQ(node->pos[i] - pos[i]);
	}
	if (dist_sq < *result_dist_sq) {
		*result = node;
		*result_dist_sq = dist_sq;
	}

	if (farther_subtree) {
		/* Get the hyperrect of the farther subtree */
		dummy = *farther_hyperrect_coord;
		*farther_hyperrect_coord = node->pos[dir];
		/* Check if we have to recurse down by calculating the closest
		 * point of the hyperrect and see if it's closer than our
		 * minimum distance in result_dist_sq. */
		if (d_hyperrect_dist_sq(rect, pos) < *result_dist_sq) {
			/* Recurse down into farther subtree */
			d_kd_nearest_i(farther_subtree, pos, result, result_dist_sq, rect);
		}
		/* Undo the slice on the hyperrect */
		*farther_hyperrect_coord = dummy;
	}
}

__device__ struct kdres *d_kd_nearest(struct kdtree *kd, const double *pos)
{
	struct kdhyperrect *rect;
	struct kdnode *result;
	struct kdres *rset;
	double dist_sq;
	int i;

	if (!kd) return 0;
	if (!kd->rect) return 0;

	/* Allocate result set */
	if(!(rset = (struct kdres *)malloc(sizeof *rset))) {
		return 0;
	}
	if(!(rset->rlist = (struct res_node *)malloc(sizeof(struct res_node)))) {
		free(rset);
		return 0;
	}
	rset->rlist->next = 0;
	rset->tree = kd;

	/* Duplicate the bounding hyperrectangle, we will work on the copy */
	if (!(rect = d_hyperrect_duplicate(kd->rect))) {
		d_kd_res_free(rset);
		return 0;
	}

	/* Our first guesstimate is the root node */
	result = kd->root;
	dist_sq = 0;
	for (i = 0; i < kd->dim; i++)
		dist_sq += SQ(result->pos[i] - pos[i]);

	/* Search for the nearest neighbour recursively */
	d_kd_nearest_i(kd->root, pos, &result, &dist_sq, rect);

	/* Free the copy of the hyperrect */
	d_hyperrect_free(rect);

	/* Store the result */
	if (result) {
		if (d_rlist_insert(rset->rlist, result, -1.0) == -1) {
			d_kd_res_free(rset);
			return 0;
		}
		rset->size = 1;
		d_kd_res_rewind(rset);
		return rset;
	} else {
		d_kd_res_free(rset);
		return 0;
	}
}

__device__ struct kdres *d_kd_nearestf(struct kdtree *tree, const float *pos)
{
	static double sbuf[16];
	double *bptr, *buf = 0;
	int dim = tree->dim;
	struct kdres *res;

	if(dim > 16) {
        if(!(bptr = buf = (double *)malloc(dim * sizeof *bptr))) {
            return 0;
        }
	} else {
		bptr = buf = sbuf;
	}

	while(dim-- > 0) {
		*bptr++ = *pos++;
	}

	res = d_kd_nearest(tree, buf);
	if(tree->dim > 16) free(buf);
	return res;
}

__device__ struct kdres *d_kd_nearest3(struct kdtree *tree, double x, double y, double z)
{
	double pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	return d_kd_nearest(tree, pos);
}

__device__ struct kdres *d_kd_nearest3f(struct kdtree *tree, float x, float y, float z)
{
	double pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	return d_kd_nearest(tree, pos);
}

__device__ struct kdres *d_kd_nearest_range(struct kdtree *kd, const double *pos, double range)
{
    printf("d_kd_nearest_range open\n");
	int ret;
	struct kdres *rset;

	if(!(rset = (struct kdres *)malloc(sizeof *rset))) {
		return 0;
	}
	if(!(rset->rlist = (struct res_node *)malloc(sizeof(struct res_node)))) {
		free(rset);
		return 0;
	}
	rset->rlist->next = 0;
	rset->tree = kd;

	if((ret = d_find_nearest(kd->root, pos, range, rset->rlist, 0, kd->dim)) == -1) {
		d_kd_res_free(rset);
		return 0;
	}
    printf("fn close\n");
	rset->size = ret;
	d_kd_res_rewind(rset);
	return rset;
}

__device__ struct kdres *d_kd_nearest_rangef(struct kdtree *kd, const float *pos, float range)
{
	static double sbuf[16];
	double *bptr, *buf = 0;
	int dim = kd->dim;
	struct kdres *res;

	if(dim > 16) {
        if(!(bptr = buf = (double *)malloc(dim * sizeof *bptr))) {
            return 0;
        }
	} else {
		bptr = buf = sbuf;
	}

	while(dim-- > 0) {
		*bptr++ = *pos++;
	}

	res = d_kd_nearest_range(kd, buf, range);
	if(kd->dim > 16) free(buf);
	return res;
}

__device__ struct kdres *d_kd_nearest_range3(struct kdtree *tree, double x, double y, double z, double range)
{
    printf("d_kd_nearest_range3 open\n");
	double buf[3];
	buf[0] = x;
	buf[1] = y;
	buf[2] = z;
	return d_kd_nearest_range(tree, buf, range);
}

__device__ struct kdres *d_kd_nearest_range3f(struct kdtree *tree, float x, float y, float z, float range)
{
	double buf[3];
	buf[0] = x;
	buf[1] = y;
	buf[2] = z;
	return d_kd_nearest_range(tree, buf, range);
}

__device__ void d_kd_res_free(struct kdres *rset)
{
    printf("d_kd_res_free open\n");
	d_clear_results(rset);
	free(rset->rlist);
	free(rset);
}

__device__ int d_kd_res_size(struct kdres *set)
{
	return (set->size);
}

__device__ void d_kd_res_rewind(struct kdres *rset)
{
    printf("d_kd_res_rewind open\n");
	rset->riter = rset->rlist->next;
}

__device__ int d_kd_res_end(struct kdres *rset)
{
	return rset->riter == 0;
}

__device__ int d_kd_res_next(struct kdres *rset)
{
	rset->riter = rset->riter->next;
	return rset->riter != 0;
}

__device__ void *d_kd_res_item(struct kdres *rset, double *pos)
{
	if(rset->riter) {
		if(pos) {
			memcpy(pos, rset->riter->item->pos, rset->tree->dim * sizeof *pos);
		}
		return rset->riter->item->data;
	}
	return 0;
}

__device__ void *d_kd_res_itemf(struct kdres *rset, float *pos)
{
	if(rset->riter) {
		if(pos) {
			int i;
			for(i=0; i<rset->tree->dim; i++) {
				pos[i] = rset->riter->item->pos[i];
			}
		}
		return rset->riter->item->data;
	}
	return 0;
}

__device__ void *d_kd_res_item3(struct kdres *rset, double *x, double *y, double *z)
{
	if(rset->riter) {
		if(x) *x = rset->riter->item->pos[0];
		if(y) *y = rset->riter->item->pos[1];
		if(z) *z = rset->riter->item->pos[2];
		return rset->riter->item->data;
	}
	return 0;
}

__device__ void *d_kd_res_item3f(struct kdres *rset, float *x, float *y, float *z)
{
	if(rset->riter) {
		if(x) *x = rset->riter->item->pos[0];
		if(y) *y = rset->riter->item->pos[1];
		if(z) *z = rset->riter->item->pos[2];
		return rset->riter->item->data;
	}
	return 0;
}

__device__ void *d_kd_res_item_data(struct kdres *set)
{
	return d_kd_res_item(set, 0);
}

/* ---- hyperrectangle helpers ---- */
__device__ static struct kdhyperrect* d_hyperrect_create(int dim, const double *min, const double *max)
{
	size_t size = dim * sizeof(double);
	struct kdhyperrect* rect = 0;

	if (!(rect = (struct kdhyperrect*)malloc(sizeof(struct kdhyperrect)))) {
		return 0;
	}

	rect->dim = dim;
	if (!(rect->min = (double *)malloc(size))) {
		free(rect);
		return 0;
	}
	if (!(rect->max = (double *)malloc(size))) {
		free(rect->min);
		free(rect);
		return 0;
	}
	memcpy(rect->min, min, size);
	memcpy(rect->max, max, size);

	return rect;
}

__device__ static void d_hyperrect_free(struct kdhyperrect *rect)
{
	free(rect->min);
	free(rect->max);
	free(rect);
}

__device__ static struct kdhyperrect* d_hyperrect_duplicate(const struct kdhyperrect *rect)
{
	return d_hyperrect_create(rect->dim, rect->min, rect->max);
}

__device__ static void d_hyperrect_extend(struct kdhyperrect *rect, const double *pos)
{
	int i;

	for (i=0; i < rect->dim; i++) {
		if (pos[i] < rect->min[i]) {
			rect->min[i] = pos[i];
		}
		if (pos[i] > rect->max[i]) {
			rect->max[i] = pos[i];
		}
	}
}

__device__ static double d_hyperrect_dist_sq(struct kdhyperrect *rect, const double *pos)
{
	int i;
	double result = 0;

	for (i=0; i < rect->dim; i++) {
		if (pos[i] < rect->min[i]) {
			result += SQ(rect->min[i] - pos[i]);
		} else if (pos[i] > rect->max[i]) {
			result += SQ(rect->max[i] - pos[i]);
		}
	}

	return result;
}


/* inserts the item. if dist_sq is >= 0, then do an ordered insert */
/* TODO make the ordering code use heapsort */
__device__ static int d_rlist_insert(struct res_node *list, struct kdnode *item, double dist_sq)
{
    printf("d_rlist_insert open\n");
	struct res_node *rnode;

	if(!(rnode = (struct res_node *)malloc(sizeof(struct res_node)))) {
		return -1;
	}
	rnode->item = item;
	rnode->dist_sq = dist_sq;

	if(dist_sq >= 0.0) {
		while(list->next && list->next->dist_sq < dist_sq) {
			list = list->next;
		}
	}
	rnode->next = list->next;
	list->next = rnode;
	return 0;
}

__device__ static void d_clear_results(struct kdres *rset)
{
	struct res_node *tmp, *node = rset->rlist->next;

	while(node) {
		tmp = node;
		node = node->next;
		free(tmp);
	}

	rset->rlist->next = 0;
}




//host
struct kdtree *h_kd_create(int k)
{
	struct kdtree *tree;

	if(!(tree = (struct kdtree *)malloc(sizeof *tree))) {
		return 0;
	}

	tree->dim = k;
	tree->root = 0;
	tree->destr = 0;
	tree->rect = 0;

	return tree;
}

void h_kd_free(struct kdtree *tree)
{
	if(tree) {
		h_kd_clear(tree);
		free(tree);
	}
}

static void h_clear_rec(struct kdnode *node, void (*destr)(void*))
{
	if(!node) return;

	h_clear_rec(node->left, destr);
	h_clear_rec(node->right, destr);
	
	if(destr) {
		destr(node->data);
	}
	free(node->pos);
	free(node);
}

void h_kd_clear(struct kdtree *tree)
{
	h_clear_rec(tree->root, tree->destr);
	tree->root = 0;

	if (tree->rect) {
		h_hyperrect_free(tree->rect);
		tree->rect = 0;
	}
}

void h_kd_data_destructor(struct kdtree *tree, void (*destr)(void*))
{
	tree->destr = destr;
}


static int h_insert_rec(struct kdnode **nptr, const double *pos, void *data, int dir, int dim)
{
	int new_dir;
	struct kdnode *node;

	if(!*nptr) {
		if(!(node = (struct kdnode *)malloc(sizeof *node))) {
			return -1;
		}
		if(!(node->pos = (double *)malloc(dim * sizeof *node->pos))) {
			free(node);
			return -1;
		}
		memcpy(node->pos, pos, dim * sizeof *node->pos);
		node->data = data;
		node->dir = dir;
		node->left = node->right = 0;
		*nptr = node;
		return 0;
	}

	node = *nptr;
	new_dir = (node->dir + 1) % dim;
	if(pos[node->dir] < node->pos[node->dir]) {
		return h_insert_rec(&(*nptr)->left, pos, data, new_dir, dim);
	}
	return h_insert_rec(&(*nptr)->right, pos, data, new_dir, dim);
}

int h_kd_insert(struct kdtree *tree, const double *pos, void *data)
{
	if (h_insert_rec(&tree->root, pos, data, 0, tree->dim)) {
		return -1;
	}

	if (tree->rect == 0) {
		tree->rect = h_hyperrect_create(tree->dim, pos, pos);
	} else {
		h_hyperrect_extend(tree->rect, pos);
	}

	return 0;
}

int h_kd_insertf(struct kdtree *tree, const float *pos, void *data)
{
	static double sbuf[16];
	double *bptr, *buf = 0;
	int res, dim = tree->dim;

	if(dim > 16) {
        if(!(bptr = buf = (double *)malloc(dim * sizeof *bptr))) {
            return -1;
        }
	} else {
		bptr = buf = sbuf;
	}

	while(dim-- > 0) {
		*bptr++ = *pos++;
	}

	res = h_kd_insert(tree, buf, data);
	if(tree->dim > 16) free(buf);
	return res;
}

int h_kd_insert3(struct kdtree *tree, double x, double y, double z, void *data)
{
	double buf[3];
	buf[0] = x;
	buf[1] = y;
	buf[2] = z;
	return h_kd_insert(tree, buf, data);
}

int h_kd_insert3f(struct kdtree *tree, float x, float y, float z, void *data)
{
	double buf[3];
	buf[0] = x;
	buf[1] = y;
	buf[2] = z;
	return h_kd_insert(tree, buf, data);
}

static int h_find_nearest(struct kdnode *node, const double *pos, double range, struct res_node *list, int ordered, int dim)
{
    printf("h_find_nearest open\n");
	double dist_sq, dx;
	int i, ret, added_res = 0;

	if(!node) return 0;
    printf("fn 1\n");
	dist_sq = 0;
	for(i=0; i<dim; i++) {
		dist_sq += SQ(node->pos[i] - pos[i]);
	}
    printf("fn 2\n");
	if(dist_sq <= SQ(range)) {
		if(h_rlist_insert(list, node, ordered ? dist_sq : -1.0) == -1) {
			return -1;
		}
		added_res = 1;
	}
    printf("fn 3\n");

	dx = pos[node->dir] - node->pos[node->dir];
    printf("fn 4\n");
	ret = h_find_nearest(dx <= 0.0 ? node->left : node->right, pos, range, list, ordered, dim);
    printf("fn 5\n");
	if(ret >= 0 && fabs(dx) < range) {
		added_res += ret;
		ret = h_find_nearest(dx <= 0.0 ? node->right : node->left, pos, range, list, ordered, dim);
	}
    printf("fn 6\n");
	if(ret == -1) {
		return -1;
	}
    printf("fn 7\n");
	added_res += ret;
    printf("fn 8\n");
	return added_res;
}

#if 0
static int h_find_nearest_n(struct kdnode *node, const double *pos, double range, int num, struct rheap *heap, int dim)
{
	double dist_sq, dx;
	int i, ret, added_res = 0;

	if(!node) return 0;
	
	/* if the photon is close enough, add it to the result heap */
	dist_sq = 0;
	for(i=0; i<dim; i++) {
		dist_sq += SQ(node->pos[i] - pos[i]);
	}
	if(dist_sq <= range_sq) {
		if(heap->size >= num) {
			/* get furthest element */
			struct res_node *maxelem = rheap_get_max(heap);

			/* and check if the new one is closer than that */
			if(maxelem->dist_sq > dist_sq) {
				rheap_remove_max(heap);

				if(rheap_insert(heap, node, dist_sq) == -1) {
					return -1;
				}
				added_res = 1;

				range_sq = dist_sq;
			}
		} else {
			if(rheap_insert(heap, node, dist_sq) == -1) {
				return =1;
			}
			added_res = 1;
		}
	}


	/* find signed distance from the splitting plane */
	dx = pos[node->dir] - node->pos[node->dir];

	ret = h_find_nearest_n(dx <= 0.0 ? node->left : node->right, pos, range, num, heap, dim);
	if(ret >= 0 && fabs(dx) < range) {
		added_res += ret;
		ret = h_find_nearest_n(dx <= 0.0 ? node->right : node->left, pos, range, num, heap, dim);
	}

}
#endif

static void h_kd_nearest_i(struct kdnode *node, const double *pos, struct kdnode **result, double *result_dist_sq, struct kdhyperrect* rect)
{
	int dir = node->dir;
	int i;
	double dummy, dist_sq;
	struct kdnode *nearer_subtree, *farther_subtree;
	double *nearer_hyperrect_coord, *farther_hyperrect_coord;

	/* Decide whether to go left or right in the tree */
	dummy = pos[dir] - node->pos[dir];
	if (dummy <= 0) {
		nearer_subtree = node->left;
		farther_subtree = node->right;
		nearer_hyperrect_coord = rect->max + dir;
		farther_hyperrect_coord = rect->min + dir;
	} else {
		nearer_subtree = node->right;
		farther_subtree = node->left;
		nearer_hyperrect_coord = rect->min + dir;
		farther_hyperrect_coord = rect->max + dir;
	}

	if (nearer_subtree) {
		/* Slice the hyperrect to get the hyperrect of the nearer subtree */
		dummy = *nearer_hyperrect_coord;
		*nearer_hyperrect_coord = node->pos[dir];
		/* Recurse down into nearer subtree */
		h_kd_nearest_i(nearer_subtree, pos, result, result_dist_sq, rect);
		/* Undo the slice */
		*nearer_hyperrect_coord = dummy;
	}

	/* Check the distance of the point at the current node, compare it
	 * with our best so far */
	dist_sq = 0;
	for(i=0; i < rect->dim; i++) {
		dist_sq += SQ(node->pos[i] - pos[i]);
	}
	if (dist_sq < *result_dist_sq) {
		*result = node;
		*result_dist_sq = dist_sq;
	}

	if (farther_subtree) {
		/* Get the hyperrect of the farther subtree */
		dummy = *farther_hyperrect_coord;
		*farther_hyperrect_coord = node->pos[dir];
		/* Check if we have to recurse down by calculating the closest
		 * point of the hyperrect and see if it's closer than our
		 * minimum distance in result_dist_sq. */
		if (h_hyperrect_dist_sq(rect, pos) < *result_dist_sq) {
			/* Recurse down into farther subtree */
			h_kd_nearest_i(farther_subtree, pos, result, result_dist_sq, rect);
		}
		/* Undo the slice on the hyperrect */
		*farther_hyperrect_coord = dummy;
	}
}

struct kdres *h_kd_nearest(struct kdtree *kd, const double *pos)
{
	struct kdhyperrect *rect;
	struct kdnode *result;
	struct kdres *rset;
	double dist_sq;
	int i;

	if (!kd) return 0;
	if (!kd->rect) return 0;

	/* Allocate result set */
	if(!(rset = (struct kdres *)malloc(sizeof *rset))) {
		return 0;
	}
	if(!(rset->rlist = (struct res_node *)malloc(sizeof(struct res_node)))) {
		free(rset);
		return 0;
	}
	rset->rlist->next = 0;
	rset->tree = kd;

	/* Duplicate the bounding hyperrectangle, we will work on the copy */
	if (!(rect = h_hyperrect_duplicate(kd->rect))) {
		h_kd_res_free(rset);
		return 0;
	}

	/* Our first guesstimate is the root node */
	result = kd->root;
	dist_sq = 0;
	for (i = 0; i < kd->dim; i++)
		dist_sq += SQ(result->pos[i] - pos[i]);

	/* Search for the nearest neighbour recursively */
	h_kd_nearest_i(kd->root, pos, &result, &dist_sq, rect);

	/* Free the copy of the hyperrect */
	h_hyperrect_free(rect);

	/* Store the result */
	if (result) {
		if (h_rlist_insert(rset->rlist, result, -1.0) == -1) {
			h_kd_res_free(rset);
			return 0;
		}
		rset->size = 1;
		h_kd_res_rewind(rset);
		return rset;
	} else {
		h_kd_res_free(rset);
		return 0;
	}
}

struct kdres *h_kd_nearestf(struct kdtree *tree, const float *pos)
{
	static double sbuf[16];
	double *bptr, *buf = 0;
	int dim = tree->dim;
	struct kdres *res;

	if(dim > 16) {
        if(!(bptr = buf = (double *)malloc(dim * sizeof *bptr))) {
            return 0;
        }
	} else {
		bptr = buf = sbuf;
	}

	while(dim-- > 0) {
		*bptr++ = *pos++;
	}

	res = h_kd_nearest(tree, buf);
	if(tree->dim > 16) free(buf);
	return res;
}

struct kdres *h_kd_nearest3(struct kdtree *tree, double x, double y, double z)
{
	double pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	return h_kd_nearest(tree, pos);
}

struct kdres *h_kd_nearest3f(struct kdtree *tree, float x, float y, float z)
{
	double pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	return h_kd_nearest(tree, pos);
}

struct kdres *h_kd_nearest_range(struct kdtree *kd, const double *pos, double range)
{
    printf("h_kd_nearest_range open\n");
	int ret;
	struct kdres *rset;

	if(!(rset = (struct kdres *)malloc(sizeof *rset))) {
		return 0;
	}
	if(!(rset->rlist = (struct res_node *)malloc(sizeof(struct res_node)))) {
		free(rset);
		return 0;
	}
	rset->rlist->next = 0;
	rset->tree = kd;

	if((ret = h_find_nearest(kd->root, pos, range, rset->rlist, 0, kd->dim)) == -1) {
		h_kd_res_free(rset);
		return 0;
	}
    printf("fn close\n");
	rset->size = ret;
	h_kd_res_rewind(rset);
	return rset;
}

struct kdres *h_kd_nearest_rangef(struct kdtree *kd, const float *pos, float range)
{
	static double sbuf[16];
	double *bptr, *buf = 0;
	int dim = kd->dim;
	struct kdres *res;

	if(dim > 16) {
        if(!(bptr = buf = (double *)malloc(dim * sizeof *bptr))) {
            return 0;
        }
	} else {
		bptr = buf = sbuf;
	}

	while(dim-- > 0) {
		*bptr++ = *pos++;
	}

	res = h_kd_nearest_range(kd, buf, range);
	if(kd->dim > 16) free(buf);
	return res;
}

struct kdres *h_kd_nearest_range3(struct kdtree *tree, double x, double y, double z, double range)
{
    printf("h_kd_nearest_range3 open\n");
	double buf[3];
	buf[0] = x;
	buf[1] = y;
	buf[2] = z;
	return h_kd_nearest_range(tree, buf, range);
}

struct kdres *h_kd_nearest_range3f(struct kdtree *tree, float x, float y, float z, float range)
{
	double buf[3];
	buf[0] = x;
	buf[1] = y;
	buf[2] = z;
	return h_kd_nearest_range(tree, buf, range);
}

void h_kd_res_free(struct kdres *rset)
{
    printf("h_kd_res_free open\n");
	h_clear_results(rset);
	free(rset->rlist);
	free(rset);
}

int h_kd_res_size(struct kdres *set)
{
	return (set->size);
}

void h_kd_res_rewind(struct kdres *rset)
{
    printf("h_kd_res_rewind open\n");
	rset->riter = rset->rlist->next;
}

int h_kd_res_end(struct kdres *rset)
{
	return rset->riter == 0;
}

int h_kd_res_next(struct kdres *rset)
{
	rset->riter = rset->riter->next;
	return rset->riter != 0;
}

void *h_kd_res_item(struct kdres *rset, double *pos)
{
	if(rset->riter) {
		if(pos) {
			memcpy(pos, rset->riter->item->pos, rset->tree->dim * sizeof *pos);
		}
		return rset->riter->item->data;
	}
	return 0;
}

void *h_kd_res_itemf(struct kdres *rset, float *pos)
{
	if(rset->riter) {
		if(pos) {
			int i;
			for(i=0; i<rset->tree->dim; i++) {
				pos[i] = rset->riter->item->pos[i];
			}
		}
		return rset->riter->item->data;
	}
	return 0;
}

void *h_kd_res_item3(struct kdres *rset, double *x, double *y, double *z)
{
	if(rset->riter) {
		if(x) *x = rset->riter->item->pos[0];
		if(y) *y = rset->riter->item->pos[1];
		if(z) *z = rset->riter->item->pos[2];
		return rset->riter->item->data;
	}
	return 0;
}

void *h_kd_res_item3f(struct kdres *rset, float *x, float *y, float *z)
{
	if(rset->riter) {
		if(x) *x = rset->riter->item->pos[0];
		if(y) *y = rset->riter->item->pos[1];
		if(z) *z = rset->riter->item->pos[2];
		return rset->riter->item->data;
	}
	return 0;
}

void *h_kd_res_item_data(struct kdres *set)
{
	return h_kd_res_item(set, 0);
}

/* ---- hyperrectangle helpers ---- */
static struct kdhyperrect* h_hyperrect_create(int dim, const double *min, const double *max)
{
	size_t size = dim * sizeof(double);
	struct kdhyperrect* rect = 0;

	if (!(rect = (struct kdhyperrect*)malloc(sizeof(struct kdhyperrect)))) {
		return 0;
	}

	rect->dim = dim;
	if (!(rect->min = (double *)malloc(size))) {
		free(rect);
		return 0;
	}
	if (!(rect->max = (double *)malloc(size))) {
		free(rect->min);
		free(rect);
		return 0;
	}
	memcpy(rect->min, min, size);
	memcpy(rect->max, max, size);

	return rect;
}

static void h_hyperrect_free(struct kdhyperrect *rect)
{
	free(rect->min);
	free(rect->max);
	free(rect);
}

static struct kdhyperrect* h_hyperrect_duplicate(const struct kdhyperrect *rect)
{
	return h_hyperrect_create(rect->dim, rect->min, rect->max);
}

static void h_hyperrect_extend(struct kdhyperrect *rect, const double *pos)
{
	int i;

	for (i=0; i < rect->dim; i++) {
		if (pos[i] < rect->min[i]) {
			rect->min[i] = pos[i];
		}
		if (pos[i] > rect->max[i]) {
			rect->max[i] = pos[i];
		}
	}
}

static double h_hyperrect_dist_sq(struct kdhyperrect *rect, const double *pos)
{
	int i;
	double result = 0;

	for (i=0; i < rect->dim; i++) {
		if (pos[i] < rect->min[i]) {
			result += SQ(rect->min[i] - pos[i]);
		} else if (pos[i] > rect->max[i]) {
			result += SQ(rect->max[i] - pos[i]);
		}
	}

	return result;
}


/* inserts the item. if dist_sq is >= 0, then do an ordered insert */
/* TODO make the ordering code use heapsort */
static int h_rlist_insert(struct res_node *list, struct kdnode *item, double dist_sq)
{
    printf("h_rlist_insert open\n");
	struct res_node *rnode;

	if(!(rnode = (struct res_node *)malloc(sizeof(struct res_node)))) {
		return -1;
	}
	rnode->item = item;
	rnode->dist_sq = dist_sq;

	if(dist_sq >= 0.0) {
		while(list->next && list->next->dist_sq < dist_sq) {
			list = list->next;
		}
	}
	rnode->next = list->next;
	list->next = rnode;
	return 0;
}

static void h_clear_results(struct kdres *rset)
{
	struct res_node *tmp, *node = rset->rlist->next;

	while(node) {
		tmp = node;
		node = node->next;
		free(tmp);
	}

	rset->rlist->next = 0;
}


__device__ unsigned int Rand(unsigned int randx)
{
    randx = randx*1103515245+12345;
    return randx&2147483647;
}

static double dist_sq( double *a1, double *a2, int dims ) {
  double dist_sq = 0, diff;
  while( --dims >= 0 ) {
    diff = (a1[dims] - a2[dims]);
    dist_sq += diff*diff;
  }
  return dist_sq;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

__global__ void normalsGPU(float* points,int point_size,int* neighbor_points_indices,int* neighbor_start_indices,int neighbor_points_count,float* normals,float* curvatures,long long int* covariance_time,long long int* eigen_time) {
    // printf("normalsGPU");
    //インデックス取得
    unsigned int ix = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int idx = ix;
    unsigned int output_id=50;
    // printf("idx = %d ,", idx);

    // if(idx==output_id) {
    //     int i, vcount = 50;
    //     void *kd, *set;
    //     printf("inserting %d random vectors... ", vcount);
    //     kd = d_kd_create(3);
    //     printf("kdcreate ok\n");
    //     for(i=0; i<vcount; i++) {
    //         float x, y, z;
    //         /*
    //         x = ((float)Rand(idx) / 4294967295.0f) * 10.0 - 5.0;
    //         y = ((float)Rand(idx) / 4294967295.0f) * 10.0 - 5.0;
    //         z = ((float)Rand(idx) / 4294967295.0f) * 10.0 - 5.0;
    //         */
    //         x = (float)i/(float)vcount *10.0 - 5.0;
    //         y = (float)i/(float)vcount *10.0 - 5.0;
    //         z = (float)i/(float)vcount *10.0 - 5.0;
    //         printf("rand ok\n");
    //         d_kd_insert3f((struct kdtree *)kd, x, y, z, 0);
    //         printf("insert ok\n");
    //     }
    //     printf("kdset ok\n");
    //     set = d_kd_nearest_range3f((struct kdtree *)kd, 0, 0, 0, 1);
    //     printf("range query returned %d items\n", d_kd_res_size((struct kdres *)set));
    //     d_kd_res_free((struct kdres *)set);
    //     d_kd_free((struct kdtree *)kd);
    // }

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
            eigenJacobiMethod(a, eigen_vector, 3);

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

extern void compute_normals(std::vector<std::vector<float>> points_array,std::vector<std::vector<int>> neighbor_points_indices,std::vector<int> neighbor_start_indices,int neighbor_points_count,std::vector<std::vector<float>>& normals_array,std::vector<float>& curvatures_array,std::vector<long long int>& covariance_compute_time,std::vector<long long int>& eigen_compute_time){
	int i, vcount = 50;
	void *kd, *set;
	printf("inserting %d random vectors... ", vcount);
	kd = h_kd_create(3);
	printf("kdcreate ok\n");
	for(i=0; i<vcount; i++) {
		float x, y, z;
		/*
		x = ((float)Rand(idx) / 4294967295.0f) * 10.0 - 5.0;
		y = ((float)Rand(idx) / 4294967295.0f) * 10.0 - 5.0;
		z = ((float)Rand(idx) / 4294967295.0f) * 10.0 - 5.0;
		*/
		x = (float)i/(float)vcount *10.0 - 5.0;
		y = (float)i/(float)vcount *10.0 - 5.0;
		z = (float)i/(float)vcount *10.0 - 5.0;
		printf("rand ok\n");
		h_kd_insert3f((struct kdtree *)kd, x, y, z, 0);
		printf("insert ok\n");
	}
	printf("kdset ok\n");
	set = h_kd_nearest_range3f((struct kdtree *)kd, 0, 0, 0, 1);//tree,pos,radius
	printf("range query returned %d items\n", h_kd_res_size((struct kdres *)set));
	
	char *pch;
	double pos[3], dist;
	double pt[3] = { 0, 0, 0 };//中心のポイント

	while( !h_kd_res_end( (struct kdres *)set ) ) {
		/* get the data and position of the current result item */
		pch = (char*)h_kd_res_item( (struct kdres *)set, pos);

		/* compute the distance of the current result from the pt */
		dist = sqrt( dist_sq( pt, pos, 3 ) );

		/* print out the retrieved data */
		printf( "node at (%.3f, %.3f, %.3f) is %.3f away\n", 
			pos[0], pos[1], pos[2], dist);
		if(pch!=0) printf( "has data=%c\n", *pch);
		// 	pos[0], pos[1], pos[2], dist, *pch );
		// printf( "node at (%.3f, %.3f, %.3f) is %.3f away and has data=%c\n", 
		// 	pos[0], pos[1], pos[2], dist, *pch );

		/* go to the next entry */
		h_kd_res_next((struct kdres *)set);
	}

	h_kd_res_free((struct kdres *)set);
	h_kd_free((struct kdtree *)kd);

    // std::cout<<"3.01"<<std::endl;
    //ホスト1次配列宣言
    std::vector<float> h_points(points_array.size() * 3);
    std::vector<int> h_neighbor_points_indices(neighbor_points_count);
    std::vector<float> h_normals(points_array.size() * 3);
    std::vector<float> h_curvatures(points_array.size());
    std::vector<long long int> h_covariance_compute_time(points_array.size());
    std::vector<long long int> h_eigen_compute_time(points_array.size());
    // std::cout<<"3.02"<<std::endl;
    //デバイス1次配列宣言
    float *d_points,*d_normals,*d_curvatures;
    int *d_neighbor_points_indices,*d_neighbor_start_indices;
    long long int *d_covariance_compute_time,*d_eigen_compute_time;
    // std::cout<<"3.03"<<std::endl;
    //メモリ確保
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
    }
    // std::cout<<"3.05"<<std::endl;
    //コピー
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
    cudaDeviceSetLimit(cudaLimitStackSize, 1024*8);
    //実行
    normalsGPU<<<grid,block>>>(d_points,points_array.size(),d_neighbor_points_indices,d_neighbor_start_indices,neighbor_points_count,d_normals,d_curvatures,d_covariance_compute_time,d_eigen_compute_time);
    // std::cout<<"normalsGPUend"<<std::endl;
    // std::cout<<"3.08"<<std::endl;
    //コピー
    cudaMemcpy(&h_normals[0], d_normals, points_array.size() * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_curvatures[0], d_curvatures, points_array.size() * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_covariance_compute_time[0], d_covariance_compute_time, points_array.size() * sizeof(long long int), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_eigen_compute_time[0], d_eigen_compute_time, points_array.size() * sizeof(long long int), cudaMemcpyDeviceToHost);
    // std::cout<<"3.09"<<std::endl;
    //2次配列化
    k=0;
    for(int i=0;i<points_array.size();i++){//点群
        for(int j=0;j<3;j++){//x,y,z
            normals_array[i][j]=h_normals[k];
            k++;
        }
        curvatures_array[i]=h_curvatures[i];
        covariance_compute_time[i]=h_covariance_compute_time[i];
        eigen_compute_time[i]=h_eigen_compute_time[i];
    }
    // std::cout<<"cu_normals : "<<normals_array[0][0]<<","<<normals_array[0][1]<<","<<normals_array[0][2]<<std::endl;
    // std::cout<<"3.10"<<std::endl;
    //メモリ解放
    cudaFree(d_points);
    cudaFree(d_neighbor_points_indices);
    cudaFree(d_neighbor_start_indices);
    cudaFree(d_normals);
    cudaFree(d_curvatures);
    cudaFree(d_covariance_compute_time);
    cudaFree(d_eigen_compute_time);
}