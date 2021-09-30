#include <stdio.h>
#include "cv_cuda/cv_cuda.hpp"

__global__ void cvhello(void) {
    printf("hello cuda !! thredIdx=%d\n", threadIdx.x);
}

void cvcallHello(void) {
    cvhello<<<1, 4>>>();
    cudaDeviceSynchronize();
}