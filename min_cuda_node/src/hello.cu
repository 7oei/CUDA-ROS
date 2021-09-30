#include <stdio.h>
#include "min_cuda_node/hello.hpp"

__global__ void hello(void) {
    printf("hello cuda !! thredIdx=%d\n", threadIdx.x);
}

void callHello(void) {
    hello<<<1, 4>>>();
    cudaDeviceSynchronize();
}