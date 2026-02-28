#ifndef RAPTOR_CAMERA__CUDA_UTILS_HPP_
#define RAPTOR_CAMERA__CUDA_UTILS_HPP_

#include <cuda_runtime.h>
#include <iostream>

// __FILE__ 파일이름
// __LINE__ 줄번호
// __func__ 함수이름
#define CUDA_CHECK(call) \
    do { \
        cudaError_t error = call; \
        if (error != cudaSuccess) { \
            std::cerr << "[CUDA error] " << cudaGetErrorString(error) \
                    << "at " << __FILE__ << " : " << __LINE__ << std::endl; \
            exit(EXIT_FAILURE); \
        } \
    } while (0)

#endif // RAPTOR_CAMERA__CUDA_UTILS_HPP_
