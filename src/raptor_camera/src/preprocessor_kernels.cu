#include "raptor_camera/cuda_utils.hpp"
#include <cuda_runtime.h>

namespace raptor_camera {

__global__ void preprocess_kernel(cudaTextureObject_t texObj, float* d_output, 
                                    float* d_gain_maps, float* d_rect_map,
                                    int src_w, int src_h, int dst_w, int dst_h, int black_level) {

    int x = blockIdx.x * blockDim.x + threadIdx.x; // 640 범위
    int y = blockIdx.y * blockDim.y + threadIdx.y; // 480 범위
    int cam_id = blockIdx.z;

    if (x < dst_w && y < dst_h) {
        // 왜곡 보정 지도에서 원본 좌표(u, v)를 가져오기
        int map_idx = (cam_id * dst_w * dst_h + y * dst_w + x) * 2;
        float u = d_rect_map[map_idx];
        float v = d_rect_map[map_idx + 1];

        // tex2D 픽셀뽑기
        float raw_val = tex2D<uint16_t>(texObj, u, v + (cam_id * src_h));

        // Black Level 제거
        int src_idx = (cam_id * src_w * src_h) + ((int)v * src_w + (int)u);
        float gain = d_gain_maps[src_idx];
        float corrected = (raw_val > black_level) ? (float)(raw_val - black_level) : 0.0f;

        // 선형화 + Vignetting 결과(640 x 480)
        d_output[cam_id * dst_w * dst_h + y * dst_w + x] = (corrected / (4095.0f - black_level)) * gain;
    }
}

void launch_preprocess(uint16_t* d_input, float* d_output, float* d_gain_maps, float* d_rect_maps,
                        int src_w, int src_h, int dst_w, int dst_h, int cam_count) {
    // 텍스처 리소스
    struct cudaResourceDesc resDesc = {};
    memset(&resDesc, 0, sizeof(resDesc));
    resDesc.resType = cudaResourceTypePitch2D;
    resDesc.res.pitch2D.devPtr = d_input;
    resDesc.res.pitch2D.desc = cudaCreateChannelDesc<uint16_t>();
    resDesc.res.pitch2D.width = src_w;
    resDesc.res.pitch2D.height = src_h * cam_count; // 전체 세로 길이 (800 * 7)
    resDesc.res.pitch2D.pitchInBytes = src_w * sizeof(uint16_t);

    // 텍스처 속성
    struct cudaTextureDesc texDesc = {};
    memset(&texDesc, 0, sizeof(texDesc));
    texDesc.readMode = cudaReadModeElementType; // 원래 타입으로 읽기
    // cudaFilterModePoint 기본
    texDesc.filterMode = cudaFilterModeLinear; // Bilinear Interpolation (픽셀 4개 참조)
    texDesc.addressMode[0] = cudaAddressModeClamp; // 이미지 밖을 참조하면 가장자리 색으로 채움 (Clamp)
    texDesc.addressMode[1] = cudaAddressModeClamp;
    texDesc.normalizedCoords = 0; // (u, v)

    // texObj 생성
    cudaTextureObject_t texObj = 0;
    CUDA_CHECK(cudaCreateTextureObject(&texObj, &resDesc, &texDesc, NULL));

    // block, grid생성
    dim3 block(16, 16);
    dim3 grid((dst_w + block.x - 1) / block.x, (dst_h + block.y - 1) / block.y, cam_count);
    int black_level = 64;
    
    // 커널 실행
    preprocess_kernel<<<grid, block>>>(texObj, d_output, d_gain_maps, d_rect_maps, 
                                        src_w, src_h, dst_w, dst_h, black_level);

    // 에러 체크
    CUDA_CHECK(cudaGetLastError());
    // texObj 해제
    CUDA_CHECK(cudaDestroyTextureObject(texObj));
}

}
