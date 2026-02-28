#ifndef RAPTOR_CAMERA__RAW_CAMERA_CAPTURER_HPP_
#define RAPTOR_CAMERA__RAW_CAMERA_CAPTURER_HPP_

#include <vector>
#include <string>
#include <memory>
#include <linux/videodev2.h>
#include "raptor_camera/preprocess_config.hpp"

namespace raptor_camera {

void launch_preprocess(uint16_t* d_input, float* d_output, float* d_gain_maps, float* d_rect_maps,
                        int src_w, int src_h, int dst_w, int dst_h, int cam_count);

class RawCameraCapturer {
public:
    RawCameraCapturer(const PreprocessConfig& config);
    ~RawCameraCapturer();

    // 7개 카메라의 프레임을 캡처하여 CUDA 메모리 포인터로 반환 (uint16_t)
    uint16_t* capture_frame(int timeout_ms = 5);
    float* get_processed_buffer() { return d_processed_buffer_; }

private:
    bool setup_v4l2();
    // void init_dmabuf();

    int fds_[7];                        // V4L2 파일 디스크립터
    void* user_buffers_[7][4];          // 7개 카메라 x 4개 버퍼 주소 저장 (CPU용)
    // int dmabuf_fds_[7][4];              // DMABUF 파일 디스크립터 (GPU)

    size_t num_cameras_;
    int epoll_fd_;
    int src_width_, src_height_;
    int dst_width_, dst_height_;
    uint16_t* d_raw_buffer_;            // GPU Raw 버퍼
    uint16_t* d_raw_prev_buffer_;       // Timeout 대비용 이전 프레임
    float* d_gain_maps_;                // 7개 카메라용 Vignetting 보정 Gain Map
    float* d_rect_maps_;                // 7개 카메라용 Rectification 보정 Map
    float* d_processed_buffer_;         // 전처리 완료된 Float 버퍼 (640 x 480 크기)
};

} // namespace raptor_camera

#endif // RAPTOR_CAMERA__RAW_CAMERA_CAPTURER_HPP_
