#include "raptor_camera/raw_camera_capture.hpp"
#include "raptor_camera/cuda_utils.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <iostream>
#include <cuda_runtime.h>
#include <fstream>

namespace raptor_camera {

RawCameraCapturer::RawCameraCapturer(const PreprocessConfig& config)
    : src_width_(config.orig_width), src_height_(config.orig_height), 
    dst_width_(config.target_width), dst_height_(config.target_height) {
    
    // user_buffers 초기화
    memset(user_buffers_, 0, sizeof(user_buffers_)); 

    // 초기화
    memset(fds_, -1, sizeof(fds_));
    num_cameras_ = config.device_paths.size();
    if (num_cameras_ > 7) num_cameras_ = 7; 

    size_t num_pixels_src = (size_t)num_cameras_ * src_width_ * src_height_;
    size_t num_pixels_dst = (size_t)num_cameras_ * dst_width_ * dst_height_;

    CUDA_CHECK(cudaMalloc(&d_raw_buffer_, num_pixels_src * sizeof(uint16_t)));
    CUDA_CHECK(cudaMalloc(&d_raw_prev_buffer_, num_pixels_src * sizeof(uint16_t)));
    CUDA_CHECK(cudaMalloc(&d_processed_buffer_, num_pixels_dst * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_gain_maps_, num_pixels_src * sizeof(float))); 
    CUDA_CHECK(cudaMalloc(&d_rect_maps_, num_pixels_dst * 2 * sizeof(float))); 

    CUDA_CHECK(cudaMemset(d_raw_prev_buffer_, 0, num_pixels_src * sizeof(uint16_t))); 

    // CPU gain map
    std::vector<float> h_gain_map(num_pixels_src, 1.0f);
    for (size_t i = 0; i < num_cameras_; i++) {
        if (i < config.gain_map_paths.size() && !config.gain_map_paths[i].empty()) {
            std::ifstream ifs(config.gain_map_paths[i], std::ios::binary);
            if (ifs.is_open()) {
                ifs.read(reinterpret_cast<char*>(&h_gain_map[i * src_width_ * src_height_]), src_width_ * src_height_ * sizeof(float));
            }
            else std::cerr << "[Warning] Gain map file open failed for camera " << i << std::endl;
        }
        else std::cerr << "[Warning] Gain map path is empty for camera " << i << std::endl;
    }
    cudaMemcpy(d_gain_maps_, h_gain_map.data(), num_pixels_src * sizeof(float), cudaMemcpyHostToDevice);

    // CPU rectify map
    std::vector<float> h_rect_map(num_pixels_dst * 2, 0.0f); // 각 픽셀당 (u, v) 좌표 2개씩
    for (size_t i = 0; i < num_cameras_; i++) {
        if (i < config.rectify_map_paths.size() && !config.rectify_map_paths[i].empty()) {
            std::ifstream ifs(config.rectify_map_paths[i], std::ios::binary);
            if (ifs.is_open()) {
                ifs.read(reinterpret_cast<char*>(&h_rect_map[i * dst_width_ * dst_height_ * 2]), 
                        dst_width_ * dst_height_ * 2 * sizeof(float));
            }
        }
    }
    cudaMemcpy(d_rect_maps_, h_rect_map.data(), num_pixels_dst * 2 * sizeof(float), cudaMemcpyHostToDevice);

    // epoll 인스턴스 생성
    epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);

    // epoll 이벤트 등록
    for (size_t i = 0; i < num_cameras_; i++) {
        const std::string& path = config.device_paths[i]; 

        int fd = open(path.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) {
            std::cerr << "[Error] Fail to connect camera " << i << ": " << path << std::endl;
            continue; // 일단 다른 카메라도 시도하여 로그를 남김
        }
        fds_[i] = fd;

        struct epoll_event ev;
        ev.events = EPOLLIN;
        ev.data.u32 = (uint32_t)i;

        epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev); 
    }

    if (!setup_v4l2()) {
        exit(EXIT_FAILURE); // 설정 실패 시 시작 안 함
    }
    // init_dmabuf();
}

RawCameraCapturer::~RawCameraCapturer() {
    // mmap으로 연결된 통로 먼저 해제
    size_t size = sizeof(fds_) / sizeof(fds_[0]);
    for (size_t cam_idx = 0; cam_idx < size; cam_idx++) {
        for (int buf_idx = 0; buf_idx < 4; buf_idx++) {
            if (user_buffers_[cam_idx][buf_idx]) {
                // RAW12 데이터 크기만큼 매핑 해제
                munmap(user_buffers_[cam_idx][buf_idx], src_width_ * src_height_ * 2); // 16비트 -> 2바이트
            }
        }
    }

    for (int fd : fds_) close(fd); // 카메라 연결 끊기
    close(epoll_fd_); // epoll 인스턴스 해제
    cudaFree(d_raw_buffer_); // GPU 메모리 반납
    cudaFree(d_raw_prev_buffer_);
    cudaFree(d_processed_buffer_);
    cudaFree(d_gain_maps_);
    cudaFree(d_rect_maps_);
}

bool RawCameraCapturer::setup_v4l2() {
    size_t success_cnt = 0;
    for (size_t i = 0; i < num_cameras_; i++) {
        int fd = fds_[i];
        if (fd < 0) continue; // 열리지 않은 카메라는 건너뜀

        // 영상 포맷 설정
        struct v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = src_width_;
        fmt.fmt.pix.height = src_height_;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_Y12;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        
        // 드라이버 요청
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            std::cerr << "[Warning] Camera " << i << " does not recognize RAW12." << std::endl;
            continue;
        }

        // 결과확인
        if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_Y12) {
            std::cerr << "[Warning] The camera rejected the RAW12 request and switched to another format." << std::endl;
            continue;
        }

        // v4l2 버퍼설정 4개 예약
        struct v4l2_requestbuffers req = {};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;  // cpu zero-copy 방식

        // 버퍼 요청
        if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            std::cerr << "[Error] Failed to request buffers for camera " << i << std::endl;
            return false;
        }

        // 예약된 빈 버퍼 4개 카메라에게 전달
        for (int j = 0; j < 4; j++) {
            struct v4l2_buffer init_buf = {};
            init_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            init_buf.memory = V4L2_MEMORY_MMAP;
            init_buf.index = (uint32_t)j;
            if (ioctl(fd, VIDIOC_QBUF, &init_buf) < 0) {
                std::cerr << "[Warning] Initial QBUF failed for camera " << i << ", buffer " << j << std::endl;
            }
            
            // 버퍼 정보 얻기 (실제 주소)
            struct v4l2_buffer query_buf = {};
            query_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            query_buf.memory = V4L2_MEMORY_MMAP;
            query_buf.index = (uint32_t)j;
            ioctl(fd, VIDIOC_QUERYBUF, &query_buf);
            user_buffers_[i][j] = mmap(NULL, query_buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, query_buf.m.offset);
        }

        // Streaming ON
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            std::cerr << "[Error] STREAMON failed for camera " << i << std::endl;
            return false;
        }

        success_cnt++;
    }   

    std::cout << "[Info] All " << success_cnt << " cameras successfully configured." << std::endl;
    return (success_cnt == num_cameras_); // 모든 카메라가 성공해야 true
}

uint16_t* RawCameraCapturer::capture_frame(int timeout_ms) {
    size_t ready_cameras = 0;
    bool camera_received[7] = {false}; // 어떤 카메라가 왔는지 체크

    while (ready_cameras < 7) {
        struct epoll_event events[7];
        int n = epoll_wait(epoll_fd_, events, 7 - ready_cameras, timeout_ms);
        
        if (n <= 0) break; // 시간초과

        for (int i = 0; i < n; i++) {
            int cam_idx = events[i].data.u32;
            int fd = fds_[cam_idx];

            // 커널요청
            struct v4l2_buffer buf = {};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            // 사진이 담긴 바구니 꺼내기
            if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) continue;

            // 이미 받은 카메라 데이터
            if (camera_received[cam_idx]) {
                // 바구니만 비워서 다시 돌려주고, 이번 턴 사용 x
                ioctl(fd, VIDIOC_QBUF, &buf);
                continue;
            }

            // int dma_fd = dmabuf_fds_[cam_idx][buf.index];
            void* src_ptr = user_buffers_[cam_idx][buf.index];
            cudaMemcpy(d_raw_buffer_ + (cam_idx * src_width_ * src_height_), 
                        src_ptr, src_width_ * src_height_ * sizeof(uint16_t), 
                        cudaMemcpyHostToDevice);

            camera_received[cam_idx] = true;
            ready_cameras++;

            // QBUF (Queue Buffer) 커널에 빈 버퍼 반납
            if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                std::cerr << "[Warning] QBUF failed." << std::endl;
            }
        }
    }

    // 모든 카메라가 다 모였으면(num_cameras_) 결과 반환, 아니면 이전 프레임 반환
    if (ready_cameras == num_cameras_) {
        launch_preprocess(d_raw_buffer_, d_processed_buffer_, d_gain_maps_, d_rect_maps_, 
                            src_width_, src_height_, dst_width_, dst_height_, num_cameras_);

        // 백업
        cudaMemcpy(d_raw_prev_buffer_, d_raw_buffer_, 
                   (size_t)num_cameras_ * src_width_ * src_height_ * sizeof(uint16_t), 
                   cudaMemcpyDeviceToDevice);

        return d_raw_buffer_;
    } else {
        std::cerr << "[Warning] Capture timeout. Only " << ready_cameras << " / " << num_cameras_ << " cameras synced." << std::endl;

        return d_raw_prev_buffer_;
    }
}

// void RawCameraCapturer::init_dmabuf() {
//     for (size_t cam_idx = 0; cam_idx < fds_.size(); cam_idx++) {
//         for (int buf_idx = 0; buf_idx < 4; buf_idx++) {
//             struct v4l2_exportbuffer expbuf = {};
//             expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//             expbuf.index = buf_idx;

//             // 커널 fd변환 요청
//             if (ioctl(fds_[cam_idx], VIDIOC_EXPBUF, &expbuf) < 0) {
//                 std::cerr << "[Critical] Failed to export DMABUF for camera " << cam_idx << std::endl;
//                 exit(EXIT_FAILURE);
//             }

//             dmabuf_fds_[cam_idx][buf_idx] = expbuf.fd;
//         }
//     }
// }

} // namespace raptor_camera
