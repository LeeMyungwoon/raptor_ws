#ifndef RAPTOR_CAMERA__PREPROCESS_CONFIG_HPP_
#define RAPTOR_CAMERA__PREPROCESS_CONFIG_HPP_

#include <string>
#include <vector>
#include <array>
#include <Eigen/Dense>

namespace raptor_camera {

struct CalibrationParams {
    Eigen::Matrix3d K;              // 원본 Intrinsic
    std::vector<double> D;          // 왜곡 계수
    Eigen::Matrix4d T_cam_robot;    // Extrinsic (카메라 -> 로봇)
    Eigen::Matrix3d new_K;          // 보정 후 Intrinsic (Crop/Resize 반영)
};

struct PreprocessConfig {
    std::vector<std::string> device_paths; // /dev/video0 ~ 6
    int orig_width = 1280;
    int orig_height = 800;
    int target_width = 640;
    int target_height = 480;
    int black_level = 64;           // 센서 실측값 기반
    std::vector<std::string> gain_map_paths;        // gain map 파일 경로
    std::vector<std::string> rectify_map_paths;     // rectify map 파일 경로
    std::array<CalibrationParams, 7> calib_params;  // 7개 카메라 calibration 파라미터
};

} // namespace raptor_camera

#endif // RAPTOR_CAMERA__PREPROCESS_CONFIG_HPP_
