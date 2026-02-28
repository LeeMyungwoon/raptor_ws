# Changelog

---
## [v.0.0.2]
## [2026-02-29]
### Architecture
- IMU package 생성
- VectorNav SDK를 ThirdParty로 추가
- raptor_msgs 패키지 생성
- raptor_vision -> raptor_camera로 변경
### Implementation
- Binary + 400hz(800/2) 설정
- 표준 sensor_msgs/Imu + raptor_msgs/TimeGroup 퍼블리시
### Build System
- 빌드 경고 제거
  - CUDA line directive
    - CUDA에만 -Wpedantic 빼서 해결
  - VectorNav maybe_unused
    - SYSTEM include로 "외부 라이브러리야" 알려줘서 해결
  - CMP0148
    - CMake policy OLD 정책 설정으로 해결
  - ThirdParty 빌드
    - COLCON_IGNORE로 설정
  - int -> size_t 변환

## [v.0.0.1]
## [2026-02-22]
### Architecture
- CUDA 에러 확인 유틸 모듈 (`cuda_utils.hpp`) 추가
- RAW 영상 처리용 kernel (`preprocessor_kernel.cu`) 초기 구현
- RawCameraCapture 클래스 구조 설계 및 분리
- (`raw_camera_capture.hpp`, `raw_camera_capture.cpp`) 초기 구현
- (`preprocess_config.hpp`) 초기 구현
### Implementation
- CUDA texture object(texObj) 기반 메모리 접근 방식 적용
  - 2D spatial locality 최적화를 위한 설계
- DMA buffer 관련 코드 주석 처리
  - Desktop + Jetson Orin 공통 환경 지원을 위해 현재는 비활성화
  - 향후 zero-copy 구조로 확장 예정
### Build System
- CMakeLists.txt CUDA 컴파일 옵션 추가
- package.xml dependency 정리