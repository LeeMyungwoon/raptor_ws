cmake_minimum_required(VERSION 3.18)

# 1. Kineto 관련 경고 억제 플래그 (find_package 전에 선언)
set(TORCH_NO_OPERATOR_REGISTRATION ON)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
#set(CMAKE_CUDA_ARCHITECTURES 89) # RTX 40 시리즈 (데스크탑) 최적화
set(CMAKE_CUDA_ARCHITECTURES 87) # OrinNX 최적화
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PyTorch 경로 설정
list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/python3.10/dist-packages/torch/share/cmake")

project(raptor_camera LANGUAGES CXX CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
  # -Wpedantic은 CXX에만 적용 (CUDA의 nvcc 생성 코드에서 line directive 경고 방지)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>)
  # 특정 경고 무시 추가
  add_compile_options("-Wno-class-memaccess")
  # Eigen 관련 memaccess 경고 확실히 제거
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-class-memaccess>)
  # NVCC가 호스트 컴파일러(gcc)를 호출할 때도 이 플래그를 전달하도록 설정
  add_compile_options($<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-class-memaccess>)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED)

# 헤더 경로 포함
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

add_library(raptor_camera_lib SHARED
  src/raw_camera_capture.cpp
  src/preprocessor_kernels.cu
)

# 의존성 링크 (ROS 2 패키지)
ament_target_dependencies(raptor_camera_lib
  rclcpp
  sensor_msgs
)

# 일반 라이브러리 링크 (CUDA, Torch)
target_link_libraries(raptor_camera_lib
  CUDA::cudart
  ${TORCH_LIBRARIES}
)

# 설치 설정
install(TARGETS raptor_camera_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
