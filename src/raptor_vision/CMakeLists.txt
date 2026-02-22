cmake_minimum_required(VERSION 3.18)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(CMAKE_CUDA_ARCHITECTURES 89) # RTX 40 시리즈 (데스크탑) 최적화
#set(CMAKE_CUDA_ARCHITECTURES 87) # OrinNX 최적화
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/python3.10/dist-packages/torch/share/cmake")

project(raptor_vision LANGUAGES CXX CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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

add_library(raptor_vision_lib SHARED
  src/raw_camera_capture.cpp
  src/preprocessor_kernels.cu
)

# 의존성 링크 (ROS 2 패키지)
ament_target_dependencies(raptor_vision_lib
  rclcpp
  sensor_msgs
)

# 일반 라이브러리 링크 (CUDA, Torch)
target_link_libraries(raptor_vision_lib
  CUDA::cudart
  ${TORCH_LIBRARIES}
)

# 설치 설정
install(TARGETS raptor_vision_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
