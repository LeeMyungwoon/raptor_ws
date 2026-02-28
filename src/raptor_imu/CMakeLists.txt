cmake_minimum_required(VERSION 3.16)
project(raptor_imu)

# C++17 (VectorNav SDK 요구사항)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# ─── Dependencies ───
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(raptor_msgs REQUIRED)

# ─── VectorNav SDK (ThirdParty) ───
set(VN_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ThirdParty/vectornav)

# SYSTEM으로 지정하면 GCC가 이 경로의 헤더 경고를 모두 무시
include_directories(SYSTEM ${VN_SDK_DIR}/include)
include_directories(include)

# oVnSensor object library 빌드 (SDK src/CMakeLists.txt 사용)
if(NOT TARGET oVnSensor)
  add_subdirectory(${VN_SDK_DIR}/src oVnSensor)
endif()

# SDK 빌드 호환을 위한 GNSS 그룹 매크로 (SDK 내부에서 참조)
target_compile_definitions(oVnSensor PUBLIC GNSS_GROUP_ENABLE=0xFFFFFFFF)
target_compile_definitions(oVnSensor PUBLIC GNSS2_GROUP_ENABLE=0xFFFFFFFF)

# SDK 자체 소스 빌드 시 경고 억제 + include 경로를 SYSTEM으로 재지정
target_compile_options(oVnSensor PRIVATE -Wno-attributes)
target_include_directories(oVnSensor SYSTEM PUBLIC ${VN_SDK_DIR}/include)

# ─── vn200_node 실행파일 ───
add_executable(vn200_node
  src/vn200_driver.cpp
  src/vn200_node.cpp
)

ament_target_dependencies(vn200_node
  rclcpp
  sensor_msgs
  geometry_msgs
  raptor_msgs
)

target_link_libraries(vn200_node oVnSensor)

# ─── Install ───
install(TARGETS
  vn200_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
