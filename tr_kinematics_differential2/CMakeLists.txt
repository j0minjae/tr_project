cmake_minimum_required(VERSION 3.5)
project(tr_kinematics_differential2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # C++17을 사용하는 것이 일반적입니다.
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package 목록에 누락된 geometry_msgs를 추가합니다.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 라이브러리(Kinematics)를 먼저 정의합니다.
add_library(${PROJECT_NAME} SHARED
  src/DiffDrive2WKinematics.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# 라이브러리가 의존하는 패키지들을 명시합니다.
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_msgs"
  "trajectory_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# 실행 파일(Node)을 정의합니다.
add_executable(tr_differential_node src/tr_kinematics_differential_node.cpp)

# 아래 라인을 추가하여 실행 파일이 헤더 파일을 찾을 수 있도록 합니다.
target_include_directories(tr_differential_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 실행 파일이 의존하는 패키지들을 명시합니다.
ament_target_dependencies(tr_differential_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_msgs"
  "trajectory_msgs"
  "tf2_ros"
)

# 실행 파일이 위에서 만든 라이브러리를 사용하도록 링크합니다.
target_link_libraries(tr_differential_node ${PROJECT_NAME})

# 설치(install) 규칙들을 정의합니다.
install(TARGETS
  ${PROJECT_NAME}
  tr_differential_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ament_export_dependencies를 사용하여 다른 패키지가 이 패키지를 사용할 때 필요한 의존성을 알려줍니다.
ament_export_dependencies(
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_msgs"
  "trajectory_msgs"
  "tf2_ros"
)

ament_package()