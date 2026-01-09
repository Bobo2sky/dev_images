cmake_minimum_required(VERSION 3.8)
project(image_saver_cpp)

# ========== 新增：设置 CMP0074 策略，解决 PCL_ROOT 警告 ==========
# 检查 CMake 版本是否支持 CMP0074（3.12+），兼容低版本
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# 强制使用C++17（解决filesystem编译错误）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 显式指定C++扩展为OFF，避免依赖编译器特定扩展
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
# 明确指定 PCL 组件，避免冗余依赖
find_package(PCL REQUIRED COMPONENTS common io)

# 编译可执行文件
add_executable(save_compressed_image src/save_compressed_image.cpp)
ament_target_dependencies(
  save_compressed_image
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  pcl_conversions
)

# 链接 PCL 库（ROS 2 推荐用 target_link_libraries 而非直接变量）
target_include_directories(save_compressed_image PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(save_compressed_image ${PCL_LIBRARIES})
target_compile_definitions(save_compressed_image PRIVATE ${PCL_DEFINITIONS})

# 安装可执行文件
install(TARGETS
  save_compressed_image
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()