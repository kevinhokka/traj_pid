cmake_minimum_required(VERSION 3.8)
project(traj_pid_node)

set(CMAKE_PREFIX_PATH "~/ws_livox/install" ${CAMKE_PREFIX_PATH})

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(planner REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译目标
add_executable(traj_pid_node src/traj_pid_node.cpp)

# 链接依赖库
ament_target_dependencies(traj_pid_node
  rclcpp
  geometry_msgs
  nav_msgs  
  sensor_msgs
  planner
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 安装可执行文件
install(TARGETS
  traj_pid_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 编译测试时的配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过检查版权的linter
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过cpplint（仅在git仓库中有效）
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装包
ament_package()
