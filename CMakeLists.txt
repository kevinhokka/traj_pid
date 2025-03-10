cmake_minimum_required(VERSION 3.8)
project(traj_pid)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖

#export LD_LIBRARY_PATH=/home/jetson/ros2_ctl/install/planner/lib:$LD_LIBRARY_PATH

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(OpenCV REQUIRED)



set(TRAJ_INTERFACES traj_pid)

set(msg_files
  "msg/Bspline.msg"
)

# rosidl_generate_interfaces(
#   ${TRAJ_INTERFACES}
#   ${msg_files}
#   DEPENDENCIES std_msgs geometry_msgs builtin_interfaces planner
#   LIBRARY_NAME ${PROJECT_NAME}
# )

find_package(planner REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces planner
)



ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  builtin_interfaces
  planner
  rosidl_typesupport_cpp
)


# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译目标
add_executable(traj_pid_node src/traj_pid_node.cpp)

# 链接依赖库
ament_target_dependencies(
  traj_pid_node
  rclcpp
  geometry_msgs
  nav_msgs  
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosidl_typesupport_cpp
  builtin_interfaces
  OpenCV
  planner
)


set(TRAJ_INTERFACES traj_pid)

set(
  PLANNER_INTERFACE
  "planner"
)

# rosidl_get_typesupport_target(
#   cpp_typesupport_target
#   ${PLANNER_INTERFACE}
#   "rosidl_typesupport_cpp"
# )


target_link_libraries(
  traj_pid_node
  /home/jetson/ros2_ws/install/planner/lib/libplanner__rosidl_typesupport_cpp.so
  ${rosidl_typesupport_cpp_LIBRARIES}
)

# jetson@jetson:~/ros2_ctl$ echo $LD_LIBRARY_PATH
# /home/jetson/ros2_ctl/install/planner/lib:/home/jetson/ros2_ctl/install/traj_pid_node/lib:/home/jetson/ros2_ctl/install/traj_pid/lib:/home/jetson/ros2_ctl/install/serial/lib:/home/jetson/ros2_ctl/install/planner/lib:/usr/local/cuda/lib64:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/usr/local/cuda/lib64:/usr/local/lib:/usr/local/lib
# jetson@jetson:~/ros2_ctl$ 


target_link_libraries(
  traj_pid_node
  "${cpp_typesupport_target}")


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
