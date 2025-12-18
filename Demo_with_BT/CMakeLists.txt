cmake_minimum_required(VERSION 3.16)
project(home_demo)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# Nav2/BT 常用的包名（如你环境实际是 behaviortree_cpp，请改回去）
find_package(behaviortree_cpp REQUIRED)
find_package(qrb_ros_amr_msgs REQUIRED)

# 库：不包含 main.cpp
add_library(home_nodes SHARED
    src/common/config.cpp
    src/nodes/battery_ok.cpp
    src/nodes/dock_and_charge.cpp
    src/nodes/relocate.cpp
    src/nodes/detect_fall.cpp
    src/nodes/tts.cpp
    src/nodes/wait_seconds.cpp
    src/nodes/navigateTo.cpp
    src/nodes/command_conditions.cpp
)

target_include_directories(home_nodes PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(home_nodes
    rclcpp
    rclcpp_action
    sensor_msgs
    geometry_msgs
    nav2_msgs
    qrb_ros_amr_msgs
    behaviortree_cpp
)

# 可执行主程序
add_executable(bt_main
    src/main.cpp
)

target_include_directories(bt_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(bt_main
    rclcpp
    rclcpp_action
    sensor_msgs
    geometry_msgs
    nav2_msgs
    behaviortree_cpp
    qrb_ros_amr_msgs
)

target_link_libraries(bt_main
    home_nodes
)

# 安装
install(TARGETS home_nodes bt_main
    EXPORT export_home_nodes
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

# （可选）导出以供其他包使用
ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp
    rclcpp_action
    sensor_msgs
    geometry_msgs
    nav2_msgs
    behaviortree_cpp
)
ament_export_targets(export_home_nodes)

ament_package()

