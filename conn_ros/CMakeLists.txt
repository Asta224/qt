cmake_minimum_required(VERSION 3.8)
project(conn_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  
find_package(tf2_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# find_package(rosidl_typesupport_cpp REQUIRED)
find_package(turtlesim REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

link_directories(/opt/ros/humble/lib)

add_executable(${PROJECT_NAME}
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    image.qrc
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    rclcpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs 
    geometry_msgs
    sensor_msgs
    nav_msgs
    turtlesim
    tf2_msgs
    diagnostic_msgs
)
ament_export_dependencies(rclcpp std_msgs conn_ros Qt5)
ament_export_targets(${PROJECT_NAME} HAS_EXECUTABLE_TARGET)
