cmake_minimum_required(VERSION 3.8)
project(ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
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
    action.cpp
    action.h
    action.ui
    service.h
    service.cpp
    service.ui
    topic.h
    topic.cpp
    topic.ui
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    rclcpp
    turtlesim::turtlesim__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    geometry_msgs
    rosidl_typesupport_cpp
    turtlesim
)
ament_export_dependencies(rclcpp std_msgs geometry_msgs turtlesim rosidl_typesupport_cpp Qt5)
ament_export_targets(${PROJECT_NAME} HAS_EXECUTABLE_TARGET)
