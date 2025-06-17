cmake_minimum_required(VERSION 3.16)
project(QtPlotter LANGUAGES CXX)

# Find required Qt modules, add PrintSupport
find_package(Qt5 REQUIRED COMPONENTS Widgets PrintSupport)
# For Qt6:
# find_package(Qt6 REQUIRED COMPONENTS Widgets PrintSupport)

add_library(QtPlotter STATIC
    qcustomplot.cpp
)

target_include_directories(QtPlotter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(QtPlotter PUBLIC Qt5::Widgets Qt5::PrintSupport)
# For Qt6:
# target_link_libraries(QtPlotter PUBLIC Qt6::Widgets Qt6::PrintSupport)
