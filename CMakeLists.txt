cmake_minimum_required(VERSION 3.16)

project(hakoniwa_drone_control_adapter LANGUAGES CXX)

add_library(hakoniwa_drone_control_adapter INTERFACE)

target_include_directories(hakoniwa_drone_control_adapter
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(hakoniwa_drone_control_adapter
    INTERFACE
        cxx_std_17
)
