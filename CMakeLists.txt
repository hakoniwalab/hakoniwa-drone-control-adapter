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

option(
    HAKONIWA_DRONE_CONTROL_ADAPTER_BUILD_TESTS
    "Build standalone control-adapter contract tests"
    OFF
)

if(HAKONIWA_DRONE_CONTROL_ADAPTER_BUILD_TESTS)
    enable_testing()

    add_executable(
        control_allocation_input_contract_test
        test/control_allocation_input_contract.cpp
    )
    target_link_libraries(
        control_allocation_input_contract_test
        PRIVATE
            hakoniwa_drone_control_adapter
    )

    add_test(
        NAME control_allocation_input_contract
        COMMAND control_allocation_input_contract_test
    )
endif()
