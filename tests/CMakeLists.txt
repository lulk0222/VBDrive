cmake_minimum_required(VERSION 3.22)
project(VBDriveHostTests LANGUAGES C CXX)

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(cyphal_register_helpers_test
    cyphal_register_helpers_test.cpp
)

target_include_directories(cyphal_register_helpers_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../Drivers/libcxxcanard
    ${CMAKE_CURRENT_LIST_DIR}/../Drivers/libcxxcanard/libs
)

add_test(NAME cyphal_register_helpers_test COMMAND cyphal_register_helpers_test)
