cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(ur3_collisions)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(UR3_COLLISIONS_VERSION 1.0.0)

find_package(inria_wbc REQUIRED)
find_package(RobotDART COMPONENTS Simu Magnum)
find_package(Utheque)

list(APPEND DEPENDENCIES_LIBS
    inria_wbc::inria_wbc
)

add_executable(test_controller src/test_controller.cpp)
target_link_libraries(test_controller PUBLIC inria_wbc::inria_wbc RobotDART::Simu  RobotDART::Magnum Utheque )
if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set_target_properties(test_controller PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")
endif()

