cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(ur3_collisions)

include(GenerateExportHeader)
include(CMakePackageConfigHelpers)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(UR3_COLLISIONS_VERSION 1.0.0)

option(COMPILE_ROBOT_DART_EXAMPLES "Compile Robot DART examples" ON)

find_package(inria_wbc REQUIRED)

list(APPEND DEPENDENCIES_LIBS
    inria_wbc::inria_wbc
)

set(UR3_COLLISIONS_SOURCES
    src/behaviors/ex_behavior.cpp
    src/controllers/ex_controller.cpp
    src/tsid/ex_task.cpp

)

set(EXAMPLE_LIST 
    src/robot_dart/test_controller.cpp
)

add_library(ur3_collisions SHARED ${UR3_COLLISIONS_SOURCES})

target_compile_definitions(ur3_collisions PUBLIC BOOST_MATH_DISABLE_FLOAT128=1)
target_compile_features(ur3_collisions PUBLIC cxx_std_14)

target_link_libraries(ur3_collisions PUBLIC ${DEPENDENCIES_LIBS})

target_include_directories(ur3_collisions
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/build/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)



######################### INSTALLATION #########################################

generate_export_header(ur3_collisions)

install(TARGETS ur3_collisions
        EXPORT targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

configure_package_config_file("cmake/ur3_collisions.cmake.in" "ur3_collisionsConfig.cmake"
                              INSTALL_DESTINATION "lib/cmake/ur3_collisions"
                              PATH_VARS
)

# CHANGE NAMESPACE AS YOU PREFER. The project is now linked as inria_wbc::ur3_collisions
install(EXPORT targets
        FILE ur3_collisionsTargets.cmake
        NAMESPACE inria_wbc::
        DESTINATION lib/cmake/ur3_collisions
)

write_basic_package_version_file("ur3_collisionsConfigVersion.cmake"
                                 VERSION ${UR3_COLLISIONS_VERSION}
                                 COMPATIBILITY AnyNewerVersion
)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/ur3_collisionsConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/ur3_collisionsConfigVersion.cmake"
        DESTINATION lib/cmake/ur3_collisions
)


########################## TEST EXAPLES USING ROBOT DART #########################

function(compile_examples)
    foreach(EXAMPLE ${ARGN})
        string(REPLACE ".cpp" "" EXAMPLE_NAME ${EXAMPLE} )
        string(REPLACE "src/robot_dart/" "" EXAMPLE_NAME ${EXAMPLE_NAME} )
        message("COMPILING EXAMPLE ${EXAMPLE_NAME}")
        add_executable(${EXAMPLE_NAME} ${EXAMPLE})
        target_link_libraries(${EXAMPLE_NAME} PUBLIC ur3_collisions)
        target_link_libraries(${EXAMPLE_NAME} PUBLIC inria_wbc::inria_wbc RobotDART::Simu Boost::program_options)
        if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            set_target_properties(${EXAMPLE_NAME} PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")
            # target_link_options(${EXAMPLE_NAME} PRIVATE "LINKER:--no-as-needed")
        endif()

        if(RobotDART_Magnum_FOUND)
            add_executable(${EXAMPLE_NAME}_graphics ${EXAMPLE})
            if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
                set_target_properties(${EXAMPLE_NAME}_graphics PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")
                # target_link_options(${EXAMPLE_NAME}_graphics PRIVATE "LINKER:--no-as-needed")
            endif()
            target_link_libraries(${EXAMPLE_NAME}_graphics PUBLIC inria_wbc::inria_wbc RobotDART::Simu RobotDART::Magnum Boost::program_options)
            target_link_libraries(${EXAMPLE_NAME}_graphics PUBLIC ur3_collisions)
        endif()
    endforeach()
endfunction()

if(${COMPILE_ROBOT_DART_EXAMPLES} STREQUAL "ON")
    find_package(RobotDART COMPONENTS Simu OPTIONAL_COMPONENTS Magnum)
    find_package(Boost REQUIRED COMPONENTS program_options)
    compile_examples(${EXAMPLE_LIST})
endif()


