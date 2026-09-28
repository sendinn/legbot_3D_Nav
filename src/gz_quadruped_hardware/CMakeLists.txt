cmake_minimum_required(VERSION 3.8)
project(gz_quadruped_hardware)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C11
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif ()
# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# Compiler options
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(ignition-gazebo6 REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}-system SHARED
        src/gz_quadruped_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}-system
        ignition-gazebo6::ignition-gazebo6
        ignition-plugin1::register
)
ament_target_dependencies(${PROJECT_NAME}-system
        ament_index_cpp
        controller_manager
        hardware_interface
        pluginlib
        rclcpp
        yaml_cpp_vendor
        rclcpp_lifecycle
)

#########

add_library(gz_quadruped_plugins SHARED
        src/gz_system.cpp
)
ament_target_dependencies(gz_quadruped_plugins
        rclcpp_lifecycle
        hardware_interface
        rclcpp
)
target_link_libraries(gz_quadruped_plugins
        ignition-gazebo6::ignition-gazebo6
)

## Install
install(TARGETS
        gz_quadruped_plugins
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY
        include/
        DESTINATION include
)

install(
        DIRECTORY xacro
        DESTINATION share/${PROJECT_NAME}/
)

# Testing and linting
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}-system gz_quadruped_plugins)

# Install directories
install(TARGETS ${PROJECT_NAME}-system
        DESTINATION lib
)

pluginlib_export_plugin_description_file(gz_quadruped_hardware gz_quadruped_hardware.xml)

# Setup the project
ament_package()
