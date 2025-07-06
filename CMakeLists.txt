cmake_minimum_required(VERSION 3.10)
project(quadrotor_teleoperation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Add compiler flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(include)
# Find BehaviorTree.CPP
find_package(behaviortree_cpp REQUIRED)
if(NOT behaviortree_cpp_FOUND)
    message(FATAL_ERROR "BehaviorTree.CPP not found. Please install it first.")
endif()

# Find pthread
find_package(Threads REQUIRED)


# Add executable
add_executable(quadrotor_teleoperation
        main.cpp src/teleop.cpp
        src/common.hpp
)
# Link libraries
target_link_libraries(quadrotor_teleoperation
        behaviortree_cpp
        Threads::Threads
)

# Install target
install(TARGETS quadrotor_teleoperation
        RUNTIME DESTINATION bin
)
