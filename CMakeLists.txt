cmake_minimum_required(VERSION 3.10)
project(VehicleTrajectoryControl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)
include_directories(C:/CppLibs/include)  # Adjust path as needed

# Find required packages
# find_package(Eigen3 REQUIRED)  # If using Eigen for future MPC

# Source files
set(SOURCES
    sim/pure_pursuit_sim.cpp
)

# Executables
add_executable(pure_pursuit_sim ${SOURCES})

# Link libraries if needed
# target_link_libraries(pure_pursuit_sim Eigen3::Eigen)

# Compiler flags
if(MSVC)
    target_compile_options(pure_pursuit_sim PRIVATE /W4)
else()
    target_compile_options(pure_pursuit_sim PRIVATE -Wall -Wextra -Wno-subobject-linkage)
endif()