cmake_minimum_required(VERSION 3.18) # setting this is required
project(orthosfm CXX) # this sets the project name

# Global Project Settings
option(ENABLE_CUDA "Enables CUDA based feature extraction" OFF)
option(ENABLE_TESTBENCH "Builds the testbench used for evaluation of the software" OFF)


# Also enable CUDA
if(ENABLE_CUDA)
    add_definitions(-DENABLE_CUDA)
    enable_language(CUDA)
    add_subdirectory(src/cuda_sift)
endif()

# Add libraries
add_subdirectory(src/algorithms)
add_subdirectory(src/bundle_adjustment)
add_subdirectory(src/data_structures)
add_subdirectory(src/matching)
add_subdirectory(src/mve)
add_subdirectory(src/sfm)
add_subdirectory(src/triangulation)
add_subdirectory(src/util)

# Add Applications
add_subdirectory(src/app)

if(ENABLE_TESTBENCH)
    add_subdirectory(src/testbench)
endif()

# Find the resources and copy them to the local build folders
file(GLOB_RECURSE data resources/*)
file(COPY ${data} DESTINATION resources)
# install(DIRECTORY resources DESTINATION example_destination) # MAYBE?
