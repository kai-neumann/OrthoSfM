# Define the library
add_library(orthosfm-ba bundle_adjustment.h bundle_adjustment.cpp)

# Set some target specific compile flags
set_target_properties(orthosfm-ba PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(orthosfm-ba PRIVATE ${OpenMP_CXX_FLAGS})

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(orthosfm-ba PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find external required libraries
find_package(Ceres CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# Define the dependencies
target_link_libraries(orthosfm-ba
        PRIVATE Ceres::ceres Eigen3::Eigen orthosfm-data)