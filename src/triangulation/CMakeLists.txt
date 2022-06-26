# Define the library
add_library(orthosfm-triangulation triangulation.h triangulation.cpp outlier_filtering.h outlier_filtering.cpp)

# Set some target specific compile flags
set_target_properties(orthosfm-triangulation PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(orthosfm-triangulation PRIVATE ${OpenMP_CXX_FLAGS})

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(orthosfm-triangulation PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find external required libraries
find_package(Eigen3 CONFIG REQUIRED)

# Define the dependencies
target_link_libraries(orthosfm-triangulation
        PRIVATE orthosfm-data Eigen3::Eigen)