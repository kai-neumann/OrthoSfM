# Define the library
add_library(orthosfm-algorithms ReconstructionAlgorithm.h ReconstructionAlgorithm.cpp tomasi_kanade.h tomasi_kanade.cpp)

# Set some target specific compile flags
set_target_properties(orthosfm-algorithms PROPERTIES CXX_STANDARD 17)

# Add sub folders
add_subdirectory(orthographic)
add_subdirectory(orthographic_quaternion)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(orthosfm-algorithms PRIVATE ${OpenMP_CXX_FLAGS})

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(orthosfm-algorithms PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find external required libraries
find_package(Ceres CONFIG REQUIRED)

# Define the dependencies
target_link_libraries(orthosfm-algorithms
        PRIVATE Ceres::ceres)