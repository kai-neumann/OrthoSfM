# Define the library
add_library(orthosfm-matching matching.h matching.cpp matching_io.h matching_io.cpp matching_mve.h matching_mve.cpp)

# Set some target specific compile flags
set_target_properties(orthosfm-matching PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(orthosfm-matching PRIVATE ${OpenMP_CXX_FLAGS})

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(orthosfm-matching PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find external required libraries
find_package(Boost 1.36.0)
find_package(OpenCV CONFIG REQUIRED)

# Define the dependencies
target_link_libraries(orthosfm-matching
        PRIVATE orthosfm-data Boost::boost ${OpenCV_LIBS} mve_math mve mve_sfm mve_util)

if(ENABLE_CUDA)
    target_link_libraries(orthosfm-matching PRIVATE cudasift)
endif(ENABLE_CUDA)