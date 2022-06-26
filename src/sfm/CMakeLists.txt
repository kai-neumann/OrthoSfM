# Define the library
add_library(orthosfm-sfm reconstruct.h reconstruct.cpp)

# Set some target specific compile flags
set_target_properties(orthosfm-sfm PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(orthosfm-sfm PRIVATE ${OpenMP_CXX_FLAGS})

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(orthosfm-sfm PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find external required libraries
find_package(OpenCV CONFIG REQUIRED)

# Define the dependencies
target_link_libraries(orthosfm-sfm
        PRIVATE orthosfm-data orthosfm-algorithms orthosfm-ba orthosfm-matching orthosfm-triangulation ${OpenCV_LIBS})