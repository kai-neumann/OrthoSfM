# Define the library
add_library(orthosfm-util timing.h timing.cpp common.h common.cpp)

# Set some target specific compile flags
set_target_properties(orthosfm-util PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(orthosfm-util PRIVATE ${OpenMP_CXX_FLAGS})

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(orthosfm-util PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find external required libraries
find_package(OpenCV CONFIG REQUIRED)
find_package(Boost 1.36.0 COMPONENTS filesystem)
find_package(Eigen3 CONFIG REQUIRED)

# Define the dependencies
target_link_libraries(orthosfm-util
        PRIVATE orthosfm-data ${OpenCV_LIBS} Boost::boost Boost::filesystem Eigen3::Eigen)