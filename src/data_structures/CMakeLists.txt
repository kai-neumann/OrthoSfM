# Define the library
add_library(orthosfm-data Camera.h Camera.cpp camera_io.h camera_io.cpp colored_point_cloud.h group.h group.cpp project_io.h project_io.cpp solver_type.h track.h track.cpp view.h view.cpp)

# Set some target specific compile flags
set_target_properties(orthosfm-data PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(orthosfm-data PRIVATE ${OpenMP_CXX_FLAGS})

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(orthosfm-data PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find external required libraries
find_package(Boost 1.36.0 COMPONENTS filesystem)
find_package(OpenCV CONFIG REQUIRED)
target_include_directories(orthosfm-data PUBLIC ${OpenCV_INCLUDE_DIRS})
find_package(Eigen3 CONFIG REQUIRED)

# Define the dependencies
target_link_libraries(orthosfm-data
        PRIVATE Eigen3::Eigen Boost::boost Boost::filesystem Eigen3::Eigen
        PUBLIC ${OpenCV_LIBS})