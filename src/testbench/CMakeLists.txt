# First define the library
add_executable(orthosfm-testbench testbench.cpp dataset_generation.h dataset_generation.cpp full_pipeline_tests.h full_pipeline_tests.cpp synthethic_tests.h synthethic_tests.cpp)

# Set some target specific compile flags
set_target_properties(orthosfm-testbench PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(orthosfm-testbench PRIVATE ${OpenMP_CXX_FLAGS})

# Find external libraries
find_package(Boost 1.36.0 COMPONENTS filesystem program_options)

# For matplotlib
find_package(Python3 3.7 COMPONENTS Interpreter NumPy REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Define the dependencies
target_link_libraries(orthosfm-testbench
        PRIVATE orthosfm-sfm orthosfm-data orthosfm-util Boost::boost Boost::filesystem Boost::program_options ${PYTHON_LIBRARIES} Python3::NumPy)

# All install commands get the same destination. this allows us to use paths
# relative to the executable.
install(TARGETS orthosfm-testbench DESTINATION OrthoSfM)

# Now comes everything we need, to create a package
# there are a lot more variables you can set, and some
# you need to set for some package types, but we want to
# be minimal here.
set(CPACK_PACKAGE_NAME "OrthoSfM_testbench")
set(CPACK_PACKAGE_VERSION "1.0.0")

# We don't want to split our program up into several incomplete pieces.
set(CPACK_MONOLITHIC_INSTALL 1)

# This must be last
include(CPack)