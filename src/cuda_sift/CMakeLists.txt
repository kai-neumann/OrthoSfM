# Define the sources
set(cuda_sources
        cudaImage.cu
        cudaImage.h
        cudaSiftH.cu
        cudaSiftH.h
        matching.cu
        cudaSiftD.h
        cudaSift.h
        cudautils.h
        )

set(sources
        geomFuncs.cpp
        mainSift.cpp
        )

# Define the library
add_library(cudasift ${cuda_sources} ${sources})

# Set some target specific compile flags
#set_target_properties(cudasift PROPERTIES CXX_STANDARD 17)

if (WIN32)
    add_definitions(-DWIN32)
    set(EXTRA_CXX_FLAGS "/DVERBOSE /D_CRT_SECURE_NO_WARNINGS ")
    list(APPEND CUDA_NVCC_FLAGS "-arch=sm_35;--compiler-options;-O2;-DVERBOSE")
endif()
if (UNIX)
    if (APPLE)
        set(EXTRA_CXX_FLAGS "-DVERBOSE -msse2")
        list(APPEND CUDA_NVCC_FLAGS "-arch=sm_35;--compiler-options;-O2;-DVERBOSE")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -msse2 ")
        list(APPEND CUDA_NVCC_FLAGS "-lineinfo;-ccbin;/usr/bin/gcc-6;--compiler-options;-O2;-D_FORCE_INLINES;-DVERBOSE_NOT")
    endif()
endif()

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(cudasift PRIVATE -Xcompiler="${OpenMP_CXX_FLAGS}")

#set_target_properties(cudasift PROPERTIES CUDA_ARCHITECTURES "35" COMPILE_FLAGS "${EXTRA_CXX_FLAGS}")
set_target_properties(cudasift PROPERTIES CUDA_ARCHITECTURES "35")

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(cudasift PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find libaries
find_package(OpenCV CONFIG REQUIRED)

# Define the dependencies
target_link_libraries(cudasift PRIVATE ${OpenCV_LIBS})