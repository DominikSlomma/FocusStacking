cmake_minimum_required(VERSION 3.22)
project(FOKUS_STACKING LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -msse4 ") # think on compiler!
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})  
set(CMAKE_CUDA_ARCHITECTURES 35)
if(POLICY CMP0091)
    cmake_policy(SET CMP0091 NEW)
endif()

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if(COMPILER_SUPPORTS_CXX17)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   add_definitions(-DCOMPILEDWITHCXX17)
   message(STATUS "Using flag -std=c++17.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()
set(CMAKE_PREFIX_PATH "/usr/local" ${CMAKE_PREFIX_PATH})

# include_directories(${PROJECT_SOURCE_DIR}/src/viewer)
# include_directories(${PROJECT_SOURCE_DIR}/src/modules)
# include_directories(${PROJECT_SOURCE_DIR}/src/modules/data)
# include_directories(${PROJECT_SOURCE_DIR}/src/modules/GT_compare)
# include_directories(${PROJECT_SOURCE_DIR}/src/modules/optimize)
# include_directories(${PROJECT_SOURCE_DIR}/src/util)

include_directories(${PROJECT_SOURCE_DIR}/test)


# find_package(SuiteSparse REQUIRED)

# if(SuiteSparse_FOUND)
#     message(STATUS "SuiteSparse found")
#     message(STATUS "SuiteSparse_INCLUDE_DIRS: ${SuiteSparse_INCLUDE_DIRS}")
#     message(STATUS "SuiteSparse_LIBRARIES: ${SuiteSparse_LIBRARIES}")
# else()
#     message(FATAL_ERROR "SuiteSparse not found!")
# endif()

# include_directories(${SUITESPARSE_INCLUDE_DIRS})
# link_directories(${SUITESPARSE_LIBRARY_DIRS})

# find_package(Cholmod)
# find_package(BLAS)
# find_package(LAPACK)
# find_package(GOTOBLAS)
# IF(CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
#   MESSAGE(STATUS "Found CHOLMOD and its dependencies")
#   SET(CHOLMOD_FOUND TRUE)
# ELSE()
#   SET(CHOLMOD_FOUND FALSE)
# ENDIF()


find_package(OpenCV 4.5.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "Use OpenCV ${OpenCV_VERSION}")

# Eigen
# find_package(Eigen3 3.3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})
# message(STATUS "Use Eigen")

# find_package(Pangolin 0.9 REQUIRED)
# message(STATUS "Pangolin found")
# include_directories(${Pangolin_INCLUDE_DIR})


# Finde die Open3D-Bibliothek
# find_package(Open3D 0.18.0 REQUIRED)
# message(STATUS "Open3D found")

# Füge den include-Ordner von Open3D hinzu
# include_directories(${Open3D_INCLUDE_DIRS})

# if(NOT Open3D_FOUND)
#     message(FATAL_ERROR "Open3D not found, please use -DCMAKE_INSTALL_PREFIX=open3d_install_dir")
# endif()

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# OpenMP
find_package(OpenMP REQUIRED)


# Set standard installation directories
set(RUNTIME_DESTINATION bin)
set(LIBRARY_DESTINATION lib)
set(ARCHIVE_DESTINATION lib)
set(INCLUDES_DESTINATION include)


# enable_language(CUDA)
# find_package(CUDA QUIET REQUIRED)
# include_directories("${CUDA_INCLUDE_DIRS}")




# add_subdirectory(src)
add_subdirectory(app)

# add_subdirectory(test)


