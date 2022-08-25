cmake_minimum_required(VERSION 3.5)
project(swarm_map)

#set(CMAKE_CXX_STANDARD 11)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

#set(CMAKE_C_FLAGS_DEBUG "-O2 -g")
#set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g")

set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O1 -g")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O1 -g")

set(CMAKE_C_FLAGS_RELEASE "-O1")
set(CMAKE_CXX_FLAGS_RELEASE "-O1")

LIST(APPEND CMAKE_C_FLAGS "-Wall -Wextra -DUSE_NVTX -Wno-deprecated-declarations -ftemplate-backtrace-limit=1")
LIST(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -DUSE_NVTX -Wno-deprecated-declarations -ftemplate-backtrace-limit=1")


find_package(CUDA QUIET REQUIRED)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
SET(CUDA_HOST_COMPILER /usr/bin/g++)
LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
    message(STATUS "Using flag -std=c++11.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++0x")
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()


set(BASE_LIBRARY "slam_core")
set(SRC_DIRECTORY "${PROJECT_SOURCE_DIR}/code")

LIST(APPEND CMAKE_MODULE_PATH ${SRC_DIRECTORY}/cmake_modules)

find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
    message("OpenCV 4 not found")
    find_package(OpenCV 3.4.4 QUIET)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 3.4.4 not found.")
    endif ()
endif ()

if(OpenCV_CUDA_VERSION)
    message("OpenCV CUDA support: ${OpenCV_CUDA_VERSION}")
endif()

list(APPEND CUDA_NVCC_FLAGS ${OpenCV_COMPUTE_CAPABILITIES})

cuda_include_directories(
        ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc
        ${CUDA_TOOLKIT_ROOT_DIR}/Common/
)


find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS serialization)

find_package(spdlog REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/debug/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/debug/bin)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/bin)


#add_library(${BASE_LIBRARY} SHARED
cuda_add_library(${BASE_LIBRARY} SHARED
        ${SRC_DIRECTORY}/src/System.cc
        ${SRC_DIRECTORY}/src/Tracking.cc
        ${SRC_DIRECTORY}/src/LocalMapping.cc
        ${SRC_DIRECTORY}/src/LoopClosing.cc
        ${SRC_DIRECTORY}/src/ORBextractor.cc
        ${SRC_DIRECTORY}/src/ORBmatcher.cc
        ${SRC_DIRECTORY}/src/FrameDrawer.cc
        ${SRC_DIRECTORY}/src/Converter.cc
        ${SRC_DIRECTORY}/src/MapPoint.cc
        ${SRC_DIRECTORY}/src/KeyFrame.cc
        ${SRC_DIRECTORY}/src/Map.cc
        ${SRC_DIRECTORY}/src/MapDrawer.cc
        ${SRC_DIRECTORY}/src/Optimizer.cc
        ${SRC_DIRECTORY}/src/PnPsolver.cc
        ${SRC_DIRECTORY}/src/Frame.cc
        ${SRC_DIRECTORY}/src/KeyFrameDatabase.cc
        ${SRC_DIRECTORY}/src/Sim3Solver.cc
        ${SRC_DIRECTORY}/src/Initializer.cc
        ${SRC_DIRECTORY}/src/Viewer.cc
        ${SRC_DIRECTORY}/src/MapUpdater.cc
        ${SRC_DIRECTORY}/src/AgentMediator.cc
        ${SRC_DIRECTORY}/src/DataSetUtil.cc
        ${SRC_DIRECTORY}/src/Timer.cc
        ${SRC_DIRECTORY}/src/MapManager.cc
        ${SRC_DIRECTORY}/src/MapSlice.cc
        ${SRC_DIRECTORY}/src/LandmarkScoring.cc
        ${SRC_DIRECTORY}/src/cuda/Allocator_gpu.cu
        ${SRC_DIRECTORY}/src/cuda/Fast_gpu.cu
        ${SRC_DIRECTORY}/src/cuda/Orb_gpu.cu
        ${SRC_DIRECTORY}/src/cuda/Cuda.cu
        ${SRC_DIRECTORY}/src/DynamicExtractor.cc
        ${SRC_DIRECTORY}/src/DynamicRunner.cc
        ${SRC_DIRECTORY}/src/MapEnhancer.cc
        ${SRC_DIRECTORY}/src/MediatorScheduler.cc
        ${SRC_DIRECTORY}/src/Mapit.cc
        ${SRC_DIRECTORY}/src/STS.cc
#        ${SRC_DIRECTORY}/src/Network/ClientService.cc
        ${SRC_DIRECTORY}/src/ClientService.cc
        ${SRC_DIRECTORY}/src/ServerService.cc
        #        ${SRC_DIRECTORY}/src/Network/ServerService.cc
        ${SRC_DIRECTORY}/src/Network/MapElementUpdate.cc
#        ${SRC_DIRECTORY}/src/Network/ConnectionService.cc
        ${SRC_DIRECTORY}/src/Network/WebSocket.cc
        )

target_include_directories(${BASE_LIBRARY} PUBLIC
        ${SRC_DIRECTORY}/
        ${SRC_DIRECTORY}/include
        ${SRC_DIRECTORY}/Thirdparty/popl
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        )

target_link_libraries(${BASE_LIBRARY}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Boost_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${SRC_DIRECTORY}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${SRC_DIRECTORY}/Thirdparty/g2o/lib/libg2o.so
        nvToolsExt
        pthread
        spdlog::spdlog
        )

add_executable(swarm_map
        ${SRC_DIRECTORY}/Examples/Monocular/swarm_map.cc)
target_link_libraries(swarm_map ${BASE_LIBRARY})
