cmake_minimum_required(VERSION 2.8.12)
project(dvo)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

find_package(Threads)
find_package(libfreenect)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system)

include_directories(src
    dvo_core/include
    dvo_core/src
    src/object_tracking/include
    src/object_tracking/include/kinect
    src/object_tracking/src/object_tracking
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_subdirectory(dvo_core)

link_directories(dvo 
    dvo_core
    dvo_visualization
    ${Boost_LIBRARY_DIRS}
    )

add_executable(cv2_example src/object_tracking/src/examples/cv2_example.cpp)
target_link_libraries(cv2_example
    freenect
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
)

add_executable(depth_tracking src/object_tracking/src/object_tracking/depth_tracking.cpp)
target_link_libraries(depth_tracking
    dvo_core
    freenect
    dvo_visualization
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
    ${Eigen3_LIBRARIES}
    ${Boost_LIBRARIES}
)
