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

include_directories(src
    include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(cv2_example src/cv2_example.cpp)
target_link_libraries(cv2_example
    freenect
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
)

add_executable(depth_tracking src/depth_tracking.cpp)
target_link_libraries(depth_tracking
    freenect
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
)