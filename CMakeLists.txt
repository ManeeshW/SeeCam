cmake_minimum_required(VERSION 3.10)
project(SeeCam)

find_package(OpenCV REQUIRED)
find_package(zenohc REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_compile_definitions(ZENOHCXX_ZENOHC)

add_executable(seecam src/main.cpp src/seecam.cpp)
target_link_libraries(seecam ${OpenCV_LIBS} zenohc::lib)