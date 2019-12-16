cmake_minimum_required(VERSION 3.5.1)

project(VideoServer CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14 -Wall -Weffc++ -Wextra")

find_package(jpeg REQUIRED)

set(SRCS
    "videoserver.cpp"
    "serialcomm.cpp"
    "imageproc.cpp"
)

include_directories(
    "${PROJECT_SOURCE_DIR}"
)

target_link_libraries(videoserver
    libjpeg
)

add_executable(videoserver
    ${SRCS}
)

