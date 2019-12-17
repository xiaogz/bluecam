cmake_minimum_required(VERSION 3.5.1)

project(VideoServer CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14 -Wall -Weffc++ -Wextra")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}")

find_package(JPEG
    REQUIRED
)

set(SRCS
    "videoserver.cpp"
    "serialcomm.cpp"
    "imageproc.cpp"
    "utility.cpp"
)

include_directories(
    "${JPEG_INCLUDE_DIR}"
    "${PROJECT_SOURCE_DIR}"
)

add_executable(videoserver
    ${SRCS}
)

target_link_libraries(videoserver
    jpeg
)

