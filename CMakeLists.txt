cmake_minimum_required(VERSION 3.5.1)

project(VideoServer CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14 -Wall -Weffc++ -Wextra")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}")

find_package(JPEG REQUIRED)
find_package(Threads REQUIRED)
find_package(SDL2 REQUIRED)

set(SRCS
    "videoserver.cpp"
    "imageproc.cpp"
    "utility.cpp"
)

include_directories(
    "${SDL2_INCLUDE_DIRS}"
    "${JPEG_INCLUDE_DIR}"
    "${PROJECT_SOURCE_DIR}"
)

add_executable(videoserver
    ${SRCS}
)

target_link_libraries(videoserver
    SDL2
    jpeg
)

