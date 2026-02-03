cmake_minimum_required(VERSION 3.5)

project(oakchina_vio_windows)

find_package(OpenCV REQUIRED)

link_directories(lib)

include_directories(
    include
    src
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(oakchina_vio_demo
    ./src/oakchina_vio_demo.cpp
)
target_link_libraries(oakchina_vio_demo
    carina_vio.lib
    ${OpenCV_LIBS}
)

add_executable(oakchina_vio_traj_demo
    ./src/oakchina_vio_traj_demo.cpp
    ./src/trajectory_viewer_opencv.cpp
)
target_link_libraries(oakchina_vio_traj_demo
    carina_vio.lib
    ${OpenCV_LIBS}
)
