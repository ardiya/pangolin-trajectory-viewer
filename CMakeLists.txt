# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(trajectory-viewer main.cpp)
target_link_libraries(trajectory-viewer ${Pangolin_LIBRARIES})
