cmake_minimum_required(VERSION 3.20)

project(SimpleTriangleRenderer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

add_executable(SimpleTriangleRenderer main.cpp)

target_link_libraries(SimpleTriangleRenderer
    PRIVATE
        Eigen3::Eigen
        ${OpenCV_LIBS}
)

target_include_directories(SimpleTriangleRenderer
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)
