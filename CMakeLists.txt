cmake_minimum_required(VERSION 3.23)
project(EasyDroneSimulator)

set(CMAKE_CXX_STANDARD 17)

if (WIN32)
    find_package(Eigen3 CONFIG REQUIRED)
    find_package(gflags CONFIG REQUIRED)
    find_package(Pangolin CONFIG REQUIRED)
    find_package(Boost COMPONENTS system REQUIRED)
    if (Boost_FOUND)
        include_directories(${Boost_INCLUDE_DIRS})
        link_directories(${Boost_LIBRARY_DIRS})
    endif ()
else()
    find_package(Eigen3 REQUIRED)
    find_package(gflags REQUIRED)
    if (MACOS)
        set(Pangolin_DIR /usr/local/)
    endif()
    find_package(Pangolin REQUIRED)
    find_package(Boost REQUIRED)
endif()

include_directories(
    ${Eigen_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/EasyDroneSimulator/include
    ${Pangolin_INCLUDE_DIRS}
)


add_executable(EasyDroneSimulator EasyDroneSimulator/main.cpp)
target_link_libraries(EasyDroneSimulator
    ${Pangolin_LIBRARIES})
