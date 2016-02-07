cmake_minimum_required(VERSION 2.8)

project(Imac3-MoteursPhysiques-Utils)

add_definitions(-std=c++11)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake)

find_package(SDL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

include_directories(${SDL_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIR} Utils/include third-party/AntTweakBar/include third-party/include)

add_subdirectory(Utils)
add_subdirectory(third-party/AntTweakBar)

set(ALL_LIBRARIES Utils AntTweakBar ${SDL_LIBRARY} ${OPENGL_LIBRARIES} ${GLEW_LIBRARY})

file(GLOB_RECURSE SRC_FILES src/*.cpp)

foreach(SRC_FILE ${SRC_FILES})
    get_filename_component(FILE ${SRC_FILE} NAME_WE)
    add_executable(${FILE} ${SRC_FILE} Utils/src/Flag.cpp Utils/include/Utils/Flag.h)
    target_link_libraries(${FILE} ${ALL_LIBRARIES})
endforeach()
