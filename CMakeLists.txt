cmake_minimum_required(VERSION 3.12)
project(SofaHaplyRobotics VERSION 0.1)

find_package(Sofa.Component.Controller REQUIRED)
find_package(Sofa.Component.Haptics REQUIRED)

set(HAPLY_PREFIX "c:/Program Files/Haply/1.1.1"
    CACHE PATH "Install folder for the SDK")

## Required include and define directives for making use of the API.
include_directories("${HAPLY_PREFIX}/include")
add_definitions(-DUNICODE -D_UNICODE)

set(SOFAHAPLYROBOTICS_SRC_DIR src/SofaHaplyRobotics)
set(HEADER_FILES
    ${SOFAHAPLYROBOTICS_SRC_DIR}/config.h.in
    ${SOFAHAPLYROBOTICS_SRC_DIR}/Haply_Defines.h
    
    ${SOFAHAPLYROBOTICS_SRC_DIR}/Haply_HapticThreadManager.h    
    ${SOFAHAPLYROBOTICS_SRC_DIR}/Haply_Inverse3Controller.h
    
    extlib/Stream.h
    extlib/SerialStream.h
    extlib/serialib.h
    extlib/Inverse3.h
)

set(SOURCE_FILES
    ${SOFAHAPLYROBOTICS_SRC_DIR}/Haply_HapticThreadManager.cpp    
    ${SOFAHAPLYROBOTICS_SRC_DIR}/Haply_Inverse3Controller.cpp
    
    ${SOFAHAPLYROBOTICS_SRC_DIR}/initSofaHaplyRoboticsPlugin.cpp
    extlib/serialib.cpp
    extlib/Inverse3.cpp
)

set(README_FILES Readme.md)

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Set define dllimport/dllexport mechanism on Windows.
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_SOFAHAPLYROBOTICS")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Component.Controller Sofa.Component.Haptics "${HAPLY_PREFIX}/lib/haply-rt.lib")

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extlib>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

## Install rules for the library; CMake package configurations files
sofa_generate_package(
    NAME ${PROJECT_NAME}
    TARGETS ${PROJECT_NAME}
    VERSION ${PROJECT_VERSION}
    INCLUDE_SOURCE_DIR ${SOFAHAPLYROBOTICS_SRC_DIR}
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )