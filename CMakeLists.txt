cmake_minimum_required(VERSION 3.12)
project(SofaHaplyRobotics VERSION 0.1)

sofa_find_package(Sofa.Component.Controller REQUIRED)
sofa_find_package(Sofa.Component.Haptics REQUIRED)

set(HAPLY_PREFIX "C:\\Program Files\\Haply\\Inverse\\"
    CACHE PATH "Install folder for the SDK")

## Required include and define directives for making use of the API.
include_directories("${HAPLY_PREFIX}\\include")
add_definitions(-DUNICODE -D_UNICODE)

set(SOFAHAPLYROBOTICS_SRC_DIR src/SofaHaplyRobotics)
set(HEADER_FILES
    ${SOFAHAPLYROBOTICS_SRC_DIR}/config.h.in
    ${SOFAHAPLYROBOTICS_SRC_DIR}/Haply_Defines.h
    
    ${SOFAHAPLYROBOTICS_SRC_DIR}/Haply_HapticThreadManager.h    
    ${SOFAHAPLYROBOTICS_SRC_DIR}/Haply_Inverse3Controller.h
)

set(SOURCE_FILES
    ${SOFAHAPLYROBOTICS_SRC_DIR}/Haply_HapticThreadManager.cpp    
    ${SOFAHAPLYROBOTICS_SRC_DIR}/Haply_Inverse3Controller.cpp
    
    ${SOFAHAPLYROBOTICS_SRC_DIR}/initSofaHaplyRoboticsPlugin.cpp
)

set(README_FILES Readme.md)

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Set define dllimport/dllexport mechanism on Windows.
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_SOFAHAPLYROBOTICS")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Component.Controller Sofa.Component.Haptics "${HAPLY_PREFIX}\\lib\\haply-inverse-c.lib")

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")


install(FILES "${HAPLY_PREFIX}\\lib\\haply-inverse-c.dll" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COMPONENT applications)

## Install rules for the library; CMake package configurations files
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )

# Organize projects into folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
SET_PROPERTY(TARGET ${PROJECT_NAME} PROPERTY FOLDER "plugins")