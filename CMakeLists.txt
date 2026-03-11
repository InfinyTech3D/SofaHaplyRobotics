##############################################################################
#                 - Copyright (C) 2022-Present InfinyTech3D -                #
#                                                                            #
#  This file is part of the SofaHaplyRobotics plugin for the SOFA framework. #
#                                                                            #
#  This file is dual-licensed:                                               #
#                                                                            #
#  1) Commercial License:                                                    #
#       This file may be used under the terms of a valid commercial license  #
#       agreement provided wih the software by InfinyTech3D.                 #
#                                                                            #
#  2) GNU General Public License (GPLv3) Usage                               #
#       Alternatively, this file may be used under the terms of the          #
#       GNU General Public License version 3 as published by the             #
#       Free Software Foundation: https://www.gnu.org/licenses/gpl-3.0.html  #
#                                                                            #
#  Contact: contact@infinytech3d.com                                         #
#  Further information: https://infinytech3d.com                             #
##############################################################################

cmake_minimum_required(VERSION 3.12)
project(SofaHaplyRobotics VERSION 0.2)

include(cmake/environment.cmake)

find_package(Sofa.Component.Controller REQUIRED)
find_package(Sofa.Component.Haptics REQUIRED)

add_definitions(-DUNICODE -D_UNICODE)

# adding external dependencies
if(NOT TARGET nlohmann_json)
    add_subdirectory(external/json)
else()
    message(STATUS "## ${PROJECT_NAME}: nlohmann_json already included in the project.")
endif()

add_subdirectory(external/libhv)


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
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Component.Controller Sofa.Component.Haptics)
target_link_libraries(${PROJECT_NAME} PRIVATE hv_static nlohmann_json::nlohmann_json)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")
#target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})

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