cmake_minimum_required(VERSION 2.8)

PROJECT (double-pendulum)

FIND_PACKAGE (Simbody REQUIRED)

INCLUDE_DIRECTORIES (${Simbody_INCLUDE_DIR})

SET (CMAKE_C_FLAGS "-Wall -g")

SET (SRC_LIST main.cpp Position.h) 
SET (EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
ADD_EXECUTABLE (double-pendulum ${SRC_LIST})
TARGET_LINK_LIBRARIES (double-pendulum ${Simbody_LIBRARIES})
