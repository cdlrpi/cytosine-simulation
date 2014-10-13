cmake_minimum_required(VERSION 2.8)

PROJECT (double-pendulum)

FIND_PACKAGE (ChronoEngine)

include_directories(${CHRONOENGINE_INCLUDES})

SET (CMAKE_C_FLAGS "-Wall -g")

SET (SRC_LIST demo_buildsystem.cpp) 
SET (EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
ADD_EXECUTABLE (double-pendulum ${SRC_LIST})
TARGET_LINK_LIBRARIES (double-pendulum ${CHRONOENGINE_LIBRARIES})
#   find_package(ChronoEngine COMPONENTS unit_POSTPROCESS unit_FEM)
#   include(${CHRONOENGINE_INCLUDES})
#   add_executable(myexe main.cpp)
#   target_link_libraries(myexe ${CHRONOENGINE_LIBRARIES})
