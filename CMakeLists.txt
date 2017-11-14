PROJECT(cartographer)
cmake_minimum_required(VERSION 2.6)

find_package(Rock)
if (ROCK_USE_CXX11)
    rock_activate_cxx11()
endif()

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(cartographerBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)
#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
