cmake_minimum_required(VERSION 2.8.3)
project(sim_env)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        cmake_modules)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include test/include
  LIBRARIES sim_env
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS EIGEN3 Boost
)

# BUILD

#set(CMAKE_CXX_COMPILER /usr/bin/clang++)
set(CMAKE_CXX_STANDARD 11)
add_definitions(-std=gnu++11)
add_definitions(-Wno-deprecated-register)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(include
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})
set(SOURCE_FILES
        src/sim_env/Controller.cpp
        src/sim_env/SimEnv.cpp
        src/sim_env/utils/EigenUtils.cpp
        src/sim_env/utils/MathUtils.cpp)
add_library(sim_env
        ${SOURCE_FILES})
