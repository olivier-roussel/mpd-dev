# Copyright (c) 2012 CNRS
# Author: Olivier Roussel
#
# This file is part of the MPD-dev package.
# MPD-dev is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# MPD-dev is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# MPD-dev.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
cmake_minimum_required(VERSION 2.6)

# include(cmake/base.cmake)
# include(cmake/boost.cmake)
include(cmake/portability.cmake)
include(cmake/platform.cmake)

set(CXX_DISABLE_WERROR ON) # blame OMPL 
set(PROJECT_NAME MPD-dev)
set(PROJECT_VERSION 0.1)
set(PROJECT_DESCRIPTION
  "Motion planning for deformable objects.")
set(PROJECT_URL "")

project(${PROJECT_NAME})

# Define & include header directory
set(CUSTOM_HEADER_DIR mpd)
include_directories("${CMAKE_SOURCE_DIR}/include")

# Fill platform specific variables
# We usually require that libs are stored separatly for each platforms and architectures
# using the form lib/{PLATFORM}/${PLATFORM_ARCH} and bin/{PLATFORM}/${PLATFORM_ARCH}
DETECT_PLATFORM()

# Use cmake/ dir for user modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Define share directory
set(SHARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/share/mpd")
CONFIG_FILES("src/config.h")
include_directories("${CMAKE_BINARY_DIR}/src")

# Dependencies
# ----------------------------------

# Use Boost 
# TODO here we force using static boost libs
# Dyn libs need to copy boost dll. 

#set(Boost_DEBUG TRUE)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON) 
set(Boost_USE_STATIC_RUNTIME OFF)
#add_definitions( -DBOOST_ALL_DYN_LINK )
set(BOOST_INCLUDEDIR "$ENV{BOOST_ROOT}/include")
set(BOOST_LIBRARYDIR "$ENV{BOOST_ROOT}/lib/${PLATFORM}/${PLATFORM_ARCH}")
find_package(Boost 1.48 COMPONENTS chrono date_time filesystem regex system thread REQUIRED)
message ("Boost_FIND_COMPONENTS=${Boost_FIND_COMPONENTS}")
# foreach(COMPONENT ${Boost_FIND_COMPONENTS})
  # message ("COMPONENT=${COMPONENT}")
# endforeach(COMPONENT)

if (Boost_FOUND)
  include_directories("${Boost_INCLUDE_DIR}")
  set(LIBS ${LIBS} ${Boost_LIBRARIES}) 
else()
  message("Could not find Boost")
endif()

# Use Eigen
if (NOT $ENV{EIGEN3_INCLUDE_DIR} STREQUAL "")
  set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
endif()
message("EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")
find_package(Eigen3 REQUIRED)
if (EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
  #set(LIBS ${EIGEN3} ${EIGEN3_LIBRARY}) 
else()
  message("Could not find Eigen3")
endif()
# Set EIGEN_DONT_ALIGN definition for 32bits platforms
if(NOT CMAKE_CL_64)
	add_definitions(-DEIGEN_DONT_ALIGN)
endif()

# SDL (for GUI)
find_package(SDL REQUIRED)
if (SDL_FOUND)
  include_directories(${SDL_INCLUDE_DIR})
  set(LIBS ${LIBS} ${SDL_LIBRARY}) 
else()
  message("Could not find SDL")
endif()

# OpenGL (for GUI)
find_package(OpenGL REQUIRED)
if (OPENGL_FOUND)
  set(LIBS ${LIBS} ${OPENGL_LIBRARIES}) 
else()
  message("Could not find OpenGL")
endif()

# Bullet
find_package(Bullet REQUIRED)
if (BULLET_FOUND)
  add_definitions( -DBT_USE_DOUBLE_PRECISION ) # remove this if you use single precision bullet
  include_directories(${BULLET_INCLUDE_DIR})
  set(LIBS ${LIBS} ${BULLET_LIBRARIES}) 
else()
  message("Could not find Bullet")
endif()

add_subdirectory(src)
#add_subdirectory(tests)
