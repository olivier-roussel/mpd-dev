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

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_BINARY_DIR}")  # for the configured config.h file


# -------------------------------------
# Dependencies
# -------------------------------------

# Use Boost
if(MSVC)
	add_definitions(-DBOOST_ALL_DYN_LINK)
endif(MSVC)
if(IS_ICPC)
    set(Boost_USE_STATIC_LIBS ON CACHE STRING "Use statically linked Boost libraries")
else(IS_ICPC)
    # Ensure dynamic linking with boost unit_test_framework
    add_definitions(-DBOOST_TEST_DYN_LINK)
endif(IS_ICPC)
find_package(Boost 1.55 COMPONENTS chrono date_time filesystem program_options regex serialization system thread unit_test_framework REQUIRED)
if (Boost_FOUND)
  include_directories("${Boost_INCLUDE_DIR}")
  set(LIBS ${LIBS} ${Boost_LIBRARIES}) 
else()
  message("Could not find Boost")
endif()

# Eigen3 (with LGSM module)
#if (NOT $ENV{EIGEN3_ROOT} STREQUAL "")
#  set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
#endif()
find_package(Eigen3 REQUIRED)
if (EIGEN3_FOUND)
  message("{EIGEN3_INCLUDE_DIR} = ${EIGEN3_INCLUDE_DIR}")
  include_directories(${EIGEN3_INCLUDE_DIR})
  include_directories(${EIGEN3_INCLUDE_DIR}/unsupported)
else()
  message("Could not find Eigen3")
endif()
# Set EIGEN_DONT_ALIGN definition for 32bits platforms
# XXX crashes if not set also under MSVC10 Win64 _ 
#if(NOT CMAKE_CL_64)
	add_definitions(-DEIGEN_DONT_ALIGN)
#endif()

# Use SDL2
find_package(SDL2 REQUIRED)
if (SDL_FOUND)
  include_directories(${SDL_INCLUDE_DIR})
  set(LIBS ${LIBS} ${SDL_LIBRARY}) 
else()
  message("Could not find SDL")
endif()

# Use Glew
find_package(Glew REQUIRED)
if (Glew_FOUND)
  include_directories(${Glew_INCLUDE_DIR})
  link_directories(${Glew_LIBRARY_DIRS})
  set(LIBS ${LIBS} ${Glew_LIBRARIES}) 
else()
  message("Could not find Glew")
endif()

# OpenGL (for GLViewer)
find_package(OpenGL REQUIRED)
if (OPENGL_FOUND)
  set(LIBS ${LIBS} ${OPENGL_LIBRARIES}) 
else()
  message("Could not find OpenGL")
endif()

# Bullet
find_package(Bullet REQUIRED)
if (BULLET_FOUND)
  #add_definitions( -DBT_USE_DOUBLE_PRECISION ) # remove this if you use single precision bullet
  include_directories(${BULLET_INCLUDE_DIR})
  set(LIBS ${LIBS} ${BULLET_LIBRARIES}) 
else()
  message("Could not find Bullet")
endif()

# -------------------------------------
# Setting up target
# -------------------------------------

set(EXECUTABLE_NAME ${PROJECT_NAME})

# Get sources from subdirectories
SOURCE_FROM_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} "")

# Get sources and create source groups
message("HEADER_FILES=${HEADER_FILES}")
SOURCE_FROM_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR})
set(${EXECUTABLE_NAME}_SOURCES ${SOURCE_FILES})
set(${EXECUTABLE_NAME}_HEADERS ${HEADER_FILES})

add_executable(${EXECUTABLE_NAME} 
  ${${EXECUTABLE_NAME}_SOURCES} 
  ${${EXECUTABLE_NAME}_HEADERS} )
  
# if(WIN32)
  # add_executable(${EXECUTABLE_NAME} WIN32 ${${EXECUTABLE_NAME}_SOURCES} ${${EXECUTABLE_NAME}_HEADERS})
# else(WIN32)
  # #Linux:
  # add_executable(${EXECUTABLE_NAME} ${${EXECUTABLE_NAME}_SOURCES} ${${EXECUTABLE_NAME}_HEADERS})
# endif(WIN32)

target_link_libraries(${EXECUTABLE_NAME} ${LIBS})

