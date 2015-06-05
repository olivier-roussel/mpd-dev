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

include(cmake/portability.cmake)
include(cmake/detect_msvc_versions.cmake)
include(cmake/common.cmake)

set(CXX_DISABLE_WERROR ON) # blame OMPL 
set(PROJECT_NAME MPD-dev)
set(PROJECT_VERSION 0.1)
set(PROJECT_DESCRIPTION
  "Motion planning for deformable objects.")
set(PROJECT_URL "")

project(${PROJECT_NAME})

if(MSVC)
	DETECT_MSVC_VERSION()
endif(MSVC)

# Define & include header directory
set(${PROJECT_NAME}_HEADER_DIR mpd)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_INSTALL_PREFIX}"/include/)

# Use cmake/ dir for user modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Configure share dir
set(SHARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/share/mpd")  
CONFIG_FILES("src/config.h")
include_directories("${CMAKE_BINARY_DIR}/src") # for the config.h file

# Special flags for MSVC
if (MSVC)
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast")
  # Enable intrinsics on MSVC in debug mode
  #set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Oi")
  if (CMAKE_CL_64)
    # Visual Studio bails out on debug builds in 64bit mode unless
	# this flag is set...
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /bigobj")
  endif ()
endif ()



add_subdirectory(src)
#add_subdirectory(tests)
