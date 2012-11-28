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
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/cpack.cmake)

SET(CXX_DISABLE_WERROR ON) # blame OMPL 
SET(PROJECT_NAME MPD-dev)
SET(PROJECT_VERSION 0.1)
SET(PROJECT_DESCRIPTION
  "Motion planning for deformable objects.")
SET(PROJECT_URL "")

SET(CUSTOM_HEADER_DIR mpd)
SET(${PROJECT_NAME}_HEADERS
  include/${CUSTOM_HEADER_DIR}/algorithm.h
  include/${CUSTOM_HEADER_DIR}/bullet_engine_wrapper.h
  include/${CUSTOM_HEADER_DIR}/bullet_utils.h
  include/${CUSTOM_HEADER_DIR}/constants.h
  include/${CUSTOM_HEADER_DIR}/environment.h
  include/${CUSTOM_HEADER_DIR}/mpd_controller.h
  include/${CUSTOM_HEADER_DIR}/physics_engine.h
  include/${CUSTOM_HEADER_DIR}/physics_thread.h
  include/${CUSTOM_HEADER_DIR}/polygon_soup.h
  include/${CUSTOM_HEADER_DIR}/types.h
)

SETUP_PROJECT()

# Always OFF on Unix systems
IF (WIN32)
 OPTION(MPD_DEBUG "Required if Debug build for linking with debug libs on Windows" OFF)
ELSE (WIN32)
 SET(MPD_DEBUG OFF)
ENDIF (WIN32)

set(SHARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/share/mpd")
CONFIG_FILES("src/config.h")
#include_directories("${CMAKE_BINARY_DIR}/include")
#include_directories("${CMAKE_BINARY_DIR}/src")

# Dependencies

# Use Boost
IF (WIN32)
	set(Boost_USE_STATIC_LIBS OFF)
	set(Boost_USE_MULTITHREADED ON) 
	set(Boost_USE_STATIC_RUNTIME OFF)
	#add_definitions( -DBOOST_ALL_DYN_LINK )
	find_package(Boost 1.48 COMPONENTS chrono date_time filesystem regex system thread )
else (WIN32)
	SEARCH_FOR_BOOST()
endif (WIN32)

# Use Eigen
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.1.1")

# Use assimp for Collada files
#ADD_REQUIRED_DEPENDENCY("assimp >= 2.0")

# Use SDL for the viewer / GUI
ADD_REQUIRED_DEPENDENCY("sdl >= 1.2.15")
#ADD_REQUIRED_DEPENDENCY("sdl2 >= 2.0.0")

# Use Bullet as one of the physical engine
IF (MPD_DEBUG)
  ADD_REQUIRED_DEPENDENCY("bulletd >= 2.80")
ELSE (MPD_DEBUG)
  ADD_REQUIRED_DEPENDENCY("bullet >= 2.80")
ENDIF (MPD_DEBUG)

# Use ompl motion planning lib
#ADD_REQUIRED_DEPENDENCY("ompl >= 0.11.1")

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)
#ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
