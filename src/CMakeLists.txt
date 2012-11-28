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
include_directories("${Boost_INCLUDE_DIR}")

# Setting up target
SET(EXECUTABLE_NAME mpd-dev-app)

# Must lists all required dependencies here
IF (MPD_DEBUG)
  SET(${EXECUTABLE_NAME}_PKG_DEPS "eigen3" "bulletd" "sdl")
ELSE (MPD_DEBUG)
  SET(${EXECUTABLE_NAME}_PKG_DEPS "eigen3" "bullet" "sdl")
ENDIF (MPD_DEBUG)

# On windows, link_directories must be called before setting up the target.
# As the PKG_CONFIG_USE_DEPENDENCY macro is calling both TARGET_LINK_LIBRARIES and
# LINK_DIRECTORIES, one will not work. So we have to separate the calls.
# TODO : update cmake / pkgconfig buildchain for windows compatibility
IF (WIN32)
  FOREACH(_PKG_DEP ${${EXECUTABLE_NAME}_PKG_DEPS})
    BUILD_PREFIX_FOR_PKG(${_PKG_DEP} _DEP_PREFIX)
    LINK_DIRECTORIES(${${_DEP_PREFIX}_LIBRARY_DIRS})
  ENDFOREACH()
ENDIF (WIN32)

FILE(GLOB ${EXECUTABLE_NAME}_HEADERS 
	${CMAKE_SOURCE_DIR}/include/${CUSTOM_HEADER_DIR}/*.h)
FILE(GLOB ${EXECUTABLE_NAME}_PRIV_HEADERS 
	${CMAKE_CURRENT_SOURCE_DIR}/*.h)
FILE(GLOB ${EXECUTABLE_NAME}_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

# App gui (TODO: make the viewer as a submodule)
FILE(GLOB ${EXECUTABLE_NAME}_GUI_HEADERS 
  ${CMAKE_CURRENT_SOURCE_DIR}/gui/*.h)
FILE(GLOB ${EXECUTABLE_NAME}_GUI_SOURCES 
  ${CMAKE_CURRENT_SOURCE_DIR}/gui/*.cc)

ADD_EXECUTABLE(${EXECUTABLE_NAME}
  ${${EXECUTABLE_NAME}_SOURCES}
  ${${EXECUTABLE_NAME}_HEADERS}
  ${${EXECUTABLE_NAME}_PRIV_HEADERS}
  ${${EXECUTABLE_NAME}_GUI_SOURCES}
  ${${EXECUTABLE_NAME}_GUI_HEADERS}
  )
  
SOURCE_GROUP("gui\\Headers" FILES ${${EXECUTABLE_NAME}_GUI_HEADERS})
SOURCE_GROUP("gui\\Sources" FILES ${${EXECUTABLE_NAME}_GUI_SOURCES})

# Use pkgconfig dependencies
FOREACH(_PKG_DEP ${${EXECUTABLE_NAME}_PKG_DEPS})
  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} ${_PKG_DEP})
ENDFOREACH()

find_package(OpenGL REQUIRED)

TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES})

INSTALL(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)

