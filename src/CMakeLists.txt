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

# Setting up target
set(EXECUTABLE_NAME mpd-dev-gui)

file(GLOB ${EXECUTABLE_NAME}_HEADERS 
	${CMAKE_SOURCE_DIR}/include/${CUSTOM_HEADER_DIR}/*.h)
file(GLOB ${EXECUTABLE_NAME}_PRIV_HEADERS 
	${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB ${EXECUTABLE_NAME}_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

# App gui (TODO: make the viewer as a lib)
file(GLOB ${EXECUTABLE_NAME}_GUI_HEADERS 
  ${CMAKE_CURRENT_SOURCE_DIR}/gui/*.h)
file(GLOB ${EXECUTABLE_NAME}_GUI_SOURCES 
  ${CMAKE_CURRENT_SOURCE_DIR}/gui/*.cc)

add_executable(${EXECUTABLE_NAME}
  ${${EXECUTABLE_NAME}_SOURCES}
  ${${EXECUTABLE_NAME}_HEADERS}
  ${${EXECUTABLE_NAME}_PRIV_HEADERS}
  ${${EXECUTABLE_NAME}_GUI_SOURCES}
  ${${EXECUTABLE_NAME}_GUI_HEADERS}
  )
  
source_group("gui\\Headers" FILES ${${EXECUTABLE_NAME}_GUI_HEADERS})
source_group("gui\\Sources" FILES ${${EXECUTABLE_NAME}_GUI_SOURCES})

target_link_libraries(${EXECUTABLE_NAME} ${LIBS})

install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)

