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
#include_directories(.)

SET(EXECUTABLE_NAME ompl-physics-tester)

ADD_EXECUTABLE(${EXECUTABLE_NAME}
  main.cc
  polygon_soup.cc
  algorithm.cc
  gui/imgui.cc
  gui/imgui_render_gl.cc
  gui/ugly_font.cc
  gui/utils.cc
  gui/gl_viewer.cc
  gui/render_helpers.cc
  gui/mpd_viewer.cc
  )

PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} eigen3)
PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} assimp)
PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} sdl)
PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} bullet)
PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} ompl)

TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${OMPL_PHYSICS_LIBS})

INSTALL(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)

