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

message("Boost_INCLUDE_DIR=${Boost_INCLUDE_DIR}")
message("Boost_LIBRARY_DIRS=${Boost_LIBRARY_DIRS}")
message("Boost_LIBRARIES=${Boost_LIBRARIES}")

SET(EXECUTABLE_NAME mpd-dev-gui)

ADD_EXECUTABLE(${EXECUTABLE_NAME}
  algorithm.cc
  bullet_engine_wrapper.cc
  bullet_utils.cc
  environment.cc
  main.cc
  mpd_controller.cc
  #  physics_manager.cc
  physics_thread.cc
  polygon_soup.cc
  gui/imgui.cc
  gui/imgui_render_gl.cc
  gui/ugly_font.cc
  gui/utils.cc
  gui/gl_viewer.cc
  gui/render_helpers.cc
  gui/mpd_viewer.cc
  gui/file_utils.cc
  gui/polygon_soup_renderer.cc
  )

PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} eigen3)
#PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} assimp)
PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} sdl)
PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} bullet)
#PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} ompl)

find_package(OpenGL REQUIRED)

message("OpenGL_INCLUDE_DIR=${OPENGL_INCLUDE_DIR}")

message("OpenGL_LIBRARIES=${OPENGL_LIBRARIES}")

TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${OMPL_PHYSICS_LIBS} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES})

INSTALL(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)

