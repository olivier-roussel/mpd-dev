/**
* Copyright (c) 2012 CNRS
* Author: Olivier Roussel
*
* This file is part of the MPD-dev package.
* MPD-dev is free software: you can redistribute it
* and/or modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation, either version
* 3 of the License, or (at your option) any later version.
*
* MPD-dev is distributed in the hope that it will be
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Lesser Public License for more details.  You should have
* received a copy of the GNU Lesser General Public License along with
* MPD-dev.  If not, see
* <http://www.gnu.org/licenses/>.
**/

#include "gui/mpd_viewer.h"

#include "gui/imgui.h"
#include "gui/render_helpers.h"
#include "gui/imgui_render_gl.h"

static const int kMainMenuWidth = 250;
static const int kMainMenuHeight = 600;

MPDViewer::MPDViewer(const std::string& label, int width, int height, int fps_max):
  GLViewer(label, width, height, fps_max),
  main_scroll_(0), is_show_algos_(false), algo_(MPA_KPIECE_OMPL), 
  is_show_envs_(false), env_name_("")
{}

MPDViewer::~MPDViewer()
{}

void MPDViewer::renderScene()
{
  const float radius = 2.f;
  const float h = 5.f;
  drawCylinder(-radius, -radius, 0.f, radius, radius, h, Eigen::Vector4f(1.f, 0.f, 0.f, 1.f));
}

void MPDViewer::handleGUI()
{
  unsigned char mouse_buttons = 0;
  if (SDL_GetMouseState(0,0) & SDL_BUTTON_LMASK)
    mouse_buttons |= IMGUI_MBUT_LEFT;
  if (SDL_GetMouseState(0,0) & SDL_BUTTON_RMASK)
    mouse_buttons |= IMGUI_MBUT_RIGHT;

  //imguiBeginFrame(mouse_pos_.x(), mouse_pos_.y(), mouse_buttons, mouse_scroll_);
  imguiBeginFrame(0, 0, mouse_buttons, mouse_scroll());

  // process GUI elements
  // Main menu
  if (imguiBeginScrollArea("Main menu", width() - kMainMenuWidth-10, height() - kMainMenuHeight -10, kMainMenuWidth, kMainMenuHeight, &main_scroll_)) 
    set_is_mouse_over_gui(true);

  imguiLabel("Environment");
  if (imguiButton(env_name_.c_str()))
  {
    is_show_envs_ = !is_show_envs_;
//    if (is_show_envs_)
//      scanDirectory(kEnvDir.c_str(), kEnvExtensions, env_files_);
  }

  imguiLabel("Physical engine");
  imguiValue("Bullet");

  imguiLabel("Algorithm");
  if (imguiButton(getMotionPlanningAlgorithmName(algo_).c_str()))
    is_show_algos_ = !is_show_algos_;
  imguiSeparator();

  imguiEndScrollArea();

  // end of GUI main frame
  imguiEndFrame();
  imguiRenderGLDraw();

}
