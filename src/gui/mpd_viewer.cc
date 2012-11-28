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
#include "gui/file_utils.h"
#include "gui/polygon_soup_renderer.h"
#include "src/config.h"

static const int kMainMenuWidth = 250;
static const int kMainMenuHeight = 600;

static const int kFileSelectorHeight = 300;
static const int kFileSelectorWidth = 250;
static const std::string kEnvDir = std::string(kShareDir) + "/resources/environments/";

MPDViewer::MPDViewer(const std::string& label, int width, int height, int fps_max, MPDController& mpd_controller):
  GLViewer(label, width, height, fps_max),
  main_scroll_(0), is_show_algos_(false), algo_(MPA_KPIECE_OMPL), 
  is_show_envs_(false), env_scroll_(0), env_name_("Pick from file..."), env_files_(),
  mpd_controller_(mpd_controller)
{}

MPDViewer::~MPDViewer()
{}

void MPDViewer::renderScene()
{
  const float radius = 2.f;
  const float h = 5.f;
//  drawCylinder(-radius, -radius, 0.f, radius, radius, h, Eigen::Vector4f(1.f, 0.f, 0.f, 0.2f));

  // render axis
  renderReferential(Eigen::Vector3d::Zero(), 1.f, 2.f);
  if (mpd_controller_.isEnvironmentSet())
    renderPolygonSoup(mpd_controller_.environment().polygon_soup());
}

void MPDViewer::handleGUI()
{
  unsigned char mouse_buttons = 0;
  if (SDL_GetMouseState(0,0) & SDL_BUTTON_LMASK)
    mouse_buttons |= IMGUI_MBUT_LEFT;
  if (SDL_GetMouseState(0,0) & SDL_BUTTON_RMASK)
    mouse_buttons |= IMGUI_MBUT_RIGHT;

  imguiBeginFrame(mouse_pos().x(), mouse_pos().y(), mouse_buttons, mouse_scroll());

  // process GUI elements
  // Main menu
  if (imguiBeginScrollArea("Main menu", width() - kMainMenuWidth-10, height() - kMainMenuHeight -10, kMainMenuWidth, kMainMenuHeight, &main_scroll_)) 
    set_is_mouse_over_gui(true);

  imguiLabel("Environment");
  if (imguiButton(env_name_.c_str()))
  {
    is_show_envs_ = !is_show_envs_;
    if (is_show_envs_)
    {
      env_files_ = listDirectory(kEnvDir, ".obj", false);
//      for (size_t i = 0 ; i < PolygonSoup::SFF_NB_SUPPORTED_FORMATS ; ++i)
    }
  }
  if (imguiButton("Switch Y/Z axis", mpd_controller_.isEnvironmentSet()))
    mpd_controller_.switchEnvironmentAxis();

  if (imguiButton("Invert triangles", mpd_controller_.isEnvironmentSet()))
    mpd_controller_.invertEnvironmentTriangles();

  imguiLabel("Physical engine");
  imguiValue("Bullet");

  imguiLabel("Algorithm");
  if (imguiButton(getMotionPlanningAlgorithmName(algo_).c_str()))
    is_show_algos_ = !is_show_algos_;
  imguiSeparator();

  imguiEndScrollArea();

  // display environments filelist if opened
  if (is_show_envs_)
  {
    if (imguiBeginScrollArea("Select environment file", width()-10-kMainMenuWidth-10-kFileSelectorWidth, height()-10-kFileSelectorHeight - 40, kFileSelectorWidth, kFileSelectorHeight, &env_scroll_))
      set_is_mouse_over_gui(true);
    int env_to_load = -1;
    for (int i = 0; i < env_files_.size() && env_to_load < 0; ++i)
    {
      if (imguiItem(env_files_[i].filename().string().c_str()))
        env_to_load = i;
    }
    if (env_to_load != -1)
    {
      env_name_ = env_files_[env_to_load].filename().string();
      if (!mpd_controller_.loadEnvironment(env_files_[env_to_load]))
        std::cout << "Could not load environment from " << env_name_ << std::endl;
      else{
        std::cout << "env loaded! nverts=" << mpd_controller_.environment().polygon_soup().verts().size() << " / nfaces=" << mpd_controller_.environment().polygon_soup().tris().size() << " / normals= " << mpd_controller_.environment().polygon_soup().normals().size() << std::endl;
        // update camera & fog to mesh bounds
        const AABB env_aabb = mpd_controller_.environment().getAABB();
        set_far_clip(static_cast<float>((env_aabb.bmax - env_aabb.bmin).norm() * 0.5));
        set_camera_pos((env_aabb.bmax + env_aabb.bmin).cast<float>()/ 2.f + Eigen::Vector3f::Identity()*far_clip());
        set_far_clip(far_clip() * 20);
        set_rot(Eigen::Vector2f(-45.f, -135.f));
        //set_rot(Eigen::Vector2f(45.f, 45.f));
        glFogf(GL_FOG_START, far_clip()*0.2f);
        glFogf(GL_FOG_END, far_clip()*1.25f);
       }
       is_show_envs_ = false;
    }
    imguiEndScrollArea();
  }
  
  // end of GUI main frame
  imguiEndFrame();
  imguiRenderGLDraw();

}
