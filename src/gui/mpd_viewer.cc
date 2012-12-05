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

#include <boost/lexical_cast.hpp>

#include "mpd/mpd_controller.h"

#include "gui/imgui.h"
#include "gui/render_helpers.h"
#include "gui/imgui_render_gl.h"
#include "gui/file_utils.h"
#include "gui/polygon_soup_renderer.h"
#include "src/config.h"

static const int kMenuWidth = 250;
static const float kMainMenuHeightRatio = 0.75f;
static const float kDisplayMenuHeightRatio = 0.25f;

static const int kFileSelectorHeight = 300;
static const int kFileSelectorWidth = 250;
static const std::string kEnvDir = std::string(kShareDir) + "/resources/environments/";
static const std::string kBodiesDir = std::string(kShareDir) + "/resources/bodies/";

static const size_t kDebugDefaultPhysicsLinesToDraw = 4096;
static const size_t kDebugDefaultPhysicsTextToDraw = 128;
static const Eigen::Vector3d kDebugTextColor(1., 0.5, 0.);
static const Eigen::Vector4f kHistoPhysicsColor(0., 0.8, 1., 1.);
static const Eigen::Vector4f kHistoGraphicsColor(0., 1., 0., 1.);

static const double kBodyFallingHeight = 5.;

MPDViewer::MPDViewer(const std::string& label, int width, int height, int fps_max, MPDController& mpd_controller):
  GLViewer(label, width, height, fps_max),
  main_scroll_(0), display_scroll_(0), algo_(MPA_KPIECE_OMPL), 
  //is_show_envs_(false), is_show_rigid_bodies_(false), is_show_soft_bodies_(false), is_show_algos_(false), 
	env_scroll_(0), env_name_("Pick from file..."), env_files_(),
  mpd_controller_(mpd_controller), physics_debug_lines_(), 
	render_physics_(true), render_referential_(true),
	mass_next_object_(1.), body_count_(0), menu_popup_y_(0),
	current_contextual_menu_(MPDViewer::CTM_NONE),
	physics_histogram_(1), graphics_histogram_(1),
	physics_histo_last_step_(0)
	{
		physics_debug_lines_.reserve(kDebugDefaultPhysicsLinesToDraw);

		std::vector<Eigen::Vector4f> color_set_histo;
		color_set_histo.push_back(kHistoPhysicsColor);
		physics_histogram_.setColorSet(color_set_histo);
		color_set_histo.clear();
		color_set_histo.push_back(kHistoGraphicsColor);
		graphics_histogram_.setColorSet(color_set_histo);
	}

MPDViewer::~MPDViewer()
{}

void MPDViewer::isRenderingPhysics(bool i_is_rendering_physics)
{
	render_physics_ = i_is_rendering_physics;
}

void MPDViewer::renderScene()
{
  // render world referential
	if (render_referential_)
		renderReferential(Eigen::Vector3d::Zero(), 1.f, 2.f);

  if (mpd_controller_.isEnvironmentSet())	// we know environement have an identity transform
    drawPolygonSoup(mpd_controller_.environment().polygon_soup(), mpd_controller_.environment().transform(), Eigen::Vector4d(1., 1., 1., 1.));

	if (mpd_controller_.isPhysicsInitialized())
	{
		for (std::map<std::string, RigidBody*>::const_iterator it_body = mpd_controller_.physics_engine().rigid_bodies().begin() ; it_body != mpd_controller_.physics_engine().rigid_bodies().end() ; ++it_body)
		{
			//if (it_body->second->name() != mpd_controller_.environment().name)
				drawPolygonSoup(it_body->second->polygon_soup(), it_body->second->transform(), Eigen::Vector4d(1., 0.5, 0., 1.));
		}
	}
	// render physics (debug)
	if (render_physics_)
	{
		boost::mutex::scoped_lock lock(debug_physics_objects_mutex_);
		drawLineArray(physics_debug_lines_);
		for (size_t i = 0 ; i < physics_debug_text_.size() ; ++i)
		{
			draw3dText(physics_debug_text_[i].get<0>(), physics_debug_text_[i].get<1>(), kDebugTextColor);
		}
	}
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
	const int main_menu_height = static_cast<int>(height() * kMainMenuHeightRatio) - 5;
  if (imguiBeginScrollArea("Main menu", width() - kMenuWidth-10, height() - main_menu_height, kMenuWidth, main_menu_height, &main_scroll_)) 
    set_is_mouse_over_gui(true);

  imguiLabel("Environment");
  if (imguiButton(env_name_.c_str()))
  {
    current_contextual_menu_ = current_contextual_menu_ == MPDViewer::CTM_ENV_FROM_MESH ? MPDViewer::CTM_NONE : MPDViewer::CTM_ENV_FROM_MESH;
    if (current_contextual_menu_ == MPDViewer::CTM_ENV_FROM_MESH)
    {
      env_files_ = listDirectory(kEnvDir, ".obj", false);
			menu_popup_y_ =  mouse_pos().y();
    }
  }
  if (imguiButton("Switch Y/Z axis", mpd_controller_.isEnvironmentSet()))
    mpd_controller_.switchEnvironmentAxis();

  if (imguiButton("Invert triangles", mpd_controller_.isEnvironmentSet()))
    mpd_controller_.invertEnvironmentTriangles();

  imguiLabel("Physical engine");
  imguiValue("Bullet");

	if (imguiButton("Start", mpd_controller_.isEnvironmentSet() && !mpd_controller_.isPhysicsInitialized()))
		mpd_controller_.initPhysics(PhysicsEngine::PE_BULLET);

	double phy_time_step = static_cast<double>(mpd_controller_.physics_time_step());
	imguiSlider("Physics time step", &phy_time_step, 1., 100., 1.);
	unsigned int new_phy_time_step = static_cast<unsigned int>(phy_time_step);
	if (new_phy_time_step != mpd_controller_.physics_time_step())
		mpd_controller_.set_physics_time_step(new_phy_time_step);

  imguiLabel("Algorithm");
  if (imguiButton(getMotionPlanningAlgorithmName(algo_).c_str()))
	{
    current_contextual_menu_ = current_contextual_menu_ == MPDViewer::CTM_MP_ALGO_SELECT ? MPDViewer::CTM_NONE : MPDViewer::CTM_MP_ALGO_SELECT;
	}
  imguiSeparator();

	imguiSlider("Mass", &mass_next_object_, 0., 10., 0.1);
	if (imguiButton("Add rigid box", mpd_controller_.isPhysicsInitialized()))
	{
		Eigen::Affine3d box_t(Eigen::Affine3d::Identity());
		box_t.translate(Eigen::Vector3d(0., 0., kBodyFallingHeight));			// add body falling from the sky :)
		mpd_controller_.addRigidBox("rigid_box_" + boost::lexical_cast<std::string>(body_count_), mass_next_object_, box_t);
		++body_count_;
	}

	if (imguiButton("Add soft box", mpd_controller_.isPhysicsInitialized()))
	{
		Eigen::Affine3d box_t(Eigen::Affine3d::Identity());
		box_t.translate(Eigen::Vector3d(0., 0., kBodyFallingHeight));			// add body falling from the sky :)
		mpd_controller_.addSoftBox("soft_box_" + boost::lexical_cast<std::string>(body_count_), mass_next_object_, box_t);
		++body_count_;
	}

	// add rigid body from mesh file
	if (imguiButton("Add rigid body from mesh", mpd_controller_.isPhysicsInitialized()))
  {
    current_contextual_menu_ = current_contextual_menu_ == MPDViewer::CTM_RIGID_BODY_FROM_MESH ? MPDViewer::CTM_NONE : MPDViewer::CTM_RIGID_BODY_FROM_MESH;
    if (current_contextual_menu_ == MPDViewer::CTM_RIGID_BODY_FROM_MESH)
		{
      bodies_files_ = listDirectory(kBodiesDir, ".obj", false);
			menu_popup_y_ =  mouse_pos().y();
		}
  }

	// add soft body from mesh file
	if (imguiButton("Add soft body from mesh", mpd_controller_.isPhysicsInitialized()))
  {
    current_contextual_menu_ = current_contextual_menu_ == MPDViewer::CTM_SOFT_BODY_FROM_MESH ? MPDViewer::CTM_NONE : MPDViewer::CTM_SOFT_BODY_FROM_MESH;
    if (current_contextual_menu_ == MPDViewer::CTM_SOFT_BODY_FROM_MESH)
		{
      bodies_files_ = listDirectory(kBodiesDir, ".obj", false);
			menu_popup_y_ =  mouse_pos().y();
		}
  }
  imguiEndScrollArea();
	// end of main menu

	// display options
	const int display_menu_height = static_cast<int>(height() * kDisplayMenuHeightRatio) - 5;
	if (imguiBeginScrollArea("Display options", width() - kMenuWidth-10, 0, kMenuWidth, display_menu_height, &display_scroll_)) 
    set_is_mouse_over_gui(true);

	if (imguiCheck("Render physics", render_physics_))
		render_physics_ = !render_physics_;

	if (imguiCheck("Render world ref", render_referential_))
		render_referential_ = !render_referential_;
  
	if (imguiButton(getRenderingModeName(render_mode()).c_str()))
		set_render_mode(static_cast<RenderingMode_t>((static_cast<int>(render_mode()) + 1) % static_cast<int>(RM_NB_RENDERING_MODES)));

  imguiEndScrollArea();

	// perfs infos
	

  // display environments filelist if opened
  if (current_contextual_menu_ == MPDViewer::CTM_ENV_FROM_MESH)
  {
    if (imguiBeginScrollArea("Select environment file", width()-10-kMenuWidth-10-kFileSelectorWidth, menu_popup_y_ - kFileSelectorHeight, kFileSelectorWidth, kFileSelectorHeight, &env_scroll_))
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
      if (mpd_controller_.loadEnvironment(env_files_[env_to_load]))
			{
				std::cout << "[PROGRESS] Loaded environment: nverts = " << mpd_controller_.environment().polygon_soup().verts().size() << " / nfaces = " << mpd_controller_.environment().polygon_soup().tris().size() << " / normals = " << mpd_controller_.environment().polygon_soup().normals().size() << std::endl;
				// update camera & fog to mesh bounds
				const AABB env_aabb = mpd_controller_.environment().polygon_soup().aabb();
				set_far_clip(static_cast<float>((env_aabb.bmax - env_aabb.bmin).norm() * 0.5));
				set_camera_pos((env_aabb.bmax + env_aabb.bmin).cast<float>()/ 2.f + Eigen::Vector3f::Identity()*far_clip());
				set_far_clip(far_clip() * 20);
				set_rot(Eigen::Vector2f(-45.f, -135.f));
				//set_rot(Eigen::Vector2f(45.f, 45.f));
				glFogf(GL_FOG_START, far_clip()*0.2f);
				glFogf(GL_FOG_END, far_clip()*1.25f);
			}else
        std::cout << "[ERROR] Could not load environment from " << env_name_ << std::endl;

			current_contextual_menu_ = MPDViewer::CTM_NONE;
    }
    imguiEndScrollArea();
  }

	// add rigid body from file menu
	if (current_contextual_menu_ == MPDViewer::CTM_RIGID_BODY_FROM_MESH)
	{
		if (imguiBeginScrollArea("Select rigid body mesh", width()-10-kMenuWidth-10-kFileSelectorWidth, menu_popup_y_ - kFileSelectorHeight, kFileSelectorWidth, kFileSelectorHeight, &env_scroll_))
      set_is_mouse_over_gui(true);
		int mesh_to_load = -1;
		for (int i = 0; i < bodies_files_.size() && mesh_to_load < 0; ++i)
    {
      if (imguiItem(bodies_files_[i].filename().string().c_str()))
        mesh_to_load = i;
    }
		if (mesh_to_load != -1)
    {
			const std::string body_name = "body_" + boost::lexical_cast<std::string>(body_count_++);

			Eigen::Affine3d body_t(Eigen::Affine3d::Identity());
			body_t.translate(Eigen::Vector3d(0., 0., kBodyFallingHeight));			// add body falling from the sky :)
			if (mpd_controller_.addRigidBodyFromMeshFile(body_name, bodies_files_[mesh_to_load], mass_next_object_, body_t))
        std::cout << "[PROGRESS] Rigid body added from mesh " << bodies_files_[mesh_to_load].filename().string() << ": nverts = " << mpd_controller_.environment().polygon_soup().verts().size() << 
				" / ntris = " << mpd_controller_.environment().polygon_soup().tris().size() << std::endl;
			else
        std::cout << "[ERROR] Could not load rigid body mesh from file " << bodies_files_[mesh_to_load].filename().string() << std::endl;

			current_contextual_menu_ = MPDViewer::CTM_NONE;
		}
    imguiEndScrollArea();
	}

	// add soft body from file menu
	if (current_contextual_menu_ == MPDViewer::CTM_SOFT_BODY_FROM_MESH)
	{
		if (imguiBeginScrollArea("Select soft body mesh", width()-10-kMenuWidth-10-kFileSelectorWidth, menu_popup_y_ - kFileSelectorHeight, kFileSelectorWidth, kFileSelectorHeight, &env_scroll_))
      set_is_mouse_over_gui(true);
		int mesh_to_load = -1;
		for (int i = 0; i < bodies_files_.size() && mesh_to_load < 0; ++i)
    {
      if (imguiItem(bodies_files_[i].filename().string().c_str()))
        mesh_to_load = i;
    }
		if (mesh_to_load != -1)
    {
			const std::string body_name = "body_" + boost::lexical_cast<std::string>(body_count_++);

			Eigen::Affine3d body_t(Eigen::Affine3d::Identity());
			body_t.translate(Eigen::Vector3d(0., 0., kBodyFallingHeight));			// add body falling from the sky :)
			if (mpd_controller_.addSoftBodyFromMeshFile(body_name, bodies_files_[mesh_to_load], mass_next_object_, body_t))
        std::cout << "[PROGRESS] Soft body added from mesh " << bodies_files_[mesh_to_load].filename().string() << ": nverts = " << mpd_controller_.environment().polygon_soup().verts().size() << 
				" / ntris = " << mpd_controller_.environment().polygon_soup().tris().size() << std::endl;
			else
        std::cout << "[ERROR] Could not load soft body mesh from file " << bodies_files_[mesh_to_load].filename().string() << std::endl;

			current_contextual_menu_ = MPDViewer::CTM_NONE;
		}
    imguiEndScrollArea();
	}

  // end of GUI main frame
  imguiEndFrame();
  imguiRenderGLDraw();

	 // Rendering histogram over GUI
	// first check if they need an update of records
	if (mpd_controller_.isPhysicsInitialized() && mpd_controller_.physics_engine().niter() != physics_histo_last_step_)
	{
		physics_histo_last_step_ = mpd_controller_.physics_engine().niter();
		const double normalized_last_step_cpu_time = mpd_controller_.physics_engine().last_step_cpu_time() / mpd_controller_.physics_engine().last_step_simulation_time();
		physics_histogram_.addMeasure(static_cast<float>(normalized_last_step_cpu_time));
	}
	glPushMatrix();
	glTranslatef(10, height() - 10 - physics_histogram_.getHeight(), 0);
	physics_histogram_.render();
	glPopMatrix();
	glPushMatrix();
	glTranslatef(10 + physics_histogram_.getWidth() + 10, height() - 10 - physics_histogram_.getHeight(), 0);
	graphics_histogram_.render();
	glPopMatrix();
}

void MPDViewer::addPhysicsLine(const Eigen::Vector3d& i_from, const Eigen::Vector3d& i_to, const Eigen::Vector3d& i_color)
{
	physics_debug_lines_.push_back(boost::make_tuple(i_from, i_to, i_color));
}

void MPDViewer::addPhysicsText(const Eigen::Vector3d& i_pos, const std::string& i_text)
{
	physics_debug_text_.push_back(boost::make_tuple(i_pos, i_text));
}

void MPDViewer::clearPhysicsObjects()
{
	physics_debug_lines_.clear();
	physics_debug_lines_.reserve(kDebugDefaultPhysicsLinesToDraw);
	physics_debug_text_.clear();
	physics_debug_text_.reserve(kDebugDefaultPhysicsTextToDraw);
}

boost::mutex& MPDViewer::getPhysicsObjectsMutex()
{
	return debug_physics_objects_mutex_;
}
