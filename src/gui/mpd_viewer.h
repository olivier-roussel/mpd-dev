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

#ifndef MPD_DEV_GUI_MPD_VIEWER_H_
#define MPD_DEV_GUI_MPD_VIEWER_H_

#include "gui/gl_viewer.h"
#include "mpd/algorithm.h"
#include "gui/perf_histogram.h"
#include "gui/render_helpers.h"

#include <boost/filesystem/path.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/thread/mutex.hpp>

class MPDController;


class MPDViewer : public GLViewer 
{
public:

	/**
	* Constructors, destructors
	*/
  MPDViewer(const std::string& label, int width, int height, int fps_max, MPDController& mpd_controller);
  virtual ~MPDViewer();

	/**
	* Debug drawing methods for physics _ to enable physics engine to draw within the viewer thread
	*/
	void addPhysicsLine(const Eigen::Vector3d& i_from, const Eigen::Vector3d& i_to, const Eigen::Vector3d& i_color);

	void addPhysicsText(const Eigen::Vector3d& i_pos, const std::string& i_text);

	void clearPhysicsObjects();

	boost::mutex& getPhysicsObjectsMutex();

private:

	RenderingConfiguration render_cfg_;

  int main_scroll_;             // Scroller for main menu
  int display_scroll_;          // Scroller for display menu
	int soft_params_scroll_;			// Scroller for soft params menu
  MotionPlanningAlgorithms algo_; // current algorithm for MP
  std::string env_name_;
  MPDController& mpd_controller_;  // controller
	bool show_soft_parameters_;

	/**
	* Internal viewer stuff
	*/
  void renderScene();
  void handleGUI();

	enum ContextualMenu_t
	{
		CTM_NONE = 0,
		CTM_ENV_FROM_MESH,
		CTM_RIGID_BODY_FROM_MESH,
		CTM_SOFT_BODY_FROM_MESH,
		CTM_MP_ALGO_SELECT,
		CTM_NB_CONTEXTUAL_MENUS
	};

	int menu_popup_y_;
  int env_scroll_;
	ContextualMenu_t current_contextual_menu_;
	double mass_next_object_;			// mass of next object to be added
	int body_count_;							// number of bodies in the scene (excepted environment)

  std::vector<boost::filesystem::path> env_files_;  // env models files
  std::vector<boost::filesystem::path> bodies_files_;  // bodies models files

	PerformanceHistogram physics_histogram_;
	PerformanceHistogram graphics_histogram_;
	unsigned int physics_histo_last_step_;

	/**
	* Debug drawing attributes for physics
	*/
	boost::mutex debug_physics_objects_mutex_;
	std::vector<boost::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> > physics_debug_lines_;	// Lines to draw from physics as a tuple (start pos, end pos, color)
	std::vector<boost::tuple<Eigen::Vector3d, std::string> > physics_debug_text_;	// 3d texts to draw from physics
};
#endif // MPD_DEV_GUI_MPD_VIEWER_H_
