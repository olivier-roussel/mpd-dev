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

	void isRenderingPhysics(bool i_is_rendering_physics);

	boost::mutex& getPhysicsObjectsMutex();

private:

	bool render_physics_;					// true if viewer must render physics
	bool render_referential_;			// true if world referential must be rendered

  int main_scroll_;             // Scroller for main menu
  int display_scroll_;          // Scroller for display menu
  MotionPlanningAlgorithms algo_; // current algorithm for MP
  std::string env_name_;
  MPDController& mpd_controller_;  // controller

	/**
	* Internal viewer stuff
	*/
  void renderScene();
  void handleGUI();

  int env_scroll_;
  bool is_show_envs_;
  bool is_show_algos_;          // true if algo selection menu visible
	double mass_next_object_;			// mass of next object to be added
	int body_count_;							// number of bodies in the scene (excepted environment)

  std::vector<boost::filesystem::path> env_files_;  // env models files

	/**
	* Debug drawing attributes for physics
	*/
	boost::mutex debug_physics_objects_mutex_;
	std::vector<boost::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> > physics_debug_lines_;	// Lines to draw from physics as a tuple (start pos, end pos, color)
	std::vector<boost::tuple<Eigen::Vector3d, std::string> > physics_debug_text_;	// 3d texts to draw from physics
};
#endif // MPD_DEV_GUI_MPD_VIEWER_H_
