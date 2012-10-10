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

class MPDViewer : public GLViewer {

public:
  MPDViewer(const std::string& label, int width, int height, int fps_max);
  virtual ~MPDViewer();

private:
  void renderScene();

  void handleGUI();


  int main_scroll_;             // Scroller for main menu
  bool is_show_algos_;          // true if algo selection menu visible
  MotionPlanningAlgorithms algo_; // current algorithm for MP
  bool is_show_envs_;
  std::string env_name_;
};
#endif // MPD_DEV_GUI_MPD_VIEWER_H_
