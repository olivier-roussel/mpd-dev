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

#ifndef MPD_DEV_GUI_GL_VIEWER_H_
#define MPD_DEV_GUI_GL_VIEWER_H_

#include <Eigen/Core>
#include "SDL.h"
#include "SDL_opengl.h"

class GLViewer {
public:
  GLViewer(const std::string& label);
  virtual ~GLViewer();

  bool init();
  void quit();
  void processEvents();

  void renderScene();           // GL rendering of the scene
  void handleGUI();             // GUI rendering

  bool is_done() const;

protected:
  std::string label_;           // Viewer label
  Eigen::Vector2i origin_pos_;  // Origin position 
  Eigen::Vector2f origin_rot_;  // Origin rotation 
  bool is_done_;
  bool is_rotate_;
  bool is_mouse_over_gui_;      // True if mouse is over the GUI
  Eigen::Vector2i mouse_pos_;   // Mouse coords on screen
  Eigen::Vector2f rot_;
  int width_;                   // Window width
  int height_;                  // Window height
  float far_clip_;              // Camera far clipping distance. Depends on the scene.
  Eigen::Vector3f camera_pos_;  // Camera position
  Uint32 last_time_;            // Previous rendering time
 // Uint32 time_;
  int mouse_scroll_;            // mouse scroll

  Eigen::Vector3f ray_start_;   // Ray start
  Eigen::Vector3f ray_end_;     // Ray end

  float move_f_;                // Forward moves
  float move_b_;                // Backward moves
  float move_l_;                // Left moves
  float move_r_;                // Right moves
  float move_u_;                // Upward moves
  float move_d_;                // Downwards moves

  int main_scroll_;             // Scroller for main menu
  
};

#endif //  MPD_DEV_GUI_GL_VIEWER_H_
