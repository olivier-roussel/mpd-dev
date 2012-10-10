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
#include <boost/thread.hpp>
#include <boost/asio.hpp>


class GLViewer {
public:
  GLViewer(const std::string& label, int width, int height, int fps_max);
  virtual ~GLViewer();

  void run();

  void join();

  int height() const;

  int width() const;

private:
  /**
   * Here are methods that must be redefined to customize 
   * the behavior of the viewer.
   * */

  /**
   * \fn
   * \brief Process to given event which has been caught.
   */
  virtual void processEvent(const SDL_Event& event) {}

  /**
   * \fn
   * \brief Called for each scene rendering.
   * Add your gl calls here.
   */ 
  virtual void renderScene() {}

  /**
   * \fn
   * \brief Called for each GUI rendering & handling.
   * Define your GUI over the scene here (see imgui* functions).
   */ 
  virtual void handleGUI() {}


protected:
  bool is_done() const;

  int mouse_scroll() const;

  void set_is_mouse_over_gui(bool is_over);

private:
  std::string label_;           // Viewer label
  boost::thread thread_;        // Viewer thread
  boost::asio::io_service io_service_;
  boost::asio::deadline_timer timer_;

  Eigen::Vector2i origin_pos_;  // Origin position 
  Eigen::Vector2f origin_rot_;  // Origin rotation 
  bool is_done_;
  bool is_rotate_;
  bool is_mouse_over_gui_;      // True if mouse is over the GUI
  Eigen::Vector2i mouse_pos_;   // Mouse coords on screen
  Eigen::Vector2f rot_;
  int width_;                   // Window width
  int height_;                  // Window height
  int fps_max_;                 // Maximum expected FPS
  float far_clip_;              // Camera far clipping distance. Depends on the scene.
  Eigen::Vector3f camera_pos_;  // Camera position
  Uint32 last_time_;            // Previous rendering time
  int mouse_scroll_;            // mouse scroll

  Eigen::Vector3f ray_start_;   // Ray start
  Eigen::Vector3f ray_end_;     // Ray end

  float move_f_;                // Forward moves
  float move_b_;                // Backward moves
  float move_l_;                // Left moves
  float move_r_;                // Right moves
  float move_u_;                // Upward moves
  float move_d_;                // Downwards moves


private:
  void _run();

  void _update();
  /**
   * \fn
   * \brief Initialize the core viewer components.
   */ 
  bool _init();

  void _processEvents();

  void _renderScene();           // GL rendering of the scene

  void _handleGUI();             // GUI rendering

  void _quit();
};

#endif //  MPD_DEV_GUI_GL_VIEWER_H_
