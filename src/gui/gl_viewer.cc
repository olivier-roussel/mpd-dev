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

#include "gui/gl_viewer.h"
#include "gui/imgui_render_gl.h"
#include "gui/imgui.h"
#include "gui/utils.h"
#include "src/config.h"

#include <iostream>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

static const float kNearClip = 0.01f;
static const float kFogColor[4] = { 0.32f, 0.25f, 0.25f, 1.f };
static const float kBackColor[3] = {0.5f, 0.5f, 0.5f};
static const float kKeybSpeed = 22.f;

static const std::string kEnvDir = std::string(kShareDir) + "/ressources/environments/";

GLViewer::GLViewer(const std::string& label, int width, int height, int fps_max):
  label_(label), origin_pos_(0, 0), origin_rot_(0.f, 0.f), is_done_(false), is_rotate_(false), 
  mouse_pos_(0, 0), rot_(45.f, 45.f), width_(width), height_(height), fps_max_(fps_max),
  far_clip_(500.f), camera_pos_(0.f, 0.f, 0.f), last_time_(0), mouse_scroll_(0),
  move_f_(0.f), move_b_(0.f), move_l_(0.f), move_r_(0.f), move_u_(0.f), move_d_(0.f),
  timer_(io_service_), io_service_()
{
}

GLViewer::~GLViewer()
{
}

void GLViewer::_quit()
{
  imguiRenderGLDestroy();
  SDL_Quit();
}

bool GLViewer::is_done() const
{
  return is_done_;
}

void GLViewer::run()
{
  thread_ = boost::thread(&GLViewer::_run, this);
}

void GLViewer::join()
{
  thread_.join();
}

int GLViewer::mouse_scroll() const
{
  return mouse_scroll_;
}

void GLViewer::set_is_mouse_over_gui(bool is_over)
{
  is_mouse_over_gui_ = is_over;
}

int GLViewer::width() const
{
  return width_;
}

int GLViewer::height() const
{
  return height_;
}

void GLViewer::_run()
{
  if (_init())
  {
    const int refresh_time = fps_max_ <= 0 ? 0 : 1000 / fps_max_;
    timer_.expires_from_now(boost::posix_time::milliseconds(refresh_time));
    timer_.async_wait(boost::bind(&GLViewer::_update, this));
    io_service_.run();
  }else
    std::cout << "Could not initialize the GL viewer." << std::endl;
}

void GLViewer::_update()
{
  _processEvents();
  _renderScene();
  _handleGUI();

  // schedule next update excepted if is_done() condition reached
  if (!is_done())
  {
    const int refresh_time = fps_max_ <= 0 ? 0 : 1000 / fps_max_;
    timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(refresh_time));
    timer_.async_wait(boost::bind(&GLViewer::_update, this));
  }else
    _quit();
}

bool GLViewer::_init()
{

   // Init SDL
   if (SDL_Init(SDL_INIT_EVERYTHING) != 0) 
   {
     std::cout << "Could not initialise SDL\n" << std::endl;
     return false;
   }
 
   // Init SDL_OpenGL
   SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
   SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
   SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
   SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
   SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
   SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
 
   if (height_ <= 0 || width_ <= 0)
   {
     const SDL_VideoInfo* vi = SDL_GetVideoInfo();
     width_ = vi->current_w - 20;
     height_ = vi->current_h - 80;
   }
   SDL_Surface* screen = SDL_SetVideoMode(width_, height_, 0, SDL_OPENGL);
   if (!screen)
   {
     std::cout << "Could not initialise SDL opengl" << std::endl;
     return false;
   }
 
   SDL_WM_SetCaption(label_.c_str(), 0);
 
   std::string font_file = std::string(kShareDir) + "/FreeMonoBold.ttf";
   if (!imguiRenderGLInit(font_file.c_str()))
   {
     std::cout << "Could not init GUI renderer." << std::endl;
     SDL_Quit();
     return false;
   }
 
   // Init OpenGL
   glEnable(GL_CULL_FACE);
   glEnable(GL_FOG);
   glFogi(GL_FOG_MODE, GL_LINEAR);
   glFogf(GL_FOG_START,20);
   glFogf(GL_FOG_END, 100);
   glFogfv(GL_FOG_COLOR, kFogColor);
 
   glEnable(GL_POINT_SMOOTH);
   glEnable(GL_LINE_SMOOTH);
 
   // init locals
   last_time_ = SDL_GetTicks();
   return true;
}

void GLViewer::_processEvents()
{
  SDL_Event event;
  mouse_scroll_ = 0;
  while(SDL_PollEvent(&event))
  {
    switch(event.type)
    {
    case SDL_KEYDOWN:
      // Handle key press 
      if (event.key.keysym.sym == SDLK_ESCAPE)
        is_done_ = true;
      break;
    case SDL_MOUSEBUTTONDOWN:
      // Handle mouse clicks here.
      if (!is_mouse_over_gui_)
      {
        if (event.button.button == SDL_BUTTON_LEFT)
        {
          // Rotate view
          is_rotate_ = true;
          origin_pos_ = mouse_pos_;
          origin_rot_ = rot_;
        }
        else if (event.button.button == SDL_BUTTON_RIGHT)
        {
          // todo : process selection
        }
      }
 
      if (event.button.button == SDL_BUTTON_WHEELUP)
        --mouse_scroll_;
 
      if (event.button.button == SDL_BUTTON_WHEELDOWN)
        ++mouse_scroll_;
      break;
    case SDL_MOUSEBUTTONUP:
      if(event.button.button == SDL_BUTTON_LEFT)
        is_rotate_ = false;
      break;
    case SDL_MOUSEMOTION:
      mouse_pos_.x() = event.motion.x;
      mouse_pos_.y() = height_-1 - event.motion.y;
      if (is_rotate_)
      {
        const int dx = mouse_pos_.x() - origin_pos_.x();
        const int dy = mouse_pos_.y() - origin_pos_.y();
        rot_.x() = origin_rot_.x() - static_cast<float>(dy)*0.25f;
        rot_.y() = origin_rot_.y() - static_cast<float>(dx)*0.25f;
      }
      break;
    case SDL_QUIT:
      is_done_ = true;
      break;
    default:
      break;
    } // end switch event type
    processEvent(event); // user defined behavior
  } // end while poll event

}

void GLViewer::_renderScene()
{
  // update times
  Uint32 time = SDL_GetTicks();
  const float dt = static_cast<float>(time - last_time_) / 1000.f;
  last_time_ = time;

  // Handle keyboard movement.
  Uint8* keystate = SDL_GetKeyState(NULL);
  move_f_ = clamp(move_f_ + dt * 4 * (keystate[SDLK_z] ? 1 : -1), 0.0f, 1.0f);
  move_b_ = clamp(move_b_ + dt * 4 * (keystate[SDLK_s] ? 1 : -1), 0.0f, 1.0f);
  move_l_ = clamp(move_l_ + dt * 4 * (keystate[SDLK_q] ? 1 : -1), 0.0f, 1.0f);
  move_r_ = clamp(move_r_ + dt * 4 * (keystate[SDLK_d] ? 1 : -1), 0.0f, 1.0f);
  move_u_ = clamp(move_u_ + dt * 4 * (keystate[SDLK_r] ? 1 : -1), 0.0f, 1.0f);
  move_d_ = clamp(move_d_ + dt * 4 * (keystate[SDLK_f] ? 1 : -1), 0.0f, 1.0f);

  // render GL
  glViewport(0, 0, width_, height_);
  glClearColor(kBackColor[0], kBackColor[1], kBackColor[2], 1.0f);
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_TEXTURE_2D);

  glEnable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(50.0f, static_cast<float>(width_)/static_cast<float>(height_), kNearClip, far_clip_);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glRotatef(rot_.x(),1,0,0);
  glRotatef(rot_.y(),0,0,1);
  glTranslatef(-camera_pos_.x(), -camera_pos_.y(), -camera_pos_.z());

  // Get hit ray position and direction.
  GLdouble proj[16];
  GLdouble model[16];
  GLint view[4];
  glGetDoublev(GL_PROJECTION_MATRIX, proj);
  glGetDoublev(GL_MODELVIEW_MATRIX, model);
  glGetIntegerv(GL_VIEWPORT, view);
  GLdouble x, y, z;
  gluUnProject(mouse_pos_.x(), mouse_pos_.y(), 0.0f, model, proj, view, &x, &y, &z);
  ray_start_.x() = static_cast<float>(x);
  ray_start_.y() = static_cast<float>(y);
  ray_start_.z() = static_cast<float>(z);
  gluUnProject(mouse_pos_.x(), mouse_pos_.y(), 1.0f, model, proj, view, &x, &y, &z);
  ray_end_.x() = static_cast<float>(x);
  ray_end_.y() = static_cast<float>(y);
  ray_end_.z() = static_cast<float>(z);

  // compute camera pos & orientation
  float final_keyb_speed = kKeybSpeed;
  if (SDL_GetModState() & KMOD_SHIFT)
    final_keyb_speed *= 4.0f;

  const float movex = (move_r_ - move_l_) * final_keyb_speed * dt;
  const float movey = (move_b_ - move_f_) * final_keyb_speed * dt;
  const float movez = (move_u_ - move_d_) * final_keyb_speed * dt;
  camera_pos_.x() += movex * static_cast<float>(model[0]);
  camera_pos_.y() += movex * static_cast<float>(model[4]);
  camera_pos_.z() += movex * static_cast<float>(model[8]);
  camera_pos_.x() += movey * static_cast<float>(model[2]);
  camera_pos_.y() += movey * static_cast<float>(model[6]);
  camera_pos_.z() += movey * static_cast<float>(model[10]);
  camera_pos_.x() += movez * static_cast<float>(model[1]);
  camera_pos_.y() += movez * static_cast<float>(model[5]);
  camera_pos_.z() += movez * static_cast<float>(model[9]);

  // render scene
  renderScene();
}

void GLViewer::_handleGUI()
{
  // handle & render GUI
  is_mouse_over_gui_ = false;
  

  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0, width_, 0, height_);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  handleGUI();

  glEnable(GL_DEPTH_TEST);
  // double buffering
  SDL_GL_SwapBuffers();
}
