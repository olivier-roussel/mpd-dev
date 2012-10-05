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

#include <iostream>

#include "Eigen/Core"
#include "SDL.h"
#include "SDL_opengl.h"

#include "gui/imgui.h"
#include "gui/imguiRenderGL.h"
//#include "gui/Utils.h"
//#include "gui/fileUtils.h"
#include "config.h"

int main()
{

  // Init SDL
  if (SDL_Init(SDL_INIT_EVERYTHING) != 0) 
  {
    std::cout << "Could not initialise SDL\n" << std::endl;
    return -1;
  }

  // Init SDL_OpenGL
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);

  const SDL_VideoInfo* vi = SDL_GetVideoInfo();

  int width = vi->current_w - 20;
  int height = vi->current_h - 80;
  SDL_Surface* screen = SDL_SetVideoMode(width, height, 0, SDL_OPENGL);
  if (!screen)
  {
    std::cout << "Could not initialise SDL opengl" << std::endl;
    return -1;
  }

  SDL_WM_SetCaption("mpd-dev-test", 0);

//  std::string font_file = std::string(SHARE_DIR) + "/FreeMonoBold.ttf";
//  if (!imguiRenderGLInit(font_file.c_str()))
//  {
//    std::cout << "Could not init GUI renderer." << std::endl;
//    SDL_Quit();
//    return -1;
//  }
//
//  // Init OpenGL
//  glEnable(GL_CULL_FACE);  
//  float fogCol[4] = { 0.32f,0.25f,0.25f,1 };
//  glEnable(GL_FOG);
//  glFogi(GL_FOG_MODE, GL_LINEAR);
//  glFogf(GL_FOG_START, 0);
//  glFogf(GL_FOG_END, 10);
//  glFogfv(GL_FOG_COLOR, fogCol);
//
//  glEnable(GL_POINT_SMOOTH);
//  glEnable(GL_LINE_SMOOTH);
//
//  // init locals
//  Uint32 lastTime = SDL_GetTicks();
//  static const float nearClip = 0.01f;
//  float farClip = 500.f;        // updated depending on the scene
//  int mx = 0, mz = 0;
//  float rx = 45, rz = 45;
//  float camx = 0, camy = 0, camz = 0;//, camr=10;
//  float moveW = 0, moveS = 0, moveA = 0, moveD = 0, moveR = 0, moveF = 0;
//  bool rotate = false;
//  float origrx, origrz;
//  int origx, origz;
//  bool mouseOverGUI = false;
//  
  std::cout << "Hello Bichon !" << std::endl;
  return 0;
}
