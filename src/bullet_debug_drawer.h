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

#ifndef MPD_DEV_BULLET_DEBUG_DRAWER_H_
#define MPD_DEV_BULLET_DEBUG_DRAWER_H_

#include <Eigen/Geometry>
#include <boost/thread/mutex.hpp>

#include "mpd/bullet_utils.h"
#include "LinearMath/btIDebugDraw.h"


class MPDViewer;

class BulletDebugDrawer : public btIDebugDraw
{
public:
  BulletDebugDrawer(MPDViewer* i_viewer);
  virtual ~BulletDebugDrawer();

	void clearDraws();

	/**
	* btIDebugDraw interface implementation
	*/
	void drawLine(const btVector3& i_from, const btVector3& i_to, const btVector3& i_color);

	void drawContactPoint(const btVector3& ,const btVector3 &,btScalar,int,const btVector3 &);

	void draw3dText(const btVector3& i_pos, const char* i_text);

	void reportErrorWarning(const char* i_text);

	void setDebugMode(int i_mode);

	int getDebugMode() const;

	boost::mutex& getPhysicsObjectsMutex();

protected:
	MPDViewer* viewer_;

	int debug_mode_;
};

#endif // MPD_DEV_BULLET_DEBUG_DRAWER_H_
