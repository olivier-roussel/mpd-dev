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

#ifndef MPD_DEV_PHYSICS_ENGINE_H_
#define MPD_DEV_PHYSICS_ENGINE_H_


/**
 * \class PhysicsEngine
 * \brief Common interface for physics engine to be used in mpd-dev.
 */
class PhysicsEngine {

public:
  virtual ~PhysicsEngine() = 0;

  virtual void init(unsigned int step_time) = 0;

  virtual void doOneStep() = 0;

  virtual void quit() = 0;

  virtual void addStaticRigidBody() = 0;

  virtual void addDynamicRigidBody() = 0;

  virtual void addDynamicSoftBody() = 0;

protected:
  PhysicsEngine() {}

};

#endif // MPD_DEV_PHYSICS_ENGINE_H_

