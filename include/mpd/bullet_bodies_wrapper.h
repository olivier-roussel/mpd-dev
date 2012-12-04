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

#ifndef MPD_DEV_BULLET_BODIES_WRAPPER_H_
#define MPD_DEV_BULLET_BODIES_WRAPPER_H_

#include "mpd/rigid_body.h"
#include "mpd/soft_body.h"

#include "btBulletDynamicsCommon.h"

struct BulletRigidBody
{
	btRigidBody* bt_rigid_body;	// owned by bullet engine wrapper
	RigidBody* rigid_body;			// owned by generic engine

	inline BulletRigidBody(btRigidBody* i_bt_rigid_body, RigidBody* i_rigid_body) : 
	bt_rigid_body(i_bt_rigid_body), rigid_body(i_rigid_body) {}
};

struct BulletSoftBody
{
	btSoftBody* bt_soft_body;		// owned by bullet engine wrapper
	SoftBody* soft_body;				// owned by generic engine

		inline BulletSoftBody(btSoftBody* i_bt_soft_body, SoftBody* i_soft_body) : 
	bt_soft_body(i_bt_soft_body), soft_body(i_soft_body) {}
};

#endif // MPD_DEV_BULLET_BODIES_WRAPPER_H_