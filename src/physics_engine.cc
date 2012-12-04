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

#include "mpd/physics_engine.h"

PhysicsEngine::PhysicsEngine() :
is_init_(false)
{
}

PhysicsEngine::~PhysicsEngine()
{
	PhysicsEngine::quit();
}

void PhysicsEngine::quit()
{
	for (std::map<std::string, RigidBody*>::iterator it_bodies = rigid_bodies_.begin() ; it_bodies != rigid_bodies_.end() ; ++it_bodies)
	{
		if (it_bodies->second)
		{
			delete it_bodies->second;
			it_bodies->second = NULL;
		}
	}
	rigid_bodies_.clear();
}

bool PhysicsEngine::init()
{
	return true;
}

const std::map<std::string, RigidBody*>& PhysicsEngine::rigid_bodies() const
{
	return rigid_bodies_;
}

bool PhysicsEngine::addStaticRigidBody(const std::string& i_name, RigidBody* i_rigid_body)
{
	if (rigid_bodies_.find(i_name) == rigid_bodies_.end())
	{
		rigid_bodies_.insert(std::make_pair(i_name, i_rigid_body));
		return true;
	}else
		return false;
}

bool PhysicsEngine::addDynamicRigidBody(const std::string& i_name, RigidBody* i_rigid_body)
{
	if (rigid_bodies_.find(i_name) == rigid_bodies_.end())
	{
		rigid_bodies_.insert(std::make_pair(i_name, i_rigid_body));
		return true;
	}else
		return false;
}

bool PhysicsEngine::addDynamicSoftBody(const std::string& i_name/*, SoftBody* i_soft_body*/)
{
	// TODO
	return true;
}

void PhysicsEngine::set_is_init(bool i_is_init)
{
	is_init_ = i_is_init;
}

bool PhysicsEngine::is_init() const
{
	return is_init_;
}