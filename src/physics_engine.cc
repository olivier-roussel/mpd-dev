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
	is_init_(false),
	is_gravity_(false)
{
}

PhysicsEngine::~PhysicsEngine()
{
	//quit();	// will call implemented _quit()
	_cleanup();
}

void PhysicsEngine::quit()
{
	if (is_init())
	{
		// call implemented _quit()
		_quit();

		_cleanup();
	}
}

void PhysicsEngine::_cleanup()
{
	is_init_ = false;
	is_gravity_ = false;
	niter_ = 0;

	for (std::map<std::string, RigidBody*>::iterator it_bodies = rigid_bodies_.begin() ; it_bodies != rigid_bodies_.end() ; ++it_bodies)
	{
		if (it_bodies->second)
		{
			delete it_bodies->second;
			it_bodies->second = NULL;
		}
	}
	rigid_bodies_.clear();

	for (std::map<std::string, SoftBody*>::iterator it_bodies = soft_bodies_.begin() ; it_bodies != soft_bodies_.end() ; ++it_bodies)
	{
		if (it_bodies->second)
		{
			delete it_bodies->second;
			it_bodies->second = NULL;
		}
	}
	soft_bodies_.clear();
}

bool PhysicsEngine::init()
{
	// if already initialized, must first be properly terminated using quit() 
	if (is_init())
		return false;

	// common init stuff
	_cleanup();

	// call implemented _init()
	if (_init())
	{
		is_init_ = true;
		return true;
	}else
		return false;
}


void PhysicsEngine::doOneStep(unsigned int i_step_time_ms)
{
	assert(is_init_ && "Cannot step physics engine as it is not initialized");

	// call implemented _doOneStep()
	_doOneStep(i_step_time_ms);
}


bool PhysicsEngine::addStaticRigidBody(const std::string& i_name, RigidBody* i_rigid_body)
{
	assert(is_init_ && "Trying to add static rigid body but physics engine is not initialized");
	assert(i_rigid_body && "Trying to add uninitialized rigid body to physics engine");

	if (rigid_bodies_.find(i_name) == rigid_bodies_.end())
	{
		rigid_bodies_.insert(std::make_pair(i_name, i_rigid_body));

		// call implemented _addStaticRigidBody()
		return _addStaticRigidBody(i_name, i_rigid_body);
	}else
		return false;
}

bool PhysicsEngine::addDynamicRigidBody(const std::string& i_name, RigidBody* i_rigid_body)
{
	assert(is_init_ && "Trying to add dynamic rigid body but physics engine is not initialized");
	assert(i_rigid_body && "Trying to add uninitialized rigid body to physics engine");

	if (rigid_bodies_.find(i_name) == rigid_bodies_.end())
	{
		rigid_bodies_.insert(std::make_pair(i_name, i_rigid_body));
		
		// call implemented _addDynamicRigidBody()
		return _addDynamicRigidBody(i_name, i_rigid_body);
	}else
		return false;
}

bool PhysicsEngine::addDynamicSoftBody(const std::string& i_name, SoftBody* i_soft_body)
{
	assert(is_init_ && "Trying to add dynamic soft body but physics engine is not initialized");
	assert(i_soft_body && "Trying to add uninitialized soft body to physics engine");

	if (soft_bodies_.find(i_name) == soft_bodies_.end())
	{
		soft_bodies_.insert(std::make_pair(i_name, i_soft_body));
		
		// call implemented _addDynamicSoftBody()
		return _addDynamicSoftBody(i_name, i_soft_body);
	}else
		return false;
}

void PhysicsEngine::enableGravity(bool i_enable_gravity)
{
		// call implemented _enableGravity()
	is_gravity_ = _enableGravity(i_enable_gravity);
}

//void PhysicsEngine::set_is_init(bool i_is_init)
//{
//	is_init_ = i_is_init;
//}

bool PhysicsEngine::is_init() const
{
	return is_init_;
}

unsigned int PhysicsEngine::niter() const
{
	return niter_;
}

const std::map<std::string, RigidBody*>& PhysicsEngine::rigid_bodies() const
{
	return rigid_bodies_;
}

const std::map<std::string, SoftBody*>& PhysicsEngine::soft_bodies() const
{
	return soft_bodies_;
}

