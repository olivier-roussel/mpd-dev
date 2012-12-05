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

#include "mpd/mpd_controller.h"
#include "mpd/bullet_engine_wrapper.h"
#include "mpd/constants.h"

MPDController::MPDController()
  :env_(NULL),
	physics_engine_(NULL),
	physics_thread_(NULL),
	physics_time_step_ms_(kDefaultPhysicsTimeStepMs),
	debug_physics_viewer_(NULL)
{
}

MPDController::~MPDController()
{
	// not needed as environment is a rigid body managed by the phsyics engine
  //if (env_)
  //{
  //  delete env_;
  //  env_ = NULL;
  //}
	if (physics_thread_)
	{
		delete physics_thread_;
    physics_thread_ = NULL;
	}
	if (physics_engine_)
	{
		delete physics_engine_;
    physics_engine_ = NULL;
	}
}

const Environment& MPDController::environment() const
{
  assert(env_ != NULL && "environment is uninitialized.");
  return *env_;
}

const PhysicsEngine& MPDController::physics_engine() const
{
  assert(physics_engine_ != NULL && "physics engine is uninitialized.");
  return *physics_engine_;
}

bool MPDController::loadEnvironment(const boost::filesystem::path& path)
{
  //if (!env_)
		//env_ = new Environment(Eigen::Affine3d::Identity());
  //return env_->loadPolygonSoup(path);

	if (env_)
	{
		delete env_;
		env_ = NULL;
	}
	PolygonSoup soup;
	if (soup.loadFromFile(path))
	{
		env_ = new Environment(soup, Eigen::Affine3d::Identity());
		return true;
	}else
		return false;
}

void MPDController::switchEnvironmentAxis()
{
  if (env_)
    env_->switchPolygonSoupAxis();
}

void MPDController::invertEnvironmentTriangles()
{
  if (env_)
    env_->invertPolygonSoupTriangles();
}

bool MPDController::isEnvironmentSet() const
{
  return env_ != NULL;
}

bool MPDController::isPhysicsEngineSet() const
{
  return physics_engine_ != NULL;
}

unsigned int MPDController::physics_time_step() const
{
	return physics_time_step_ms_;
}

void MPDController::set_physics_time_step(unsigned int i_physics_time_step_ms)
{
	physics_time_step_ms_ = i_physics_time_step_ms;
}

bool MPDController::isPhysicsInitialized() const
{
	return (physics_engine_ && physics_engine_->is_init());
}

void MPDController::quitPhysics()
{
	if (physics_thread_)
	{
		physics_thread_->set_is_done(true);
		physics_thread_->join();
		delete physics_thread_;
    physics_thread_ = NULL;
	}
	if (physics_engine_)
	{
		physics_engine_->quit();
		delete physics_engine_;
    physics_engine_ = NULL;
	}

}

bool MPDController::initPhysics(const PhysicsEngine::ImplementationType i_physics_engine_type)
{
	assert (static_cast<int>(i_physics_engine_type) >= 0 && static_cast<int>(i_physics_engine_type) < PhysicsEngine::NB_IMPLEMENTATION_TYPES && "invalid physics engine implementation");
	assert (physics_time_step_ms_ != 0 && "physics loop time must be above 0");

	// delete if any engine running
	if (physics_engine_)
	{
		quitPhysics();
	}

	// init a new engine
	switch (i_physics_engine_type)
	{
	case PhysicsEngine::PE_BULLET:
		physics_engine_ = new BulletEngineWrapper(debug_physics_viewer_);
		break;
	default:
		std::cout << "Unknown physics engine implementation type" << std::endl;
		return false;
	}

	// init phsyics engine 
	if (!physics_engine_->init())
	{
		std::cout << "MPDController::initPhysics : Could not initialize physics engine" << std::endl;
		return false;
	}

	// synchronize physics with existing data
	if (isEnvironmentSet())
	{
		physics_engine_->addStaticRigidBody("environment", env_);
	}
	
	// init new physics thread
	physics_thread_ = new PhysicsThread(physics_engine_);
	physics_thread_->run(physics_time_step_ms_);

	return true;
}

void MPDController::enableGravity(bool i_enable_gravity)
{
	assert (isPhysicsInitialized() && "cannot add rigid bodies if physics engine not initialized");

	physics_engine_->enableGravity(i_enable_gravity);
}

void MPDController::addRigidBox(const std::string& i_name, double i_mass, const Eigen::Affine3d& i_transform)
{
	assert (isPhysicsInitialized() && "cannot add rigid bodies if physics engine not initialized");

	const PolygonSoup box_geom = PolygonSoup::createBox(1.);

	RigidBody* box_body = new RigidBody(box_geom, i_mass, i_transform);

	physics_engine_->addDynamicRigidBody(i_name, box_body);
}

void MPDController::addSoftBox(const std::string& i_name, double i_mass, const Eigen::Affine3d& i_transform)
{
	assert (isPhysicsInitialized() && "cannot add soft bodies if physics engine not initialized");

	const PolygonSoup box_geom = PolygonSoup::createBox(1.);
	std::vector<double> nodes_masses(box_geom.verts().size(), 0.);
	SoftBody* box_body = new SoftBody(box_geom, i_mass, nodes_masses, i_transform);

	physics_engine_->addDynamicSoftBody(i_name, box_body);
}

bool MPDController::addRigidBodyFromMeshFile(const std::string& i_name, const boost::filesystem::path& path, double i_mass, const Eigen::Affine3d& i_transform)
{
	assert (isPhysicsInitialized() && "cannot add rigid bodies if physics engine not initialized");

	PolygonSoup soup;
	if (soup.loadFromFile(path))
	{
		RigidBody* body = new RigidBody(soup, i_mass, i_transform);

		physics_engine_->addDynamicRigidBody(i_name, body);
		return true;
	}else
		return false;
}

bool MPDController::addSoftBodyFromMeshFile(const std::string& i_name, const boost::filesystem::path& path, double i_mass, const Eigen::Affine3d& i_transform)
{
	assert (isPhysicsInitialized() && "cannot add rigid bodies if physics engine not initialized");

	PolygonSoup soup;
	if (soup.loadFromFile(path))
	{
		std::vector<double> nodes_masses(soup.verts().size(), 0.);
		SoftBody* body = new SoftBody(soup, i_mass, nodes_masses, i_transform);

		physics_engine_->addDynamicSoftBody(i_name, body);
		return true;
	}else
		return false;
}

void MPDController::setPhysicsDebugDrawer(MPDViewer* i_viewer)
{
	assert (i_viewer != NULL && "debug drawer for physics engine is NULL");

	debug_physics_viewer_ = i_viewer;
}

