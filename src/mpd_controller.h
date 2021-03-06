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

#ifndef MPD_DEV_MPD_CONTROLLER_H_
#define MPD_DEV_MPD_CONTROLLER_H_

#include <boost/filesystem/path.hpp>

#include "rigid_body.h"
#include "environment.h"
#include "physics_engine.h"
#include "physics_thread.h"

class MPDViewer; // for debug drawing physics

/**
* \class MPDController
* \brief Handles scene data and physics engine, and synchronize scene data with physics.
*/

class MPDController 
{
public:
  MPDController();
  virtual ~MPDController();

	bool initPhysics(const PhysicsEngine::ImplementationType i_physics_engine_type);
	
	void quitPhysics();

  bool loadEnvironment(const boost::filesystem::path& path);

  void switchEnvironmentAxis();

  void invertEnvironmentTriangles();

	void setPhysicsDebugDrawer(MPDViewer* i_viewer);

	/**
	*
	* \pre Physics must have been initialized.
	*/
	//void enableGravity(bool i_enable_gravity);
  
	/**
	*
	* \pre Physics must have been initialized.
	*/
	void addRigidBox(const std::string& i_name, double i_mass, const Eigen::Affine3d& i_transform);

	/**
	*
	* \pre Physics must have been initialized.
	*/
	void addSoftBox(const std::string& i_name, double i_mass, const Eigen::Affine3d& i_transform);

	void addSoftRope(const std::string& i_name, double i_mass, int i_nnodes, double i_len, const Eigen::Affine3d& i_transform);

	void runRopeExperiment(double i_mass, int i_nnodes);

	/**
	*
	* \pre Physics must have been initialized.
	*/
	bool addRigidBodyFromMeshFile(const std::string& i_name, const boost::filesystem::path& path, double i_mass, const Eigen::Affine3d& i_transform);

	/**
	*
	* \pre Physics must have been initialized.
	*/
	bool addSoftBodyFromMeshFile(const std::string& i_name, const boost::filesystem::path& path, double i_mass, const Eigen::Affine3d& i_transform);
	
	/**
	*
	* \pre Physics must have been initialized.
	*/
	void setPhysicsPaused(bool i_physics_paused);

	/**
	* Accessors
	*/

	bool isPhysicsInitialized() const;

	const Environment& environment() const;

	const PhysicsEngine& physics_engine() const;

	unsigned int physics_time_step() const;

	void set_physics_time_step(unsigned int i_physics_time_step_ms);

  bool isEnvironmentSet() const;

	bool isPhysicsEngineSet() const;

	bool isPhysicsPaused() const;

	PhysicsEngine* physics_engine_mutable();	// for debugging purpose

private:
  Environment* env_;								// owned by the physics engine
	PhysicsEngine* physics_engine_;		// owned
	PhysicsThread* physics_thread_;		// owned

	unsigned int physics_time_step_ms_;
	bool is_physics_paused_;

	/**
	* Debug viewer for physics
	*/
	MPDViewer* debug_physics_viewer_;

};

#endif // MPD_DEV_MPD_CONTROLLER_H_
