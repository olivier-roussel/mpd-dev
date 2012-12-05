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

#include <map>
#include <Eigen/Geometry>
#include "mpd/rigid_body.h"
#include "mpd/soft_body.h"

class PolygonSoup;

/**
* Debug drawer for physics
*/
class MPDViewer;

/**
 * \class PhysicsEngine
 * \brief Common interface for physics engine to be used in mpd-dev.
 */
class PhysicsEngine 
{

public:
  
  enum ImplementationType
  {
    PE_BULLET = 0,
    //PE_ODE, // TODO
    //PE_XDE, // TODO
    NB_IMPLEMENTATION_TYPES
  };

	virtual ~PhysicsEngine();

	
	/**
	* \brief This is the actual method called by the physics thread at each time step, which will call 
	* itself the doOneStep() method that must be reimplemented in inherited classes.
	* It proceed to meta stuff, such iterations count, time measurements, etc.
	*/
  void doOneStep(unsigned int i_step_time_ms);

	bool init();

	void quit();

	bool addStaticRigidBody(const std::string& i_name, RigidBody* i_rigid_body);

	bool addDynamicRigidBody(const std::string& i_name, RigidBody* i_rigid_body); 

	bool addDynamicSoftBody(const std::string& i_name, SoftBody* i_soft_body); 

	void enableGravity(bool i_enable_gravity);

	/**
	* ------------------------------------
	* Interface
	* ------------------------------------
	*/

	/**
	* ------------------------------------
	* Accessors
	* ------------------------------------
	*/

	/**
	*
	* \brief Returns world transforms of all bodies in simulation.
	*/
	//virtual const std::map<std::string, Eigen::Affine3d> getRigidBodiesWorldTransform() const = 0;

	const std::map<std::string, RigidBody*>& rigid_bodies() const;

	const std::map<std::string, SoftBody*>& soft_bodies() const;

	bool is_init() const;

	unsigned int niter() const;

	/**
	* \brief Returns the last procedeed step cpu time in milliseconds.
	*/
	double last_step_cpu_time() const;

	/**
	* \brief Returns the last procedeed step simulation time in milliseconds.
	*/
	double last_step_simulation_time() const;

protected:
	/**
	* ------------------------------------
	* Protected methods
	* ------------------------------------
	*/
  PhysicsEngine();

	//void set_is_init(bool i_is_init);


	/**
	* ------------------------------------
	* Attributes
	* ------------------------------------
	*/
	unsigned int niter_;		// number of steps procedeed since last init
  bool is_init_;					// true if engine is initialized
	bool is_gravity_;

	std::map<std::string, RigidBody*> rigid_bodies_; // owned
	std::map<std::string, SoftBody*> soft_bodies_;	// owned

	double last_step_simu_time_;
	double last_step_cpu_time_;

protected:

	/**
	* \warning Implementations of doOneStep() method must update bodies transformations of this class at each step.
	*/
	virtual void _doOneStep(unsigned int i_step_time_ms) = 0;

  virtual bool _init() = 0;

  virtual void _quit() = 0;

	virtual bool _addStaticRigidBody(const std::string& i_name, RigidBody* i_rigid_body) = 0;

	virtual bool _addDynamicRigidBody(const std::string& i_name, RigidBody* i_rigid_body) = 0; 

  virtual bool _addDynamicSoftBody(const std::string& i_name, SoftBody* i_soft_body) = 0; 

	/**
	* \brief Must returns true if gravity have been enabled, false otherwise.
	*/
	virtual bool _enableGravity(bool i_enable_gravity) = 0;

private:

	/**
	* \brief Cleanup all data of physics engine base class.
	*/
	void _cleanup();

};

#endif // MPD_DEV_PHYSICS_ENGINE_H_

