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
#include <boost/thread.hpp>
#include <boost/optional.hpp>
#include "rigid_body.h"
#include "soft_body.h"

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
  
	struct PerformanceTimes
	{
		double last_step_dostep_cpu_time;			/**< Last step CPU time for _doOneStep() call in ms.*/ 
		double last_step_update_cpu_time;			/**< Last step CPU time for _updateBodies() call in ms.*/ 
		double last_step_simu_time;						/**< Last step simulation desired step time in ms.*/ 
	};

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

	bool removeSoftBody(const std::string& i_name);

	//void removeRigidBody(const std::string& i_name);	// TODO

	void enableGravity(bool i_enable_gravity);

	void enableEngineDebugDrawer(bool i_enable_drawer);

	void setWorldAABB(const Eigen::Vector3d& i_aabb_min, const Eigen::Vector3d& i_aabb_max);

	/**
	* \brief Set new parameters for given SoftBody.\n
	* Will also call _setSoftBodyParameters() implementation for implementation specific requirements.
	*/
	void setSoftBodyParameters(const std::string& i_name, const SoftBodyParameters& i_params);

	/**
	* ------------------------------------
	* Accessors
	* ------------------------------------
	*/

	// these would not be threadsafe
	//const std::map<std::string, RigidBody*>& rigid_bodies() const;
	//const std::map<std::string, SoftBody*>& soft_bodies() const;

	// Threadsafe accessors to 
	const std::vector<std::pair<std::string, RigidBody> > getRigidBodies() const;

	const std::vector<std::pair<std::string, SoftBody> > getSoftBodies() const;

	const boost::optional<const RigidBody> getRigidBody(const std::string& i_name) const;

	const boost::optional<const SoftBody> getSoftBody(const std::string& i_name) const;

	const std::vector<std::string> getRigidBodiesNames() const;

	const std::vector<std::string> getSoftBodiesNames() const;

	bool is_init() const;

	bool is_gravity() const;

	unsigned int niter() const;

	const PerformanceTimes performance_times() const;

	const size_t getNumberOfSoftBodies() const;

	const size_t getNumberOfRigidBodies() const;

	const Eigen::Vector3d& getWorldAABBmin() const;

	const Eigen::Vector3d& getWorldAABBmax() const;

protected:
	/**
	* ------------------------------------
	* Protected methods
	* ------------------------------------
	*/
  PhysicsEngine();


	/**
	* ------------------------------------
	* Attributes
	* ------------------------------------
	*/
	unsigned int niter_;		// number of steps procedeed since last init
  bool is_init_;					// true if engine is initialized
	bool is_gravity_;
	bool is_debug_drawer_;	// true if should use physics engine own drawer (for debugging purpose, and if availbale)
	Eigen::Vector3d world_aabb_min_, world_aabb_max_;				// world aabb

	std::map<std::string, RigidBody*> rigid_bodies_;	// owned
	std::map<std::string, SoftBody*> soft_bodies_;		// owned
	mutable boost::mutex bodies_mutex_;

	PerformanceTimes perf_times_;
	mutable boost::mutex perf_times_mutex_;

protected:

	/**
	* ------------------------------------
	* Interface
	* ------------------------------------
	*/

	/**
	* \warning Implementations of doOneStep() method must update bodies transformations of this class at each step.
	*/
	virtual void _doOneStep(unsigned int i_step_time_ms) = 0;

  virtual bool _init() = 0;

  virtual void _quit() = 0;

	virtual bool _addStaticRigidBody(const std::string& i_name, RigidBody* i_rigid_body) = 0;

	virtual bool _addDynamicRigidBody(const std::string& i_name, RigidBody* i_rigid_body) = 0; 

  virtual bool _addDynamicSoftBody(const std::string& i_name, SoftBody* i_soft_body) = 0; 

  virtual bool _removeSoftBody(const std::string& i_name) = 0; 

	virtual void _updateBodies() = 0;

	virtual void _setSoftBodyParameters(const std::string& i_name, const SoftBodyParameters& i_params) = 0;

	virtual void 	_setWorldAABB(const Eigen::Vector3d& i_aabb_min, const Eigen::Vector3d& i_aabb_max) = 0;

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

