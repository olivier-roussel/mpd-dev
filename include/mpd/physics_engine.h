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
	* ------------------------------------
	* Interface
	* ------------------------------------
	*/

	/**
	* \warning Inherited classes that reimplements this method should call PhysicsEngine::init() at first during their own init().
	*/
  virtual bool init();

	/**
	* \warning Implementations of doOneStep() method must update bodies transformations of this class at each step.
	*/
  virtual void doOneStep(unsigned int i_step_time_ms) = 0;

	/**
	* \warning Inherited classes that reimplements this method should call PhysicsEngine::quit() at end during their own quit().
	*/
  virtual void quit();

	/**
	* \warning Inherited classes that reimplements this method should call PhysicsEngine::addStaticRigidBody() at first during their own addStaticRigidBody().
	*/
	virtual bool addStaticRigidBody(const std::string& i_name, RigidBody* i_rigid_body);

	/**
	* \warning Inherited classes that reimplements this method should call PhysicsEngine::addDynamicRigidBody() at first during their own addDynamicRigidBody().
	*/
	virtual bool addDynamicRigidBody(const std::string& i_name, RigidBody* i_rigid_body); 

	/**
	* \warning Inherited classes that reimplements this method should call PhysicsEngine::addDynamicSoftBody() at first during their own addDynamicSoftBody().
	*/
  virtual bool addDynamicSoftBody(const std::string& i_name/*, SoftBody* i_soft_body*/); // TODO

	virtual void enableGravity(bool i_enable_gravity) = 0;

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

	bool is_init() const;

protected:
	/**
	* ------------------------------------
	* Protected methods
	* ------------------------------------
	*/
  PhysicsEngine();
	void set_is_init(bool i_is_init);


	/**
	* ------------------------------------
	* Attributes
	* ------------------------------------
	*/
  bool is_init_;
	std::map<std::string, RigidBody*> rigid_bodies_; // owned

};

#endif // MPD_DEV_PHYSICS_ENGINE_H_

