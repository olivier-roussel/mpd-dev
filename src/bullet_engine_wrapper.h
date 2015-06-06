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

#ifndef MPD_DEV_BULLET_ENGINE_WRAPPER_H_
#define MPD_DEV_BULLET_ENGINE_WRAPPER_H_

#include <string>
#include <map>
#include <Eigen/Geometry>

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "physics_engine.h"
#include "bullet_utils.h"
#include "bullet_bodies_wrapper.h"

/**
* Debug drawer for physics
*/
class MPDViewer;
class BulletDebugDrawer;

/**
 * \class BulletEngineWrapper
 */
class BulletEngineWrapper : public PhysicsEngine
{

public:
  BulletEngineWrapper(MPDViewer* i_debug_physics_viewer = NULL);
  virtual ~BulletEngineWrapper();

protected:
  bool _init();

  void _quit();

	// XXX so far, only add once ! TODO add possibility to add many static objects
  bool _addStaticRigidBody(const std::string& i_name, RigidBody* i_rigid_body); 

	/**
	* \pre Body must be convex
	*/
  bool _addDynamicRigidBody(const std::string& i_name, RigidBody* i_rigid_body);

  bool _addDynamicSoftBody(const std::string& i_name, SoftBody* i_soft_body);

  bool _removeSoftBody(const std::string& i_name); 
	
	bool _removeRigidBody(const std::string& i_name);

	bool _enableGravity(bool i_enable_gravity);

	void _applyForceOnRigidBody(const std::string& i_name, const Eigen::Vector3d& i_force);

	void _applyForceOnSoftBody(const std::string& i_name, const Eigen::Vector3d& i_force, int i_node_index);

  void _doOneStep(unsigned int i_step_time_ms);

	void _updateBodies();

	void _setSoftBodyParameters(const std::string& i_name, const SoftBodyParameters& i_params);

	void _setWorldAABB(const Eigen::Vector3d& i_aabb_min, const Eigen::Vector3d& i_aabb_max);

private:

  std::vector<btCollisionShape*> collision_shapes_;	// so far, one collision shape for one body (TODO : share this between bodies)
  //std::vector<std::pair<btCollisionShape*, int> > collision_shapes_;	// second is number of btCollisionObjects that uses this shape (for memory management)
  btDefaultCollisionConfiguration* collision_config_;
  btCollisionDispatcher* dispatcher_;
  //bt32BitAxisSweep3* overlapping_pair_cache_;
	btDbvtBroadphase* overlapping_pair_cache_;
  btSequentialImpulseConstraintSolver* solver_; // TODO try others solvers
  btSoftRigidDynamicsWorld* dynamics_world_;
	btSoftBodyWorldInfo	world_soft_config_;

  std::map<std::string, btTriangleIndexVertexArray*> tris_indexes_arrays_;
  std::map<std::string, BulletRigidBody> bt_rigid_bodies_;
  std::map<std::string, BulletSoftBody> bt_soft_bodies_;

	/**
	* Debug drawer for physics
	*/
	MPDViewer* debug_physics_viewer_;
	BulletDebugDrawer* debug_physics_drawer_;
};

#endif // MPD_DEV_BULLET_ENGINE_WRAPPER_H_


