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

#include "mpd/physics_engine.h"
#include "mpd/bullet_utils.h"
#include "mpd/polygon_soup.h"


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
  ~BulletEngineWrapper();

  bool init();

	bool is_init();

  void doOneStep(unsigned int i_step_time_ms);

  void quit();

	// XXX so far, only add once ! TODO add possibility to add many static objects
  void addStaticRigidBody(const std::string& i_name, const RigidBody& i_rigid_body); 

	/**
	* \pre Body must be convex
	*/
  void addDynamicRigidBody(const std::string& i_name, const RigidBody& i_rigid_body);

  void addDynamicSoftBody(const Eigen::Affine3d& i_transform);

	void enableGravity(bool i_enable_gravity);

private:
  unsigned int step_time_;   /**< Step time in microseconds. */
  bool is_init_;
	bool is_gravity_;

  std::vector<btCollisionShape*> 	 collision_shapes_;
  btDefaultCollisionConfiguration* collision_config_;
  btCollisionDispatcher* dispatcher_;
  btBroadphaseInterface* overlapping_pair_cache_;
  btSequentialImpulseConstraintSolver* solver_; // TODO check if solver appropriated
  btDiscreteDynamicsWorld* dynamics_world_;

  std::map<std::string, btTriangleIndexVertexArray*> tris_indexes_arrays_;
  std::map<std::string, BulletBaseArray<btVector3>* > bodies_verts_;
  std::map<std::string, BulletBaseArray<int>* > bodies_tris_;
  std::map<std::string, btRigidBody*> rigid_bodies_;
  std::map<std::string, btSoftBody*> soft_bodies_;

	/**
	* Debug drawer for physics
	*/
	MPDViewer* debug_physics_viewer_;
	BulletDebugDrawer* debug_physics_drawer_;
};

#endif // MPD_DEV_BULLET_ENGINE_WRAPPER_H_


