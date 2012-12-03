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

#include "mpd/bullet_engine_wrapper.h"
#include "mpd/constants.h"
#include "bullet_debug_drawer.h"
#include <boost/thread/mutex.hpp>

static const bool kUseQuantizedAabbCompression = true;

BulletEngineWrapper::BulletEngineWrapper(MPDViewer* i_debug_physics_viewer) :
  step_time_(0), is_init_(false), 
  collision_shapes_(), collision_config_(NULL), dispatcher_(NULL), overlapping_pair_cache_(NULL),
  solver_(NULL), dynamics_world_(NULL), 
  tris_indexes_arrays_(),
  bodies_verts_(),
  bodies_tris_(),
  rigid_bodies_(),
  soft_bodies_(),
	debug_physics_drawer_(NULL),
	debug_physics_viewer_(i_debug_physics_viewer)
{
}

BulletEngineWrapper::~BulletEngineWrapper()
{
  quit();
}

bool BulletEngineWrapper::init()
{
  if (!is_init_)
  {
    //collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collision_config_= new btDefaultCollisionConfiguration();

    //use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher_= new btCollisionDispatcher(collision_config_);

    //btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlapping_pair_cache_ = new btDbvtBroadphase();

    //the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver_ = new btSequentialImpulseConstraintSolver;

    dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher_, overlapping_pair_cache_, solver_, collision_config_);
  
		// if a viewer as been given for physics debug rendering, associate a bullet physics renderer 
		if (debug_physics_viewer_)
		{
			debug_physics_drawer_ = new BulletDebugDrawer(debug_physics_viewer_);
			dynamics_world_->setDebugDrawer(debug_physics_drawer_);
			debug_physics_drawer_->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
		}

		if (is_gravity_)
		{
			dynamics_world_->setGravity(toBtVector3(Z_2_Y_Matrix * kGravity));
		}

    is_init_ = true;
  }else
    return false; // already initialized
}

void BulletEngineWrapper::doOneStep(unsigned int i_step_time_ms)
{
  if (is_init_)
	{
    dynamics_world_->stepSimulation(static_cast<float>(i_step_time_ms *1.e-3f));

		// debug drawing
		{
			boost::mutex::scoped_lock lock(debug_physics_drawer_->getPhysicsObjectsMutex());
			debug_physics_drawer_->clearDraws();
			dynamics_world_->debugDrawWorld();
		}
	}
}

void BulletEngineWrapper::enableGravity(bool i_enable_gravity)
{
	is_gravity_ = i_enable_gravity;
	if (is_gravity_ && is_init_)
	{
		dynamics_world_->setGravity(toBtVector3(Z_2_Y_Matrix * kGravity));
		//m_softBodyWorldInfo.m_gravity.setValue(0,-9.8,0);
		//m_softBodyWorldInfo.m_sparsesdf.Initialize();
	}
}

bool BulletEngineWrapper::is_init()
{
	return is_init_;
}

void BulletEngineWrapper::quit()
{
  if (is_init_)
  {
    // Release bodies data
    for (std::map<std::string, btTriangleIndexVertexArray*>::iterator it = tris_indexes_arrays_.begin() ; it != tris_indexes_arrays_.end() ; ++it)
    {
      if (it->second)
        delete it->second;
    }
    tris_indexes_arrays_.clear();

		for (std::map<std::string, BulletBaseArray<btVector3>* >::iterator it = bodies_verts_.begin() ; it != bodies_verts_.end() ; ++it)
    {
			if (it->second->data)
			{
        delete [] it->second->data;
				it->second->data = NULL;
			}
			delete it->second;
			it->second = NULL;
    }
    bodies_verts_.clear();

		for (std::map<std::string, BulletBaseArray<int>* >::iterator it = bodies_tris_.begin() ; it != bodies_tris_.end() ; ++it)
    {
			if (it->second->data)
			{
        delete [] it->second->data;
				it->second->data = NULL;
			}
			delete it->second;
			it->second = NULL;
    }
    bodies_tris_.clear();

    // Global bullet handlers
    if (dynamics_world_ )
    {
      delete dynamics_world_ ;
      dynamics_world_ = NULL;
    }
    if (solver_ )
    {
      delete solver_ ;
      solver_ = NULL;
    }
    if (overlapping_pair_cache_)
    {
      delete overlapping_pair_cache_;
      overlapping_pair_cache_= NULL;
    }
    if (dispatcher_)
    {
      delete overlapping_pair_cache_;
      overlapping_pair_cache_= NULL;
    }
    if (collision_config_)
    {
      delete collision_config_;
      collision_config_ = NULL;
    }

		if (debug_physics_drawer_)
		{
      delete debug_physics_drawer_;
			debug_physics_drawer_ = NULL;
		}
    is_init_ = false;
  }
}

void BulletEngineWrapper::addDynamicRigidBody(const std::string& i_name, const RigidBody& i_rigid_body)
{
  // add polygon soup to bullet engine   
	// Vertices
	BulletBaseArray<btVector3>* verts = new BulletBaseArray<btVector3>();
	verts->size = i_rigid_body.polygon_soup().verts().size();
	verts->data = new btVector3[i_rigid_body.polygon_soup().verts().size()];
	toBtVector3Array(i_rigid_body.polygon_soup().verts(), Z_2_Y_Matrix, verts->data);

	bodies_verts_.insert(std::make_pair(i_name, verts)); 

	// Triangles
	BulletBaseArray<int>* tris = new BulletBaseArray<int>();
	tris->size = i_rigid_body.polygon_soup().tris().size()*3;
	tris->data = new int[i_rigid_body.polygon_soup().tris().size()*3];

	for (size_t i = 0 ; i < i_rigid_body.polygon_soup().tris().size() ; ++i)
	{
		const Triangle& t = i_rigid_body.polygon_soup().tris()[i];
		tris->data[i*3 + 0] = t.get<0>();
		tris->data[i*3 + 1] = t.get<1>();
		tris->data[i*3 + 2] = t.get<2>();
	}

	bodies_tris_.insert(std::make_pair(i_name, tris));

	btTriangleIndexVertexArray* tris_idx_array = new btTriangleIndexVertexArray(
		tris->size/3, tris->data, sizeof(int) * 3,
		verts->size, (btScalar*) &verts->data[0].x(), sizeof(btVector3));

	tris_indexes_arrays_.insert(std::make_pair(i_name, tris_idx_array));

	btConvexTriangleMeshShape* trimesh_shape = new btConvexTriangleMeshShape(tris_idx_array, true);
	collision_shapes_.push_back(trimesh_shape);

	btScalar mass(i_rigid_body.mass()); 
	btVector3 local_inertia;
	trimesh_shape->calculateLocalInertia(mass, local_inertia);

	btDefaultMotionState* motion_state = new btDefaultMotionState(toBtTransform(i_rigid_body.transform()));
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motion_state, trimesh_shape, local_inertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	//btRigidBody* body = new btRigidBody(mass, 0, trimesh_shape, local_inertia);

	//add the body to the dynamics world
	dynamics_world_->addRigidBody(body);
}

void BulletEngineWrapper::addDynamicSoftBody(const Eigen::Affine3d& i_transform)
{
  // TODO
}

void BulletEngineWrapper::addStaticRigidBody(const std::string& i_name,const RigidBody& i_rigid_body)
{
	assert(is_init_ && "Trying to add static rigid body but Bullet physics is not initialized");
	assert(i_rigid_body.mass() == 0. && "Static bodies must have null mass");

	// add polygon soup to bullet engine   
	// Vertices
	BulletBaseArray<btVector3>* verts = new BulletBaseArray<btVector3>();
	verts->size = i_rigid_body.polygon_soup().verts().size();
	verts->data = new btVector3[i_rigid_body.polygon_soup().verts().size()];
	toBtVector3Array(i_rigid_body.polygon_soup().verts(), Z_2_Y_Matrix, verts->data);

	bodies_verts_.insert(std::make_pair(i_name, verts)); 

	// Triangles
	BulletBaseArray<int>* tris = new BulletBaseArray<int>();
	tris->size = i_rigid_body.polygon_soup().tris().size()*3;
	tris->data = new int[i_rigid_body.polygon_soup().tris().size()*3];

	for (size_t i = 0 ; i < i_rigid_body.polygon_soup().tris().size() ; ++i)
	{
		const Triangle& t = i_rigid_body.polygon_soup().tris()[i];
		tris->data[i*3 + 0] = t.get<0>();
		tris->data[i*3 + 1] = t.get<1>();
		tris->data[i*3 + 2] = t.get<2>();
	}

	bodies_tris_.insert(std::make_pair(i_name, tris));

	btTriangleIndexVertexArray* tris_idx_array = new btTriangleIndexVertexArray(
		tris->size/3, tris->data, sizeof(int) * 3,
		verts->size, (btScalar*) &verts->data[0].x(), sizeof(btVector3));

	tris_indexes_arrays_.insert(std::make_pair(i_name, tris_idx_array));

	btScalar mass(i_rigid_body.mass()); // rigid body is static -> mass must be 0
	btVector3 local_inertia(0., 0., 0.);

	// get poly soup AABB in Bullet referential (Y up)
	const AABB soup_aabb = i_rigid_body.polygon_soup().aabb();
	btVector3 aabb_min = toBtVector3((Z_2_Y_Matrix * soup_aabb.bmin).cwiseMin(Z_2_Y_Matrix * soup_aabb.bmax));
	btVector3 aabb_max = toBtVector3((Z_2_Y_Matrix * soup_aabb.bmin).cwiseMax(Z_2_Y_Matrix * soup_aabb.bmax));
	btBvhTriangleMeshShape* trimesh_shape = new btBvhTriangleMeshShape(tris_idx_array, kUseQuantizedAabbCompression, aabb_min, aabb_max, true);
	collision_shapes_.push_back(trimesh_shape);

	btDefaultMotionState* motion_state = new btDefaultMotionState(toBtTransform(i_rigid_body.transform()));
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motion_state, trimesh_shape, local_inertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	//btRigidBody* body = new btRigidBody(mass, 0, trimesh_shape, local_inertia);

	//add the body to the dynamics world
	dynamics_world_->addRigidBody(body);
}

