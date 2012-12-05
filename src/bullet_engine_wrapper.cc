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
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

static const bool kUseQuantizedAabbCompression = true;

BulletEngineWrapper::BulletEngineWrapper(MPDViewer* i_debug_physics_viewer) :
  collision_shapes_(), collision_config_(NULL), dispatcher_(NULL), overlapping_pair_cache_(NULL),
  solver_(NULL), dynamics_world_(NULL), 
  tris_indexes_arrays_(),
  bodies_verts_(),
  bodies_tris_(),
  bt_rigid_bodies_(),
  bt_soft_bodies_(),
	debug_physics_drawer_(NULL),
	debug_physics_viewer_(i_debug_physics_viewer),
	world_soft_config_()
{
}

BulletEngineWrapper::~BulletEngineWrapper()
{
  _quit();
}

bool BulletEngineWrapper::_init()
{

	//collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	collision_config_= new btSoftBodyRigidBodyCollisionConfiguration();

	//use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	dispatcher_= new btCollisionDispatcher(collision_config_);

	//btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btVector3 worldMin(-200, -200, -200);	// XXX 
	btVector3 worldMax(200, 200, 200);
	overlapping_pair_cache_ = new bt32BitAxisSweep3(worldMin, worldMax, 30000);
	//overlapping_pair_cache_ = new btDbvtBroadphase();

	//the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	solver_ = new btSequentialImpulseConstraintSolver();

	dynamics_world_ = new btSoftRigidDynamicsWorld(dispatcher_, overlapping_pair_cache_, solver_, collision_config_);

	// soft bodies configuration
	world_soft_config_.m_sparsesdf.Initialize();
	world_soft_config_.m_dispatcher = dispatcher_;
	world_soft_config_.m_broadphase = overlapping_pair_cache_;

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
		world_soft_config_.m_gravity = toBtVector3(Z_2_Y_Matrix * kGravity);
	}

	return true;
}

bool BulletEngineWrapper::_enableGravity(bool i_enable_gravity)
{
	if (is_gravity_ && is_init())
	{
		dynamics_world_->setGravity(toBtVector3(Z_2_Y_Matrix * kGravity));
		//m_softBodyWorldInfo.m_gravity.setValue(0,-9.8,0);
		//m_softBodyWorldInfo.m_sparsesdf.Initialize();
		return true;
	}else
		return false;
}

void BulletEngineWrapper::_quit()
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

	// XXX fis this
	for (std::map<std::string, BulletRigidBody>::iterator it = bt_rigid_bodies_.begin() ; it != bt_rigid_bodies_.end() ; ++it)
	{
		if (it->second.bt_rigid_body)
		{
			delete it->second.bt_rigid_body;
			it->second.bt_rigid_body = NULL;
		}
	}
	bt_rigid_bodies_.clear();

	for (std::map<std::string, BulletSoftBody>::iterator it = bt_soft_bodies_.begin() ; it != bt_soft_bodies_.end() ; ++it)
	{
		if (it->second.bt_soft_body)
		{
			delete it->second.bt_soft_body;
			it->second.bt_soft_body = NULL;
		}
	}
	bt_soft_bodies_.clear();

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

	world_soft_config_ = btSoftBodyWorldInfo();

}

bool BulletEngineWrapper::_addDynamicRigidBody(const std::string& i_name, RigidBody* i_rigid_body)
{
  // add polygon soup to bullet engine   
	// Vertices
	const PolygonSoup& soup = i_rigid_body->polygon_soup();
	BulletBaseArray<btVector3>* verts = new BulletBaseArray<btVector3>();
	verts->size = soup.verts().size();
	verts->data = new btVector3[soup.verts().size()];
	toBtVector3Array(soup.verts(), verts->data);

	bodies_verts_.insert(std::make_pair(i_name, verts)); 

	// Triangles
	BulletBaseArray<int>* tris = new BulletBaseArray<int>();
	tris->size = soup.tris().size()*3;
	tris->data = new int[soup.tris().size()*3];

	for (size_t i = 0 ; i < soup.tris().size() ; ++i)
	{
		const Triangle& t = soup.tris()[i];
		tris->data[i*3 + 0] = t[0];
		tris->data[i*3 + 1] = t[1];
		tris->data[i*3 + 2] = t[2];
	}

	bodies_tris_.insert(std::make_pair(i_name, tris));

	btTriangleIndexVertexArray* tris_idx_array = new btTriangleIndexVertexArray(
		tris->size/3, tris->data, sizeof(int) * 3,
		verts->size, (btScalar*) &verts->data[0].x(), sizeof(btVector3));

	tris_indexes_arrays_.insert(std::make_pair(i_name, tris_idx_array));

	btConvexTriangleMeshShape* trimesh_shape = new btConvexTriangleMeshShape(tris_idx_array, true);
	collision_shapes_.push_back(trimesh_shape);

	btScalar mass(i_rigid_body->mass()); 
	btVector3 local_inertia;
	trimesh_shape->calculateLocalInertia(mass, local_inertia);

	btDefaultMotionState* motion_state = new btDefaultMotionState(toBtTransform(Z_2_Y_Matrix * i_rigid_body->transform()));
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motion_state, trimesh_shape, local_inertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	//add the body to the dynamics world
	dynamics_world_->addRigidBody(body);

	// add body to bullet wrapper and generic engine
	BulletRigidBody wrapped_body(body, i_rigid_body);
	bt_rigid_bodies_.insert(std::make_pair(i_name, wrapped_body));

	return true;
}

bool BulletEngineWrapper::_addDynamicSoftBody(const std::string& i_name, SoftBody* i_soft_body)
{
	unsigned int nnodes = i_soft_body->nb_nodes();

	const PolygonSoup& soup = i_soft_body->polygon_soup();
	btAlignedObjectArray<btVector3> verts;
	verts.resize(nnodes);
	for (size_t i = 0 ; i < nnodes ; ++i)
	{
		const Eigen::Vector3d& v = soup.verts()[i];
		verts[i] = toBtVector3(v);
	}

	btSoftBody* body = new btSoftBody(&world_soft_config_, nnodes, &verts[0], 0);

	int ntris = static_cast<int>(soup.tris().size());
	btAlignedObjectArray<bool> chks;
	chks.resize(nnodes * nnodes, false);
	for (size_t i = 0 ; i < ntris ; ++i)
	{
		const Triangle& tri = soup.tris()[i];
		for (int prevk = 2, k = 0 ; k < 3 ; prevk = k++)
		{
			if (!chks[tri[k] * nnodes + tri[prevk]])
			{
				chks[tri[k] * nnodes + tri[prevk]] = true;
				chks[tri[prevk] * nnodes + tri[k]] = true;
				body->appendLink(tri[prevk], tri[k]);
			}
		}
		body->appendFace(tri[0], tri[1], tri[2]);
	}

	// define soft body material
	btSoftBody::Material*	material = body->appendMaterial();
	material->m_kLST = 0.3;
	material->m_kAST = 0.2;
	material->m_kVST = 0.5;

	body->generateBendingConstraints(2, material);
	body->m_cfg.piterations =	2;
	body->m_cfg.collisions = btSoftBody::fCollision::SDF_RS+
		btSoftBody::fCollision::CL_SS +
		btSoftBody::fCollision::CL_SELF;

	body->m_cfg.kDF = 0.5;
	body->randomizeConstraints();
	body->transform(toBtTransform(Z_2_Y_Matrix * i_soft_body->transform()));
	body->setTotalMass(i_soft_body->mass(), true);
	body->generateClusters(16);	

	//add the body to the dynamics world
	dynamics_world_->addSoftBody(body);

	// add body to bullet wrapper and generic engine
	BulletSoftBody wrapped_body(body, i_soft_body);
	bt_soft_bodies_.insert(std::make_pair(i_name, wrapped_body));

	return true;
}

bool BulletEngineWrapper::_addStaticRigidBody(const std::string& i_name, RigidBody* i_rigid_body)
{
	assert(i_rigid_body->mass() == 0. && "Static bodies must have null mass");

	// add polygon soup to bullet engine   
	// Vertices
	const PolygonSoup& soup = i_rigid_body->polygon_soup();
	BulletBaseArray<btVector3>* verts = new BulletBaseArray<btVector3>();
	verts->size = soup.verts().size();
	verts->data = new btVector3[soup.verts().size()];
	toBtVector3Array(soup.verts(), verts->data);

	bodies_verts_.insert(std::make_pair(i_name, verts)); 

	// Triangles
	BulletBaseArray<int>* tris = new BulletBaseArray<int>();
	tris->size = soup.tris().size()*3;
	tris->data = new int[soup.tris().size()*3];

	for (size_t i = 0 ; i < soup.tris().size() ; ++i)
	{
		const Triangle& t = soup.tris()[i];
		tris->data[i*3 + 0] = t[0];
		tris->data[i*3 + 1] = t[1];
		tris->data[i*3 + 2] = t[2];
	}

	bodies_tris_.insert(std::make_pair(i_name, tris));

	btTriangleIndexVertexArray* tris_idx_array = new btTriangleIndexVertexArray(
		tris->size/3, tris->data, sizeof(int) * 3,
		verts->size, (btScalar*) &verts->data[0].x(), sizeof(btVector3));

	tris_indexes_arrays_.insert(std::make_pair(i_name, tris_idx_array));

	btScalar mass(i_rigid_body->mass()); // rigid body is static -> mass must be 0
	btVector3 local_inertia(0., 0., 0.);

	// get poly soup AABB 
	const AABB soup_aabb = soup.aabb();
	const btVector3 aabb_min = toBtVector3(soup_aabb.bmin);
	const btVector3 aabb_max = toBtVector3(soup_aabb.bmax);
	btBvhTriangleMeshShape* trimesh_shape = new btBvhTriangleMeshShape(tris_idx_array, kUseQuantizedAabbCompression, aabb_min, aabb_max, true);
	collision_shapes_.push_back(trimesh_shape);

	btDefaultMotionState* motion_state = new btDefaultMotionState(toBtTransform(Z_2_Y_Matrix * i_rigid_body->transform()));
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motion_state, trimesh_shape, local_inertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	//add the body to the dynamics world
	dynamics_world_->addRigidBody(body);

	// add body to bullet wrapper and generic engine
	BulletRigidBody wrapped_body(body, i_rigid_body);
	bt_rigid_bodies_.insert(std::make_pair(i_name, wrapped_body));

	return true;
}

void BulletEngineWrapper::_doOneStep(unsigned int i_step_time_ms)
{
	dynamics_world_->stepSimulation(static_cast<float>(i_step_time_ms *1.e-3f));

	// update generic bodies transformations
	for (std::map<std::string, BulletRigidBody>::iterator it_body = bt_rigid_bodies_.begin() ; it_body != bt_rigid_bodies_.end() ; ++it_body)
	{
		it_body->second.rigid_body->set_transform(Y_2_Z_Matrix * toETransform(it_body->second.bt_rigid_body->getWorldTransform()));
	}

	// TODO soft bodies

	// debug drawing
	{
		boost::mutex::scoped_lock lock(debug_physics_drawer_->getPhysicsObjectsMutex());
		debug_physics_drawer_->clearDraws();
		dynamics_world_->debugDrawWorld();
	}

}