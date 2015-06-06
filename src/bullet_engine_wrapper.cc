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

#include "bullet_engine_wrapper.h"
#include "constants.h"
#include "common_utils.h"
#include "bullet_debug_drawer.h"
#include <boost/thread/mutex.hpp>
#include <deque>
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

static const bool kUseQuantizedAabbCompression = true;

BulletEngineWrapper::BulletEngineWrapper(MPDViewer* i_debug_physics_viewer) :
  collision_shapes_(), collision_config_(NULL), dispatcher_(NULL), overlapping_pair_cache_(NULL),
  solver_(NULL), dynamics_world_(NULL), 
  tris_indexes_arrays_(),
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
	//overlapping_pair_cache_ = new bt32BitAxisSweep3(btVector3(-200., -200., -200.), btVector3(200., 200., 200.), 30000);	// world aabb will be set when environment will be
	//overlapping_pair_cache_ = new bt32BitAxisSweep3(toBtVector3(world_aabb_min_), toBtVector3(world_aabb_max_), 30000);	// world aabb will be set when environment will be
	overlapping_pair_cache_ = new btDbvtBroadphase();

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

	_enableGravity(is_gravity_);

	return true;
}

bool BulletEngineWrapper::_enableGravity(bool i_enable_gravity)
{
	if (i_enable_gravity)
	{
		dynamics_world_->setGravity(toBtVector3(Z_2_Y_Matrix * kGravity));
		world_soft_config_.m_gravity = toBtVector3(Z_2_Y_Matrix * kGravity);
		world_soft_config_.m_sparsesdf.Initialize();
	}else{
		dynamics_world_->setGravity(btVector3(0., 0., 0.));
		world_soft_config_.m_gravity = btVector3(0., 0., 0.);
		world_soft_config_.m_sparsesdf.Initialize();
	}
	return i_enable_gravity;
}

void BulletEngineWrapper::_quit()
{
	if (!dynamics_world_)
		return; 

	// Release bodies data
	// THese should not be necessary as handled by bullet _ but this would be nice to check this 
	//for (std::map<std::string, btTriangleIndexVertexArray*>::iterator it = tris_indexes_arrays_.begin() ; it != tris_indexes_arrays_.end() ; ++it)
	//{
	//	if (it->second)
	//		delete it->second;
	//}
	tris_indexes_arrays_.clear();

	// Remove bodies
	for (int i = dynamics_world_->getNumCollisionObjects()-1 ; i >= 0 ; --i)
	{
		btCollisionObject* obj = dynamics_world_->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);			// will return NULL pointer if not rigid body
		if (body && body->getMotionState())
			delete body->getMotionState();

		dynamics_world_->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (size_t i = 0 ; i < collision_shapes_.size() ; ++i)
	{
		btCollisionShape* shape = collision_shapes_[i];
		collision_shapes_[i] = NULL;
		delete shape;
	}
	collision_shapes_.clear();

	bt_rigid_bodies_.clear();
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

	//bodies_verts_.insert(std::make_pair(i_name, verts)); 

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

	//bodies_tris_.insert(std::make_pair(i_name, tris));

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

	delete verts;
	delete tris;

	return true;
}

bool BulletEngineWrapper::_addDynamicSoftBody(const std::string& i_name, SoftBody* i_soft_body)
{
	unsigned int nnodes = i_soft_body->nb_nodes();

	const PolygonSoup& soup = i_soft_body->base_geometry();
	btAlignedObjectArray<btVector3> verts;
	verts.resize(nnodes);
	btScalar* m = new btScalar[nnodes];
	for (size_t i = 0 ; i < nnodes ; ++i)
	{
		const Eigen::Vector3d& v = soup.verts()[i];
		verts[i] = toBtVector3(v);
		m[i] = 1.;
	}

	btSoftBody* body = new btSoftBody(&world_soft_config_, nnodes, &verts[0], m);

	delete [] m;

	int ntris = static_cast<int>(soup.tris().size());
	const bool is_linear = ntris == 0;		// true if one dimensionnal soft body (rope)
	// 2d - 3d soft bodies
	if (!is_linear)
	{
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
	}else{
		// 1d soft bodies
		int nlinks = i_soft_body->edges().size();
		if (nlinks)
		{
			for (size_t i = 0 ; i < nlinks ; ++i)
				body->appendLink(i_soft_body->edges()[i].first, i_soft_body->edges()[i].second);
			
		}else{
			// soft body have no triangles and edges
			delete body;
			return false;
		}
	}

	body->m_cfg.collisions = btSoftBody::fCollision::SDF_RS+
		btSoftBody::fCollision::CL_SS + 
		btSoftBody::fCollision::CL_SELF;

	btCollisionShape* body_shape = body->getCollisionShape();
	collision_shapes_.push_back(body_shape);

	// define soft body material
	btSoftBody::Material*	material = body->appendMaterial();

	body->generateBendingConstraints(2, material);

	body->randomizeConstraints();
	body->transform(toBtTransform(Z_2_Y_Matrix * i_soft_body->transform()));
	body->setTotalMass(i_soft_body->mass(), !is_linear);
	body->setPose(true, false);

	body->generateClusters(16);	

	//add the body to the dynamics world
	dynamics_world_->addSoftBody(body);

	// add body to bullet wrapper and generic engine
	BulletSoftBody wrapped_body(body, i_soft_body);
	bt_soft_bodies_.insert(std::make_pair(i_name, wrapped_body));

	_setSoftBodyParameters(i_name, i_soft_body->parameters());

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

	//bodies_verts_.insert(std::make_pair(i_name, verts)); 

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

	//bodies_tris_.insert(std::make_pair(i_name, tris));

	btTriangleIndexVertexArray* tris_idx_array = new btTriangleIndexVertexArray(
		tris->size/3, tris->data, sizeof(int) * 3,
		verts->size, (btScalar*) &verts->data[0].x(), sizeof(btVector3));

	tris_indexes_arrays_.insert(std::make_pair(i_name, tris_idx_array));

	btScalar mass(i_rigid_body->mass()); // rigid body is static -> mass must be 0
	btVector3 local_inertia(0., 0., 0.);

	// get poly soup AABB 
	const btVector3 aabb_min = toBtVector3(soup.aabbmin());
	const btVector3 aabb_max = toBtVector3(soup.aabbmax());
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

	delete verts;
	delete tris;

	return true;
}

bool BulletEngineWrapper::_removeRigidBody(const std::string& i_name)
{
	std::map<std::string, BulletRigidBody>::iterator it_body = bt_rigid_bodies_.find(i_name);
	if (it_body != bt_rigid_bodies_.end())
	{
		btRigidBody* body = it_body->second.bt_rigid_body;
		if (body->getMotionState())
			delete body->getMotionState();

		btCollisionShape* col_shape = body->getCollisionShape();
		if (col_shape)
		{
			for (size_t i = 0 ; i < collision_shapes_.size() ; ++i)
			{
				if (col_shape == collision_shapes_[i])
				{
					delete collision_shapes_[i];
					collision_shapes_[i] = NULL;
					collision_shapes_.erase(collision_shapes_.begin() + i);
				}
				break;
			}
			// For shared collision shapes between bodies
			//for (size_t i = 0 ; i < collision_shapes_.size() ; ++i)
			//{
			//	if (collision_shapes_[i].first == col_shape)
			//	{
			//		collision_shapes_[i].second--;
			//		if (collision_shapes_[i].second == 0)
			//		{
			//			delete collision_shapes_[i].first;
			//			collision_shapes_.erase(collision_shapes_.begin() + i);
			//		}
			//		break;
			//	}
			//}
		}else
			std::cout << "[ERROR] BulletEngineWrapper::_removeRigidBody() : Body " << i_name << "do not have collision shape associated with." << std::endl;
		dynamics_world_->removeRigidBody(body);
		delete body;
		it_body->second.bt_rigid_body = NULL;	
		it_body->second.rigid_body = NULL;	// should have been deleted by generic physics engine
		bt_rigid_bodies_.erase(it_body);
		return true;
	}else
		return false;
}

bool BulletEngineWrapper::_removeSoftBody(const std::string& i_name)
{
	std::map<std::string, BulletSoftBody>::iterator it_body = bt_soft_bodies_.find(i_name);
	if (it_body != bt_soft_bodies_.end())
	{
		btSoftBody* body = it_body->second.bt_soft_body;
		btCollisionShape* col_shape = body->getCollisionShape();
		if (col_shape)
		{
			for (size_t i = 0 ; i < collision_shapes_.size() ; ++i)
			{
				if (col_shape == collision_shapes_[i])
				{
					delete collision_shapes_[i];
					collision_shapes_[i] = NULL;
					collision_shapes_.erase(collision_shapes_.begin() + i);
				}
				break;
			}
			// For shared collision shapes between bodies
			//for (size_t i = 0 ; i < collision_shapes_.size() ; ++i)
			//{
			//	if (collision_shapes_[i].first == col_shape)
			//	{
			//		collision_shapes_[i].second--;
			//		if (collision_shapes_[i].second == 0)
			//		{
			//			delete collision_shapes_[i].first;
			//			collision_shapes_.erase(collision_shapes_.begin() + i);
			//		}
			//		break;
			//	}
			//}
		}else
			std::cout << "[ERROR] BulletEngineWrapper::_removeSoftBody() : Body " << i_name << "do not have collision shape associated with." << std::endl;

		dynamics_world_->removeSoftBody(body);
		delete body;
		it_body->second.bt_soft_body = NULL;	
		it_body->second.soft_body = NULL;	// should have been deleted by generic physics engine
		bt_soft_bodies_.erase(it_body);
		return true;
	}else
		return false;
}

void BulletEngineWrapper::_doOneStep(unsigned int i_step_time_ms)
{
	dynamics_world_->stepSimulation(static_cast<float>(i_step_time_ms *1.e-3f));

	// debug drawing
	if (is_debug_drawer_)
	{
		boost::mutex::scoped_lock lock(debug_physics_drawer_->getPhysicsObjectsMutex());
		debug_physics_drawer_->clearDraws();
		dynamics_world_->debugDrawWorld();
	}

}

void BulletEngineWrapper::_updateBodies()
{
	// update generic rigid bodies transformations
	for (std::map<std::string, BulletRigidBody>::iterator it_body = bt_rigid_bodies_.begin() ; it_body != bt_rigid_bodies_.end() ; ++it_body)
	{
		it_body->second.rigid_body->set_transform(Y_2_Z_Matrix * toETransform(it_body->second.bt_rigid_body->getWorldTransform()));
	}

	// update generic soft bodies transformations & deformations
	for (std::map<std::string, BulletSoftBody>::iterator it_body = bt_soft_bodies_.begin() ; it_body != bt_soft_bodies_.end() ; ++it_body)
	{
		SoftBody* soft_body = it_body->second.soft_body;
		std::vector<Eigen::Vector3d>& nodes_pos = soft_body->nodes_positions_mutable();
		std::vector<Eigen::Vector3d>& nodes_normals = soft_body->nodes_normals_mutable();
		const btSoftBody const* bt_soft_body = it_body->second.bt_soft_body;
		const btSoftBody::tNodeArray& bt_nodes(bt_soft_body->m_nodes);
		if (bt_nodes.size() == nodes_pos.size())
		{
			for (size_t i = 0 ; i < bt_nodes.size() ; ++i)
			{
				const btSoftBody::Node& bt_node = bt_nodes[i];
				if (isValidVector3<btVector3>(bt_node.m_x))	// true if components of vector are finite numbers
				{
					nodes_pos[i] = Y_2_Z_Matrix * toEVector3(bt_node.m_x);
					nodes_normals[i] = Y_2_Z_Matrix * toEVector3(bt_node.m_n);
				}else{
					//std::cout << "[WARNING] BulletEngineWrapper::_updateBodies() : Soft body " << it_body->first << " has an invalid node" << std::endl;
					invalidated_soft_bodies_.push_back(it_body->first);
				}
			}
		}else
			std::cout << "[ERROR] BulletEngineWrapper::_updateBodies() : Soft body " << it_body->first << " number of nodes differs from Bullet number of nodes" << std::endl;
	}

}

void BulletEngineWrapper::_setSoftBodyParameters(const std::string& i_name, const SoftBodyParameters& i_params)
{
	std::map<std::string, BulletSoftBody>::iterator it_body = bt_soft_bodies_.find(i_name);
	if (it_body !=  bt_soft_bodies_.end())
	{
		btSoftBody* bt_soft_body = it_body->second.bt_soft_body;

		bt_soft_body->m_cfg.kVCF = static_cast<btScalar>(i_params.k_ERP);
		bt_soft_body->m_cfg.kDP = static_cast<btScalar>(i_params.k_DP);
		bt_soft_body->m_cfg.kPR = static_cast<btScalar>(i_params.k_PR);
		bt_soft_body->m_cfg.kVC = static_cast<btScalar>(i_params.k_VC);
		bt_soft_body->m_cfg.kDF = static_cast<btScalar>(i_params.k_DF);
		bt_soft_body->m_cfg.kMT = static_cast<btScalar>(i_params.k_MT);
		bt_soft_body->m_cfg.kCHR = static_cast<btScalar>(i_params.k_CHR);
		bt_soft_body->m_cfg.kKHR = static_cast<btScalar>(i_params.k_KHR);
		bt_soft_body->m_cfg.kSHR = static_cast<btScalar>(i_params.k_SHR);
		bt_soft_body->m_cfg.kAHR = static_cast<btScalar>(i_params.k_AHR);

		bt_soft_body->m_cfg.viterations = i_params.v_niters;
		bt_soft_body->m_cfg.piterations = i_params.p_niters;
		bt_soft_body->m_cfg.diterations = i_params.d_niters;

		// XXX so far we assume only one material per body
		bt_soft_body->m_materials[0]->m_kLST = static_cast<btScalar>(i_params.k_LST);
		bt_soft_body->m_materials[0]->m_kAST = static_cast<btScalar>(i_params.k_AST);
		bt_soft_body->m_materials[0]->m_kVST = static_cast<btScalar>(i_params.k_VST);

	}else
		std::cout << "[ERROR] BulletEngineWrapper::_updateSoftBodyParameters() : Soft body " << i_name << " not associated with Bullet engine" << std::endl;
}

void BulletEngineWrapper::_setWorldAABB(const Eigen::Vector3d& i_aabb_min, const Eigen::Vector3d& i_aabb_max)
{
	if (is_init())
	{
		const Eigen::Vector3d aabbmin_y = (Z_2_Y_Matrix * i_aabb_min).cwiseMin(Z_2_Y_Matrix * i_aabb_max);
		const Eigen::Vector3d aabbmax_y = (Z_2_Y_Matrix * i_aabb_min).cwiseMin(Z_2_Y_Matrix * i_aabb_max);
		// TODO update Bullet broadphase world AABB if required...
	}
}

void BulletEngineWrapper::_applyForceOnRigidBody(const std::string& i_name, const Eigen::Vector3d& i_force)
{
	std::map<std::string, BulletRigidBody>::iterator it_body = bt_rigid_bodies_.find(i_name);

	if (it_body != bt_rigid_bodies_.end())
	{
		//it_body->second.bt_rigid_body->applyForce(); // TODO
	}
}

void BulletEngineWrapper::_applyForceOnSoftBody(const std::string& i_name, const Eigen::Vector3d& i_force, int i_node_index)
{
	std::map<std::string, BulletSoftBody>::iterator it_body = bt_soft_bodies_.find(i_name);

	if (it_body != bt_soft_bodies_.end())
	{
		it_body->second.bt_soft_body->addForce(toBtVector3(Z_2_Y_Matrix * i_force), i_node_index);
	}else
		std::cout << "[ERROR] BulletEngineWrapper::_applyForceOnSoftBody() : Soft body " << i_name << " not associated with Bullet engine" << std::endl;
}