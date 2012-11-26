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

BulletEngineWrapper::BulletEngineWrapper() :
  step_time_(0), is_init_(false), 
  collision_shapes_(), collision_config_(NULL), dispatcher_(NULL), overlapping_pair_cache_(NULL),
  solver_(NULL), dynamics_world_(NULL), 
  tris_indexes_arrays_(),
  bodies_verts_(),
  bodies_tris_(),
  rigid_bodies_(),
  soft_bodies_()
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
    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collision_config_= new btDefaultCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher_= new btCollisionDispatcher(collision_config_);

    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlapping_pair_cache_ = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver_ = new btSequentialImpulseConstraintSolver;

    dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher_, overlapping_pair_cache_, solver_, collision_config_);
  
    is_init_ = true;
  }else
    return false; // already initialized
}

void BulletEngineWrapper::doOneStep(unsigned int i_step_time_ms)
{
  if (is_init_)
    dynamics_world_->stepSimulation(static_cast<float>(i_step_time_ms *1.e-3f));
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

    // BulletBaseArray<> deletes memory at destructor call, so we can just clear maprs
    bodies_verts_.clear();
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
    is_init_ = false;
  }
}

void BulletEngineWrapper::addDynamicRigidBody(const Eigen::Affine3d& i_transform)
{
  // TOdo
}

void BulletEngineWrapper::addDynamicSoftBody(const Eigen::Affine3d& i_transform)
{
  // TOdo
}

void BulletEngineWrapper::addStaticRigidBody(const std::string& i_name, const PolygonSoup& i_soup, const Eigen::Affine3d& i_transform)
{
  if (is_init_)
  {
    // TODO !!! switch to bullet referential (Y up)
    // add polygon soup to bullet engine   
    // Vertices
    BulletBaseArray<btVector3> verts;
    verts.size = i_soup.verts().size();
    verts.data = new btVector3[i_soup.verts().size()];

    //e2bt::toBtVector3Array(i_soup.verts(), &(verts.data[0]));
    e2bt::toBtVector3Array(i_soup.verts(), verts.data);

    bodies_verts_.insert(std::make_pair(i_name, verts));

    // Triangles
    BulletBaseArray<int> tris;
    tris.size = i_soup.tris().size();
    tris.data = new int[i_soup.tris().size()];

    for (size_t i = 0 ; i < i_soup.tris().size() ; ++i)
    {
      const Triangle& t = i_soup.tris()[i];
      tris.data[i*3 + 0] = t.get<0>();
      tris.data[i*3 + 1] = t.get<1>();
      tris.data[i*3 + 2] = t.get<2>();
    }

    bodies_tris_.insert(std::make_pair(i_name, tris));

    btTriangleIndexVertexArray* tris_idx_array = new btTriangleIndexVertexArray(
      tris.size, tris.data, sizeof(int) * 3,
      verts.size, (btScalar*) &verts.data[0].x(), sizeof(btScalar) * 3);

    tris_indexes_arrays_.insert(std::make_pair(i_name, tris_idx_array));
    
    btScalar mass(0.); // rigid body is static
    btVector3 local_inertia(0., 0., 0.);

    // TODO !!!!!
    // Must convert eigen transformation to bullet one
    /*
     * btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    //add the body to the dynamics world
    dynamics_world_->addRigidBody(body);
    */
  }
}
