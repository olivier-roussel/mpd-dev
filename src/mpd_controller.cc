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

#include "mpd_controller.h"
#include "bullet_engine_wrapper.h"
#include "constants.h"

// for rope expriment
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>

MPDController::MPDController()
  :env_(NULL),
  physics_engine_(NULL),
  physics_thread_(NULL),
  physics_time_step_ms_(kDefaultPhysicsTimeStepMs),
  debug_physics_viewer_(NULL),
  is_physics_paused_(false)
{
}

MPDController::~MPDController()
{
  // not needed as environment is a rigid body managed by the phsyics engine
  //if (env_)
  //{
  //  delete env_;
  //  env_ = NULL;
  //}
  env_ = NULL;
  if (physics_thread_)
  {
    delete physics_thread_;
    physics_thread_ = NULL;
  }
  if (physics_engine_)
  {
    delete physics_engine_;
    physics_engine_ = NULL;
  }
}

const Environment& MPDController::environment() const
{
  assert(env_ != NULL && "environment is uninitialized.");
  return *env_;
}

const PhysicsEngine& MPDController::physics_engine() const
{
  assert(physics_engine_ != NULL && "physics engine is uninitialized.");
  return *physics_engine_;
}

PhysicsEngine* MPDController::physics_engine_mutable() 
{
  return physics_engine_;
}

bool MPDController::loadEnvironment(const boost::filesystem::path& path)
{
  //if (env_)
  //{
  //  delete env_;
  //  env_ = NULL;
  //}
  PolygonSoup soup;
  if (soup.loadFromFile(path))
  {
    env_ = new Environment(soup, Eigen::Affine3d::Identity());
    if (physics_engine_)
      physics_engine_->setWorldAABB(env_->transform() * env_->polygon_soup().aabbmin(), env_->transform() * env_->polygon_soup().aabbmax());

    return true;
  }else
    return false;
}

void MPDController::switchEnvironmentAxis()
{
  if (env_)
    env_->switchPolygonSoupAxis();
}

void MPDController::invertEnvironmentTriangles()
{
  if (env_)
    env_->invertPolygonSoupTriangles();
}

bool MPDController::isEnvironmentSet() const
{
  return env_ != NULL;
}

bool MPDController::isPhysicsEngineSet() const
{
  return physics_engine_ != NULL;
}

unsigned int MPDController::physics_time_step() const
{
  return physics_time_step_ms_;
}

void MPDController::set_physics_time_step(unsigned int i_physics_time_step_ms)
{
  physics_time_step_ms_ = i_physics_time_step_ms;
}

bool MPDController::isPhysicsInitialized() const
{
  return (physics_engine_ && physics_engine_->is_init());
}

void MPDController::quitPhysics()
{
  if (physics_thread_)
  {
    physics_thread_->set_is_done(true);
    physics_thread_->join();
    delete physics_thread_;
    physics_thread_ = NULL;
  }
  if (physics_engine_)
  {
    physics_engine_->quit();
    delete physics_engine_;
    physics_engine_ = NULL;
  }
}

bool MPDController::initPhysics(const PhysicsEngine::ImplementationType i_physics_engine_type)
{
  assert (static_cast<int>(i_physics_engine_type) >= 0 && static_cast<int>(i_physics_engine_type) < PhysicsEngine::NB_IMPLEMENTATION_TYPES && "invalid physics engine implementation");
  assert (physics_time_step_ms_ != 0 && "physics loop time must be above 0");

  // delete if any engine running
  if (physics_engine_)
  {
    quitPhysics();
  }

  // init a new engine
  switch (i_physics_engine_type)
  {
  case PhysicsEngine::PE_BULLET:
    physics_engine_ = new BulletEngineWrapper(debug_physics_viewer_);
    break;
  default:
    std::cout << "Unknown physics engine implementation type" << std::endl;
    return false;
  }

  // init phsyics engine 
  if (!physics_engine_->init())
  {
    std::cout << "MPDController::initPhysics : Could not initialize physics engine" << std::endl;
    return false;
  }

  // synchronize physics with existing data
  if (isEnvironmentSet())
  {
    physics_engine_->addStaticRigidBody("environment", env_);
    physics_engine_->setWorldAABB(env_->transform() * env_->polygon_soup().aabbmin(), env_->transform() * env_->polygon_soup().aabbmax());
  }
  
  // init new physics thread
  physics_thread_ = new PhysicsThread(physics_engine_);
  physics_thread_->run(physics_time_step_ms_, 1.);

  return true;
}

//void MPDController::enableGravity(bool i_enable_gravity)
//{
//  assert (isPhysicsInitialized() && "cannot add rigid bodies if physics engine not initialized");
//
//  physics_engine_->enableGravity(i_enable_gravity);
//}

void MPDController::addRigidBox(const std::string& i_name, double i_mass, const Eigen::Affine3d& i_transform)
{
  assert (isPhysicsInitialized() && "cannot add rigid bodies if physics engine not initialized");

  const PolygonSoup box_geom = PolygonSoup::createBox(1.);

  RigidBody* box_body = new RigidBody(box_geom, i_mass, i_transform);

  physics_engine_->addDynamicRigidBody(i_name, box_body);
}

void MPDController::addSoftBox(const std::string& i_name, double i_mass, const Eigen::Affine3d& i_transform)
{
  assert (isPhysicsInitialized() && "cannot add soft bodies if physics engine not initialized");

  const PolygonSoup box_geom = PolygonSoup::createBox(1.);
  std::vector<double> nodes_masses(box_geom.verts().size(), 0.);  // unused so far
  SoftBody* box_body = new SoftBody(box_geom, i_mass, nodes_masses, i_transform);

  physics_engine_->addDynamicSoftBody(i_name, box_body);
}

void MPDController::addSoftRope(const std::string& i_name, double i_mass, int i_nnodes, double i_len, const Eigen::Affine3d& i_transform)
{
  assert (isPhysicsInitialized() && "cannot add soft bodies if physics engine not initialized");

  PolygonSoup rope_geom;
	const double delta_len = i_len / static_cast<double>(i_nnodes - 1);
	for (int i = 0 ; i < i_nnodes ; ++i)
    rope_geom.addVertex(Eigen::Vector3d(delta_len * i, 0., 0.));
  std::vector<double> nodes_masses(rope_geom.verts().size(), 0.); // unused so far
  SoftBody* box_body = new SoftBody(rope_geom, i_mass, nodes_masses, i_transform);
  
  for (size_t i = 0 ; i < i_nnodes-1 ; ++i)
    box_body->edges_mutable().push_back(std::make_pair(i, i+1));
  physics_engine_->addDynamicSoftBody(i_name, box_body);
}

bool MPDController::addRigidBodyFromMeshFile(const std::string& i_name, const boost::filesystem::path& path, double i_mass, const Eigen::Affine3d& i_transform)
{
  assert (isPhysicsInitialized() && "cannot add rigid bodies if physics engine not initialized");

  PolygonSoup soup;
  if (soup.loadFromFile(path))
  {
    RigidBody* body = new RigidBody(soup, i_mass, i_transform);

    physics_engine_->addDynamicRigidBody(i_name, body);
    return true;
  }else
    return false;
}

bool MPDController::addSoftBodyFromMeshFile(const std::string& i_name, const boost::filesystem::path& path, double i_mass, const Eigen::Affine3d& i_transform)
{
  assert (isPhysicsInitialized() && "cannot add rigid bodies if physics engine not initialized");

  PolygonSoup soup;
  if (soup.loadFromFile(path))
  {
    std::vector<double> nodes_masses(soup.verts().size(), 0.);
    SoftBody* body = new SoftBody(soup, i_mass, nodes_masses, i_transform);

    physics_engine_->addDynamicSoftBody(i_name, body);
    return true;
  }else
    return false;
}

void MPDController::setPhysicsDebugDrawer(MPDViewer* i_viewer)
{
  assert (i_viewer != NULL && "debug drawer for physics engine is NULL");

  debug_physics_viewer_ = i_viewer;
}

void MPDController::setPhysicsPaused(bool i_physics_paused)
{
  assert (physics_thread_ != NULL && "physics has not been initialized yet");

  physics_thread_->set_is_paused(i_physics_paused);
}

bool MPDController::isPhysicsPaused() const
{
  assert (physics_thread_ != NULL && "physics has not been initialized yet");

  return physics_thread_->is_paused();
}


void MPDController::runRopeExperiment(double i_mass, int i_nnodes)
{
  assert (isPhysicsInitialized() && "cannot add soft bodies if physics engine not initialized");

	std::cout << "[PROGRESS] MPDController::runRopeExperiment() : Starting... " << std::endl;
	srand(3150);
	const int nropes = 100;
	const double rope_len = 50.;

	const double force_ampl = 40.;
	const int nforces = 50;

	SoftBodyParameters params;
	params.k_DP = 0.05;
	params.p_niters = 100;
	params.v_niters = 10;
	//params.d_niters = 20;

	const std::string nforces_str = boost::lexical_cast<std::string>(nforces);
	const std::string exp_suffix = "f" + nforces_str + "_n" + boost::lexical_cast<std::string>(i_nnodes) + 
		"_m" + boost::lexical_cast<std::string>(nropes) + "_l" +  boost::lexical_cast<std::string>(rope_len);
	for (int i = 0 ; i < nropes ; ++i)
	{
		const std::string rope_name = "exp_rope_" + boost::lexical_cast<std::string>(i);
		Eigen::Affine3d transform(Eigen::Affine3d::Identity());
		transform.translate(Eigen::Vector3d(0., 0., 40. + 50. * i));
		addSoftRope(rope_name, i_mass, i_nnodes, rope_len, transform);
		physics_engine_->setSoftBodyParameters(rope_name, params);
		if (i % (nropes / 10) == 0)
			std::cout << "[PROGRESS] MPDController::runRopeExperiment() : Creating ropes: " << i * 100 / nropes << "% done." << std::endl;

	}
	std::cout << "[PROGRESS] MPDController::runRopeExperiment() : Ropes created. Now applying random forces on it. " << std::endl;

	// randomize deformations
	const int force_wait_iter = 100;
	for (int j = 0 ; j < nforces ; ++j)
	{
		std::cout << "[PROGRESS] MPDController::runRopeExperiment() : Applying forces _ iter = " << j << std::endl;
		for (int i = 0 ; i < nropes ; ++i)
		{
			const std::string rope_name = "exp_rope_" + boost::lexical_cast<std::string>(i);
			const Eigen::Vector3d in_force = Eigen::Vector3d::Random().normalized() * force_ampl;
			//const Eigen::Vector3d in_force = Eigen::Vector3d(0., 1., 0.) * force_ampl;
			const int node = rand() % i_nnodes;
			physics_engine_->applyForceOnSoftBody(rope_name, in_force, node);
		}
		physics_thread_->doSteps(physics_time_step_ms_, force_wait_iter);
	}

  // wait for moves terminates
	const int stabilize_wait_iter = 800;
	const int start_iter = physics_engine_->niter();
	std::cout << "[PROGRESS] MPDController::runRopeExperiment() : Now wait for ropes to stabilize... " << std::endl;

	physics_thread_->doSteps(physics_time_step_ms_, stabilize_wait_iter);
	physics_thread_->set_is_paused(true);

	const unsigned int left_ropes = physics_engine_->getNumberOfSoftBodies();
	std::cout << "[PROGRESS] MPDController::runRopeExperiment() : After run n_ropes = " << left_ropes << " ( before n_ropes = " << nropes << std::endl;

	// output deformations measures matrix
	std::ofstream d_mat_file;
  d_mat_file.open("E:\\flecto\\reducing_deformation_dimensionality\\d_" + exp_suffix + ".mat");
	d_mat_file << "# name: D" << nforces_str << std::endl;
	d_mat_file << "# type: matrix" << std::endl;
	d_mat_file << "# rows: " << left_ropes << std::endl;
	d_mat_file << "# columns: " << i_nnodes * 3 << std::endl;

	// output base geometry matrix
	std::ofstream p_mat_file;
	p_mat_file.open("E:\\flecto\\reducing_deformation_dimensionality\\p_" + exp_suffix + ".mat");
	p_mat_file << "# name: P" << nforces_str << std::endl;
	p_mat_file << "# type: matrix" << std::endl;
	p_mat_file << "# rows: " << left_ropes << std::endl;
	p_mat_file << "# columns: " << i_nnodes * 3 << std::endl;

	for (int i = 0 ; i < nropes ; ++i)
	{
		const std::string rope_name = "exp_rope_" + boost::lexical_cast<std::string>(i);
		const boost::optional<const SoftBody> soft_body = physics_engine_->getSoftBody(rope_name);
		if (soft_body)
		{
			for (int j = 0 ; j < i_nnodes ; ++j)
			{
				const SoftBody debug_body = *soft_body;	// for debugging
				const Eigen::Vector3d& node_pos = soft_body->nodes_positions()[j];
				const Eigen::Vector3d base_vert = soft_body->transform() * soft_body->base_geometry().verts()[j];
				d_mat_file << " " << node_pos.x() - base_vert.x() << " " << node_pos.y() - base_vert.y() << " " << node_pos.z() - base_vert.z();
				p_mat_file << " " << base_vert.x() << " " << base_vert.y() << " " << base_vert.z();
			}
			d_mat_file << std::endl;
			p_mat_file << std::endl;
		}
	}
	d_mat_file.close();
	p_mat_file.close();

}
