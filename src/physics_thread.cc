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

#include "physics_thread.h"

#include "physics_engine.h"

PhysicsThread::PhysicsThread(PhysicsEngine* i_physics_engine) : 
  is_done_(true),
	is_paused_(false),
  physics_engine_(i_physics_engine),
  loop_time_ms_(0),
  thread_(),
  timer_(io_service_),
  io_service_(),
	speed_factor_(1.)
{
  assert(physics_engine_ != NULL && "physics thread initialized with NULL physics engine");
}

PhysicsThread::~PhysicsThread()
{
  // no op
}

void PhysicsThread::run(unsigned int i_loop_time_ms, double i_speed_factor)
{
  is_done_ = false;
	is_paused_ = false;
  loop_time_ms_ = i_loop_time_ms;
	speed_factor_ = i_speed_factor;
  thread_ = boost::thread(&PhysicsThread::_run, this);
}

void PhysicsThread::doSteps(unsigned int i_loop_time_ms, unsigned int i_nsteps)
{
	timer_.cancel();

	for (unsigned int i = 0 ; i < i_nsteps ; ++i)
		physics_engine_->doOneStep(static_cast<double>(i_loop_time_ms));
}

void PhysicsThread::join()
{
  thread_.join();
}

void PhysicsThread::set_is_done(bool i_is_done)
{
  is_done_ = i_is_done;
}

bool PhysicsThread::is_done() const
{
  return is_done_;
}

bool PhysicsThread::is_paused() const
{
	return is_paused_;
}

void PhysicsThread::set_is_paused(bool i_is_paused)
{
	is_paused_ = i_is_paused;
	if (!is_paused_)
	{
		_update();
	}else{
		timer_.cancel();
	}
}

void PhysicsThread::_run()
{
  if (physics_engine_->is_init())
  {
    timer_.expires_from_now(boost::posix_time::milliseconds(loop_time_ms_));
    timer_.async_wait(boost::bind(&PhysicsThread::_update, this));
    io_service_.run();
  }else{
    std::cout << "PhysicsThread::_run() : Physics engine not initialized" << std::endl;
    is_done_ = true;
  }
}

void PhysicsThread::_update()
{
	if (!is_paused_)
	{
		physics_engine_->doOneStep(static_cast<double>(loop_time_ms_));

		// schedule next update excepted if is_done() condition not reached
		if (!is_done())
		{
			timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(loop_time_ms_));
			timer_.async_wait(boost::bind(&PhysicsThread::_update, this));
		}else
			_quit();
	}
}

void PhysicsThread::_quit()
{
  // no op
	timer_.cancel();
}
