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

#include "mpd/physics_thread.h"

PhysicsThread::PhysicsThread(PhysicsEngine* i_physics_engine) : 
  is_done_(true),
  physics_engine_(i_physics_engine),
  loop_time_ms_(0),
  thread_(),
  timer_(io_service_),
  io_service_()
{
  assert(physics_engine_ != NULL && "physics thread initialized with NULL physics engine");
}

PhysicsThread::~PhysicsThread()
{
  // no op
}

void PhysicsThread::run(unsigned int i_loop_time_ms)
{
  is_done_ = false;
  loop_time_ms_ = i_loop_time_ms;
  thread_ = boost::thread(&PhysicsThread::_run, this);
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
  physics_engine_->doOneStep(static_cast<double>(loop_time_ms_ * 10e3));

  // schedule next update excepted if is_done() condition not reached
  if (!is_done())
  {
    timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(loop_time_ms_));
    timer_.async_wait(boost::bind(&PhysicsThread::_update, this));
  }else
    _quit();
}

void PhysicsThread::_quit()
{
  // no op
}
