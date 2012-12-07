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

#ifndef MPD_DEV_PHYSICS_THREAD_H_
#define MPD_DEV_PHYSICS_THREAD_H_

#include <boost/thread.hpp>
#include <boost/asio.hpp>

class PhysicsEngine;

/**
 * \class PhysicsThread
 */
class PhysicsThread 
{

public:
  PhysicsThread(PhysicsEngine* i_physics_engine);
  virtual ~PhysicsThread();
  
  void run(unsigned int i_loop_time_ms);

  void join();

  void set_is_done(bool i_is_done);

	void set_is_paused(bool i_is_paused);

  bool is_done() const;

	bool is_paused() const;

protected:
  /**
   * Attributes
   */
  bool is_done_;
	bool is_paused_;
  
  boost::thread thread_;        //  thread handle
  boost::asio::io_service io_service_;
  boost::asio::deadline_timer timer_;

  unsigned int loop_time_ms_;
  PhysicsEngine* physics_engine_;

  /**
   * Protected methods
   */

  void _update();

  void _run();

  void _quit();
};

#endif // MPD_DEV_PHYSICS_THREAD_H_

