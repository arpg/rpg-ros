/*!
 * drrobot_joystick_teleop.cpp
 * Copyright (c) 2011, Dr Robot Inc
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * please referee the http://www.ros.org/wiki/joy/Tutorials/ConfiguringALinuxJoystick.
 *
 * need check the joy stick first, here suppose joy stick is js0 on the computer.
 * ls /dev/input/
 * sudo jstest /dev/input/js0
 * ls -l /dev/input/js0
 * sudo chmod a+rw /dev/input/js0
 * rosparam set joy_node/dev "/dev/input/js0"
 * rosrun joy joy_node
 * rostopic echo joy
 *
 */

/*!

@mainpage
  drrobot_joystick_teleop for demonstration and testing published geometry_msgs/Twist message to drrobot_player by Joystick control.
  It will use Logitech Extreme 3D PRO joystickto control robot move around
  at    ( 0, 0)   position will stop robot.
  at    ( 0, 1)   position will let robot go forward
  at    ( 0,-1)  position will let robot go backward
  at    ( 1, 0)   position will let robot turn left
  at    (-1, 0)  position will let robot turn right
<hr>

@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.
          For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
          then run drrobot_player first. then start ros joy node.
@verbatim
$ drrobot_joystick_teleop
@endverbatim

<hr>
@section topic ROS topics

Publishes to (name / type):
-@b cmd_vel: will publish drrobot_cmd_vel Message to drrobot_player. For robot from Dr Robot Inc, we only need provide linear.x
    as going forward/backward speed, and angular.z as turning speed. jaguar_base will transform these command value to encoder control
    command value and send them to motion control system on the robot
<hr>
*/


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class JoystickTeleopMode
{
public:
  JoystickTeleopMode();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;
  geometry_msgs::Twist cmdvel_;
  int linear_, angular_, flipper_f, flipper_r;
  double l_scale_, a_scale_, f_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};

//Map the default axes to joystick axes 1,2,3
JoystickTeleopMode::JoystickTeleopMode():
  linear_(3),
  angular_(2),
  flipper_f(1),
  flipper_r(0)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("axis_flipper_f", flipper_f, flipper_f);
  nh_.param("axis_flipper_r", flipper_r, flipper_r);
 
  l_scale_ = -0.5;
  a_scale_ = -0.5;
  
  
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  f_scale_ = 0.5;
  nh_.param("scale_flipper", f_scale_, f_scale_);
  ROS_INFO("Using flipper scale: %1.2f", f_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoystickTeleopMode::joyCallback, this);

}

void JoystickTeleopMode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_INFO("Got callback");
  cmdvel_.linear.x = l_scale_*joy->axes[linear_];
  cmdvel_.angular.z = a_scale_*joy->axes[angular_];
  cmdvel_.angular.y = f_scale_*joy->axes[flipper_f];
  cmdvel_.angular.x = f_scale_*joy->axes[flipper_r];
  ROS_INFO("Send control command [ %1.2f, %1.2f, %1.2f, %1.2f]", cmdvel_.linear.x, 
	   cmdvel_.angular.z, cmdvel_.angular.y, cmdvel_.angular.x);
  vel_pub_.publish(cmdvel_);


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_teleop");
  JoystickTeleopMode joystick_teleop;

  ros::spin();
}
