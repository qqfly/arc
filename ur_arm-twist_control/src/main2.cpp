/*
 *    Universal Robots UR5 ROS node
 *    Copyright (C) 2012 Wouter Caarls
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ur_arm/ur_arm_node.h>

using namespace ur_arm;

/// Entry point for UR5 arm controller node
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_arm_controller");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  std::string host = std::string("192.168.11.104");
  int port = 30002;

  ros::param::get("host", host);
  ros::param::get("port", port);

  Arm *arm = new Arm(host, port);
  ArmNode arm_node(arm);
  ToolTwist speed1;
  speed1.x=0.05;
  speed1.y=0;
  speed1.z=0;
  speed1.roll=0;
  speed1.pitch=-0.1;
  speed1.yaw=0;
  int i=0;
  
while(ros::ok())
{
  arm->setToolSpeed(speed1, 0.1, 0.1);
  i=i+1;
  ros::WallDuration(0.1).sleep();
  std::cout<<"i: "<<i<<std::endl;
  ros::spinOnce();
  loop_rate.sleep();
}

  arm_node.spin();

  return 0;
}
