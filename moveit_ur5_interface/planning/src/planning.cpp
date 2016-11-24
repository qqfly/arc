/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  Shanghai Jiao Tong University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Qiang Qiu
 *********************************************************************/
#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// MoveIt!
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

using namespace std;

typedef map<int, vector<geometry_msgs::Pose> > C_POSE;

bool executeResult;

bool mPlanning(string movegroup, string refFrame, double target[]);

bool CPlanning(string movegroup, string refFrame, vector<geometry_msgs::Pose> waypoints);

void executeCB(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& msg);

void executeCB_(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg);

int main(int argc, char **argv)
{
   ROS_INFO("Start!");
   ros::init (argc, argv, "planning");
   ros::AsyncSpinner spinner(1);
   spinner.start();

   ros::NodeHandle nh;
   ros::Rate loop_rate(10);
   //ros::Subscriber sub = nh.subscribe("joint_trajectory_action/status",1000,executeCB);
   ros::Subscriber sub = nh.subscribe("follow_joint_trajectory/result",1000,executeCB);
   ros::Subscriber sub_ = nh.subscribe("follow_joint_trajectory/goal",1000,executeCB_);

   /* Initial */
   string fpath = ros::package::getPath("moveit_ur5_interface") + "/planning/data/pose.txt";
   ifstream readFile(fpath.c_str());
   double buffer;
   vector<double> v_buffer;
   vector<vector<double> > vv_buffer;
   v_buffer.clear();
   vv_buffer.clear();
   if (!readFile)
   {
      ROS_ERROR("Cannot Open File");
   } else {
      while (readFile >> buffer)
      {
         v_buffer.push_back(buffer);
         if (v_buffer.size()==10)
         {
            vv_buffer.push_back(v_buffer);
            v_buffer.clear();
         }
      }
      readFile.close();
   }

   //double mj[6] = {-1.62, -1.76, -1.84, 0.49, 1.32, -0.05};
   vector<double> m_joint = vv_buffer[0];       // Ready joint Position
   m_joint.resize(6);                           // only 6 dof
   //cout<<"["<<m_joint[0]<<", "<<m_joint[1]<<", "<<m_joint[2]<<", "<<m_joint[3]<<", "<<m_joint[4]<<", "<<m_joint[5]<<", "<<endl;

   vector<geometry_msgs::Pose> waypoints;

   geometry_msgs::Pose target_pose;
   C_POSE pre_pose;
   for (int i=1;i<vv_buffer.size();i++)
   {
      waypoints.clear();
      
      target_pose.position.x = vv_buffer[i][0];
      target_pose.position.y = vv_buffer[i][1];
      target_pose.position.z = vv_buffer[i][2];
      target_pose.orientation.x = vv_buffer[i][3];
      target_pose.orientation.y = vv_buffer[i][4];
      target_pose.orientation.z = vv_buffer[i][5];
      target_pose.orientation.w = vv_buffer[i][6];
      waypoints.push_back(target_pose);

      target_pose.position.x = vv_buffer[i][7];
      target_pose.position.y = vv_buffer[i][8];
      target_pose.position.z = vv_buffer[i][9];
      waypoints.push_back(target_pose);

      pre_pose[i-1] = waypoints;
   }

   /*for (int i=0;i<pre_pose[0].size();i++)
   {
      cout<<"No."<<i<<"is ["<<pre_pose[0][i].position.x<<", "<<pre_pose[0][i].position.y<<", "<<pre_pose[0][i].position.z<<", "<<pre_pose[0][i].orientation.x<<", "<<pre_pose[0][i].orientation.y<<", "<<pre_pose[0][i].orientation.z<<", "<<pre_pose[0][i].orientation.w<<"]"<<endl;
   }*/

   /* Initial */

   string movegroup = "manipulator";
   double target[7];
   bool result = false;
   executeResult = true;

   //min place
   
   moveit::planning_interface::MoveGroup Jointgroup("manipulator");
   Jointgroup.setPoseReferenceFrame("base_link");
   Jointgroup.setPlannerId("RRTConnectkConfigDefault");

   executeResult = false;
   Jointgroup.setJointValueTarget(m_joint);
   Jointgroup.move();
   while (!executeResult)
   {
      ros::WallDuration(0.5).sleep();
      ROS_INFO("Waiting for executing result!");
   }
   ros::WallDuration(3.0).sleep();

   string refFrame = "base_link";

   executeResult = false;
   do{
      result = CPlanning(movegroup,refFrame,pre_pose[0]);
      ROS_INFO("The result of motion planning is %ld", (long int)result);
   } while(!result);
   while (!executeResult)
   {
      ros::WallDuration(0.5).sleep();
      ROS_INFO("Waiting for executing result!");
   }
   ros::WallDuration(8.0).sleep();

   target_pose.position.y -= 0.1;
   waypoints.clear();
   waypoints.push_back(target_pose);
   executeResult = false;
   do{
      result = CPlanning(movegroup,refFrame,waypoints);
      ROS_INFO("The result of motion planning is %ld", (long int)result);
   } while(!result);
   while (!executeResult)
   {
      ros::WallDuration(0.5).sleep();
      ROS_INFO("Waiting for executing result!");
   }
   ros::WallDuration(3.0).sleep();

   executeResult = false;
   Jointgroup.move();
   while (!executeResult)
   {
      ros::WallDuration(0.5).sleep();
      ROS_INFO("Waiting for executing result!");
   }
   ros::WallDuration(1.0).sleep();
   

   ROS_INFO("Done!");
   return 0;
}

bool mPlanning(string movegroup, string refFrame, double target[]){
   
   moveit::planning_interface::MoveGroup group(movegroup);
   group.setPoseReferenceFrame(refFrame);
   group.setPlannerId("RRTConnectkConfigDefault");

   geometry_msgs::Pose target_pose;
   target_pose.position.x = target[0];
   target_pose.position.y = target[1];
   target_pose.position.z = target[2];
   target_pose.orientation.x = target[3];
   target_pose.orientation.y = target[4];
   target_pose.orientation.z = target[5];
   target_pose.orientation.w = target[6]; 
   group.setPoseTarget(target_pose);

   //Call the planner and compute the plan
   moveit::planning_interface::MoveGroup::Plan my_plan;
   //ROS_INFO("finish planning1");
   bool success = group.plan(my_plan);
   //ROS_INFO(success)
   ROS_INFO("Visualizing plan 1 (pose goal) %s", success?"":"FAILED");
   //Sleep to give Rviz time to visualize the plan.
   //sleep(0.5);
   //Execute
   group.move();
   ROS_INFO("Motion planning finished!");

   if(success)
   {
     return 1;
   }
   else{
     return 0;
   }
}

bool CPlanning(string movegroup, string refFrame, vector<geometry_msgs::Pose> waypoints){
   moveit::planning_interface::MoveGroup group(movegroup);
   group.setPoseReferenceFrame(refFrame);

  moveit_msgs::RobotTrajectory trajectory_msg;

  //moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                             0.01,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory_msg,true);
  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");

  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);
  
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);

  // Finally plan and execute the trajectory
  moveit::planning_interface::MoveGroup::Plan plan;
  plan.trajectory_ = trajectory_msg;
  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);    
  // sleep(5.0);
  group.execute(plan);
  return true;
}

void executeCB(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& msg)
{
   //ROS_INFO("I heard execute result %d", int(msg->status_list.size()));
   ROS_ERROR("************RETRERER********");
   if (msg->result.error_code == 0)
   {
      executeResult = true;
   } else {
      executeResult = false;
   }
}

void executeCB_(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
   ROS_ERROR("************GOAL");
   executeResult = false;  // active
}










