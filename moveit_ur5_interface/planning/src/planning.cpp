#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <std_msgs/String.h>
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include <control_msgs/GripperCommandActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//collision abject geometric
#include <shape_tools/solid_primitive_dims.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"


static const std::string ROBOT_DESCRIPTION="robot_description";

bool mPlanning(std::string movegroup, std::string refFrame, double target[]){
   
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


bool CPlanning(std::string movegroup, std::string refFrame, std::vector<geometry_msgs::Pose> waypoints){
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

int main(int argc, char **argv)
{
   ROS_INFO("Start!");
   ros::init (argc, argv, "planning");
   ros::AsyncSpinner spinner(1);
   spinner.start();

   ros::NodeHandle nh;
   ros::Rate loop_rate(10);

   std::vector<double> right_m_joint; //Initialize

   right_m_joint.resize(6);
   right_m_joint[0] = -1.62;   //should_pan_joint    
   right_m_joint[1] = -1.76;   //should_lift_joint
   right_m_joint[2] = -1.84;   //elbow_joint
   right_m_joint[3] = 0.49;   //wrist_1_joint 
   right_m_joint[4] = 1.32;   //wrist_2_joint
   right_m_joint[5] = -0.05;   //wrist_3_joint  


   std::string movegroup = "manipulator";
   double target[7];

   bool result = false;

   //min place
   
   moveit::planning_interface::MoveGroup Jointgroup("manipulator");
   Jointgroup.setPoseReferenceFrame("base_link");
   Jointgroup.setPlannerId("RRTConnectkConfigDefault");

   Jointgroup.setJointValueTarget(right_m_joint);
   Jointgroup.move();
   ros::WallDuration(5.0).sleep();
   //Jointgroup.move();

   std::string refFrame = "base_link";

   std::vector<geometry_msgs::Pose> waypoints;
   waypoints.clear();
   geometry_msgs::Pose target_pose;
   target_pose.position.x = -0.28645;
   target_pose.position.y = 0.47591;
   target_pose.position.z = 0.59776;
   target_pose.orientation.x = -0.0094907;
   target_pose.orientation.y = -0.026997;
   target_pose.orientation.z = 0.66772;
   target_pose.orientation.w = 0.74387;
   waypoints.push_back(target_pose);

   target_pose.position.y += 0.1;
   waypoints.push_back(target_pose);

   target_pose.position.y -= 0.1;
   waypoints.push_back(target_pose);
   do{
      result = CPlanning(movegroup,refFrame,waypoints);
      ROS_INFO("The result of motion planning is %ld", (long int)result);
   } while(!result);
   ros::WallDuration(5.0).sleep();

   Jointgroup.move();
   

   ROS_INFO("Done!");
   return 0;
}

















