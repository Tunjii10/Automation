/*********************************************************************
    Reference
    *    Title: pick_place_tutorial.cpp source code
    *    Author: Ros-Planning
    *    Date: 2021
    *    Code version: 1.0
    *    Availability:
    https://github.com/ros-planning/moveit_tutorials/blob/master/doc/pick_place/src/pick_place_tutorial.cpp
    *    Changes Made: addition of extra c++ libraries, position pose and
    *    orientation values changed,
    *    addition of collision objects using meshes, conversion of node
    *       to ROS service server node

*********************************************************************/

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit/robot_state/conversions.h>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit_msgs/GetPlanningScene.h>

#include "automation/movePose.h"
#include <shape_msgs/Mesh.h>
// #include <shape_msgs/MeshTriangle.h>
// #include <shape_msgs/Plane.h>
// #include <shape_msgs/SolidPrimitive.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory &posture) {
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory &posture) {
  /* Add both finger joints of panda robot. */

  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.03;
  posture.points[0].positions[1] = 0.03;
  posture.points[0].effort.resize(2);
  posture.points[0].effort[0] = 1.500;
  posture.points[0].effort[1] = 1.500;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface &move_group,
          geometry_msgs::PoseStamped &object_pose,
          std::vector<float> &service_request) {
  // Single vector of grasp is created
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  grasps[0].grasp_pose.header.frame_id = "world";
  // we use initial object pose to initiate grasp
  grasps[0].grasp_pose.pose.orientation.x = object_pose.pose.orientation.x;
  grasps[0].grasp_pose.pose.orientation.y = object_pose.pose.orientation.y;
  grasps[0].grasp_pose.pose.orientation.z = object_pose.pose.orientation.z;
  grasps[0].grasp_pose.pose.orientation.w = object_pose.pose.orientation.w;
  // service client position arguments are passed for grasp final pose
  grasps[0].grasp_pose.pose.position.x = service_request[0];
  grasps[0].grasp_pose.pose.position.y = service_request[1];
  grasps[0].grasp_pose.pose.position.z = service_request[2];

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
  grasps[0].pre_grasp_approach.direction.vector.z = -1;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.65;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.65;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);

  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);

  // Set support surfacefor pick operation
  move_group.setSupportSurfaceName("table2");

  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface &group) {
  // single vector of place location is created
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "world";
  // place orientation is defined using radians and converted to quaternion
  tf2::Quaternion orientation;
  orientation.setRPY(tau / 4, 0, tau / 2);

  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
  place_location[0].place_pose.pose.orientation.w = 0.000;
  /* For place location, we set the value to the exact location of the center of
   * the object. */
  place_location[0].place_pose.pose.position.x = 0.0700; // 0.07900;
  place_location[0].place_pose.pose.position.y = 0.46;   // 0.443612;
  place_location[0].place_pose.pose.position.z = 1.35;   // 1.2905;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++/
  place_location[0].pre_place_approach.direction.header.frame_id = "world";
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.55;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  place_location[0].post_place_retreat.direction.header.frame_id = "world";
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  openGripper(place_location[0].post_place_posture);

  // Set support surface a
  group.setSupportSurfaceName("table1");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface
                             &planning_scene_interface) {

  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(8);

  // Add the first table
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type =
      collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.5;
  collision_objects[0].primitives[0].dimensions[1] = 0.8;
  collision_objects[0].primitives[0].dimensions[2] = 1.03;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = -0.023537;
  collision_objects[0].primitive_poses[0].position.y = 0.672206;
  collision_objects[0].primitive_poses[0].position.z = 0.515;

  // Add the objectto the Collision object vector
  collision_objects[0].operation = collision_objects[0].ADD;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Add the second table
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type =
      collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 1.5;
  collision_objects[1].primitives[0].dimensions[1] = 0.8;
  collision_objects[1].primitives[0].dimensions[2] = 1.03;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -0.032827;
  collision_objects[1].primitive_poses[0].position.y = -0.64186;
  collision_objects[1].primitive_poses[0].position.z = 0.515;

  // Add the objectto the Collision object vector
  collision_objects[1].operation = collision_objects[1].ADD;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Add the target object, the first petri dish
  collision_objects[2].header.frame_id = "world";
  collision_objects[2].id = "object";

  //  Vector scale of the object and its path
  Eigen::Vector3d vectorScale(0.0007, 0.001, 0.001);
  shapes::Mesh *m = shapes::createMeshFromResource(
      "package://automation/src/meshes/petriDish.stl", vectorScale);

  // convert to mesh  message
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  // Define pose and orientation of the object
  geometry_msgs::Pose obj_pose;
  obj_pose.position.x = -0.136153;
  obj_pose.position.y = -0.575922;
  obj_pose.position.z = 1.0300;
  tf2::Quaternion orientation;
  orientation.setRPY(-1.57079633, -3.14159, 0.000);
  obj_pose.orientation = tf2::toMsg(orientation);

  // Add the mesh to the Collision object vector
  collision_objects[2].meshes.push_back(mesh);
  collision_objects[2].mesh_poses.push_back(obj_pose);
  collision_objects[2].operation = collision_objects[2].ADD;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Add the 2nd petri dish
  collision_objects[3].header.frame_id = "world";
  collision_objects[3].id = "petri_0";

  // Define mesh path
  m = shapes::createMeshFromResource(
      "package://automation/src/meshes/petriDish.stl", vectorScale);

  // convert to mesh  message
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  // Define pose and orientation of the object
  obj_pose.position.x = -0.141291;
  obj_pose.position.y = -0.430403;
  obj_pose.position.z = 1.0300;
  orientation.setRPY(-1.57079633, -3.14159, 0.000);
  obj_pose.orientation = tf2::toMsg(orientation);

  // Add the mesh to the Collision object vector
  collision_objects[3].meshes.push_back(mesh);
  collision_objects[3].mesh_poses.push_back(obj_pose);
  collision_objects[3].operation = collision_objects[3].ADD;
  //   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Add the 3rd petri dish
  collision_objects[4].header.frame_id = "world";
  collision_objects[4].id = "petri_1";

  //  Define mesh path
  m = shapes::createMeshFromResource(
      "package://automation/src/meshes/petriDish.stl", vectorScale);

  // convert to mesh  message
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  // Define pose and orientation of the object
  obj_pose.position.x = 0.139964;
  obj_pose.position.y = -0.552287;
  obj_pose.position.z = 1.0300;
  orientation.setRPY(-1.57079633, -3.14159, 0.000);
  obj_pose.orientation = tf2::toMsg(orientation);

  // Add the mesh to the Collision object vector
  collision_objects[4].meshes.push_back(mesh);
  collision_objects[4].mesh_poses.push_back(obj_pose);
  collision_objects[4].operation = collision_objects[4].ADD;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Add the 4th petri dish
  collision_objects[5].header.frame_id = "world";
  collision_objects[5].id = "petri_2";

  //  Define mesh path
  m = shapes::createMeshFromResource(
      "package://automation/src/meshes/petriDish.stl", vectorScale);

  // convert to mesh  message
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  // Define pose and orientation of the object
  obj_pose.position.x = -0.008752;
  obj_pose.position.y = -0.494031;
  obj_pose.position.z = 1.0300;
  orientation.setRPY(-1.57079633, -3.14159, 0.000);
  obj_pose.orientation = tf2::toMsg(orientation);

  // Add the mesh to the Collision object vector
  collision_objects[5].meshes.push_back(mesh);
  collision_objects[5].mesh_poses.push_back(obj_pose);
  collision_objects[5].operation = collision_objects[5].ADD;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Add the cardboard box
  collision_objects[6].header.frame_id = "world";
  collision_objects[6].id = "myCardboard";

  //  Vector scale of the object and its path
  Eigen::Vector3d vectorScale2(0.3, 0.37, 0.35);
  m = shapes::createMeshFromResource(
      "package://automation/src/meshes/myCardboard.stl", vectorScale2);

  // convert to mesh  message
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  // Define pose and orientation of the object
  obj_pose.position.x = -0.026411;
  obj_pose.position.y = -0.535304;
  obj_pose.position.z = 1.0300;
  orientation.setRPY(-1.57079633, -3.14159, 0.000);
  obj_pose.orientation = tf2::toMsg(orientation);

  // Add the mesh to the Collision object vector
  collision_objects[6].meshes.push_back(mesh);
  collision_objects[6].mesh_poses.push_back(obj_pose);
  collision_objects[6].operation = collision_objects[6].ADD;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Add the cardboard box
  collision_objects[7].header.frame_id = "world";
  collision_objects[7].id = "enclosure";

  //  Vector scale of the object and its path
  Eigen::Vector3d vectorScale3(0.001, 0.001, 0.001);
  m = shapes::createMeshFromResource(
      "package://automation/src/meshes/enclosure.stl", vectorScale3);

  // convert to mesh  message
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  // Define pose and orientation of the object
  obj_pose.position.x = 0.059913;
  obj_pose.position.y = 0.448577;
  obj_pose.position.z = 1.030000;
  orientation.setRPY(-1.57079633, -3.14159, 0.000);
  obj_pose.orientation = tf2::toMsg(orientation);

  // Add the mesh to the Collision object vector
  collision_objects[7].meshes.push_back(mesh);
  collision_objects[7].mesh_poses.push_back(obj_pose);
  collision_objects[7].operation = collision_objects[7].ADD;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

// service handle
bool move(automation::movePose::Request &req,
          automation::movePose::Response &res) {
  // instantiate moveit member classes moveit
  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  ros::WallDuration(1.0).sleep();

  // clear collision objects from scene
  std::vector<std::string> object_ids = {"object",  "enclosure", "myCardboard",
                                         "petri_0", "petri_1",   "petri_2",
                                         "table1",  "table2"};

  planning_scene_interface.removeCollisionObjects(object_ids);
  ros::WallDuration(1.0).sleep();

  // set goal tolerance and planning time
  group.setGoalTolerance(3.5);
  group.setPlanningTime(100.0);

  // add collision objects
  addCollisionObjects(planning_scene_interface);

  // get initial pose of robot
  geometry_msgs::PoseStamped object_pose;
  object_pose = group.getCurrentPose();

  ros::WallDuration(1.0).sleep();
  std::vector<float> service_request = {req.posX, req.posY, req.posZ};

  // call pick function
  pick(group, object_pose, service_request);
  ros::WallDuration(1.0).sleep();

  //   call place function
  place(group);

  //   group.setPoseTarget(object_pose);
  ros::WallDuration(1.0).sleep();
  return true;
}

int main(int argc, char **argv) {
  // initialize node and handler
  ros::init(argc, argv, "move_pose_server");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("move_pose", move);
  ROS_INFO("Ready to add take tool coordinates.");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::waitForShutdown();
  return 0;
}
