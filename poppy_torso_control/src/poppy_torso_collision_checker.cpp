#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <moveit/kinematic_constraints/utils.h>

#include "poppy_torso_control/CollisionDistance.h"

std::vector<std::string> no_col_links = {"base", "bust_motors", "chest", "head", "l_forearm", "l_hand", 
                                         "l_shoulder", "l_shoulder_motor", "l_upper_arm", "neck", "spine"};

std::vector<std::string> group_links = {"r_shoulder_y", "r_shoulder_x", "r_arm_z", "r_elbow_y"};

bool joint_states_received = false;
robot_state::RobotState *current_state;
sensor_msgs::JointState::ConstPtr joint_states;

collision_detection::AllowedCollisionMatrix acm;
planning_scene::PlanningScene *planning_scene_ptr;

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // ROS_INFO("I heard: [%s]", msg->name[0].c_str() );
  joint_states = msg;
  joint_states_received = true;
}

bool collisionDistanceCallback( poppy_torso_control::CollisionDistance::Request  &req,
                                poppy_torso_control::CollisionDistance::Response &res)
{
  // ROS_INFO("I heard: [%s]", joint_states->name[0].c_str() );
  current_state->setVariableValues(*joint_states);
  // current_state->printStatePositions(std::cout);
  //acm.print(std::cout); 

  if(req.offset){
    for(int i=0; i<group_links.size(); i++){
      std::string link = group_links[i];
      // double link_value = current_state->getVariablePosition(link) + req.offset_values[i];
      double link_value = req.offset_values[i];
      current_state->setVariablePosition(link, link_value);
    }
  }

  // current_state->printStatePositions(std::cout);

  
  std::cout << current_state->getVariablePosition("r_shoulder_y") << std::endl;

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.distance = true;
  
  planning_scene_ptr->checkSelfCollision(collision_request, collision_result, *current_state, acm);

  res.distance = collision_result.distance;
  res.collision = collision_result.collision;

  ROS_INFO_STREAM("/poppy_torso_collision_checker Distance " << res.distance);
  ROS_INFO_STREAM("/poppy_torso_collision_checker Test 1: Current state is " << (res.collision ? "in" : "not in") << " self collision");

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poppy_torso_collision_checker");
  ros::NodeHandle n;
  //ros::AsyncSpinner spinner(1);
  //spinner.start();
  std::size_t count = 0;

  ros::Subscriber sub = n.subscribe("/joint_states", 1, jointStatesCallback);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene_ptr = new planning_scene::PlanningScene(kinematic_model);

  current_state = new robot_state::RobotState(planning_scene_ptr->getCurrentState());
  // current_state = planning_scene_ptr->getCurrentState();

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  acm = planning_scene_monitor_ptr->getPlanningScene()->getAllowedCollisionMatrixNonConst();
  for(std::string link1: no_col_links)
    for(std::string link2: no_col_links)
      if(link1!=link2)
        acm.setEntry(link1, link2, true);
  // acm.print(std::cout); 

  ros::ServiceServer service = n.advertiseService("/poppy_collision_distance", collisionDistanceCallback);
  ROS_INFO("/poppy_collision_distance service ready");
  ros::spin();

  return 0;

}