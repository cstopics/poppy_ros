#include <iostream>
#include <memory>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "poppy_torso_control/ForwardKinematics.h"

std::map<std::string, robot_state::JointModelGroup*> groups;
std::vector<std::string> group_names;
robot_state::RobotStatePtr kinematic_state_global;

bool serviceForwardKinematicsCallback(poppy_torso_control::ForwardKinematics::Request  &req,
                                      poppy_torso_control::ForwardKinematics::Response &res)
{
    std::string group = req.group;
    std::vector<double> pos;
    ROS_INFO("/poppy_torso_kinematics Receiving forward kinematics request");

    bool valid_group = false;
    for (std::size_t i = 0; i < group_names.size(); ++i){
      if(group==group_names[i]){
        valid_group = true;
        break;
      }
    }    
    if(!valid_group){
      ROS_INFO("/poppy_torso_kinematics Invalid group: %s", group.c_str());      
      res.error = 1;
      return true;
    }
    ROS_INFO("/poppy_torso_kinematics Valid group: %s", group.c_str());      

    for (std::size_t i = 0; i < req.joints.size(); ++i){
        // ROS_INFO("num: %f", req.joints[i]);
        pos.push_back(req.joints[i]);
    }

    kinematic_state_global->setJointGroupPositions(groups[group], pos);
    std::string end_effector_name = groups[group]->getEndEffectorName();
    end_effector_name = end_effector_name.substr(0, end_effector_name.size()-9);
    const Eigen::Affine3d& end_effector_state = kinematic_state_global->getGlobalLinkTransform(end_effector_name);
    // ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    // ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    res.error = 0;
    res.end_pos = { (float)(end_effector_state.translation()[0]),
                    (float)(end_effector_state.translation()[1]),
                    (float)(end_effector_state.translation()[2]),};

    return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poppy_torso_kinematics");
  ros::NodeHandle n;
  

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state_global = kinematic_state;
  kinematic_state->setToDefaultValues();
  group_names = kinematic_model->getJointModelGroupNames();

  
  for (std::size_t i = 0; i < group_names.size(); ++i){
    groups.insert( std::pair<std::string, robot_state::JointModelGroup*>(
        group_names[i],
        kinematic_model->getJointModelGroup(group_names[i]) ) );
  }

  ros::ServiceServer serviceForwardKinematics = n.advertiseService("/poppy_forward_kinematics", serviceForwardKinematicsCallback);

  ROS_INFO("Ready!");

  ros::spin();

  ros::shutdown();
  return 0;

  // std::string group = "r_arm_4";
  // std::vector<double> pos { -1.5707961320877077, 
  //                           1.5707966089248657, 
  //                           -2.384185791015625e-07, 
  //                           1.5707961320877077};

  // const std::vector<std::string>& joint_names = groups[group]->getVariableNames();

  // kinematic_state_global->setJointGroupPositions(groups[group], pos);
  // const Eigen::Affine3d& end_effector_state = kinematic_state_global->getGlobalLinkTransform("r_hand");
  // ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  // ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  // for (std::size_t i = 0; i < joint_names.size(); ++i){
  //    ROS_INFO("joint %s", joint_names[i].c_str());
  // }



  //   std::size_t attempts = 10;
  // double timeout = 0.1;
  // bool found_ik = kinematic_state->setFromIK(groups[group], end_effector_state, attempts, timeout);

  // // Now, we can print out the IK solution (if found):
  // if (found_ik)
  // {
  //   std::vector<double> joint_values;
  //   kinematic_state->copyJointGroupPositions(groups[group], joint_values);
  //   for (std::size_t i = 0; i < joint_names.size(); ++i)
  //   {
  //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  //   }
  // }
  // else
  // {
  //   ROS_INFO("Did not find IK solution");
  // }


  // ros::spin();

  // ros::shutdown();
  // return 0;

}