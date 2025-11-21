/******************************************************************************
BSD-3-Clause
******************************************************************************/

#include <string>
#include <memory>

#include <ros/init.h>

#include <xpp_eqr2/inverse_kinematics_eqr2.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "eqr2_urdf_visualizer");

  const std::string joint_desired_topic = "xpp/joint_eqr2_des";

  auto eqr2_ik = std::make_shared<InverseKinematicsEqr2>();
  CartesianJointConverter inv_kin_converter(eqr2_ik,
					    xpp_msgs::robot_state_desired,
					    joint_desired_topic);

  // URDF joint names from eqr2 URDF
  int n_ee = eqr2_ik->GetEECount();
  int n_j  = 3;
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
  joint_names.at(n_j*LF + 0) = "fl_leg_1";
  joint_names.at(n_j*LF + 1) = "fl_leg_2";
  joint_names.at(n_j*LF + 2) = "fl_leg_3";
  joint_names.at(n_j*RF + 0) = "fr_leg_1";
  joint_names.at(n_j*RF + 1) = "fr_leg_2";
  joint_names.at(n_j*RF + 2) = "fr_leg_3";
  joint_names.at(n_j*LH + 0) = "hl_leg_1";
  joint_names.at(n_j*LH + 1) = "hl_leg_2";
  joint_names.at(n_j*LH + 2) = "hl_leg_3";
  joint_names.at(n_j*RH + 0) = "hr_leg_1";
  joint_names.at(n_j*RH + 1) = "hr_leg_2";
  joint_names.at(n_j*RH + 2) = "hr_leg_3";

  // Use standard parameter name robot_description
  std::string urdf = "eqr2_rviz_urdf_robot_description";
  UrdfVisualizer eqr2_desired(urdf, joint_names, "base_link", "world",
			      joint_desired_topic, "eqr2_des");

  ::ros::spin();
  return 0;
}


