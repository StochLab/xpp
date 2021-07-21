#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

#include <xpp_stochlite/stochlite_inverse_kinematics.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "stochlite_urdf_visualizer");

  const std::string joint_desired_stochlite = "xpp/joint_stochlite_des";

  auto stochlite_ik = std::make_shared<StochliteInverseKinematics>();
  CartesianJointConverter inv_kin_converter(stochlite_ik,
					    xpp_msgs::robot_state_desired,
					    joint_desired_stochlite);

  // urdf joint names
  int n_ee = 4;
  int n_j  = 3;
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);

  joint_names.at( n_j*0 + 0 ) = "fl_abd_joint";
  joint_names.at( n_j*0 + 1 ) = "fl_hip_joint";
  joint_names.at( n_j*0 + 2 ) = "fl_knee_joint";
  joint_names.at( n_j*1 + 0 ) = "fr_abd_joint";
  joint_names.at( n_j*1 + 1 ) = "fr_hip_joint";
  joint_names.at( n_j*1 + 2 ) = "fr_knee_joint";
  joint_names.at( n_j*2 + 0 ) = "bl_abd_joint";
  joint_names.at( n_j*2 + 1 ) = "bl_hip_joint";
  joint_names.at( n_j*2 + 2 ) = "bl_knee_joint";
  joint_names.at( n_j*3 + 0 ) = "br_abd_joint";
  joint_names.at( n_j*3 + 1 ) = "br_hip_joint";
  joint_names.at( n_j*3 + 2 ) = "br_knee_joint";

  std::string urdf = "stochlite_rviz_urdf_robot_description";
  UrdfVisualizer stochlite_desired(urdf, joint_names, "dummy", "world",
			     joint_desired_stochlite, "stochlite_des");

  ::ros::spin();

  return 1;
}