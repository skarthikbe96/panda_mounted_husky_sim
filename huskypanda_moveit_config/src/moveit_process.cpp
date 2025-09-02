#include <memory>
#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true); // get robot_description passed from launch
  auto node = rclcpp::Node::make_shared("robot_model_and_state_tutorial", opts);
  const auto LOGGER = node->get_logger();

  robot_model_loader::RobotModelLoader loader(node);
  auto kinematic_model = loader.getModel();
  if (!kinematic_model) {                                    // ★
    RCLCPP_ERROR(LOGGER, "Failed to load RobotModel (check that robot_description is set for this node).");
    return 1;
  }
  RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

  auto robot_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
  robot_state->setToDefaultValues();

  const std::string group_name = "panda_arm";
  const moveit::core::JointModelGroup* jmg = kinematic_model->getJointModelGroup(group_name);
  if (!jmg) {                                                // ★
    RCLCPP_ERROR(LOGGER, "JointModelGroup '%s' not found. Check your SRDF group names.", group_name.c_str());
    return 1;
  }

  // current joint values
  std::vector<std::string> joint_names = jmg->getVariableNames();
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(jmg, joint_values);
  for (size_t i = 0; i < joint_names.size(); ++i)
    RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);

  // bounds check / enforce
  joint_values[0] = 5.57;                         // deliberately out of bounds
  robot_state->setJointGroupPositions(jmg, joint_values);
  RCLCPP_INFO(LOGGER, "State is %s", robot_state->satisfiesBounds() ? "valid" : "NOT valid");
  robot_state->enforceBounds();
  RCLCPP_INFO(LOGGER, "After enforceBounds(): %s", robot_state->satisfiesBounds() ? "valid" : "NOT valid");

  // Forward kinematics on the group's tip (robust to naming)                // ★
  const std::string tip_link = jmg->getLinkModelNames().back();
  const Eigen::Isometry3d& ee_pose = robot_state->getGlobalLinkTransform(tip_link);
  RCLCPP_INFO_STREAM(LOGGER, "Tip link: " << tip_link);
  RCLCPP_INFO_STREAM(LOGGER, "Translation:\n" << ee_pose.translation());
  RCLCPP_INFO_STREAM(LOGGER, "Rotation:\n" << ee_pose.rotation());

  // Inverse kinematics (specify the same tip to be explicit)                // ★
  const double timeout = 0.1;
  bool ok = robot_state->setFromIK(jmg, ee_pose, tip_link, timeout);
  if (!ok) {
    RCLCPP_WARN(LOGGER, "Did not find IK solution");
  } else {
    robot_state->copyJointGroupPositions(jmg, joint_values);
    for (size_t i = 0; i < joint_names.size(); ++i)
      RCLCPP_INFO(LOGGER, "IK %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Jacobian at the tip
  Eigen::Vector3d ref(0,0,0);
  Eigen::MatrixXd J;
  robot_state->getJacobian(jmg, kinematic_model->getLinkModel(tip_link), ref, J);
  RCLCPP_INFO_STREAM(LOGGER, "Jacobian:\n" << J);

  rclcpp::shutdown();
  return 0;
}
