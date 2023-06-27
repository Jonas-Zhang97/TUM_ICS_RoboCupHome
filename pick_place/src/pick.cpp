#include <pick_place/pick.h>

bool Pick::init()
{
  pick_target_topic_ =  "/obj_pose";
  pick_done_topic_ = "/pick_done";

  pick_target_sub_ = nh_.subscribe(pick_target_topic_, 1, &Pick::poseCallback, this);

  pick_done_pub_ = nh_.advertise<std_msgs::Bool>(pick_done_topic_, 1);

  command_ = false;
  pick_done_.data = true;

  arm_ready_value_ = {0.15, 0.7, 0, -1.57, 1.57, -1.57, 0.87, 0};

  ref_frame_ = "base_footprint";

  ROS_INFO_STREAM("End effector: " << arm_torso_group.getEndEffector());
}

void Pick::update()
{
  if (command_)
  {
    pick();
    pick_done_pub_.publish(pick_done_);
    command_ = false;
  }
}

void Pick::pick()
{
  prePickApproach();
  openGripper();
  toPickPose();
  closeGripper();
  postPickRetreat();
  toTransportPose();
}

void Pick::prePickApproach()
{
  arm_torso_group.setJointValueTarget(arm_ready_value_);
  arm_torso_group.plan(arm_plan_);
  arm_torso_group.move();

  arm_torso_group.setPoseTarget(pre_approach_pose_);
  bool succ = (arm_torso_group.plan(arm_plan_) == moveit_msgs::MoveItErrorCodes::SUCCESS);

  if (!succ)
  {
    ROS_INFO_STREAM("Planning failed");
  }

  arm_torso_group.move();
  ROS_INFO_STREAM("Pre-pick goal reached");
}

void Pick::openGripper()
{
  std::vector<double> open_value = {0.044, 0.044};
  
  gripper_group.setJointValueTarget(open_value);
  gripper_group.plan(gripper_plan_);
  gripper_group.move();
  ROS_INFO_STREAM("Gripper opened");
}

void Pick::toPickPose()
{
  pick_pose_ = pre_approach_pose_.pose;
  pick_pose_.position.x += 0.2;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pre_approach_pose_.pose);
  waypoints.push_back(pick_pose_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = arm_torso_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  arm_torso_group.execute(trajectory);
  ROS_INFO_STREAM("Ready to pick");
}

void Pick::closeGripper()
{
  std::vector<double> close_value = {0.001, 0.001};
  
  gripper_group.setJointValueTarget(close_value);
  gripper_group.plan(gripper_plan_);
  gripper_group.move();
  ROS_INFO_STREAM("Gripper closed");
}

void Pick::postPickRetreat()
{
  retreat_pose_ = pick_pose_;
  retreat_pose_.position.z += 0.2;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pick_pose_);
  waypoints.push_back(retreat_pose_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = arm_torso_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  arm_torso_group.execute(trajectory);

  ROS_INFO_STREAM("Retreated");
}

void Pick::toTransportPose()
{
  arm_torso_group.setJointValueTarget(arm_ready_value_);
  arm_torso_group.plan(arm_plan_);
  arm_torso_group.move();

  ROS_INFO_STREAM("Ready to go");
}

void Pick::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  command_ = true;

  target_position_ << msg->pose.position.x,
               msg->pose.position.y,
               msg->pose.position.z;

  pre_approach_pose_.header.frame_id = ref_frame_;
  pre_approach_pose_.pose.position = msg->pose.position;

  pre_approach_pose_.pose.position.x -= 0.25;

  tf2::Quaternion quaternion;
  quaternion.setRPY(1.57, 0.0, 0.0);

  pre_approach_pose_.pose.orientation = tf2::toMsg(quaternion);
  
  ROS_INFO_STREAM("Command received, position: " << target_position_.transpose());
}