#include <pick_place/place.h>

bool Place::init()
{
  place_target_topic_ =  "/obj_pose";
  place_done_topic_ = "/place_done";

  place_target_sub_ = nh_.subscribe(place_target_topic_, 1, &Place::poseCallback, this);

  place_done_pub_ = nh_.advertise<std_msgs::Bool>(place_done_topic_, 1);

  command_ = false;
  place_done_.data = true;

  arm_home_value_ = {0.15, 0.19, -1.34, -0.19, 1.94, -1.57, 1.36, 0};

  ref_frame_ = "base_footprint";

  ROS_INFO_STREAM("End effector: " << arm_torso_group.getEndEffector());
}

void Place::update()
{
  if (command_)
  {
    place();
    place_done_pub_.publish(place_done_);
    command_ = false;
  }
}

void Place::place()
{
  prePlaceApproach();
  toPlacePose();
  openGripper();
  postPlaceRetreat();
  closeGripper();
  homing();
}

void Place::prePlaceApproach()
{
  arm_torso_group.setPoseTarget(pre_approach_pose_);
  bool succ = (arm_torso_group.plan(arm_plan_) == moveit_msgs::MoveItErrorCodes::SUCCESS);

  if (!succ)
  {
    ROS_INFO_STREAM("Planning failed");
  }

  arm_torso_group.move();

  geometry_msgs::Pose pre_approach_pose_1;
  pre_approach_pose_1 = pre_approach_pose_.pose;
  
  pre_approach_pose_2_ = pre_approach_pose_1;
  pre_approach_pose_2_.position.x += 0.25;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pre_approach_pose_1);
  waypoints.push_back(pre_approach_pose_2_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = arm_torso_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  arm_torso_group.execute(trajectory);


  ROS_INFO_STREAM("Pre-place goal reached");
}

void Place::toPlacePose()
{
  place_pose_ = pre_approach_pose_2_;
  place_pose_.position.z -= 0.2;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pre_approach_pose_2_);
  waypoints.push_back(place_pose_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = arm_torso_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  arm_torso_group.execute(trajectory);
}

void Place::openGripper()
{
  std::vector<double> open_value = {0.044, 0.044};
  
  gripper_group.setJointValueTarget(open_value);
  gripper_group.plan(gripper_plan_);
  gripper_group.move();
  ROS_INFO_STREAM("Gripper opened");
}

void Place::postPlaceRetreat()
{
  retreat_pose_ = place_pose_;
  retreat_pose_.position.z += 0.2;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(place_pose_);
  waypoints.push_back(retreat_pose_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = arm_torso_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  arm_torso_group.execute(trajectory);

  geometry_msgs::Pose retreat_pose_2;
  retreat_pose_2 = retreat_pose_;
  retreat_pose_2.position.x -= 0.25;

  std::vector<geometry_msgs::Pose> waypoints2;
  waypoints2.push_back(retreat_pose_);
  waypoints2.push_back(retreat_pose_2);

  moveit_msgs::RobotTrajectory trajectory2;

  double fraction2 = arm_torso_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);

  arm_torso_group.execute(trajectory2);
}

void Place::closeGripper()
{
  std::vector<double> close_value = {0.001, 0.001};
  
  gripper_group.setJointValueTarget(close_value);
  gripper_group.plan(gripper_plan_);
  gripper_group.move();
  ROS_INFO_STREAM("Gripper closed");
}

void Place::homing()
{
  arm_torso_group.setJointValueTarget(arm_home_value_);
  arm_torso_group.plan(arm_plan_);
  arm_torso_group.move();

  ROS_INFO_STREAM("Ready to go");
}

void Place::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  command_ = true;

  target_position_ << msg->pose.position.x,
               msg->pose.position.y,
               msg->pose.position.z;

  pre_approach_pose_.header.frame_id = ref_frame_;
  pre_approach_pose_.pose.position = msg->pose.position;

  pre_approach_pose_.pose.position.x -= 0.25;
  pre_approach_pose_.pose.position.z += 0.2;

  tf2::Quaternion quaternion;
  quaternion.setRPY(1.57, 0.0, 0.0);

  pre_approach_pose_.pose.orientation = tf2::toMsg(quaternion);
  
  ROS_INFO_STREAM("Command received, position: " << target_position_.transpose());
}