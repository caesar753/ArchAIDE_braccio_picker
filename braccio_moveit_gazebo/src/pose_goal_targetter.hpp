#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_goal_targetter");
  ros::NodeHandle nh;

  moveit::planning_interface::MoveGroupInterface move_group("braccio_arm");
  moveit::planning_interface::MoveGroupInterface gripper_group("braccio_gripper");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  move_group.setPlannerId("RRTConnectkConfigDefault");

  geometry_msgs::Pose pose_goal;
  pose_goal.position.x = 0.0;
  pose_goal.position.y = 0.0;
  pose_goal.position.z = 0.0;
  pose_goal.orientation.w = 1.0;

  move_group.setPoseTarget(pose_goal);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    move_group.execute(plan);
  }

  return 0;
}
