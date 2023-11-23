//
// Author: Jarosław Karwowski,
// Robot Programming and Machine Perception Group,
// Warsaw University of Technology
// 2023
//
// Based on DWAPlannerROS implementation by Eitan Marder-Eppstein
// Ref: https://github.com/ros-planning/navigation
//
#include <external_local_planner/external_local_planner_ros.h>

#include <Eigen/Core>
#include <cmath>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <tf2/utils.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(external_local_planner::ExternalLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace external_local_planner {

  ExternalLocalPlannerROS::ExternalLocalPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false), cmd_vel_apply_cnt_(0) {
  }

  void ExternalLocalPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros
  ) {
    if (isInitialized()) {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }

    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("local_goal", 1);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ros_->getRobotPose(current_pose_);

    // create an interface for the external planner
    planner_sub_ = private_nh.subscribe<geometry_msgs::Twist>(
      "external_cmd_vel",
      5,
      &ExternalLocalPlannerROS::externalTwistCb,
      this
    );

    // make sure to update the costmap we'll use for this cycle
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

    if( private_nh.getParam( "odom_topic", odom_topic_ )) {
      odom_helper_.setOdomTopic( odom_topic_ );
    }

    base_local_planner::LocalPlannerLimits limits;
    private_nh.param<double>("xy_goal_tolerance", limits.xy_goal_tolerance, 0.2);
    private_nh.param<double>("yaw_goal_tolerance", limits.yaw_goal_tolerance, 0.2);
    private_nh.param<double>("trans_stopped_vel", limits.trans_stopped_vel, 0.1);
    private_nh.param<double>("theta_stopped_vel", limits.theta_stopped_vel, 0.1);
    planner_util_.reconfigureCB(limits, false);

    initialized_ = true;
  }
  
  bool ExternalLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    // when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latched_stop_rotate_controller_.resetLatching();

    ROS_INFO("Got new plan");
    planner_util_.setPlan(orig_global_plan);
  }

  bool ExternalLocalPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(latched_stop_rotate_controller_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  void ExternalLocalPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  void ExternalLocalPlannerROS::externalTwistCb(const geometry_msgs::TwistConstPtr& msg) {
    std::lock_guard lock(ext_twist_mutex_);
    cmd_vel_ext_ = *msg;
    cmd_vel_apply_cnt_ = 0;
  }

  bool ExternalLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      ROS_WARN_NAMED("external_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("external_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());
    // publish local goal
    l_goal_pub_.publish(transformed_plan.back());

    std::lock_guard<std::mutex> lock(ext_twist_mutex_);
    // evaluate if the external command is valid or outdated
    bool cmd_valid = cmd_vel_apply_cnt_ <= EXT_CMD_MAX_USAGE_COUNT;

    // increment the counter
    cmd_vel_apply_cnt_++;

    if (cmd_valid) {
      publishGlobalPlan(transformed_plan);
      // send the newest command to the mobile base
      cmd_vel = cmd_vel_ext_;
      ROS_DEBUG_NAMED(
        "external_local_planner",
        "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z
      );
    } else {
      std::vector<geometry_msgs::PoseStamped> empty_plan;
      publishGlobalPlan(empty_plan);
      // send stop command
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      ROS_WARN_NAMED(
        "external_local_planner",
        "External planner failed to produce trajectory in time (%dx). Sending stop command",
        cmd_vel_apply_cnt_
      );
    }
  }

} // namespace external_local_planner
