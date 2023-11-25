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

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <tf2/utils.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(external_local_planner::ExternalLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace external_local_planner {

  ExternalLocalPlannerROS::ExternalLocalPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false), cmd_vel_apply_cnt_(0), prev_mode_(EXTERNAL_PLANNER) {
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

    private_nh.param<double>("local_goal_distance", local_goal_distance_, -1.0);
    private_nh.param<bool>("enable_stop_rotate_controller", enable_stop_rotate_controller_, false);

    base_local_planner::LocalPlannerLimits limits;
    // not required by the stop&rotate controller
    private_nh.param<double>("max_vel_trans", limits.max_vel_trans, 0.5);
    private_nh.param<double>("min_vel_trans", limits.min_vel_trans, 0.1);
    private_nh.param<double>("max_vel_x", limits.max_vel_x, 0.5);
    private_nh.param<double>("min_vel_x", limits.min_vel_x, -0.1);
    private_nh.param<double>("max_vel_y", limits.max_vel_y, 0.0);
    private_nh.param<double>("min_vel_y", limits.min_vel_y, 0.0);
    // used by the stop&rotate controller (+ latch_xy_goal_tolerance)
    private_nh.param<double>("max_vel_theta", limits.max_vel_theta, 1.05);
    private_nh.param<double>("min_vel_theta", limits.min_vel_theta, 1.05);
    private_nh.param<double>("acc_lim_x", limits.acc_lim_x, 1.0);
    private_nh.param<double>("acc_lim_y", limits.acc_lim_y, 0.0);
    private_nh.param<double>("acc_lim_theta", limits.acc_lim_theta, 1.05);
    private_nh.param<double>("acc_lim_trans", limits.acc_lim_trans, limits.acc_lim_x);
    private_nh.param<double>("xy_goal_tolerance", limits.xy_goal_tolerance, 0.2);
    private_nh.param<double>("yaw_goal_tolerance", limits.yaw_goal_tolerance, 0.2);
    private_nh.param<double>("trans_stopped_vel", limits.trans_stopped_vel, 0.1);
    private_nh.param<double>("theta_stopped_vel", limits.theta_stopped_vel, 0.1);
    planner_util_.reconfigureCB(limits, false);

    std::string controller_frequency_param;
    private_nh.searchParam("controller_frequency", controller_frequency_param);
    double controller_frequency = 10.0; // default
    if (private_nh.param(controller_frequency_param, controller_frequency, controller_frequency)) {
      sim_period_ = 1.0 / controller_frequency;
      ROS_INFO(
        "Sim period set to %6.3f s. Computed based on `controller_frequency` which is %6.3f Hz",
        sim_period_,
        controller_frequency
      );
    }
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
      prev_mode_ = EXTERNAL_PLANNER;
      return true;
    } else {
      return false;
    }
  }

  bool ExternalLocalPlannerROS::checkTrajectory(
    Eigen::Vector3f pos,
    Eigen::Vector3f /*vel*/,
    Eigen::Vector3f vel_samples
  ) {
    base_local_planner::CostmapModel world_model(*costmap_ros_->getCostmap());
    auto footprint_spec = costmap_ros_->getRobotFootprint();

    // current pose
    double cost_current = world_model.footprintCost(pos[0], pos[1], pos[2], footprint_spec);

    // predicted pose
    // NOTE: calculations based on base_local_planner::SimpleTrajectoryGenerator::computeNewPositions
    double new_x = pos[0] + (vel_samples[0] * std::cos(pos[2]) + vel_samples[1] * std::cos(M_PI_2 + pos[2])) * sim_period_;
    double new_y = pos[1] + (vel_samples[0] * std::sin(pos[2]) + vel_samples[1] * std::sin(M_PI_2 + pos[2])) * sim_period_;
    double new_yaw = pos[2] + vel_samples[2] * sim_period_;
    double cost_next = world_model.footprintCost(new_x, new_y, new_yaw, footprint_spec);

    if (cost_current < 0 || cost_next < 0) {
      return false;
    }
    double cost = std::max(cost_current, cost_next);
    if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }

    return true;
  }

  void ExternalLocalPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  void ExternalLocalPlannerROS::externalTwistCb(const geometry_msgs::TwistConstPtr& msg) {
    std::lock_guard lock(ext_twist_mutex_);
    cmd_vel_ext_ = *msg;
    cmd_vel_apply_cnt_ = 0;
    ROS_INFO_ONCE_NAMED("external_local_planner", "Received the first external twist command!");
  }

  bool ExternalLocalPlannerROS::findLocalGoal(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    geometry_msgs::PoseStamped& local_goal
  ) {
    if (plan.empty()) {
      ROS_ERROR_NAMED("external_local_planner", "Plan is empty! Selecting the current pose as a local goal");
      local_goal = current_pose_;
      return false;
    }

    if (plan.size() == 1) {
      ROS_INFO_NAMED("external_local_planner", "Plan has only 1 pose, using it as a local goal");
      local_goal = plan.back();
      return true;
    }

    // selecting a closer goal pose disabled
    if (local_goal_distance_ < 0.0) {
      local_goal = plan.back();
      return true;
    }

    // helper function that computes squared distance between poses given by {x1, y1} and {x2, y2}
    auto computeSqDist = [](double x1, double y1, double x2, double y2) {
      double dist_x = x2 - x1;
      double dist_y = y2 - y1;
      double dist_sq = dist_x * dist_x + dist_y * dist_y;
      return dist_sq;
    };

    // computes Euclidean distance between 2 points
    auto computeDist = [computeSqDist](double x1, double y1, double x2, double y2) {
      return std::sqrt(computeSqDist(x1, y1, x2, y2));
    };

    double dist_sum_along_path = 0.0;
    for (
      // NOTE: starting from the [1] instead of [0]
      std::vector<geometry_msgs::PoseStamped>::const_iterator it = std::next(plan.begin());
      it != plan.end();
      ++it
    ) {
      // the iterator is always valid as we start from the second element
      auto it_prev = std::prev(it);

      // accumulate distances
      double dist = computeDist(
        it_prev->pose.position.x,
        it_prev->pose.position.y,
        it->pose.position.x,
        it->pose.position.y
      );
      dist_sum_along_path += dist;
      if (dist_sum_along_path < local_goal_distance_) {
        // not far enough
        continue;
      }
      local_goal = *it;
      return true;
    }
    // plan is too short - choosing the last pose
    local_goal = plan.back();
    return true;
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
    // select and publish local goal
    geometry_msgs::PoseStamped goal_local;
    if (!findLocalGoal(transformed_plan, goal_local)) {
      ROS_ERROR("Could not select a local goal");
      return false;
    }
    l_goal_pub_.publish(goal_local);

    /*
     * There can be 2 modes of operation:
     * - following the external planner until the goal pose is reached (@ref enable_stop_rotate_controller_ is false)
     * - following the external planner until the goal position is reached and then using latched stop&rotate method
     */
    if (
      enable_stop_rotate_controller_
      && latched_stop_rotate_controller_.isPositionReached(&planner_util_, current_pose_)
    ) {
      if (prev_mode_ == EXTERNAL_PLANNER) {
        ROS_INFO_NAMED(
          "external_local_planner",
          "Goal position has been reached. Starting the rotation to the goal orientation."
        );
      }
      prev_mode_ = STOP_AND_ROTATE;
      return latched_stop_rotate_controller_.computeVelocityCommandsStopRotate(
        cmd_vel,
        planner_util_.getCurrentLimits().getAccLimits(),
        sim_period_,
        &planner_util_,
        odom_helper_,
        current_pose_,
        std::bind(
          &ExternalLocalPlannerROS::checkTrajectory,
          this,
          std::placeholders::_1,
          std::placeholders::_2,
          std::placeholders::_3
        )
      );
    }

    if (prev_mode_ == STOP_AND_ROTATE) {
      ROS_INFO_NAMED(
        "external_local_planner",
        "Switched back from stop and rotate control to the commands from an external local planner."
      );
    }
    prev_mode_ = EXTERNAL_PLANNER;

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
