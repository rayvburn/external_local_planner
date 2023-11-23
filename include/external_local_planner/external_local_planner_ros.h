//
// Author: Jarosław Karwowski,
// Robot Programming and Machine Perception Group,
// Warsaw University of Technology
// 2023
//
// Based on DWAPlannerROS implementation by Eitan Marder-Eppstein
// Ref: https://github.com/ros-planning/navigation
//
#pragma once

#include <angles/angles.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>

#include <mutex>

namespace external_local_planner {
  /**
   * @class ExternalLocalPlannerROS
   * @brief ROS Wrapper for an external planner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class ExternalLocalPlannerROS : public nav_core::BaseLocalPlanner {
    /// How many times externally received twist command can be applied ('timeout')
    static constexpr int EXT_CMD_MAX_USAGE_COUNT = 2;

    enum Mode {
      EXTERNAL_PLANNER = 0,
      STOP_AND_ROTATE = 1
    };

    public:
      /**
       * @brief  Constructor for ExternalLocalPlannerROS wrapper
       */
      ExternalLocalPlannerROS();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~ExternalLocalPlannerROS() = default;

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

      /// Stop and rotate controller uses this to validate commands
      bool checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples);

      bool isInitialized() {
        return initialized_;
      }

    private:
      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void externalTwistCb(const geometry_msgs::TwistConstPtr& msg);

      tf2_ros::Buffer* tf_; ///< @brief Used for transforming point clouds

      // publisher of global plan for visualisation
      ros::Publisher g_plan_pub_;
      // publisher of local goal
      ros::Publisher l_goal_pub_;

      // interface with an external local planner
      ros::Subscriber planner_sub_;

      base_local_planner::LocalPlannerUtil planner_util_;

      costmap_2d::Costmap2DROS* costmap_ros_;

      bool setup_;
      geometry_msgs::PoseStamped current_pose_;

      bool enable_stop_rotate_controller_;
      base_local_planner::LatchedStopRotateController latched_stop_rotate_controller_;

      bool initialized_;
      /// The period at which the local planner is expected to run
      double sim_period_;
      /// Which mode was previously used during velocity command computation
      enum Mode prev_mode_;

      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;

      // Velocity command received from an external module
      geometry_msgs::Twist cmd_vel_ext_;
      std::mutex ext_twist_mutex_;
      // Counter of how many times the externally received command was already applied ("count-out" instead of timeout)
      int cmd_vel_apply_cnt_;
  };
} // namespace external_local_planner
