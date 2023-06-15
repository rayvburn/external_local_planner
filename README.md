# external_local_planner

An implementation of the ROS Navigation Stack local planner plugin that passes externally computed velocity commands to the move_base framework.
This can be used as an interface for methods implemented not as the ROS Navigation Stack plugin, e.g., written as separate nodes in Python.

By default, looks for external Twist commands at the `/move_base/ExternalLocalPlannerROS/external_cmd_vel` topic.
