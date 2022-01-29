#ifndef ND_LOCAL_PLANNER_H
#define ND_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

#include <chrono>
#include <vector>

#include <angles/angles.h>

namespace nd_local_planner{

class NDLocalPlanner: public nav_core::BaseLocalPlanner{
private:
	costmap_2d::Costmap2DROS* costmap_ros_;
	costmap_2d::Costmap2D* costmap_;
	tf2_ros::Buffer* tf_;	// for tf transformations

	std::vector<geometry_msgs::PoseStamped> global_plan_;

	bool initialized_;

	/// ND Algorithm parameters
	// Robot
	double robot_radius_;
	double goal_factor_;
	double distance_robot_bounds_;
	double v_max_;
	double w_max_;

	// Sensor
	double d_max_;

	// Others
	double security_distance_;
	double security_nearness_;

	int ls2_diff_;

	int num_sectors_;
	int wide_valley_;

	long unsigned int exec_time = 0;
	long unsigned int n_times = 0;

public:

	NDLocalPlanner() : costmap_(NULL), initialized_(false){};
	NDLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

	// override functions from interface nav_core::BaseLocalPlanner
	void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
	bool isGoalReached();
};

};

#endif
