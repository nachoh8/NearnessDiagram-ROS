#include <math.h>

#include <pluginlib/class_list_macros.h>
#include <utility>
#include <string>
#include <chrono>

#include "ND/nd_local_planner.h"



//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(nd_local_planner::NDLocalPlanner, nav_core::BaseLocalPlanner)


using namespace std;

namespace nd_local_planner{
	struct Valley {
		int s_rd; // sector corresponding to the rising discontinuity closest to goal
		int s_od; // other discontinuity
		int s_l;
		int s_r;
		int distance_to_goal; // distance from goal to s_rd in sectors
	};

	bool sort_valley(const Valley& lv, const Valley& rv) {
		return lv.distance_to_goal < rv.distance_to_goal;
	}

	/// ND Algotithm Functions
	inline double euclideanDistance(const geometry_msgs::Pose pose1, const geometry_msgs::Pose pose2){
		double ex = pose2.position.x - pose1.position.x;
		double ey = pose2.position.y - pose1.position.y;

		return std::sqrt(ex*ex+ey*ey);
	}

	inline double diffAngles(const double th1, const double th2) {
		double th_diff = th1 - th2;
		if (th_diff > M_PI) {
			return th_diff - 2 * M_PI;
		} else if (th_diff < -M_PI) {
			return th_diff + 2 * M_PI;
		} else {
			return th_diff;
		}
	}

	inline int diffSectors(const int s1, const int s2, const int num_sectors) {
		int df = abs(s1 - s2);
		return min(df, num_sectors - df);
	}

	inline double clampAngle(const double angle) {
		if (angle > M_PI_2) {
			return M_PI_2;
		} else if (angle < -M_PI_2) {
			return -M_PI_2;
		} else {
			return angle;
		}
	}

	bool isValleyNavigable(const geometry_msgs::Pose& robot_pose, const double robot_radius,
							const geometry_msgs::Pose& goal, const std::vector<geometry_msgs::Pose>& L,
							const std::vector<double>& Ld, const std::vector<double>& Lth) {
		const double robot_diameter = 2 * robot_radius;

		double th_goal = atan2(goal.position.y, goal.position.x);

		double Px = goal.position.x - robot_pose.position.x;
		double Py = goal.position.y - robot_pose.position.y;
		double d_gr = std::sqrt(Px*Px + Py*Py);

		std::vector<int> FL, FR; // store L idx
		for (size_t i = 0; i < L.size(); i++) {
			/// 1) d(goal, Li) < R -> goal cannot be reached
			double d_og = euclideanDistance(L[i], goal);
			if (d_og < robot_radius) {
				ROS_INFO("ND - isValleyNavigable: 1) Goal can't be reached");
				return false;
			}

			/// 2) Eliminate from L if:

			// b) d(Li, robot) > d(goal, robot)
			double d_or = Ld[i];
			if (d_or > d_gr) {
				continue;
			}

			// a) Li belongs to BL or BR
			double th_diff = diffAngles(Lth[i], th_goal);
			if (abs(th_diff) > M_PI_2) {
				continue;
			}

			// c) d(Li, P) > 2R
			// double d_oP = pointlineDistance(robot_pose, L[i], P);
			double Lx = L[i].position.x - robot_pose.position.x;
			double Ly = L[i].position.y - robot_pose.position.y;
			double d_oP = abs(Px*Ly - Py*Lx) / d_gr;
			if (d_oP > robot_diameter) {
				continue;
			}

			/// else: Add Li to FL or FR
			if (th_diff >= 0) {
				FR.push_back(i);
			} else {
				FL.push_back(i);
			}
    	}

		/// 3) d(Li_FR, Lj_FL) > 2R;
		if (FR.size() > 0 && FL.size() > 0) {
			for (size_t i = 0; i < FR.size(); i++) {
				geometry_msgs::Pose obs_fr = L[FR[i]];
				for (size_t j = 0; j < FL.size(); j++) {
					geometry_msgs::Pose obs_fl = L[FL[j]];
					double d = euclideanDistance(obs_fl, obs_fr);
					if (d <= robot_diameter) {
						std::cout << "ND - isValleyNavigable: Goal cant be reached in 3)\n"
								<< "\tobs_fr: " << obs_fr.position.x << ", " << obs_fr.position.y
								<< "\tobs_fl: " << obs_fl.position.x << ", " << obs_fl.position.y
								<< "\tdist: " << d << ", robot_diameter" << robot_diameter
								<< std::endl;
						return false;
					}
				}
			}
		}

		return true;
	}

	inline double bisectorAngle(const int sector, const int num_sectors) {
		return M_PI - (2.0f * M_PI * (double)(sector+1)) / (double)num_sectors;
	}
	
	inline int angleToSector(const double angle, const int num_sectors) {
		double aux =  (M_PI - angle)  * (num_sectors / 2.0f) / M_PI;
		return ceil(aux) - 1;
	}

	NDLocalPlanner::NDLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
		initialize(name, tf, costmap_ros);
	}

	void NDLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){

		if (!initialized_) {

			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros_->getCostmap();
			tf_ = tf;

			ros::NodeHandle nh("~/" + name);
			ros::NodeHandle nh_local("~/local_costmap/");

			// nh_local.getParam("robot_radius", robot_radius_);
			nh.getParam("robot_radius_nd", robot_radius_);
			nh.getParam("distance_goal_factor", goal_factor_);
			nh.getParam("distance_robot_bounds", distance_robot_bounds_);
			nh.getParam("max_linear_velocity", v_max_);
			nh.getParam("max_rotational_velocity", w_max_);
			nh.getParam("ls2_diff", ls2_diff_);
			nh.getParam("security_distance", security_distance_);
			nh.getParam("max_sensor_distance", d_max_);
			security_nearness_ = d_max_ - security_distance_;
			nh.getParam("num_sectors", num_sectors_);
			int wide_angle;
			nh.getParam("wide_area_angle", wide_angle);
			wide_valley_ = num_sectors_ / (360 / wide_angle);

			initialized_ = true;

		}else{
			ROS_WARN("This planner has already been initialized, doing nothing.");
		}

	}

	bool NDLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){

		// std:cout << "setPlan ..." << std::endl;

		if(!initialized_){
			ROS_ERROR("THe planner has not been initialized.");
			return false;
		}

		//reset the global plan
		global_plan_.clear();

		// update obstacle info from costmap
		costmap_ = costmap_ros_->getCostmap();

		//Prune the plan to store the set of points within the local costmap
		for (auto it = plan.begin(); it != plan.end(); ++it){

			unsigned mx, my;
			if (costmap_->worldToMap((*it).pose.position.x, (*it).pose.position.y, mx, my))
				global_plan_.push_back((*it));
		}

		if (global_plan_.empty()){
			ROS_WARN("NDLocalPlanner: Global plan empty");
			return false;
		}

	   	return true;
	}

	bool NDLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
	    //Compute the velocity command (v,w) for a differential-drive robot
		auto begin = std::chrono::steady_clock::now();

		if (!initialized_) {
			ROS_ERROR("NDLocalPlanner: The planner has not been initialized.");
			return false;
		}

		// Get robot pose in the global frame of the local costmap
		geometry_msgs::PoseStamped robot_global_pose;
		if (!costmap_ros_->getRobotPose(robot_global_pose)) {
			ROS_ERROR("NDLocalPlanner: cant get current robot pose");
			return false;
		}

		// Get transform from global to robot
		geometry_msgs::TransformStamped globalToRobot, robotToGlobal;
		try {
			globalToRobot = tf_->lookupTransform(costmap_ros_->getBaseFrameID(),
								costmap_ros_->getGlobalFrameID(),
								ros::Time(0),
								ros::Duration(0.5)
								);
			robotToGlobal = tf_->lookupTransform(costmap_ros_->getGlobalFrameID(),
								costmap_ros_->getBaseFrameID(),
								ros::Time(0),
								ros::Duration(0.5)
								);
		} catch(tf2::TransformException& e) {
			ROS_ERROR("NDLocalPlanner: Error in lookupTransform");
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
        	return false;
		}

		std::cout << "----------------------------------\n";

		// Transform robot and goal pose to robot frame
		geometry_msgs::PoseStamped robot_pose, goal;
		tf2::doTransform(robot_global_pose, robot_pose, globalToRobot);
		tf2::doTransform(global_plan_.back(), goal, globalToRobot);

		costmap_ = costmap_ros_->getCostmap();


		/// 1. Read obstacle information from the costmap
		std::vector<double> di(num_sectors_, d_max_ + 10.0f); // min distance to each sector
		std::vector<geometry_msgs::Pose> L; // obstacle position in robot frame
		std::vector<double> L_d; // distance obstacle - robot
		std::vector<double> L_th; // angle obstacle - robot

		for (unsigned int x = 0; x < costmap_->getSizeInCellsX(); x++) {
			for (unsigned int y = 0; y < costmap_->getSizeInCellsY(); y++){
				if (costmap_->getCost(x, y) == costmap_2d::LETHAL_OBSTACLE) {
					// Distance to robot
					geometry_msgs::Pose obs_global_pose, obs_pose;					
					costmap_->mapToWorld(x, y, obs_global_pose.position.x, obs_global_pose.position.y);
					
					tf2::doTransform(obs_global_pose, obs_pose, globalToRobot);
					L.push_back(obs_pose);
					

					double dist = euclideanDistance(obs_pose, robot_pose.pose);
					L_d.push_back(dist);

					// Angle from robot
					double angle = atan2(obs_pose.position.y, obs_pose.position.x);
					L_th.push_back(angle);
					int sector = angleToSector(angle, num_sectors_);

					// Update min distance sector
					if (dist < di[sector]) {
						di[sector] = dist;
					}
				}
			}
		}

		/// 2. Compute PND and RND
		const double ROBOT_FIT = 2 * robot_radius_;

		std::vector<double> PND(num_sectors_, 0.0f);
		std::vector<double> RND(num_sectors_, 0.0f);
		bool obstacles = false;
		
		for (int i = 0; i < num_sectors_; i++) {
			double dist = di[i];
			if (dist > 0 && dist <= d_max_) {
				PND[i] = d_max_ + ROBOT_FIT - dist;
				RND[i] = d_max_ + distance_robot_bounds_ - dist;

				obstacles = true;
			}
		}

		double angle = atan2(goal.pose.position.y, goal.pose.position.x);
		int goal_sector = angleToSector(angle, num_sectors_);
		double goal_dist = euclideanDistance(goal.pose, robot_pose.pose);
		if (goal_dist < di[goal_sector]) {
			PND[goal_sector] = 0;
		}

		std::cout << "Goal sector: " << goal_sector << "\n";

		/// 3-4. Identify Gaps and Regions
		std::vector<Valley> valleys;

		if (obstacles) {
			/// 3. Identify Gaps
			std::vector<int> gaps; // guarda el indice del sector donde empieza el gap
			for (int i = 0; i < num_sectors_; i++) {
				int j = (i + 1) % num_sectors_;
				double d = PND[i] - PND[j]; 
				if (abs(d) > ROBOT_FIT) {
					if (d >= 0) {
						gaps.push_back(j);
					} else {
						gaps.push_back(i);
					}
				}
			}

			/// 4. Identify Regions
			for (int i = 0; i < gaps.size(); i++) {
				int startSector = gaps[i];
				int nextGap = (i + 1 ) % gaps.size();
				if (nextGap < gaps.size()) { // puede existir region
					int endSector = gaps[nextGap];

					bool risingDiscontinuity = PND[(startSector - 1) % num_sectors_] > PND[startSector] ||
												PND[(endSector + 1) % num_sectors_] > PND[endSector];

					if (!risingDiscontinuity) {
						continue;
					}
					
					Valley v;
					v.s_l = startSector;
					v.s_r = endSector;
					int d_start = diffSectors(startSector, goal_sector, num_sectors_);
					int d_end = diffSectors(endSector, goal_sector, num_sectors_);
					if (d_start >= d_end) {
						v.s_rd = endSector;
						v.s_od = startSector;
						v.distance_to_goal = d_end;
					} else {
						v.s_rd = startSector;
						v.s_od = endSector;
						v.distance_to_goal = d_start;
					}
					
					bool isRegion = true;
					int s = startSector;
					while (s != endSector) {
						int sj = (s+1) % num_sectors_;
						if (abs(PND[s] - PND[sj]) > ROBOT_FIT) {
							isRegion = false;
							break;
						}

						s = sj;
					}

					if (!isRegion) {
						continue;
					}

					valleys.push_back(v);
				}
			}

		} else {
			Valley v;
			v.s_l = 0;
			v.s_r = num_sectors_ - 1;
			v.s_rd = v.s_l;
			v.s_od = v.s_r;
			v.distance_to_goal = 0;
			valleys.push_back(v);
		}

		/// 5. Free walking area
		if (valleys.size() == 0){
			ROS_ERROR("NDLocalPlanner: NO VALLEYS");
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			return false;
		}

		std::sort(valleys.begin(), valleys.end(), &sort_valley); // sort the valleys by proximity to the goal

		Valley valley;
		int idx_valley;
		bool goal_in_valley = false;
		
		for (idx_valley = 0; idx_valley < valleys.size(); idx_valley++) {
			valley = valleys[idx_valley];
			if (valley.s_l <= valley.s_r) {
				goal_in_valley = valley.s_l <= goal_sector && goal_sector <= valley.s_r;
			} else {
				goal_in_valley = !(valley.s_r < goal_sector && goal_sector < valley.s_l);
			}

			geometry_msgs::Pose goal_v;
			if (goal_in_valley) {
				goal_v = goal.pose;
			} else {
				int s = valley.s_rd;
				int s_2 = (valley.s_rd == valley.s_l ? s - 1 : s + 1) % num_sectors_;

				double dist = abs(di[s] - di[s_2]) / 2.0f + di[s_2];
				double th = bisectorAngle(s, num_sectors_);

				goal_v.position.x = cos(th) * dist;
				goal_v.position.y = sin(th) * dist;
			}

			if (isValleyNavigable(robot_pose.pose, robot_radius_, goal_v, L, L_d, L_th)) {
				break;
			}
		}

		if (idx_valley == valleys.size()) {
			ROS_ERROR("NDLocalPlanner: No valley navigable");
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			return false;
		}
		
		std::cout << "NDLocalPlanner - Valley choose:\n"
				<< "\tS_rd: " << valley.s_rd << " (S_l, S_r): " << valley.s_l << ", " << valley.s_r << std::endl
				<< "\tDistance:"<< valley.distance_to_goal << std::endl;

		/// 6. Choose situation and compute action
		// RND sectors that exceed the security nearness at both sides of s_rd
		int s_ml = -1, s_mr = -1;
		double d_obs_l = d_max_ + 10.0f, d_obs_r = d_max_ + 10.0f;
		double max_ml = 0, max_mr = 0;
		for (int i = num_sectors_/4; i < 3*num_sectors_/4; i++) {
			double rnd = RND[i];
			if (rnd > security_nearness_) {
				if (i <= valley.s_rd && rnd > max_ml) { // left
					s_ml = i;
					max_ml = rnd;
					d_obs_l = di[i];
				} else if (i > valley.s_rd && rnd > max_mr) { // right
					s_mr = i;
					max_mr = rnd;
					d_obs_r = di[i];
				}
			}
		}

		// Decision Tree
		double v = 0, w = 0, theta = 0;

		// Criterion 1: Safety criterion
		bool obs_l = (s_ml != -1);
		bool obs_r = (s_mr != -1);

		if (obs_l || obs_r) { // Low Safety
			// Criterion 2: Dangerous obstacle distribution criterion
			int s_th;
			double d_obs = (d_obs_l < d_obs_r ? d_obs_l : d_obs_r) - robot_radius_;

			if (obs_l && obs_r) { // Low Safety 2
				std::cout << "Low Safety 2" << std::endl;
				
				int s_med = (s_ml + s_mr) / 2;
				float coeff = 1.0f - d_obs / (d_obs_r + d_obs_l); // [0.5-1]
				int c = round((float)ls2_diff_ * coeff);

				if (d_obs_l > d_obs_r){
					c = -c;
				}
				s_th = s_med + c;
			
			} else { // Low Safety 1
				std::cout << "Low Safety 1\n";

				int sign = 0;
				if (valley.s_rd == valley.s_l && obs_l) {
					sign = 1;
				} else if (valley.s_rd == valley.s_r && obs_r) {
					sign = -1;
					s_ml = s_mr;
				}

				int s_p = 1;
				if (sign != 0) {
					int diff = diffSectors(valley.s_rd, s_ml, num_sectors_);
					int cte = wide_valley_ / 2;
					float rho =  1 - (float)(diff) / (float)(num_sectors_/2);
					s_p = cte/2 * rho + cte;
				}

				s_th = valley.s_rd + sign * s_p;
			}
			
			s_th = s_th % num_sectors_;
			theta = bisectorAngle(s_th, num_sectors_);
			theta = clampAngle(theta);

			// Translational Velocity
			v = v_max_ * (d_obs / security_distance_) * abs(1 - abs(theta) / M_PI_2);

		} else { // High Safety
			// Criterion 3: Goal within the free walking area criterion
			if (goal_in_valley) { // High Safety Goal in Region
				std::cout << "High Safety Goal in Region" << std::endl;
				theta = bisectorAngle(goal_sector, num_sectors_);
			} else {
				// Criterion 4: Free walking area width criterion
				int s_th;
				int walley_width = diffSectors(valley.s_l, valley.s_r, num_sectors_) + 1;
				if (walley_width >= wide_valley_) { // High Safety Wide Region
					if (valley.s_rd == valley.s_l) {
						s_th = valley.s_rd + wide_valley_ / 4;
					} else {
						s_th = valley.s_rd - wide_valley_ / 4;
					}
					std::cout << "High Safety Wide Region" << std::endl;
					
				} else { // High Safety Narrow Region
					std::cout << "High Safety Narrow Region\n";
					if (valley.s_l <= valley.s_r) {
						s_th = (valley.s_l + valley.s_r) / 2;
					} else {
						s_th = valley.s_l + (walley_width - 1) / 2;
					}
					
				}
				
				s_th = s_th % num_sectors_;
				theta = bisectorAngle(s_th, num_sectors_);
			}
			
			theta = clampAngle(theta);

			// Translational Velocity
			v = v_max_ * abs(1 - abs(theta) / M_PI_2);
		}

		
		// Rotational Velocity
		w = w_max_ * theta / M_PI_2;

		std::cout << "Theta: " << theta << std::endl;
		std::cout << "V: " << v << " W: " << w << std::endl;
		cmd_vel.linear.x = v;
		cmd_vel.angular.z = w;

		auto end = std::chrono::steady_clock::now();
		unsigned int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count();
		std::cout << "Exec. time: " << elapsed << " ms\n";

		exec_time += elapsed;
		n_times++;
		std::cout << "Average Exec. time: " << (float)exec_time / (float)n_times << " ms\n";
		
		return true;
	}

	bool NDLocalPlanner::isGoalReached(){
		//Check if the robot has reached the position and orientation of the goal

		if (! initialized_) {
			ROS_ERROR("This planner has not been initialized.");
			return false;
		}

		//Get robot and goal pose in the global frame of the local costmap
		geometry_msgs::PoseStamped robot_pose;
		costmap_ros_->getRobotPose(robot_pose);
		const geometry_msgs::PoseStamped goal = global_plan_.back();

		// bool goalReached = false;



		//implement here the condition(s) to have reached the goal
		float ex = robot_pose.pose.position.x - goal.pose.position.x;
		float ey = robot_pose.pose.position.y - goal.pose.position.y;
		float dist = sqrt(ex*ex + ey*ey);


		return dist < robot_radius_ * goal_factor_;
	}

};
