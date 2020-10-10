#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <waypoint_maker/Lane.h>
#include <waypoint_maker/State.h>
#include <nav_msgs/Odometry.h>

#include <vector>
#include <cmath>
#include <math.h>

using namespace std;

class WaypointFollower {
private:
	double prev_steer_;
	bool isfirst_steer_;
	int waypoints_size_;
	int mission_state_;

	vector<waypoint_maker::Waypoint> waypoints_;

	double init_speed_;

	double lookahead_dist_;
	double init_lookahead_dist_;

	int current_mission_state_;
	int next_mission_state_;
	int next_mission_index_;
	int next_waypoint_index_;
	int target_index_;
	int waypoint_target_index_;

	int first_state_index_;
	int second_state_index_;
	int third_state_index_;
	int fourth_state_index_;
	int fifth_state_index_;
	int sixth_state_index_;
	int seventh_state_index_;
	int eighth_state_index_;

	bool is_pose_;
	bool is_course_;
	bool is_lane_;
	bool is_state_change_;
	bool is_control_;

	bool mission_A_;

//for mission
	int parking_count_;
	int detected_number_;
	bool return_sign_;




	geometry_msgs::PoseStamped cur_pose_;

	double cur_course_;
					  
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::Publisher ackermann_pub_;
	ros::Publisher state_pub_;
	ros::Publisher current_state_pub_;

	ros::Subscriber odom_sub_;
	ros::Subscriber course_sub_;
	ros::Subscriber lane_sub_;
	
	ros::Time return_time_;	

	ackermann_msgs::AckermannDriveStamped ackermann_msg_;
	waypoint_maker::Waypoint state_msg_;
	waypoint_maker::State current_state_msg_;//for Vision

public:
	WaypointFollower() {
			  initSetup();
	}

	~WaypointFollower() {
			  waypoints_.clear();
	}

void initSetup() {
	ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("gps_ackermann", 10);
	state_pub_ = nh_.advertise<waypoint_maker::Waypoint>("target_state", 10);
	current_state_pub_ = nh_.advertise<waypoint_maker::State>("current_state", 10);

	odom_sub_ = nh_.subscribe("odom", 10, &WaypointFollower::OdomCallback, this);
   course_sub_ = nh_.subscribe("course", 10, &WaypointFollower::CourseCallback, this);
   lane_sub_ = nh_.subscribe("final_waypoints", 10, &WaypointFollower::LaneCallback, this);

	private_nh_.getParam("/waypoint_follower_node/init_speed", init_speed_);
   private_nh_.getParam("/waypoint_follower_node/init_lookahead_distance", init_lookahead_dist_);
	private_nh_.getParam("/waypoint_follower_node/current_mission_state", current_mission_state_);
	
	private_nh_.getParam("/waypoint_follower_node/first_state_index", first_state_index_);
	private_nh_.getParam("/waypoint_follower_node/second_state_index", second_state_index_);
	private_nh_.getParam("/waypoint_follower_node/third_state_index", third_state_index_);
	private_nh_.getParam("/waypoint_follower_node/fourth_state_index", fourth_state_index_);
	private_nh_.getParam("/waypoint_follower_node/fifth_state_index", fifth_state_index_);
	private_nh_.getParam("/waypoint_follower_node/sisth_state_index", sixth_state_index_);
	private_nh_.getParam("/waypoint_follower_node/seventh_state_index", seventh_state_index_);
	private_nh_.getParam("/waypoint_follower_node/eighth_state_index", eighth_state_index_);

<<<<<<< HEAD
	private_nh_.getParam("/detected_number",detected_number_);
	private_nh_.getParam("/return_sign",return_sign_);
	private_nh_.getParam("/mission_A",mission_A_);

=======
	private_nh_.setParam("/return_sign",false);
>>>>>>> 2a30bc3482363a702d5e5d3e8e532157403428d0
	ROS_INFO("WAYPOINT FOLLOWER INITIALIZED.");

	parking_count_ = -1;

	isfirst_steer_ = true;
	prev_steer_ = 0;

	next_mission_state_ = current_mission_state_ + 1;
	is_pose_ = false;
	is_course_ = false;
	is_lane_ = false;

	is_state_change_ = false;
	is_control_ = false;
}

float calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2) {
	float dist = sqrtf(powf(pose1.pose.position.x - pose2.pose.position.x, 2) + powf(pose1.pose.position.y - pose2.pose.position.y, 2));
   	return dist;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
	cur_pose_.header = odom_msg->header;
	cur_pose_.pose.position = odom_msg->pose.pose.position;
   	is_pose_ = true;
//	ROS_INFO("CURRENT POSE CALLBACK");
}


void CourseCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &course_msg) {
	cur_course_ = course_msg->drive.steering_angle;
   	is_course_ = true;
}

void LaneCallback(const waypoint_maker::Lane::ConstPtr &lane_msg) {
  	waypoints_.clear();
  	waypoints_ = lane_msg->waypoints;
  	waypoints_size_ = waypoints_.size();

	is_lane_ = true;
	
	for(int i=0;i<waypoints_size_;i++) {
		int index = waypoints_[i].waypoint_index;
		if(index == first_state_index_ || index == second_state_index_ || index == third_state_index_ || index == fourth_state_index_ || index == fifth_state_index_ || index == sixth_state_index_ || index == seventh_state_index_ || index ==eighth_state_index_) {
			next_mission_state_ = waypoints_[i].mission_state;
			next_mission_index_ = index;
			next_waypoint_index_ = i;
			is_state_change_ = true;
//			ROS_INFO("%d STATE CHANGE DETECTED.", next_mission_state_);
			return;
		}	
	}
}


double calcSteeringAngle() {
	for(int i=0;i<waypoints_size_;i++) {
   	double dist = calcPlaneDist(cur_pose_, waypoints_[i].pose);
      	if(dist>lookahead_dist_){
			target_index_=i;
			waypoint_target_index_ = waypoints_[i].waypoint_index;
//		   ROS_INFO("target_index: %d ld: %f",target_index_,lookahead_dist_);
		   break;
      }
	}

        double steering_angle;

        double target_x = waypoints_[target_index_].pose.pose.position.x;
        double target_y = waypoints_[target_index_].pose.pose.position.y;

  //      ROS_INFO("TARGET X=%f, TARGET Y=%f", target_x, target_y);

        double dx = target_x - cur_pose_.pose.position.x +0.000000001;
        double dy = target_y - cur_pose_.pose.position.y;

        double heading = atan(dy/dx);
        double angle = heading * 180 / 3.141592;
        double cur_steer;


		  if(dx>=0 && dy>0) {
			  cur_steer = 90.0 - angle - cur_course_;
			  if(cur_steer<-270) cur_steer = cur_steer + 360;
		  }
		  else if(dx>=0 && dy<0) cur_steer = 90.0 - angle - cur_course_;
        else if(dx<0 && dy<0) cur_steer = 270.0 - angle  - cur_course_;
        else if(dx<0 && dy>0) {
			  cur_steer = 270.0 - angle - cur_course_;
			  if(cur_steer>270) cur_steer= cur_steer-360 ; 
		  }

        if(cur_steer<-180) cur_steer = 360 +cur_steer;
        else if(cur_steer>180) cur_steer = cur_steer -360 ;
	

        if(isfirst_steer_) {
			  prev_steer_ = cur_steer;
			  isfirst_steer_ = false;
        }
        else {
               // if(abs(cur_steer - prev_steer_) < 5.0) cur_steer = prev_steer_;
        }		  
        return cur_steer;
}

double calcAngularVelocity(double steering_angle){
	double angular_velocity ;
	double time_to_target ;
		  
	time_to_target = (lookahead_dist_ + 0.5) / init_speed_ ;
	angular_velocity = -steering_angle / time_to_target;
		  
	return angular_velocity;
}


void process() {
	double speed;
	double dist = 0;
	current_state_msg_.dist =-1.0;// for vision stop_lineTODO stop line delete
	if(is_pose_ && is_course_ && is_lane_ ) {
		is_control_ = true;                
		if(is_state_change_) {
			double dist = calcPlaneDist(cur_pose_, waypoints_[next_waypoint_index_].pose);
			cout << "DIST : " << dist << endl;

			current_state_msg_.dist = dist;// for vision stop_line
			current_state_msg_.current_state = waypoints_[0].mission_state;//for Vision
			current_state_pub_.publish(current_state_msg_);
			
			if(dist < 1.5 && next_mission_state_ == 1){
				parking_count_ = 0;
				private_nh_.setParam("/waypoint_loader_node/parking_state", 0);		
			}
			else if(dist < 1.5 && next_mission_state_ == 2){
				parking_count_ =1;
				private_nh_.setParam("/waypoint_loader_node/parking_state", 1);		
			}

			else if(dist < 2.5 && next_mission_state_ == 3) {//배달 장소에 도착했습니다.Duration과 관련된 Logic을 추가하세요.
				while(1){
					private_nh_.getParam("/return_sign",return_sign_);
					ackermann_msg_.header.stamp = ros::Time::now();
					ackermann_msg_.drive.speed = 0.0;
					ackermann_msg_.drive.steering_angle = 0.0;

					ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
		
					if(!return_sign_){
						return_time_ = ros::Time::now();
						break;	
					}					
					else{
						if((ros::Time::now() - return_time_).toSec() <= 2){
							ackermann_msg_.header.stamp = ros::Time::now();
			      				ackermann_msg_.drive.speed = 0.0;
							ackermann_msg_.drive.steering_angle = 0.0;

							ackermann_pub_.publish(ackermann_msg_);
							is_control_ = false;
						}
						else if(mission_A_&&(ros::Time::now() - return_time).toSec() <= 6){
							ackermann_msg_.header.stamp = ros::Time::now();
			      				ackermann_msg_.drive.speed = 0.0;
							ackermann_msg_.drive.steering_angle = 60;

							ackermann_pub_.publish(ackermann_msg_);
							is_control_ = false;					
							break;
						}
						else {
							parking_count_ =2;
							private_nh_.setParam("/waypoint_loader_node/parking_state", 2);
							private_nh_.setParam("/return_sign",false);							
							break;
						}
					}
				}	
			}

			else if(dist < 1.5 && next_mission_state_ == 4){
				parking_count_ =-2;
				private_nh_.setParam("/waypoint_loader_node/parking_state", -2);		
				}

			else if(detected_number_ ==-1 && waypoints_[0].mission_state == 5){
				private_nh_.getParam("/detected_number", detected_number_);
			}
			else if(dist < 2.25 && next_mission_state_ == (detected_number_+ 5)){
				is_control_ = false;
				ackermann_msg_.header.stamp = ros::Time::now();
				ackermann_msg_.drive.speed = 0.0;
				ackermann_msg_.drive.steering_angle = 0.0;
               			
				ackermann_pub_.publish(ackermann_msg_);
					
				ros::shutdown();
				}
		}
		if(is_control_) {
			speed = init_speed_;
			lookahead_dist_ = init_lookahead_dist_;

			double cur_steer = calcSteeringAngle();

			cur_steer = calcAngularVelocity(cur_steer);
			ackermann_msg_.header.stamp = ros::Time::now();
	 		ackermann_msg_.drive.speed = speed;
			ackermann_msg_.drive.steering_angle_velocity = cur_steer;

			ackermann_pub_.publish(ackermann_msg_);
			
		}
		else {
			//ROS_INFO("IS_CONTROL IS FALSE, NOT PUBLISH.");
		}		
		is_pose_ = false;
      		is_course_ = false;
		//is_lane_ = false;
		is_state_change_ = false;
		
		state_msg_.waypoint_index = waypoints_[target_index_].waypoint_index;
		state_msg_.mission_state = waypoints_[target_index_].mission_state;
		state_pub_.publish(state_msg_);
	}
		
}

};

int main(int argc, char **argv) {
   ros::init(argc, argv, "waypoint_follower");
   WaypointFollower wf;
	ros::Rate loop_rate(10);
	
	while(ros::ok()) {
		wf.process();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
