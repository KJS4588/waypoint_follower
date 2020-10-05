#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <waypoint_maker/Lane.h>
#include <waypoint_maker/Waypoint.h>
#include <waypoint_maker/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>

#include<iostream>
#include<iomanip>

#include <vector>
#include <cmath>
#include <math.h>
#include "ros/time.h"

#include "time.h"

using namespace std;

class WaypointFollower {
private:
	double prev_steer_;
	bool isfirst_steer_;
	bool parking_trigger_;
	int waypoints_size_;
	int mission_state_;

	// 0. before parking state
	// 1. enter parking area
	// 2. after parking state(backward)
	// 3. retrieve and go(forward)
	// 4. go to the main Lane
	/*======================*/
	// 5. static avoidance state 1
	// 6. intersection state A(KID ZONE Straight)
	// 7. intersection state B(AnJeon elemet. Left)
	// 8. dynamic avoidance
	// 9. static avoidance state 2

	/*======================*/
	// 10. intersection state C
	// 11. intersection state D
	// 12. intersection state E
	// 13. intersection state F
	// 14. intersection state G
	// 15. goalpoint

	vector<waypoint_maker::Waypoint> waypoints_;

	//speed
	double init_speed_;
	double decelate_speed_;
	//double accelate_speed_;
	double parking_speed_;
	double backward_movement_speed_;

	//lookahead_dist
	double lookahead_dist_;
	double init_lookahead_dist_;
	double decelate_lookahead_dist_;
	//double accelate_lookahead_dist_;
	double parking_lookahead_dist_;

	//state, index, lane number
	int current_mission_state_;
	int next_mission_state_;
	int next_mission_index_;
	int next_waypoint_index_;
	int target_index_;
	int waypoint_target_index_;
	int lane_number_;
	int static_lane_change_count_;

	//state list
	int first_state_index_;
	int second_state_index_;
	int third_state_index_;
	int fourth_state_index_;
	int fifth_state_index_;
	int sixth_state_index_;
	int seventh_state_index_;
	int eighth_state_index_;
	int nineth_state_index_;
	int tenth_state_index_;
	int eleventh_state_index_;
	int twelveth_state_index_;
	int thirteenth_state_index_;
	int fourteenth_state_index_;
	int fifteenth_state_index_;

	//parking
	int parking_count_;
	bool is_backward_;
	bool is_retrieve_;
	int parking_test_count_;
	int lane_final_;
	int ex_lane_final_;


	//flags
	bool is_pose_;
	bool is_course_;
	bool is_lane_;
	bool is_state_change_;
	bool is_control_;
	bool is_parking_area_;
	bool is_parking_test_;
	bool before_intersection_;
	bool is_onLane_;
	bool is_obs_detect_;

	//local_system_polifit
	double x_list[15];
	double y_list[15];
	double curvature_;


	//pose and course
	geometry_msgs::PoseStamped cur_pose_;
	double cur_course_;


	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Publisher ackermann_pub_;
	ros::Publisher index_pub_;
	ros::Publisher state_pub_;
	ros::Publisher current_state_pub_;//for Vision
	ros::Publisher parking_info_pub_;//for Lidar_parking

	ros::Subscriber odom_sub_;
	ros::Subscriber course_sub_;
	ros::Subscriber lane_sub_;
	ros::Subscriber parking_area_sub_;
	ros::Subscriber static_lidar_sub_;

	ackermann_msgs::AckermannDriveStamped ackermann_msg_;
	waypoint_maker::Waypoint index_msg_;
	waypoint_maker::State state_msg_;
	waypoint_maker::State current_state_msg_;//for Vision
	geometry_msgs::Pose2D parking_info_msg_;//for LiDAR_parking

public:
	WaypointFollower() {
        initSetup();
	}

	~WaypointFollower() {
        waypoints_.clear();
	}

	void initSetup() {
		ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("gps_ackermann", 10);
		index_pub_ = nh_.advertise<waypoint_maker::Waypoint>("target_state", 10);
		state_pub_ = nh_.advertise<waypoint_maker::State>("gps_state", 10);
		current_state_pub_ = nh_.advertise<waypoint_maker::State>("current_state", 10);//for vision(current_state

		parking_info_pub_ = nh_.advertise<geometry_msgs::Pose2D>("parking_info", 10);// for LiDAR_parking

		odom_sub_ = nh_.subscribe("odom", 10, &WaypointFollower::OdomCallback, this);
		course_sub_ = nh_.subscribe("course", 10, &WaypointFollower::CourseCallback, this);
		lane_sub_ = nh_.subscribe("final_waypoints", 10, &WaypointFollower::LaneCallback, this);
		parking_area_sub_ = nh_.subscribe("parking_area",10,&WaypointFollower::ParkingAreaCallback,this);
		static_lidar_sub_ = nh_.subscribe("static_obs",10,&WaypointFollower::StaticLidarCallback,this);

		private_nh_.getParam("/waypoint_follower_node/init_speed", init_speed_);
		private_nh_.getParam("/waypoint_follower_node/decelate_speed", decelate_speed_);
		private_nh_.getParam("/waypoint_follower_node/parking_speed", parking_speed_);
		private_nh_.getParam("/waypoint_follower_node/backward_movement_speed", backward_movement_speed_);

		private_nh_.getParam("/waypoint_follower_node/init_lookahead_distance", init_lookahead_dist_);
		private_nh_.getParam("/waypoint_follower_node/decelate_lookahead_distance", decelate_lookahead_dist_);
		private_nh_.getParam("/waypoint_follower_node/parking_lookahead_distance", parking_lookahead_dist_);

		private_nh_.getParam("/waypoint_follower_node/current_mission_state", current_mission_state_);
		
		private_nh_.getParam("/waypoint_follower_node/first_state_index", first_state_index_);
		private_nh_.getParam("/waypoint_follower_node/second_state_index", second_state_index_);
		private_nh_.getParam("/waypoint_follower_node/third_state_index", third_state_index_);
		private_nh_.getParam("/waypoint_follower_node/fourth_state_index", fourth_state_index_);
		private_nh_.getParam("/waypoint_follower_node/fifth_state_index", fifth_state_index_);
		private_nh_.getParam("/waypoint_follower_node/sixth_state_index", sixth_state_index_);
		private_nh_.getParam("/waypoint_follower_node/seventh_state_index", seventh_state_index_);
		private_nh_.getParam("/waypoint_follower_node/eighth_state_index", eighth_state_index_);
		private_nh_.getParam("/waypoint_follower_node/nineth_state_index", nineth_state_index_);	
		private_nh_.getParam("/waypoint_follower_node/tenth_state_index", tenth_state_index_);
		private_nh_.getParam("/waypoint_follower_node/eleventh_state_index", eleventh_state_index_);
		private_nh_.getParam("/waypoint_follower_node/twelveth_state_index", twelveth_state_index_);
		private_nh_.getParam("/waypoint_follower_node/thirteenth_state_index", thirteenth_state_index_);
		private_nh_.getParam("/waypoint_follower_node/fourteenth_state_index", fourteenth_state_index_);
		private_nh_.getParam("/waypoint_follower_node/fifteenth_state_index", fifteenth_state_index_);
		private_nh_.getParam("/waypoint_follower_node/lane_final", lane_final_);

        ex_lane_final_ = lane_final_;

		ROS_INFO("WAYPOINT FOLLOWER INITIALIZED.");

		parking_count_ = -1;
		parking_test_count_=0;
		lookahead_dist_ = 7.0;

		isfirst_steer_ = true;
		prev_steer_ = 0;

		next_mission_state_ = current_mission_state_ + 1;
		is_pose_ = false;
		is_course_ = false;
		is_lane_ = false;
		is_state_change_ = false;
		is_control_ = true;
		parking_trigger_ = false;
		is_backward_ = false;
		is_retrieve_ = false;
		is_parking_area_ = true; //is_parking_area true이면 경로 유지
		is_parking_test_ = false; //callback이 들어오면 true

		is_onLane_ = false;
		is_obs_detect_ = false;
		static_lane_change_count_ = 0;

		cur_course_ = 0.0;
		lane_number_ = 0;
		waypoints_size_ = 0;

		before_intersection_ = false;

		x_list[15] ={0.0,};
		y_list[15] ={0.0,};
		curvature_=0.0;
	}

	float calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2) {
        return sqrtf(powf(pose1.pose.position.x - pose2.pose.position.x, 2) + powf(pose1.pose.position.y - pose2.pose.position.y, 2));
	}

	void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
		cur_pose_.header = odom_msg->header;
		cur_pose_.pose.position = odom_msg->pose.pose.position;
        is_pose_ = true;
		ROS_INFO("CURRENT POSE CALLBACK");
	}

	void CourseCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &course_msg) {
		cur_course_ = course_msg->drive.steering_angle;
		is_course_ = true;
		ROS_INFO("COURSE CALLBACK");
	}

	void ParkingAreaCallback(const std_msgs::Bool::ConstPtr &parking_area_msg) {
		is_parking_area_ = parking_area_msg->data;
		is_parking_test_ = true;
	}

	void StaticLidarCallback(const std_msgs::Bool::ConstPtr &static_lidar_msg) {
		is_obs_detect_ = static_lidar_msg->data;
	}

	void LaneCallback(const waypoint_maker::Lane::ConstPtr &lane_msg) {
		waypoints_.clear();
		waypoints_ = lane_msg->waypoints;
		waypoints_size_ = waypoints_.size();
		ROS_INFO("LANE CALLBACK");

		is_onLane_ = lane_msg->onlane;

		is_lane_ = true;

        for(int i=0;i<waypoints_size_;i++) {
			int index = waypoints_[i].waypoint_index;
			if( (index == first_state_index_ || index == second_state_index_ || index == third_state_index_ || index == fourth_state_index_ || index == fifth_state_index_ || index == sixth_state_index_ || index == seventh_state_index_ || index == eighth_state_index_ || index == nineth_state_index_ || index == tenth_state_index_ || index == eleventh_state_index_ || index == twelveth_state_index_ || index == thirteenth_state_index_ || index == fourteenth_state_index_|| index == fifteenth_state_index_)) {
				next_mission_state_ = waypoints_[i].mission_state;
				next_mission_index_ = index;
				next_waypoint_index_ = i;
				is_state_change_ = true;
				ROS_INFO("%d STATE CHANGE DETECTED LOOP 1.", next_mission_state_);
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
				ROS_INFO("target_index: %d ld: %f",target_index_,lookahead_dist_);

				if(parking_count_ == 0 || parking_count_ == 1 ){
					if((waypoints_[i].waypoint_index - waypoints_[0].waypoint_index == 1) || (waypoints_[i].waypoint_index - waypoints_[0].waypoint_index == 2  )) {
						ROS_INFO("1");
						target_index_ = i;
						waypoint_target_index_ = waypoints_[i].waypoint_index;
						break;
					}
				}

				if(waypoints_[i].mission_state == 3) {
					is_retrieve_ = false;
				}
				break;
			}
        }

        if(parking_trigger_ || is_retrieve_){
			if(cur_course_ < 180) cur_course_ += 180;
			else cur_course_ -= 180;
			
			ROS_INFO("CURRENT COURSE INVERTED.");
		}

        double steering_angle;
        double target_x = waypoints_[target_index_].pose.pose.position.x;
        double target_y = waypoints_[target_index_].pose.pose.position.y;

        ROS_INFO("TARGET X=%f, TARGET Y=%f", target_x, target_y);

        double dx = target_x - cur_pose_.pose.position.x +0.000000001;
        double dy = target_y - cur_pose_.pose.position.y;

        double heading = atan(dy/dx);
        double angle = heading * 180 / 3.141592;
        double cur_steer;

		if( (parking_count_==-2)&&waypoints_size_>20){
			transform2local();
			curvature_ = polifit_with_localsystem();
		}

		if(dx>=0 && dy>0) {
			cur_steer = 90.0 - angle - cur_course_;
			if(cur_steer<-270) cur_steer = cur_steer + 360;
		}

		else if(dx>=0 && dy<0) cur_steer = 90.0 - angle - cur_course_;

		else if(dx<0 && dy<0) cur_steer = 270.0 - angle - cur_course_;

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

        return cur_steer;
	}

	void parking_info(){
		float local_x = 0.0;
		float local_y = 0.0;

		float point_x = 0.0;
		float point_y = 0.0;

		float local_theta = 0.0;
		int mission_index_err = second_state_index_ - first_state_index_-4;

		float temp_wayposition_x = waypoints_.at(next_waypoint_index_ + mission_index_err).pose.pose.position.x;
		float temp_wayposition_y = waypoints_.at(next_waypoint_index_ + mission_index_err).pose.pose.position.y;
		
		float temp_posepoint_x = cur_pose_.pose.position.x;
		float temp_posepoint_y = cur_pose_.pose.position.y;

		point_x = temp_wayposition_x - temp_posepoint_x;
		point_y = temp_wayposition_y - temp_posepoint_y;

		//transform to r,theta system
		float local_r= sqrtf(powf(point_x, 2) + powf(point_y, 2));
		if ((point_x > 0) && (point_y > 0)){
			local_theta = atan(point_x/point_y)*180/M_PI - cur_course_;
			if (local_theta < -270) local_theta = local_theta + 360;
		}

		else if ((point_x > 0) && (point_y < 0)){
			local_theta = 90 - atan(point_y/point_x)*180/M_PI  - cur_course_;
		}

		else if ((point_x < 0) && (point_y < 0)){
			local_theta = 180 + atan(point_x/point_y)*180/M_PI - cur_course_;
		}

		else if ((point_x < 0) && (point_y > 0)){
			local_theta = 270 - atan(point_y/point_x)*180/M_PI - cur_course_;
			if (local_theta > 270) local_theta = local_theta - 360;		//0도 부근에서 날 수 있는 오차를 보정해주기 위함
		}
			
	
		float local_theta_rad = local_theta*M_PI/180;	
		
		local_x = local_r * cos(local_theta_rad);
		local_y = -local_r * sin(local_theta_rad);

		parking_info_msg_.x = local_x;
		parking_info_msg_.y = local_y;
		parking_info_msg_.theta = local_theta;
		parking_info_pub_.publish(parking_info_msg_);
	}

	void transform2local(){
		x_list[15] = {0,};
		y_list[15] = {0,};
		for (int i=0;i<15;i++){
			double local_x = 0.0;
			double local_y = 0.0;

			double point_x = 0.0;
			double point_y = 0.0;

			//double local_r = 0.0;
			double local_theta = 0.0;

			double temp_wayposition_x = waypoints_[i].pose.pose.position.x;
			double temp_wayposition_y = waypoints_[i].pose.pose.position.y;
			double temp_posepoint_x = cur_pose_.pose.position.x;
			double temp_posepoint_y = cur_pose_.pose.position.y;
			//ROS_INFO("way,pose =  %f,%f",temp_wayposition_x,temp_posepoint_x);
			point_x = temp_wayposition_x - temp_posepoint_x;
			point_y = temp_wayposition_y - temp_posepoint_y;
			//ROS_INFO("point_offset = %f,%f",point_x,point_y);

		//transform to r,theta system
			float local_r= sqrtf(powf(point_x, 2) + powf(point_y, 2));

			if ((point_x > 0) && (point_y > 0)){
				local_theta = atan(point_x/point_y)*180/M_PI - cur_course_;
				if (local_theta < -270) local_theta = local_theta + 360;
			}
			else if ((point_x > 0) && (point_y < 0)){
				local_theta = 90 - atan(point_y/point_x)*180/M_PI  - cur_course_;
			}
			else if ((point_x < 0) && (point_y < 0)){
				local_theta = 180 + atan(point_x/point_y)*180/M_PI - cur_course_;
			}
			else if ((point_x < 0) && (point_y > 0)){
				local_theta = 270 - atan(point_y/point_x)*180/M_PI - cur_course_;
				if (local_theta > 270) local_theta = local_theta - 360;		//0도 부근에서 날 수 있는 오차를 보정해주기 위함
			}
			
			float local_theta_rad = local_theta*M_PI/180;
		
			local_x = local_r * cos(local_theta_rad);
			local_y = -(local_r * sin(local_theta_rad));
		
			x_list[i] = local_x;
			y_list[i] = local_y;
			//ROS_INFO("local_point= %f,%f",local_r,local_theta);
		}
	}

	double polifit_with_localsystem(){
    	int i,j,k,n,N;
    	cout.precision(4);                        //set precision
    	cout.setf(ios::fixed);

		N = 15;
		n = 2;                             // n is the degree of Polynomial 

    	double X[2*n+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)

    	for (i=0;i<2*n+1;i++){
        	X[i]=0;
        	for (j=0;j<N;j++) 
			X[i]=X[i]+pow(x_list[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    	}

    	double B[n+1][n+2],a[n+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients

    	for (i=0;i<=n;i++){
        	for (j=0;j<=n;j++)
        		B[i][j]=X[i+j];
		}            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix

    	double Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)

    	for (i=0;i<n+1;i++){    
        	Y[i]=0;
        	for (j=0;j<N;j++) 
			Y[i]=Y[i]+pow(x_list[j],i)*y_list[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    	}

    	for (i=0;i<=n;i++)
        	B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
			n=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
    	for (i=0;i<n;i++){                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        	for (k=i+1;k<n;k++){
            	if (B[i][i]<B[k][i]){
                	for (j=0;j<=n;j++){
            			double temp=B[i][j];
            			B[i][j]=B[k][j];
            			B[k][j]=temp;
            		}
				}
			}
		}
     
    	for (i=0;i<n-1;i++){            //loop to perform the gauss elimination
        	for (k=i+1;k<n;k++){
                double t=B[k][i]/B[i][i];
            	for (j=0;j<=n;j++)
            		B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
            }
		}
    	for (i=n-1;i>=0;i--){                //back-substitution
		                        //x is an array whose values correspond to the values of x,y,z..
        	a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
        	for (j=0;j<n;j++){
				if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value is being calculated
					a[i]=a[i]-B[i][j]*a[j];
			}
       		a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    	}
/*    	cout<<"\nThe values of the coefficients are as follows:\n";
    	for (i=0;i<n;i++)
        	cout<<"x^"<<i<<"="<<a[i]<<endl;            // Print the values of x^0,x^1,x^2,x^3,....*/    
/*    	cout<<"\nHence the fitted Polynomial is given by:\ny=";
    	for (i=0;i<n;i++)
        	cout<<" + ("<<a[i]<<")"<<"x^"<<i;
    	cout<<"\n";*/

		double temp_curvature = a[2]; //coefficient of x^2
		ROS_INFO("CURVATURE: %f", temp_curvature);

		return temp_curvature;
	}

	void process() {
			
		double speed = 0;
		double dist = 0;
		current_state_msg_.dist =-1.0;// for vision stop_lineTODO stop line delete

		if((lane_number_!=0)&& (parking_count_>-2)){
			ex_lane_final_ = lane_final_;
			private_nh_.getParam("/waypoint_follower_node/first_state_index", first_state_index_);
			private_nh_.getParam("/waypoint_follower_node/second_state_index", second_state_index_);
			private_nh_.getParam("/waypoint_follower_node/third_state_index", third_state_index_);
			private_nh_.getParam("/waypoint_follower_node/fourth_state_index", fourth_state_index_);
			private_nh_.getParam("/waypoint_follower_node/lane_final", lane_final_);
			if(lane_final_ != ex_lane_final_){
					parking_test_count_ = 0;
			}
		}
		else if((static_lane_change_count_ ==1 || static_lane_change_count_==2)&&(waypoints_[0].mission_state==9)){//원래는 주차 밑에, static에 해당하는 state로 기입해야 함. 이를위해 parking_count를 -2로 init함.
			ex_lane_final_ = lane_final_;
			private_nh_.getParam("/waypoint_follower_node/lane_final", lane_final_);
			if(lane_final_ != ex_lane_final_){
			private_nh_.getParam("/waypoint_follower_node/tenth_state_index", tenth_state_index_);
			cout<<"next_mission_index"<<next_mission_index_<<endl;		
			}
		}

        if(is_pose_ && is_course_ && is_lane_) {
			is_control_ = true;
			if(is_state_change_) {
				dist = calcPlaneDist(cur_pose_, waypoints_[next_waypoint_index_].pose);
				current_state_msg_.dist = dist;// for vision stop_line
				current_state_msg_.current_state = waypoints_[0].mission_state;//for Vision
				current_state_pub_.publish(current_state_msg_);

				ROS_INFO("CURRENT TARGET STATE INDEX=%d, MISSION_INDEX=%d, DIST=%f, PARKING_COUNT=%d", next_mission_index_, next_mission_state_, dist,parking_count_);	
				
				if(dist < 1.7 && next_mission_state_ == 1) {
					while(1){
						if(parking_count_==-1){
							ROS_INFO("PARKING SIGN DETECTED. WAITING FOR SIGN.");
							ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
							is_control_ = false;
							//is_lane_ = false;
							parking_count_ =0;
							private_nh_.setParam("/waypoint_loader_node/parking_state", 0);
							ros::Duration(2.0).sleep();
//LiDAR한테 PUB
/*						if(!is_parking_test_&&(parking_test_count_==0)){
							parking_count_ = 0;
							parking_test_count_++;
							ackermann_msg_.header.stamp = ros::Time::now();
							ackermann_msg_.drive.speed = 0.0;
							ackermann_msg_.drive.steering_angle = 0.0;

							ackermann_pub_.publish(ackermann_msg_);
							parking_info();
							is_control_ = false;
						}
					
						else if(is_parking_test_){	
							if(!is_parking_area_){
								lane_number_ += 1;
								is_lane_ = false;
								is_parking_test_ = false;
								break;
							}

						else if(is_parking_area_){
							is_control_ = true;
							private_nh_.setParam("/waypoint_loader_node/parking_state", parking_count_);
							is_lane_ = false; //0908
							break;
							}
						}*/}
						break;
					}	
				}
		
				else if( dist < 1.5 && next_mission_state_ == 2) {

					if(parking_count_ == 0) {
						parking_trigger_ = true;
						parking_count_++;
						is_backward_ = true;
						ROS_INFO("CURRENTLY ARRIVED AT PARKING POINT.");
						ROS_INFO("STOP FOR 4SECONDS.");

						ackermann_msg_.header.stamp = ros::Time::now();
						ackermann_msg_.drive.speed = 0.0;
						ackermann_msg_.drive.steering_angle = 0.0;

						ackermann_pub_.publish(ackermann_msg_);

						is_control_ = false;
						ros::Time::init();
						ros::Duration(7.0).sleep();

						private_nh_.setParam("/waypoint_loader_node/parking_state", parking_count_);
						ackermann_msg_.header.stamp = ros::Time::now();
						ackermann_msg_.drive.speed = 0.0;
						ackermann_msg_.drive.steering_angle = 0.0;

						ackermann_pub_.publish(ackermann_msg_);
					}				
					is_control_ = true;
				}

				else if( dist < 1.5 && next_mission_state_ == 3) {
					ROS_INFO("PARKING MISSION IS DONE.");
					if(parking_count_ == 1) {

						ackermann_msg_.header.stamp = ros::Time::now();
						ackermann_msg_.drive.speed = 0.0;
						ackermann_msg_.drive.steering_angle = 0.0;

						ackermann_pub_.publish(ackermann_msg_);

						parking_trigger_ = false;
						parking_count_++;
						private_nh_.setParam("/waypoint_loader_node/parking_state", parking_count_);
						is_retrieve_ = true;
						is_backward_=false;					
					}
				}
			
				else if( dist < 5.0 && next_mission_state_ == 4) {
					ROS_INFO("GET BACK TO MAIN LANE");

					if(lane_number_!=0){
					    lane_number_ = 0;
						is_lane_ = false;
						private_nh_.setParam("/waypoint_loader_node/parking_state", -2);
					}
					parking_count_ = -2;
					private_nh_.setParam("/waypoint_loader_node/parking_state", -2);
				}
			
				else if(dist < 8.0 && next_mission_state_ == 5) {
                           	//TODO:정적장애물.//속도느리게해주기~~.

					before_intersection_ = true;
					private_nh_.setParam("/is_intersection",true);
				}

				else if(dist < 7.0 && next_mission_state_ == 6) {
					//Intersection
					//before_intersection_ = true;
					//private_nh_.setParam("/is_intersection",true);
	
				}

				else if(dist<7.0 && next_mission_state_ == 7){
				//Kidszone
					//before_intersection_ = true;
					//private_nh_.setParam("/is_intersection",true);

				}
		    	else if(dist < 7.0 && next_mission_state_ == 8) {
				//TODO: 동적장애물.//속도느리게해주기~~.
				}

				else if (next_mission_state_ == 10){//TODO: waypoints_[0].mission_stae = 9 일떄로 수정 해야함
					if(is_obs_detect_){
						cout<<"lane_number : "<< lane_number_ <<endl;
						cout<<"is_onlane : "<< is_onLane_ <<endl;

						if(static_lane_change_count_ < 2 && is_onLane_){
							if(lane_number_ ==0){
								lane_number_ = 1;
								is_lane_ = false;
								//is_control_ = false;
								static_lane_change_count_ = 1;
								is_onLane_ =false;
								cout<<"ONLANE FALSE"<<endl;

							}
							else if(lane_final_ ==1){

								lane_number_ = 0;
								is_lane_ = false;
								//is_control_ = false;
								static_lane_change_count_ = 2;
								is_onLane_ =false ;
								cout<<"ONLANE FALSE"<<endl;
			
							}
						cout<<"Lane change to"<<lane_number_<< endl;
						cout<<"static_lane_change_count"<<static_lane_change_count_ <<endl;
						}
					}
					else{
						//cout<<"#####None obs######"<<endl;
					}
				}
				/*
		    	else if(dist < 3.0 && next_mission_state_ == 10) {
					//TODO:intersection
					//before_intersection_ = true;
					//private_nh_.setParam("/is_intersection",true);

					ROS_INFO("GOAL POINT READCHED. TERMINATING WAYPOINT FOLLOWER.");//for 학교 운동장 		
					
					is_control_ = false;
					ackermann_msg_.header.stamp = ros::Time::now();
					ackermann_msg_.drive.speed = 0.0;
					ackermann_msg_.drive.steering_angle = 0.0;
                			
					ackermann_pub_.publish(ackermann_msg_);
					
					ros::shutdown();
				}
				*/
		    	else if(dist < 7.0 && next_mission_state_ == 11) {
					//TODO:intersection
					before_intersection_ = true;
					private_nh_.setParam("/is_intersection",true);
				}

		    	else if(dist < 7.0 && next_mission_state_ == 12) {
					//TODO:intersection
					before_intersection_ = true;
					private_nh_.setParam("/is_intersection",true);
				}

		    	else if(dist < 7.0 && next_mission_state_ == 13) {
					//TODO:intersection
						before_intersection_ = true;
						private_nh_.setParam("/is_intersection",true);
				}

				else if(dist < 7.0 && next_mission_state_ == 14) {
					//TODO:intersection
					before_intersection_ = true;
					private_nh_.setParam("/is_intersection",true);
				}
		
				else if(dist < 3.0 && next_mission_state_ == 15){
					ROS_INFO("GOAL POINT READCHED. TERMINATING WAYPOINT FOLLOWER.");		
					
					is_control_ = false;
					ackermann_msg_.header.stamp = ros::Time::now();
					ackermann_msg_.drive.speed = 0.0;
					ackermann_msg_.drive.steering_angle = 0.0;
                			
					ackermann_pub_.publish(ackermann_msg_);
					
					ros::shutdown();
				}
			}

			if(is_control_) {
				double cur_steer = calcSteeringAngle();
			//curvature 계산이 잘 된다면 아래처럼 구간별 제어가 아닌 curvature를 통한 제어를 넣는다.
		
				if(parking_count_== -1){
					speed = decelate_speed_;
					lookahead_dist_= decelate_lookahead_dist_;
				}

				else if((parking_count_== 0)&&(is_backward_==false)){
					speed = parking_speed_;
					lookahead_dist_= parking_lookahead_dist_;
				}
			
				else if(parking_count_ == 1 || is_backward_ ) {
					speed = backward_movement_speed_;
					cur_steer = -cur_steer;
				}

				else if(parking_count_ ==2) {
					speed = decelate_speed_;
					lookahead_dist_=5.0;
				}
		
				else if(before_intersection_) {				//decelerate for traffic_sign detection
					speed = decelate_speed_;
					lookahead_dist_= decelate_lookahead_dist_;
				}

				else if(next_mission_state_ ==10 && is_onLane_){
					speed = 2.0;
					lookahead_dist_= 6.5;
				}
				else if (next_mission_state_ ==10 && !is_onLane_){
					speed = 2.0;
					lookahead_dist_= 4.5;
				}
			
				 else if((waypoints_[0].mission_state)>=4 && (waypoints_[0].mission_state)<=9){		//decelerate for KID ZONE
					speed=decelate_speed_;
					lookahead_dist_=decelate_lookahead_dist_;
				}

				 else if((parking_count_==-2)&&(abs(curvature_) > 0.03||abs(cur_steer) > 13)) { //curvature value need to change
					speed = decelate_speed_; //decel = 2.5
					lookahead_dist_ = decelate_lookahead_dist_;
				}

				else {
					speed = init_speed_;
					lookahead_dist_=init_lookahead_dist_;
				}

				ROS_INFO("SPEED=%f, STEER=%f", speed, cur_steer);

				ackermann_msg_.header.stamp = ros::Time::now();
				ackermann_msg_.drive.speed = speed;
				ackermann_msg_.drive.steering_angle = cur_steer;

				ackermann_pub_.publish(ackermann_msg_);
			}

			parking_trigger_ = false;	
			is_pose_ = false;
			is_course_ = false;
			//is_lane_ = false;
			is_state_change_ = false;
			is_retrieve_ = false;
			before_intersection_ =false;

			index_msg_.waypoint_index = waypoints_[target_index_].waypoint_index;
			index_msg_.lane_number = lane_number_;
			index_msg_.mission_state = waypoints_[target_index_].mission_state;
			index_pub_.publish(index_msg_);
			
			state_msg_.current_state = waypoints_[target_index_].mission_state;
			state_pub_.publish(state_msg_);
		}
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "waypoint_follower");
	WaypointFollower wf;
	
	while(ros::ok()) {
		wf.process();
		ros::spinOnce();
	}
	return 0;
}

