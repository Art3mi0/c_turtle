#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <c_turtle/Turtlecontrol.h>
#include <ur5e_control/Plan.h>
#include <stdio.h>
#include <stdlib.h>

// Variables
turtlesim::Pose pos_msg;
c_turtle::Turtlecontrol control_msg;
ur5e_control::Plan plan;
geometry_msgs::Twist point1;
geometry_msgs::Twist point2;
geometry_msgs::Twist point3;
bool got_pose = false;
bool got_control = false;

// Function for receiving turtle position
void pose_callback(const turtlesim::Pose & _data){
	pos_msg = _data;
	got_pose = true;
}

// Function for receiving control data
void control_callback(const c_turtle::Turtlecontrol & _data){
	control_msg = _data;
	got_control = true;
}
	
int main(int argc, char * argv[]){
	// Initialize node
	ros::init(argc, argv, "c_turtle");
	ros::NodeHandle nh_;
	
	// Declare subscribers and publisher
	ros::Subscriber pose_sub = nh_.subscribe("/turtle1/pose", 1, pose_callback);
	ros::Subscriber control_sub = nh_.subscribe("/turtle1/control_params", 1, control_callback);
	ros::Publisher cmd_pub = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
	ros::Publisher plan_pub = nh_.advertise<ur5e_control::Plan>("/plan", 1);
	
	// Variables
	int loop_freq = 10;
	ros::Rate loop_rate(loop_freq);
	geometry_msgs::Twist vel_cmd;
	bool plan_created = false;
	
	while(ros::ok()){
		if(got_pose == true && (got_control == true)){
			// Formula for moving turtle towards designated position
			vel_cmd.linear.x = (control_msg.kp * (control_msg.xd - pos_msg.x));
			cmd_pub.publish(vel_cmd);
		}
		if((got_pose == true) && (plan_created == false)){
			plan_created = true;
			point1.linear.x = pos_msg.x;
			point2.linear.x = 1;
			point3.linear.x = 8;
			plan.points = {point1, point3};
			plan_pub.publish(plan);
		}
		loop_rate.sleep();
		
		ros::spinOnce();
	}
	return 0;
}
	
