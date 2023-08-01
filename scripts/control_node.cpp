#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <c_turtle/Turtlecontrol.h>

// Variables
turtlesim::Pose pos_msg;
c_turtle::Turtlecontrol control_msg;
geometry_msgs::Twist ref_msg;

bool got_ref = false;
bool got_pose = false;
bool got_control = false;

// Function for receiving turtle reference
void ref_callback(const geometry_msgs::Twist & _data){
	ref_msg = _data;
	got_ref = true;
}

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
	ros::Subscriber ref_sub = nh_.subscribe("/turtle1/ref", 1, ref_callback);
	ros::Publisher cmd_pub = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
	
	// Variables
	int loop_freq = 10;
	ros::Rate loop_rate(loop_freq);
	geometry_msgs::Twist vel_cmd;
	
	while(ros::ok()){
		if(got_pose == true && got_ref == true){//(got_control == true)){
			// Formula for moving turtle towards designated position
//			vel_cmd.linear.x = (control_msg.kp * (control_msg.xd - pos_msg.x));
			vel_cmd.linear.x = (1* (ref_msg.linear.x - pos_msg.x));
			cmd_pub.publish(vel_cmd);
		}
		loop_rate.sleep();
		
		ros::spinOnce();
	}
	return 0;
}
	
