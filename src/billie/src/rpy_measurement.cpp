#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <tf/tf.h>
FILE *rpy;
void keyboardCommands();
void keyCommandIndicator(const std_msgs::String::ConstPtr& msg);
void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void position_trajectory(const ros::TimerEvent& event);
geometry_msgs::TwistStamped velocity;
geometry_msgs::TwistStamped qc_velocity;
geometry_msgs::PoseStamped position;
geometry_msgs::PoseStamped qc_position;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::State current_state;
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;
ros::Publisher position_publisher;
void state_callback(const mavros_msgs::State::ConstPtr& msg);
void doTakeOff();
int velocity_ctrl = 0;
int position_ctrl = 0;
int position_trajectory_begin = 0;
int pid_ctrl = 0;
ros::Time last_keyboard_command;
ros::Time starting_trajectory;
char g_key_command = 'o';
bool g_key_command_allow = true;
double gain_constant = 0;
double waktu_mulai = 0;
double waktu_sekarang = 0;
double position_reff_x = 0;
double position_reff_y = 0;
double current_error_x = 0;
double current_error_y = 0;
double total_error_x = 0;
double total_error_y = 0;
double error_prv_x = 0;
double error_prv_y = 0;
double dt = 0.01;
double current_position_x = 0;
double current_position_y = 0;
double dv = 0;
double current_velocity_x = 0;
double current_velocity_y = 0;
double velocity_prv = 0;
double position_cmd_x = 25;
double position_cmd_y = 15;
const double KEYBOARD_CALL_DURATION = 0.5;
const double SAMPLING_TIME = 0.01;
const double kp = 100;
const double ki = 0.0001;
const double kd = 1200;
const double pi = 3.14159265358979323846;
double duration = 20;
double acceleration = 0;
double velocity_reff_prv = 0;
double roll, pitch, yaw;

int main(int argc, char **argv)
{
    ros::init(argc,argv, "velocity_limit");
	ros::NodeHandle nh;
	position_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
	ros::Subscriber key_command_subscriber = nh.subscribe<std_msgs::String>("key_commands", 1000, keyCommandIndicator);
	ros::Subscriber state_subscriber = nh.subscribe<mavros_msgs::State>("mavros/state",10, state_callback);
	ros::Subscriber velocity_subscriber = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity",100,velocity_callback);
	ros::Subscriber position_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",100,position_callback);
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::Timer position_trajectory_interrupt = nh.createTimer(ros::Duration(SAMPLING_TIME),position_trajectory);
	
	ros::Rate rate(100);
	ROS_INFO_STREAM("Program dimulai ");
	while(ros::ok())
	{
		ros::spinOnce();	
		 
		if(position_ctrl == 1)
		{
			position_publisher.publish(position);

		}
		if(velocity_ctrl == 1)
		{
			velocity_publisher.publish(velocity);
			rate.sleep();
			fprintf(rpy, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",position_reff_x, current_position_x, qc_velocity.twist.linear.x, position_reff_y, current_position_y, qc_velocity.twist.linear.y, roll, pitch, yaw);
		}
		
		//The rate at which the program will be able to receive keyboard commands.
		if (ros::Time::now() - last_keyboard_command > ros::Duration(KEYBOARD_CALL_DURATION)) {
			last_keyboard_command = ros::Time::now();
			keyboardCommands();	
		}

	}
	fclose(rpy);
}

void keyboardCommands(){
	if (g_key_command_allow) {
		switch(g_key_command){
		case 'a':
			ROS_INFO_STREAM("Tombol A telah ditekan");
			position_ctrl = 0;
			velocity_ctrl = 1;
			pid_ctrl = 1;
			g_key_command = 'x';
			position_trajectory_begin = 1;
			starting_trajectory = ros::Time::now();
			waktu_mulai = starting_trajectory.toSec();
			break;
		case ']':
			ROS_INFO_STREAM("Take off ke ketinggian 3 meter");
			doTakeOff();
			break;
		
		case 's':
			ROS_INFO_STREAM("Tombol S telah ditekan");
			position_ctrl = 0;
			velocity_ctrl = 1;
			position_reff_x = 15;
			position_reff_y = 15;
			pid_ctrl = 1;
			g_key_command = 'x';
			break;
		case 'd':
			ROS_INFO_STREAM("Tombol D telah ditekan");
			position_ctrl = 0;
			velocity_ctrl = 1;
//				velocity.twist.angular.z = 0;
			velocity.twist.linear.x = 5;
			pid_ctrl = 0;
			g_key_command = 'x';
			break;

		}
	}
	
	
	return;
}
void doTakeOff()
{
	position_ctrl = 1;
	velocity_ctrl = 0;
	position.pose.position.x = 0;
	position.pose.position.y = 0;
	position.pose.position.z = 3;
	rpy = fopen("rpy.csv","w");
	while(current_state.mode != "OFFBOARD")
	{
			arm_cmd.request.value = true;
			arming_client.call(arm_cmd);
			offb_set_mode.request.custom_mode = "OFFBOARD";
			set_mode_client.call(offb_set_mode);	
			position_publisher.publish(position);
			ros::spinOnce();	
}
}
void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}
void keyCommandIndicator(const std_msgs::String::ConstPtr& msg){
	g_key_command = *msg->data.c_str();
}
void position_trajectory(const ros::TimerEvent& event)
{
	ros::spinOnce();
	if(position_trajectory_begin == 1)
	{	
		waktu_sekarang = ros::Time::now().toSec();
		gain_constant =  waktu_sekarang- waktu_mulai;
		/*
		if(gain_constant < duration) 
		{
			position_reff_x = gain_constant*(position_cmd_x/duration);	
			position_reff_y = gain_constant*(position_cmd_y/duration);	
		}
		*/
		if(gain_constant >= 0 && gain_constant < duration) 
		{
			position_reff_x = position_cmd_x*(exp((10/duration)*(gain_constant-(duration/2)))/(exp((10/duration)*(gain_constant-(duration/2)))+1));	
			position_reff_y = position_cmd_y*(exp((10/duration)*(gain_constant-(duration/2)))/(exp((10/duration)*(gain_constant-(duration/2)))+1));	
		}
		
		else if(gain_constant > duration)
		{
			velocity.twist.linear.x = 0;
			velocity.twist.linear.y = 0;
			position_trajectory_begin = 0;
			position_ctrl = 1;
			velocity_ctrl = 0;
		}
	}
	if(pid_ctrl == 1){
		current_position_x = qc_position.pose.position.x;
		current_error_x = position_reff_x - current_position_x;
		total_error_x = total_error_x + current_error_x*dt;
		dv = (kp * current_error_x) + (ki * total_error_x) + (kd *(current_error_x - error_prv_x));
		error_prv_x = current_error_x;
		velocity.twist.linear.x = dv*dt;
		current_position_y = qc_position.pose.position.y;
		current_error_y = position_reff_y - current_position_y;
		total_error_y = total_error_y + current_error_y*dt;
		dv = (kp * current_error_y) + (ki * total_error_y) + (kd *(current_error_y - error_prv_y));
		error_prv_y = current_error_y;
		velocity.twist.linear.y = dv*dt;
		ROS_INFO_STREAM("Nilai Posisi X sekarang adalah " << current_position_x);
		ROS_INFO_STREAM("Nilai Posisi X reference adalah " << position_reff_x);
		ROS_INFO_STREAM("Nilai posisi Y sekarang adalah " << current_position_y);
		ROS_INFO_STREAM("Nilai posisi Y reference adalah " << position_reff_y);
	}
	
}
	
void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	qc_velocity = *msg;
}
void position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	qc_position = *msg;
	tf::Quaternion q(
        qc_position.pose.orientation.x,
        qc_position.pose.orientation.y,
        qc_position.pose.orientation.z,
        qc_position.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
	roll = roll/pi*180;
	pitch = pitch/pi*180;
	yaw = yaw/pi*180;
}
