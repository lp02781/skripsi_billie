#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <tf/tf.h>
#include <sensor_msgs/NavSatFix.h>
FILE *rpy;
void keyboardCommands();
void keyCommandIndicator(const std_msgs::String::ConstPtr& msg);
void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
/*void rotor1_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void rotor0_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void rotor2_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void rotor3_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);*/
void position_trajectory(const ros::TimerEvent& event);
geometry_msgs::TwistStamped velocity;
geometry_msgs::TwistStamped qc_velocity;
geometry_msgs::PoseStamped position;
geometry_msgs::PoseStamped qc_position;
/*geometry_msgs::PoseStamped rotor0_position;
geometry_msgs::PoseStamped rotor1_position;
geometry_msgs::PoseStamped rotor2_position;
geometry_msgs::PoseStamped rotor3_position;
*/mavros_msgs::CommandBool arm_cmd;
mavros_msgs::CommandTOL land_cmd;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::State current_state;
sensor_msgs::NavSatFix qc_gps;
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;
ros::ServiceClient landing;
ros::Publisher position_publisher;
void state_callback(const mavros_msgs::State::ConstPtr& msg);
void doTakeOff();
int velocity_ctrl = 0;
int position_ctrl = 0;
int position_trajectory_begin = 0;
int pid_ctrl = 0;
int init_flag = 1;
ros::Time last_keyboard_command;
ros::Time starting_trajectory;
char g_key_command = 'o';
bool g_key_command_allow = true;
double gain_constant = 0;
double waktu_mulai = 0;
double waktu_sekarang = 0;
double longitude, latitude,altitude;
double position_reff_x = 0;
double position_reff_z = 3;
double current_error_x = 0;
double current_error_z = 0;
double total_error_x = 0;
double total_error_z = 0;
double error_prv_x = 0;
double error_prv_z = 0;
double dt = 0.01;
double current_position_x = 0;
double current_position_z = 0;
double dv = 0;
double current_velocitzzy_x = 0;
double current_velocity_z = 0;
double velocity_prv = 0;
const double KEYBOARD_CALL_DURATION = 0.5;
const double SAMPLING_TIME = 0.01;
const double kp = 4.5;
const double ki = 2.5;
const double kd = 1;
const double pi = 3.14159265358979323846;
double acceleration = 0;
double velocity_reff_prv = 0;
double radius = 16;
double roll, pitch, yaw;
double rotor0_yaw = 0;
double rotor1_yaw = 0;
double rotor2_yaw = 0;
double rotor3_yaw = 0;
double x_awal = 0;
double z_awal = 3;
double absolute_error_x = 0;
double absolute_error_z = 0;
double total_mse_x = 0;
double total_mse_z = 0;
double mse_x = 0;
double mse_z = 0;
double n = 1;
double position_cmd_x = 0;
double position_cmd_z = 10;
double take_off_duration = 7;
double constant_duration = 0;
double duration = 30;
int main(int argc, char **argv)
{
	ros::init(argc,argv, "power_consumption");
	ros::NodeHandle nh;
	position_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
	ros::Subscriber key_command_subscriber = nh.subscribe<std_msgs::String>("key_commands", 1000, keyCommandIndicator);
	ros::Subscriber state_subscriber = nh.subscribe<mavros_msgs::State>("mavros/state",10, state_callback);
	ros::Subscriber velocity_subscriber = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity",100,velocity_callback);
/*	ros::Subscriber rotor0_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("/rotor0",100,rotor0_callback);
	ros::Subscriber rotor1_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("/rotor1",100,rotor1_callback);
	ros::Subscriber rotor2_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("/rotor2",100,rotor2_callback);
	ros::Subscriber rotor3_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("/rotor3",100,rotor3_callback);*/
	ros::Subscriber position_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",100,position_callback);
	ros::Subscriber gps_subscriber = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global",100,gps_callback);
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	landing = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/landing");
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
		}
		
		//The rate at which the program will be able to receive keyboard commands.
		if (ros::Time::now() - last_keyboard_command > ros::Duration(KEYBOARD_CALL_DURATION)) {
			last_keyboard_command = ros::Time::now();
			keyboardCommands();	
		}

	}
}

void keyboardCommands(){
	if (g_key_command_allow) {
		switch(g_key_command){
		case 'a':
			rpy = fopen("power_consumption.csv","w");
			ROS_INFO_STREAM("Tombol A telah ditekan");
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
			x_awal = qc_position.pose.position.x;
			z_awal = qc_position.pose.position.z;
			pid_ctrl = 1;
			position_trajectory_begin = 1;
			starting_trajectory = ros::Time::now();
			waktu_mulai = starting_trajectory.toSec();
			g_key_command = 'x';
			break;
		case 'd':
			ROS_INFO_STREAM("Tombol D telah ditekan");
			position_ctrl = 0;
			velocity_ctrl = 1;
				velocity.twist.angular.z = 0;
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
		fprintf(rpy, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f \n",position_reff_x, qc_position.pose.position.x, position_reff_z, qc_position.pose.position.z, current_error_x, current_error_z, mse_x, mse_z, roll, pitch);
		waktu_sekarang = ros::Time::now().toSec();
		gain_constant =  waktu_sekarang- waktu_mulai;
		if(gain_constant < take_off_duration) 
		{
			position_reff_x = 0;
			position_reff_z = 3;
			position_ctrl = 0;
			velocity_ctrl = 1;
			pid_ctrl = 1;
			while(current_state.mode != "OFFBOARD")
				{
			arm_cmd.request.value = true;
			arming_client.call(arm_cmd);
			offb_set_mode.request.custom_mode = "OFFBOARD";
			set_mode_client.call(offb_set_mode);	
			position_publisher.publish(position);
			ros::spinOnce();	
		}
		x_awal = qc_position.pose.position.x;
		z_awal = qc_position.pose.position.z;
		}
		
		if(gain_constant < (duration+take_off_duration) && gain_constant > take_off_duration) 
		{
			pid_ctrl = 1;
			velocity_ctrl = 1;
			position_ctrl = 0;
			position_reff_x = x_awal + (gain_constant-take_off_duration)/(duration)*(position_cmd_x);	
			position_reff_z = z_awal + (gain_constant-take_off_duration)/(duration)*(position_cmd_z);	
		}
	    if(gain_constant < (duration+take_off_duration+constant_duration) && gain_constant > (take_off_duration + duration)) 
		{
			pid_ctrl = 1;
			velocity_ctrl = 1;
			position_ctrl = 0;
			position_reff_x = x_awal + (gain_constant-take_off_duration)/(duration+constant_duration)*(position_cmd_x);	
		}
		else if(gain_constant > duration+take_off_duration+constant_duration)
		{
			fclose(rpy);

			while(ros::ok()){

			offb_set_mode.request.custom_mode = "AUTO.LAND";
			set_mode_client.call(offb_set_mode);	
			}
			velocity.twist.linear.x = 0;
			velocity.twist.linear.z = 0;
			position_trajectory_begin = 0;
			position_ctrl = 0;
			velocity_ctrl = 0;
			position_reff_x = 0;
			position_reff_z = 0;
			init_flag = 1;
			pid_ctrl = 0;
			g_key_command = 'x';
		}
	}
	if(pid_ctrl == 1){
		current_position_x = qc_position.pose.position.x;
		current_error_x = position_reff_x - current_position_x;
		total_mse_x = total_mse_x + (current_error_x*current_error_x);
		mse_x = total_mse_x / n;
		total_error_x = total_error_x + current_error_x*dt;
		dv = (kp * current_error_x) + (ki * total_error_x * dt) + (kd *(current_error_x - error_prv_x)/dt);
		error_prv_x = current_error_x;
		velocity.twist.linear.x = dv;
		current_position_z = qc_position.pose.position.z;
		current_error_z = position_reff_z - current_position_z;
		total_mse_z = total_mse_z + (current_error_z*current_error_z);
		mse_z = total_mse_z / n;
		n++;
		total_error_z = total_error_z + current_error_z*dt;
		dv = (kp * current_error_z) + (ki * total_error_z)*dt + (kd *(current_error_z - error_prv_z)/dt);
		error_prv_z = current_error_z;
		velocity.twist.linear.z = dv;
		ROS_INFO_STREAM("Nilai Posisi X sekarang adalah " << current_position_x);
		ROS_INFO_STREAM("Nilai Posisi X reference adalah " << position_reff_x);
		ROS_INFO_STREAM("Nilai posisi Z sekarang adalah " << current_position_z);
		ROS_INFO_STREAM("Nilai posisi Z reference adalah " << position_reff_z);
		ROS_INFO_STREAM("Nilai MSE X adalah " << mse_x);
		ROS_INFO_STREAM("Nilai MSE Z adalah " << mse_z);
		ROS_INFO_STREAM("Nilai gain constant adalah " << gain_constant);

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
/*
void rotor0_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	rotor0_position = *msg;
	tf::Quaternion qrotor0(
        rotor0_position.pose.orientation.x,
        rotor0_position.pose.orientation.y,
        rotor0_position.pose.orientation.z,
        rotor0_position.pose.orientation.w);
    tf::Matrix3x3 mrotor0(qrotor0);
    mrotor0.getRPY(roll, pitch, rotor0_yaw);
	rotor0_yaw = rotor0_yaw/pi*180;
}
void rotor1_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	rotor1_position = *msg;
	tf::Quaternion qrotor1(
        rotor1_position.pose.orientation.x,
        rotor1_position.pose.orientation.y,
        rotor1_position.pose.orientation.z,
        rotor1_position.pose.orientation.w);
    tf::Matrix3x3 mrotor1(qrotor1);
    mrotor1.getRPY(roll, pitch, rotor1_yaw);
	rotor1_yaw = rotor1_yaw/pi*180;
}
void rotor2_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	rotor2_position = *msg;
	tf::Quaternion qrotor2(
        rotor2_position.pose.orientation.x,
        rotor2_position.pose.orientation.y,
        rotor2_position.pose.orientation.z,
        rotor2_position.pose.orientation.w);
    tf::Matrix3x3 mrotor2(qrotor2);
    mrotor2.getRPY(roll, pitch, rotor2_yaw);
	rotor2_yaw = rotor2_yaw/pi*180;
}
void rotor3_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	rotor3_position = *msg;
	tf::Quaternion qrotor3(
        rotor3_position.pose.orientation.x,
        rotor3_position.pose.orientation.y,
        rotor3_position.pose.orientation.z,
        rotor3_position.pose.orientation.w);
    tf::Matrix3x3 mrotor3(qrotor3);
    mrotor3.getRPY(roll, pitch, rotor3_yaw);
	rotor3_yaw = rotor3_yaw/pi*180;
}
*/
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
		qc_gps = *msg;	
}
