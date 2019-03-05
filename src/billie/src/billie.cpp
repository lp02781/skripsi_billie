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
#include <std_msgs/Float64.h>
namespace plant_sim
{
  // Global so it can be passed from the callback fxn to main
  static double control_effort = 0.0;
  static bool reverse_acting = false;
}
using namespace plant_sim;
FILE *velocity_data;
void keyboardCommands();
void keyCommandIndicator(const std_msgs::String::ConstPtr& msg);
void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void velocity_trajectory(const ros::TimerEvent& event);
void ControlEffortCallback(const std_msgs::Float64& control_effort_input);
geometry_msgs::TwistStamped velocity;
geometry_msgs::TwistStamped qc_velocity;
geometry_msgs::PoseStamped position;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::State current_state;
std_msgs::Float64 setpoint;
std_msgs::Float64 plant_state;

ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;
ros::Publisher position_publisher;
ros::Publisher setpoint_pub;
ros::Publisher servo_state_pub;
void state_callback(const mavros_msgs::State::ConstPtr& msg);
void doTakeOff();
int velocity_ctrl = 0;
int position_ctrl = 0;
int velocity_trajectory_begin = 0;
ros::Time last_keyboard_command;
ros::Time starting_trajectory;
char g_key_command;
bool g_key_command_allow = true;
double gain_constant = 0;
double waktu_mulai = 0;
double waktu_sekarang = 0;
double velocity_reff = 0;
double current_error = 0;
double total_error = 0;
double error_prv = 0;
double dt = 0.01;
double current_velocity = 0;
double dv = 0;
const double KEYBOARD_CALL_DURATION = 0.5;
const double SAMPLING_TIME = 0.01;

int main(int argc, char **argv)
{
	ros::init(argc,argv, "velocity_trapezium");
	ros::NodeHandle nh;
	position_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
	setpoint_pub = nh.advertise<std_msgs::Float64>("setpoint",1);
	ros::Subscriber key_command_subscriber = nh.subscribe<std_msgs::String>("key_commands", 1000, keyCommandIndicator);
	ros::Subscriber state_subscriber = nh.subscribe<mavros_msgs::State>("mavros/state",10, state_callback);
	ros::Subscriber velocity_subscriber = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity",100,velocity_callback);
	ros::Subscriber sub = nh.subscribe("control_effort", 1, ControlEffortCallback );
	servo_state_pub = nh.advertise<std_msgs::Float64>("state", 1);
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::Timer velocity_trajectory_interrupt = nh.createTimer(ros::Duration(SAMPLING_TIME),velocity_trajectory);
	
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
			fprintf(velocity_data, "%f, %f, %f\n",velocity_reff, current_velocity, velocity.twist.linear.x);
	
		}
		
		//The rate at which the program will be able to receive keyboard commands.
		if (ros::Time::now() - last_keyboard_command > ros::Duration(KEYBOARD_CALL_DURATION)) {
			last_keyboard_command = ros::Time::now();
			keyboardCommands();	
		}

	}
	fclose(velocity_data);
}

void keyboardCommands(){
	if (g_key_command_allow) {
		switch(g_key_command){

		case 'a':
			ROS_INFO_STREAM("Tombol A telah ditekan");
			position_ctrl = 0;
			velocity_ctrl = 1;
			velocity_trajectory_begin = 1;
			starting_trajectory = ros::Time::now();
			waktu_mulai = starting_trajectory.toSec();
			g_key_command = 'x';
			break;
		case ']':
			ROS_INFO_STREAM("Take off ke ketinggian 3 meter");
			doTakeOff();
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
	velocity_data = fopen("velocity_data.dat","w");
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
void velocity_trajectory(const ros::TimerEvent& event)
{
	ros::spinOnce();
	if(velocity_trajectory_begin == 1)
	{	
		waktu_sekarang = ros::Time::now().toSec();
		gain_constant =  waktu_sekarang- waktu_mulai;
		if(gain_constant < 5)
		{
			velocity_reff = 0;
		}
		else if(gain_constant >= 5 && gain_constant < 15) 
		{
			velocity_reff = exp(gain_constant-10)/(exp(gain_constant-10)+1);	
		}
		else if(gain_constant >= 15 && gain_constant <=25)
		{
			velocity_reff = 1;
		}
		else if(gain_constant > 25 && gain_constant <= 35)
		{
			velocity_reff = 1 - (exp(gain_constant-30)/(exp(gain_constant-30)+1));			
		}
		else if(gain_constant > 35 && gain_constant <= 40)
		{
			velocity_reff = 0;
		}
		else if(gain_constant > 40)
		{
			velocity_reff = 0;
			velocity.twist.linear.x = 0;
			velocity_trajectory_begin = 0;
			position_ctrl = 1;
			velocity_ctrl = 0;
		}
		setpoint.data = velocity_reff;
		setpoint_pub.publish(setpoint);
		current_velocity = qc_velocity.twist.linear.x;
		plant_state.data = current_velocity;
		servo_state_pub.publish(plant_state);
		velocity.twist.linear.x = current_velocity + control_effort;
		
		
		ROS_INFO_STREAM("besarnya kecepatan kendali " << velocity.twist.linear.x);
		ROS_INFO_STREAM("besarnya referensi adalah " << velocity_reff);
		ROS_INFO_STREAM("besarnya kecepatan sekarang " << current_velocity);

		
	}
}
	
void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	qc_velocity = *msg;
}

void ControlEffortCallback(const std_msgs::Float64& control_effort_input)
{
  // the stabilizing control effort
  if (reverse_acting)
  {
    control_effort = -control_effort_input.data;
  }
  else
  {
    control_effort = control_effort_input.data;
  }
}

