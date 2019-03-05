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
#include <unistd.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
FILE *rpy;
void keyboardCommands();
void keyCommandIndicator(const std_msgs::String::ConstPtr& msg);
void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void ll_callback(const nav_msgs::Odometry::ConstPtr& msg);
void position_trajectory(const ros::TimerEvent& event);
ros::Publisher ll_publisher;
geometry_msgs::TwistStamped velocity;
geometry_msgs::TwistStamped qc_velocity;
geometry_msgs::PoseStamped position;
geometry_msgs::PoseStamped qc_position;
nav_msgs::Odometry ll;
nav_msgs::Odometry qc_ll;
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
int init_flag = 1;
ros::Time last_keyboard_command;
ros::Time starting_trajectory;
char g_key_command = 'o';
bool g_key_command_allow = true;
double gain_constant = 0;
double waktu_mulai = 0;
double waktu_sekarang = 0;
double position_reff_x = 0;
double position_reff_y = 0;
double position_reff_x_prv = 0;
double position_reff_y_prv = 0;
double yaw_reff = 0;
double current_error_x = 0;
double current_error_y = 0;
double current_error_yaw = 0;
double total_error_x = 0;
double total_error_y = 0;
double total_error_yaw = 0;
double error_prv_x = 0;
double error_prv_y = 0;
double error_prv_yaw = 0;
double dt = 0.01;
double current_position_x = 0;
double current_position_y = 0;
double current_yaw = 0;
double dv = 0;
double current_velocity_x = 0;
double current_velocity_y = 0;
double current_velocity_yaw = 0;
double velocity_prv = 0;
const double KEYBOARD_CALL_DURATION = 0.5;
const double SAMPLING_TIME = 0.1;
const double kp = 2;
const double ki = 0.02;
const double kd = 0;
const double kp_yaw = 0.005;
const double ki_yaw = 0.0001;
const double kd_yaw = 0;

const double pi = 3.14159265358979323846;
double duration[15];
double acceleration = 0;
double velocity_reff_prv = 0;
double radius = 16;
double roll, pitch, yaw;
double x_awal = 0;
double y_awal = 0;
double absolute_error_x = 0;
double absolute_error_y = 0;
double yaw_prv = 0;
double yaw_adder = 0;
double yaw_reff_adder = 0;
double yaw_reff_prv = 0;
double current_yaw_reff = 0;
double total_mse_x = 0;
double total_mse_y = 0;
double mse_x = 0;
double mse_y = 0;
double n = 1;
double home_latitude = -6.365666;
double home_longitude = 106.821598;
double x_home = 42807.0067371;
double y_home = 704636.131078;
double latitude_cmd[15];
double longitude_cmd[15];
double total_duration = 0;
int cmd_count = 0;
double x_awal_baru = 0;
double y_awal_baru = 0;
double kecepatan_depan;
double kecepatan_depan_prv;
int main(int argc, char **argv)
{
	duration[0] = 90;
	duration[1] = 70;
	duration[2] = 70;
	duration[3] = 120;
	duration[4] = 120;
	duration[5] = 60;
	duration[6] = 44;
	duration[7] = 32;
	duration[8] = 52;
	duration[9] = 80;
	duration[10] = 42;
	duration[11] = 60;
	duration[12] = 40;
	latitude_cmd[0] = -6.361761;
	latitude_cmd[1] = -6.359640;
	latitude_cmd[2] = -6.359629;
	latitude_cmd[3] = -6.363688;
	latitude_cmd[4] = -6.367825;
	latitude_cmd[5] = -6.368902;
	latitude_cmd[6] = -6.370833;
	latitude_cmd[7] = -6.371600;
	latitude_cmd[8] = -6.371483;
	latitude_cmd[9] = -6.368849;
	latitude_cmd[10] = -6.366930;
	latitude_cmd[11] = -6.366774;
	latitude_cmd[12] = home_latitude;
	longitude_cmd[0] = 106.822687;
	longitude_cmd[1] = 106.824955;
	longitude_cmd[2] = 106.827895;
	longitude_cmd[3] = 106.832018;
	longitude_cmd[4] = 106.832052;
	longitude_cmd[5] = 106.831054;
	longitude_cmd[6] = 106.830861;
	longitude_cmd[7] = 106.829896;
	longitude_cmd[8] = 106.827525;
	longitude_cmd[9] = 106.82509;
	longitude_cmd[10] = 106.82504;
	longitude_cmd[11] = 106.822590;
	longitude_cmd[12] = home_longitude;

	ros::init(argc,argv, "gps");
	ros::NodeHandle nh;
	position_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
	ll_publisher = nh.advertise<nav_msgs::Odometry>("/nav_odom",10);
	ros::Subscriber key_command_subscriber = nh.subscribe<std_msgs::String>("key_commands", 1000, keyCommandIndicator);
	ros::Subscriber state_subscriber = nh.subscribe<mavros_msgs::State>("mavros/state",10, state_callback);
	ros::Subscriber velocity_subscriber = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity",100,velocity_callback);
	ros::Subscriber position_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",100,position_callback);
	ros::Subscriber ll_subscriber = nh.subscribe<nav_msgs::Odometry>("/geonav_odom",100,ll_callback);
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
			fprintf(rpy, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f \n",position_reff_x, current_position_x, position_reff_y, current_position_y,qc_velocity.twist.linear.x, qc_velocity.twist.linear.y, current_error_x, current_error_y, yaw_reff, current_yaw, mse_x, mse_y, roll, pitch, kecepatan_depan,acceleration);
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
			if(init_flag == 1)
			{
				ll.pose.pose.position.y = home_latitude; 
				ll.pose.pose.position.x = home_longitude;
				ll.pose.pose.position.z = 20;
				ll_publisher.publish(ll);
				x_awal = qc_ll.pose.pose.position.x;
				y_awal = qc_ll.pose.pose.position.y;
				init_flag = 0;
			}
			break;
		case ']':
			ROS_INFO_STREAM("Take off ke ketinggian 3 meter");
			ll.pose.pose.position.y = home_latitude; 
			ll.pose.pose.position.x = home_longitude;
			ll.pose.pose.position.z = 20;
			ll_publisher.publish(ll);

			doTakeOff();
			break;
		
		case 's':
			ROS_INFO_STREAM("Tombol S telah ditekan");
			position_ctrl = 0;
			velocity_ctrl = 1;
			x_awal = qc_position.pose.position.x;
			y_awal = qc_position.pose.position.y;
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
	position.pose.position.x = 0;
	position.pose.position.y = 0;
	position.pose.position.z = 20;
	rpy = fopen("gps.csv","w");
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
		
		if(cmd_count < 13) // 13 didapat dari jumlah command yang ada
		{
			if(gain_constant < (total_duration + duration[cmd_count]))
			{
				ll.pose.pose.position.y = home_latitude + ((gain_constant-total_duration)/duration[cmd_count])*(latitude_cmd[cmd_count]-home_latitude); 
				ll.pose.pose.position.x = home_longitude + ((gain_constant-total_duration)/duration[cmd_count])*(longitude_cmd[cmd_count]-home_longitude);
				ll.pose.pose.position.z = 20;
				ll_publisher.publish(ll);

				position_reff_x = qc_ll.pose.pose.position.x - x_awal;
				position_reff_y = qc_ll.pose.pose.position.y - y_awal;
				current_yaw_reff = (atan2(position_reff_y-position_reff_y_prv,position_reff_x-position_reff_x_prv)/pi*180);
				yaw_reff = current_yaw_reff + yaw_reff_adder;
				position_reff_x_prv = position_reff_x;
				position_reff_y_prv = position_reff_y;
		if(current_yaw_reff - yaw_reff_prv < -170)
		{
			yaw_reff_adder = yaw_reff_adder + 360;
			yaw_reff = yaw_reff + yaw_reff_adder;
		}
		if(current_yaw_reff - yaw_reff_prv > 170)
		{
			yaw_reff_adder = yaw_reff_adder - 360;
			yaw_reff = yaw_reff + yaw_reff_adder;
		}
		yaw_reff_prv = current_yaw_reff;
			}
			if(gain_constant > (total_duration + duration[cmd_count]))
			{
				home_latitude = latitude_cmd[cmd_count];
				home_longitude = longitude_cmd[cmd_count];
				total_duration = total_duration + duration[cmd_count]; 
				cmd_count++;
			}

/*			if(gain_constant < (total_duration + duration[cmd_count]))
			{
				
				ll.pose.pose.position.y = home_latitude + ((gain_constant-total_duration)/(total_duration+duration[cmd_count]))*(latitude_cmd[cmd_count]-home_latitude); 
				ll.pose.pose.position.x = home_longitude + ((gain_constant-total_duration)/(total_duration+duration[cmd_count]))*(longitude_cmd[cmd_count]-home_longitude);
				ll.pose.pose.position.z = 20;
				ll_publisher.publish(ll);
				position_reff_x = (qc_ll.pose.pose.position.x - x_awal) + x_awal_baru;	
				position_reff_y = (qc_ll.pose.pose.position.y - y_awal) + y_awal_baru;	
			}
			if(gain_constant > (total_duration + duration[cmd_count]))
			{
				x_awal_baru = position_reff_x;
				y_awal_baru = position_reff_y;
				home_latitude = latitude_cmd[cmd_count];
				home_longitude = longitude_cmd[cmd_count];
				total_duration = total_duration + duration[cmd_count]; 
				cmd_count++;
			}
	*/		
			
		}
		else if(cmd_count >= 13) // 13 didapat dari jumlah command yang ada
		{
			velocity.twist.linear.x = 0;
			velocity.twist.linear.y = 0;
			position_trajectory_begin = 0;
			position_ctrl = 1;
			velocity_ctrl = 0;
			position_reff_x = 0;
			position_reff_y = 0;
			init_flag = 1;
			pid_ctrl = 0;
			position.pose.position.x = qc_position.pose.position.x;
			position.pose.position.y = qc_position.pose.position.y;
			position.pose.position.z = qc_position.pose.position.z;
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
		current_position_y = qc_position.pose.position.y;
		current_error_y = position_reff_y - current_position_y;
		total_mse_y = total_mse_y + (current_error_y*current_error_y);
		mse_y = total_mse_y / n;
		n++;
		total_error_y = total_error_y + current_error_y*dt;
		dv = (kp * current_error_y) + (ki * total_error_y)*dt + (kd *(current_error_y - error_prv_y)/dt);
		error_prv_y = current_error_y;
		velocity.twist.linear.y = dv;
		kecepatan_depan = sqrt(fabs(pow(qc_velocity.twist.linear.x,2)+pow(qc_velocity.twist.linear.y,2)));
		acceleration = (kecepatan_depan-kecepatan_depan_prv)/dt;
		kecepatan_depan_prv = kecepatan_depan;
		position_reff_x_prv = position_reff_x;
		position_reff_y_prv = position_reff_y;
		current_yaw = yaw + yaw_adder;
		if(yaw - yaw_prv < -170)
		{
			yaw_adder = yaw_adder + 360;
			current_yaw = current_yaw + yaw_adder;
		}
		if(yaw - yaw_prv > 170)
		{
			yaw_adder = yaw_adder - 360;
			current_yaw = current_yaw + yaw_adder;
		}
		yaw_prv = yaw;
		current_error_yaw = yaw_reff - current_yaw;
		total_error_yaw = total_error_yaw + current_error_yaw*dt;
		dv = (kp_yaw * current_error_yaw) + (ki_yaw * total_error_yaw)*dt + (kd_yaw *(current_error_yaw - error_prv_yaw)/dt);
		error_prv_yaw = current_error_yaw;
		velocity.twist.angular.z = dv;

		ROS_INFO_STREAM("Posisi X reff adalah " << position_reff_x);
		ROS_INFO_STREAM("Posisi Y reff adalah " << position_reff_y);
		ROS_INFO_STREAM("Posisi X sekarang adalah " << qc_position.pose.position.x);
		ROS_INFO_STREAM("Posisi Y sekarang adalah " << qc_position.pose.position.y);
		ROS_INFO_STREAM("Waktu sekarang adalah " << gain_constant);
		ROS_INFO_STREAM("Waypoint ke - " << cmd_count);
		ROS_INFO_STREAM("Nilai cmd count adalah " << cmd_count);
		ROS_INFO_STREAM("Nilai x cmd adalah " << (qc_ll.pose.pose.position.x - x_awal - x_awal_baru) + x_awal_baru);
		ROS_INFO_STREAM("Nilai y cmd adalah " << (qc_ll.pose.pose.position.y - y_awal - y_awal_baru) + y_awal_baru);
		ROS_INFO_STREAM("Nilai yaw reff adalah " << yaw_reff);
		ROS_INFO_STREAM("Nilai current yaw adalah " << current_yaw);
	}
}
	
void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	qc_velocity = *msg;
}
void ll_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	qc_ll = *msg;
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
