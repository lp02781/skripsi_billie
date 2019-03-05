//TODO vectorized the variables used to define position, the code can be made shorter, and it can save memory
//TODO function documentation

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

FILE *trajectory_planning_data;
FILE *controller_data;
FILE *quadrotor_position_data;
FILE *quadrotor_velocity_data;

//Constants
const double G_FIRST_CONTROL_TRIGGER_PERIOD = 10.0;//interval between program start and control activating
const double G_ARM_CALL_DURATION = 0.5;
const double G_KEYBOARD_COMMAND_CALL_DURATION = 0.5;//Keyboard command interval between different commands
const double G_CURRENT_POSITION_PRINT_DURATION = 0.5;//display current position at terminal
const double G_CONTROL_LOOP_INTERVAL_DURATION = 0.030;//control loop update time in s
const double G_Z_POSITION_LIMIT_UPPER = 10.0;
const double G_Z_POSITION_LIMIT_LOWER = 3.0;

//Position Controller PID Parameters
const double G_KP_POSITIONCONTROL_X = 1;//1
const double G_KI_POSITIONCONTROL_X = 0.58;//0.58
const double G_KD_POSITIONCONTROL_X = 1.6;//1.6
const double G_KP_POSITIONCONTROL_Y = 1;
const double G_KI_POSITIONCONTROL_Y = 0.58;
const double G_KD_POSITIONCONTROL_Y = 1.4;
const double G_KP_POSITIONCONTROL_Z = 1;
const double G_KI_POSITIONCONTROL_Z = 1.5;
const double G_KD_POSITIONCONTROL_Z = 0.3;

//Velocity Controller PID Parameters
const double G_KP_VELOCITYCONTROL_X = 1;//9/3/3
const double G_KI_VELOCITYCONTROL_X = 0;//11/1.5/4
const double G_KD_VELOCITYCONTROL_X = 0.;//2/1/2.2
const double G_KP_VELOCITYCONTROL_Y = 1;
const double G_KI_VELOCITYCONTROL_Y = 0;
const double G_KD_VELOCITYCONTROL_Y = 0;
const double G_KP_VELOCITYCONTROL_Z = 1;
const double G_KI_VELOCITYCONTROL_Z = 0;
const double G_KD_VELOCITYCONTROL_Z = 0;

const double NODE_RUN_RATE = 100.0;//Publish loop_rate, must be faster than 2Hz
const double DT = G_CONTROL_LOOP_INTERVAL_DURATION;//sample time according to control loop interval duration
const double PI = 3.14159265;
const double G_LANDING_Z_POSITION = 0.3;

//Global Variables
mavros_msgs::State g_current_state;
mavros_msgs::SetMode g_position_control_set_mode;
geometry_msgs::PoseStamped g_position_command;
geometry_msgs::PoseStamped g_position_current_fcu;
geometry_msgs::TwistStamped g_velocity_command;
geometry_msgs::TwistStamped g_velocity_current_fcu;
mavros_msgs::CommandBool g_arming_command;
ros::Time g_last_arm_request;
ros::Time g_last_keyboard_command;
ros::Time g_last_current_position_print;
ros::Time g_control_loop_last_execute_time;
ros::Time g_first_control_trigger_time;
std_msgs::String g_key_command_received;

int g_trajectory_planning_mode = 0;
//mode 0: sigmoid trajectory
//mode 1: sin wave trajectory
//mode 2: zigzag wave trajectory
//mode 3: step signal trajectory control
//mode 4: Ramp signal trajectory control
//mode 5: smooth ramp signal trajectory
bool g_controller_active = true;// to activate / deactivate the controller(command goes straight to pose)

bool g_key_any = false;
bool g_shutdown_indicator = false;
char g_key_command;
char g_current_flight_mode[20];
char g_flight_mode_command[20];

double g_x_position_reference_previous = 0;
double g_y_position_reference_previous = 0;
double g_z_position_reference_previous = 0;
double g_x_velocity_manualcount = 0;
double g_y_velocity_manualcount = 0;
double g_z_velocity_manualcount = 0;

double g_time_command = 0;
double g_time_final = 0;
double g_time_delta = 0;
double g_time_difference = 0;
double g_time_current = 0;
double g_time_initial = 0;
double g_time_init = 0;
double g_time_last_command = 0;
double g_data_timestamp = 0;
double g_data_timestamp_old = 0;

//used to calculate the multiplier for the command value of x y and z
double g_delta_command_multiplier = 0;
double g_delta_command_dot_multiplier = 0;
double g_delta_command_doubledot_multiplier = 0;
double g_delta_command_integral_multiplier = 0;

bool g_x_command_changed_indicator_linear = false;
bool g_x_command_changed_indicator_wave = false;
bool g_y_command_changed_indicator_linear = false;
bool g_y_command_changed_indicator_wave = false;
bool g_z_command_changed_indicator_linear = false;
bool g_z_command_changed_indicator_wave = false;

//variables for position control
double g_x_position_reference = 0;
double g_x_position_delta_command = 0;
double g_x_position_initial = 0;
double g_x_position_error = 0;
double g_x_position_error_previous = 0;
double g_x_position_error_derivative = 0;
double g_x_position_error_integral = 0;
double g_y_position_reference = 0;
double g_y_position_delta_command = 0;
double g_y_position_initial = 0;
double g_y_position_error = 0;
double g_y_position_error_previous = 0;
double g_y_position_error_derivative = 0;
double g_y_position_error_integral = 0;
double g_z_position_reference = 0;
double g_z_position_delta_command = 0;
double g_z_position_initial = 0;
double g_z_position_error = 0;
double g_z_position_error_previous = 0;
double g_z_position_error_derivative = 0;
double g_z_position_error_integral = 0;
double g_x_position_fcu_current = 0;
double g_y_position_fcu_current = 0;
double g_z_position_fcu_current = 0;
double g_x_position_control_signal = 0;
double g_y_position_control_signal = 0;
double g_z_position_control_signal = 0;
double g_x_position_control_signal_previous = 0;
double g_y_position_control_signal_previous = 0;
double g_z_position_control_signal_previous = 0;
double g_x_position_control_signal_derivative = 0;
double g_y_position_control_signal_derivative = 0;
double g_z_position_control_signal_derivative = 0;
double position_reference_x = 0;
double position_reference_y = 0;
double position_reference_z = 0;
double key = 1;
//variables for velocity control
double g_x_velocity_reference = 0;
double g_x_velocity_delta_command = 0;
double g_x_velocity_initial = 0;
double g_x_velocity_error = 0;
double g_x_velocity_error_previous = 0;
double g_x_velocity_error_derivative = 0;
double g_x_velocity_error_integral = 0;
double g_y_velocity_reference = 0;
double g_y_velocity_delta_command = 0;
double g_y_velocity_initial = 0;
double g_y_velocity_error = 0;
double g_y_velocity_error_previous = 0;
double g_y_velocity_error_derivative = 0;
double g_y_velocity_error_integral = 0;
double g_z_velocity_reference = 0;
double g_z_velocity_delta_command = 0;
double g_z_velocity_initial = 0;
double g_z_velocity_error = 0;
double g_z_velocity_error_previous = 0;
double g_z_velocity_error_derivative = 0;
double g_z_velocity_error_integral = 0;
double g_x_velocity_fcu_current = 0;
double g_y_velocity_fcu_current = 0;
double g_z_velocity_fcu_current = 0;
double g_x_velocity_control_signal = 0;
double g_y_velocity_control_signal = 0;
double g_z_velocity_control_signal = 0;

//Period and frequency parameter used for the wave trajectory
double g_wave_trajectory_period = 1;//period of every wave motion
double g_wave_trajectory_halfperiod = 0.5;
double g_wave_trajectory_amplitude = 1;
double g_wave_trajectory_amplitude_x = 0;
double g_wave_trajectory_amplitude_y = 0;
double g_wave_trajectory_amplitude_z = 0;
double g_wave_delta = 0;
double g_wave_time = 0;
double g_wave_trajectory_amplitude_default = 0.5;//default setting

double g_initial_position_z = 0;

bool g_first_run = true;
bool g_key_command_allow = true;

//For ramp calculation
double g_ramp_trajectory_x_gradient = 0;
double g_ramp_trajectory_y_gradient = 0;
double g_ramp_trajectory_z_gradient = 0;
double g_ramp_trajectory_time_multiplier = 0;

//for smooth ramp calculation
double g_smoothramp_time_speedincrease_duration = 0;
double g_smoothramp_time_speeddecrease_duration = 0;
double g_smoothramp_time_speedincrease_endperiod = 0;
double g_smoothramp_time_speeddecrease_startperiod = 0;
double g_smoothramp_x_maxvelocity = 0;
double g_smoothramp_x_speedgradient = 0;
double g_smoothramp_y_maxvelocity = 0;
double g_smoothramp_y_speedgradient = 0;
double g_smoothramp_z_maxvelocity = 0;
double g_smoothramp_z_speedgradient = 0;
double g_smoothramp_x_velocity = 0;
double g_smoothramp_x_velocity_previous = 0;
double g_smoothramp_y_velocity = 0;
double g_smoothramp_y_velocity_previous = 0;
double g_smoothramp_z_velocity = 0;
double g_smoothramp_z_velocity_previous = 0;
double sample_time;

//Prototype functions
void stateIndicator(const mavros_msgs::State::ConstPtr& msg);
void keyCommandIndicator(const std_msgs::String::ConstPtr& msg);
void currentFCUPositionIndicator(const geometry_msgs::PoseStamped::ConstPtr& msg);
void currentFCUVelocityIndicator(const geometry_msgs::TwistStamped::ConstPtr& msg);
void keyboardCommands();
void initiateTrajectoryPlanning();
void trajectoryPlanning();
void runControlAlgorithm(const ros::TimerEvent& event);
void positionControlLoop();
void velocityControlLoop();
void positionCommandCalculation();
void positionCommandFinalValueCalculationX();
void positionCommandFinalValueCalculationY();
void positionCommandFinalValueCalculationZ();
void resetDeltaPositionCommand();
void trajectoryDeltaWaveCalculation();
void waveCurrentTimeCalculation();
void resetKeyboardCommand();
void initializeTakeoff();
void initializeLanding();
void resetInitialValues();
void disarmQuadrotor();
void recordData();
void velocityCommandCalculation();
void rampCommandCalculation();
void squareCommandCalculation();
void stepCommandCalculation();
void sigmoidCommandCalculation();
void sinWaveCommandCalculation();
void zigzagCommandCalculation();
void smoothrampCommandCalculation();
void callback1(const ros::TimerEvent&);

int main(int argc, char **argv){
	ros::init(argc, argv,"position_control");//Initialize ros, and also naming the node
	ros::NodeHandle node_handle;//Name the node's handle
	
	//Initialize subscriber, publisher, and services used for this node
	ros::Subscriber key_command_subscriber = node_handle.subscribe<std_msgs::String>
		("key_commands", 1000, keyCommandIndicator);
	ros::Publisher key_command_received_publisher = node_handle.advertise<std_msgs::String>
		("key_command_receive",1000);
	ros::Subscriber state_subscriber = node_handle.subscribe<mavros_msgs::State>
		("mavros/state", 10, stateIndicator);
	ros::Publisher position_command_publisher = node_handle.advertise<geometry_msgs::PoseStamped>
		("mavros/setpoint_position/local", 10);
	ros::Publisher velocity_command_publisher = node_handle.advertise<geometry_msgs::TwistStamped>
		("mavros/setpoint_velocity/cmd_vel",10);
	ros::ServiceClient arming_client = node_handle.serviceClient<mavros_msgs::CommandBool>
		("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = node_handle.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode");
	ros::Subscriber current_position_FCU_subscriber = node_handle.subscribe<geometry_msgs::PoseStamped>
		("mavros/local_position/pose", 10, currentFCUPositionIndicator);
	ros::Subscriber current_velocity_FCU_subscriber = node_handle.subscribe<geometry_msgs::TwistStamped>
		("mavros/local_position/velocity", 10, currentFCUVelocityIndicator);

	//Initial start
	puts("\nPackage successfully starts!\n");
	ros::Rate usleep(0.5);
	ros::Rate loop_rate(NODE_RUN_RATE);
	while (ros::ok() && g_current_state.connected) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	puts("MAVROS connected to PX4!\n");
	//Setting up the timers used for interrupt
	ros::Timer position_control_interrupt = node_handle.createTimer(ros::Duration(G_CONTROL_LOOP_INTERVAL_DURATION), runControlAlgorithm);
	while(ros::ok()){//Main program loop
		position_command_publisher.publish(g_position_command);

		//The rate at which the program will be able to receive keyboard commands.
		if (ros::Time::now() - g_last_keyboard_command > ros::Duration(G_KEYBOARD_COMMAND_CALL_DURATION)) {
			g_last_keyboard_command = ros::Time::now();
			keyboardCommands();
			if(g_shutdown_indicator){// node shutdown command
				if (g_arming_command.request.value) {
					puts("\nNode Shutdown Rejected! Land and disarm first!");
					resetKeyboardCommand();
					g_shutdown_indicator = false;
				}
				else if (!g_arming_command.request.value) {
					//Stopping the node from running
					key_command_received_publisher.publish(g_key_command_received);
					puts("Node shutting down...\n");
					ros::shutdown();
				}
			}
		}

		if(ros::Time::now() - g_last_arm_request > ros::Duration(G_ARM_CALL_DURATION)){
			g_last_arm_request = ros::Time::now();
			if (g_current_state.mode != g_flight_mode_command) {
				set_mode_client.call(g_position_control_set_mode);
			}
			if (g_current_state.armed != g_arming_command.request.value){
				arming_client.call(g_arming_command);
			}
		}

		if ((ros::Time::now() - g_last_current_position_print > ros::Duration(G_CURRENT_POSITION_PRINT_DURATION))){
			g_last_current_position_print = ros::Time::now();
			printf("\n-----------------------------------------\n");
			if (!g_key_any) {
				printf("\nNo key press!\n");
			}
			else if (g_key_any) {
				printf("\nKey press started!\n");
			}
			if (!g_controller_active || !g_key_any) {
				printf("\nController not active!\n");
			}
			else if (g_controller_active) {
				printf("\nController active!\n");
			}
			printf("\nCurrent Quadcopter Position (from FCU):\n\nX: %f\nY: %f\nZ: %f\n",
				g_position_current_fcu.pose.position.x, g_position_current_fcu.pose.position.y, g_position_current_fcu.pose.position.z);
			ROS_INFO_STREAM(g_current_state);
			printf("\nCurrent Quadcopter Position Command:\n\nX: %f\nY: %f\nZ: %f\n", g_x_position_reference, g_y_position_reference, g_z_position_reference);
			printf("\n-----------------------------------------\n");
		}

		key_command_received_publisher.publish(g_key_command_received);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return(0);
}

void recordData(){
	 g_data_timestamp_old = g_data_timestamp;
	 g_data_timestamp = g_time_current - g_time_last_command;
	 sample_time = g_data_timestamp - g_data_timestamp_old;
	 velocityCommandCalculation();
	 //Obtaining current velocity
	 g_x_velocity_fcu_current = g_velocity_current_fcu.twist.linear.x;
	 g_y_velocity_fcu_current = g_velocity_current_fcu.twist.linear.y;
	 g_z_velocity_fcu_current = g_velocity_current_fcu.twist.linear.z;
	 fprintf(trajectory_planning_data, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
	 	g_data_timestamp, g_time_initial, g_time_final, g_wave_time, g_time_command, g_delta_command_multiplier, g_wave_delta, g_x_position_reference, g_y_position_reference, g_z_position_reference);
	fprintf(quadrotor_position_data,"%f, %f, %f, %f, %f, %f, %f\n",
		g_data_timestamp, g_x_position_reference, g_y_position_reference, g_z_position_reference, g_position_current_fcu.pose.position.x, g_position_current_fcu.pose.position.y, g_position_current_fcu.pose.position.z);
	 fprintf(controller_data, "%f, %f, %f, %f, %f, %f, %f\n",
	 	g_data_timestamp, g_x_position_error, g_y_position_error, g_z_position_error, g_position_command.pose.position.x, g_position_command.pose.position.y, g_position_command.pose.position.z);
	 fprintf(quadrotor_velocity_data, "%f, %f, %f, %f, %f, %f, %f\n",
	 	g_data_timestamp, g_x_velocity_manualcount, g_y_velocity_manualcount, g_z_velocity_manualcount, g_velocity_current_fcu.twist.linear.x, g_velocity_current_fcu.twist.linear.y, g_velocity_current_fcu.twist.linear.z);
	fprintf(quadrotor_velocity_data, "%f, %f, %f, %f, %f, %f, %f\n",
		g_data_timestamp, g_smoothramp_x_velocity, g_smoothramp_y_velocity, g_smoothramp_z_velocity, g_velocity_current_fcu.twist.linear.x, g_velocity_current_fcu.twist.linear.y, g_velocity_current_fcu.twist.linear.z);
	return;
}

void velocityCommandCalculation(){
	g_x_velocity_manualcount = (g_x_position_reference - g_x_position_reference_previous) / sample_time;
	g_y_velocity_manualcount = (g_y_position_reference - g_y_position_reference_previous) / sample_time;
	g_z_velocity_manualcount = (g_z_position_reference - g_z_position_reference_previous) / sample_time;
	return;
}

void disarmQuadrotor() {
	//Closing all data file recorded during runtime when the program
	fclose(quadrotor_position_data);
	fclose(trajectory_planning_data);
	fclose(controller_data);
	fclose(quadrotor_velocity_data);
	g_arming_command.request.value = false;
	g_key_any = false;
	puts("\nQuadrotor disarmed!\n");
	resetKeyboardCommand();
	return;
}

void resetInitialValues(){
	g_trajectory_planning_mode = 0;
	// g_controller_active = false;// to activate / deactivate the controller(command goes straight to pose)
	g_key_any = false;
	g_shutdown_indicator = false;

	g_time_command = 0;
	g_time_final = 0;
	g_time_delta = 0;
	g_time_difference = 0;
	g_time_current = 0;
	g_time_initial = 0;
	g_time_init = 0;
	g_time_last_command = 0;
	g_data_timestamp = 0;
	g_data_timestamp_old = 0;
	//used to calculate the multiplier for the command value of x y and z
	g_delta_command_multiplier = 0;
	g_delta_command_dot_multiplier = 0;
	g_delta_command_doubledot_multiplier = 0;
	g_delta_command_integral_multiplier = 0;
	g_x_position_reference_previous = 0;
	g_y_position_reference_previous = 0;
	g_z_position_reference_previous = 0;
	g_x_velocity_manualcount = 0;
	g_y_velocity_manualcount = 0;
	g_z_velocity_manualcount = 0;

	g_x_command_changed_indicator_linear = false;
	g_x_command_changed_indicator_wave = false;
	g_y_command_changed_indicator_linear = false;
	g_y_command_changed_indicator_wave = false;
	g_z_command_changed_indicator_linear = false;
	g_z_command_changed_indicator_wave = false;

	//variables for position control
	g_x_position_reference = 0;
	g_x_position_delta_command = 0;
	g_x_position_initial = 0;
	g_x_position_error = 0;
	g_x_position_error_previous = 0;
	g_x_position_error_derivative = 0;
	g_x_position_error_integral = 0;
	g_y_position_reference = 0;
	g_y_position_delta_command = 0;
	g_y_position_initial = 0;
	g_y_position_error = 0;
	g_y_position_error_previous = 0;
	g_y_position_error_derivative = 0;
	g_y_position_error_integral = 0;
	g_z_position_reference = 0;
	g_z_position_delta_command = 0;
	g_z_position_initial = 0;
	g_z_position_error = 0;
	g_z_position_error_previous = 0;
	g_z_position_error_derivative = 0;
	g_z_position_error_integral = 0;
	g_x_position_fcu_current = 0;
	g_y_position_fcu_current = 0;
	g_z_position_fcu_current = 0;
	g_x_position_control_signal = 0;
	g_y_position_control_signal = 0;
	g_z_position_control_signal = 0;
	g_x_position_control_signal_previous = 0;
	g_y_position_control_signal_previous = 0;
	g_z_position_control_signal_previous = 0;
	g_x_position_control_signal_derivative = 0;
	g_y_position_control_signal_derivative = 0;
	g_z_position_control_signal_derivative = 0;

	//variables for velocity control
	g_x_velocity_reference = 0;
	g_x_velocity_delta_command = 0;
	g_x_velocity_initial = 0;
	g_x_velocity_error = 0;
	g_x_velocity_error_previous = 0;
	g_x_velocity_error_derivative = 0;
	g_x_velocity_error_integral = 0;
	g_y_velocity_reference = 0;
	g_y_velocity_delta_command = 0;
	g_y_velocity_initial = 0;
	g_y_velocity_error = 0;
	g_y_velocity_error_previous = 0;
	g_y_velocity_error_derivative = 0;
	g_y_velocity_error_integral = 0;
	g_z_velocity_reference = 0;
	g_z_velocity_delta_command = 0;
	g_z_velocity_initial = 0;
	g_z_velocity_error = 0;
	g_z_velocity_error_previous = 0;
	g_z_velocity_error_derivative = 0;
	g_z_velocity_error_integral = 0;
	g_x_velocity_fcu_current = 0;
	g_y_velocity_fcu_current = 0;
	g_z_velocity_fcu_current = 0;
	g_x_velocity_control_signal = 0;
	g_y_velocity_control_signal = 0;
	g_z_velocity_control_signal = 0;

	g_ramp_trajectory_x_gradient = 0;
	g_ramp_trajectory_y_gradient = 0;
	g_ramp_trajectory_z_gradient = 0;
	g_ramp_trajectory_time_multiplier = 0;

	//Period and frequency parameter used for the wave trajectory
	g_wave_trajectory_period = 1;//period of every wave motion
	g_wave_trajectory_halfperiod = 0.5;
	g_wave_trajectory_amplitude = 1;
	g_wave_trajectory_amplitude_x = 0;
	g_wave_trajectory_amplitude_y = 0;
	g_wave_trajectory_amplitude_z = 0;
	g_wave_delta = 0;
	g_wave_time = 0;
	g_wave_trajectory_amplitude_default = 0.5;//default setting

	g_smoothramp_time_speedincrease_duration = 0;
	g_smoothramp_time_speeddecrease_duration = 0;
	g_smoothramp_x_maxvelocity = 0;
	g_smoothramp_x_speedgradient = 0;
	g_smoothramp_y_maxvelocity = 0;
	g_smoothramp_y_speedgradient = 0;
	g_smoothramp_z_maxvelocity = 0;
	g_smoothramp_z_speedgradient = 0;
	g_smoothramp_time_speedincrease_endperiod = 0;
	g_smoothramp_time_speeddecrease_startperiod = 0;
	g_smoothramp_x_velocity = 0;
	g_smoothramp_y_velocity = 0;
	g_smoothramp_z_velocity = 0;
	g_smoothramp_x_velocity_previous = 0;
	g_smoothramp_y_velocity_previous = 0;
	g_smoothramp_z_velocity_previous = 0;
	return;
}

void resetDeltaPositionCommand() {//default trajectory planning settings
	g_x_position_delta_command = 0;
	g_y_position_delta_command = 0;
	g_z_position_delta_command = 0;
	g_time_delta = 0.0;
	g_trajectory_planning_mode = 0;
	g_wave_trajectory_period = 1;
	g_x_position_reference_previous = 0;
	g_y_position_reference_previous = 0;
	g_z_position_reference_previous = 0;
	g_smoothramp_time_speedincrease_duration = 0;
	g_smoothramp_time_speeddecrease_duration = 0;
	g_smoothramp_x_maxvelocity = 0;
	g_smoothramp_x_speedgradient = 0;
	g_smoothramp_y_maxvelocity = 0;
	g_smoothramp_y_speedgradient = 0;
	g_smoothramp_z_maxvelocity = 0;
	g_smoothramp_z_speedgradient = 0;
	g_smoothramp_x_velocity = 0;
	g_smoothramp_y_velocity = 0;
	g_smoothramp_z_velocity = 0;
	g_smoothramp_x_velocity_previous = 0;
	g_smoothramp_y_velocity_previous = 0;
	g_smoothramp_z_velocity_previous = 0;
	return;
}

void initializeTakeoff(){//function to run takeoff
	//Initialize the files that will be written for documenting various data
	trajectory_planning_data = fopen("trajectory_planning_data.dat","w");
	controller_data = fopen("controller_data.dat","w");
	quadrotor_position_data = fopen("quadrotor_position_data.dat","w");
	quadrotor_velocity_data = fopen("quadrotor_velocity_data.dat","w");

	resetInitialValues();
	strcpy(g_flight_mode_command,"OFFBOARD");
	g_position_control_set_mode.request.custom_mode = g_flight_mode_command;
	puts("Quadrotor armed!\n");
	printf("Mode enabled: %s\n",g_flight_mode_command);
	g_arming_command.request.value = true;//Arming
	g_last_arm_request = ros::Time::now();//Get current time
	g_time_current = g_last_arm_request.toSec();
	g_time_last_command = g_last_arm_request.toSec();
	g_first_control_trigger_time = g_last_arm_request;
	g_last_keyboard_command = g_last_arm_request;
	g_last_current_position_print = g_last_arm_request;
	g_position_command.pose.position.x  = 0;
	g_position_command.pose.position.y  = 0;
	g_position_command.pose.position.z  = G_Z_POSITION_LIMIT_LOWER;
	g_x_position_initial = 0;
	g_y_position_initial = 0;
	g_z_position_initial = G_Z_POSITION_LIMIT_LOWER;
	return;
}

void initializeLanding(){
	g_trajectory_planning_mode = 3;
	g_z_position_delta_command = (-1 * g_position_current_fcu.pose.position.z);//dont make it directly go to 0
	initiateTrajectoryPlanning();
	return;
}

void runControlAlgorithm(const ros::TimerEvent& event){
	g_time_current = ros::Time::now().toSec();
	if (g_first_run) {
		g_time_last_command = g_time_current;
		g_first_run = false;
	}
	g_data_timestamp_old = g_data_timestamp;
	g_data_timestamp = g_time_current - g_time_last_command;
	sample_time = g_data_timestamp - g_data_timestamp_old;
	g_x_position_reference_previous = g_x_position_reference;
	g_y_position_reference_previous = g_y_position_reference;
	g_z_position_reference_previous = g_z_position_reference;
	trajectoryPlanning();
	if(g_key_any && g_controller_active){//sending command values to px4 with control
		positionControlLoop();
		// velocityCommandCalculation();
		// velocityControlLoop();
		// g_velocity_command.twist.linear.x = g_x_velocity_control_signal;
		// g_velocity_command.twist.linear.y = g_y_velocity_control_signal;
		// g_velocity_command.twist.linear.z = g_z_velocity_control_signal;
		g_position_command.pose.position.x = g_x_position_control_signal;
		g_position_command.pose.position.y = g_y_position_control_signal;
		g_position_command.pose.position.z = g_z_position_control_signal;
		recordData();
		return;
	}
	g_position_command.pose.position.x = g_x_position_reference;
	g_position_command.pose.position.y = g_y_position_reference;
	g_position_command.pose.position.z = g_z_position_reference;
	if (g_arming_command.request.value) {//if takeoff command is initialized
		recordData();//BUG cannot run function before takeoff
		return;
	}
	return;
}

void initiateTrajectoryPlanning(){
	g_time_last_command = ros::Time::now().toSec(); // Enable if the command starts at 0 rather that any specific time
	g_time_initial = g_time_last_command;
	g_time_final = g_time_delta + g_time_initial;
	//Linear motion indicator
	if(g_x_position_delta_command != 0){
		g_x_command_changed_indicator_linear = true;
	}
	else if (g_x_position_delta_command ==  0) {
		g_x_command_changed_indicator_linear = false;
	}

	if(g_y_position_delta_command != 0){
		g_y_command_changed_indicator_linear = true;
	}
	else if (g_y_position_delta_command == 0) {
		g_y_command_changed_indicator_linear = false;
	}

	if(g_z_position_delta_command != 0){
		g_z_command_changed_indicator_linear = true;
	}
	else if (g_z_position_delta_command ==  0) {
		g_z_command_changed_indicator_linear = false;
	}

	//Wave motion indicator
	if(g_wave_trajectory_amplitude_x != 0){
		g_x_command_changed_indicator_wave = true;
	}
	else if (g_wave_trajectory_amplitude_x ==  0) {
		g_x_command_changed_indicator_wave = false;
	}

	if(g_wave_trajectory_amplitude_y != 0){
		g_y_command_changed_indicator_wave = true;
	}
	else if (g_wave_trajectory_amplitude_y == 0) {
		g_y_command_changed_indicator_wave = false;
	}

	if(g_wave_trajectory_amplitude_z != 0){
		g_z_command_changed_indicator_wave = true;
	}
	else if (g_wave_trajectory_amplitude_z ==  0) {
		g_z_command_changed_indicator_wave = false;
	}
	g_x_position_initial = g_x_position_reference;
	g_y_position_initial = g_y_position_reference;
	g_z_position_initial = g_z_position_reference;
	g_x_position_error = 0;
	g_y_position_error = 0;
	g_z_position_error = 0;
	g_x_velocity_error = 0;
	g_y_velocity_error = 0;
	g_z_velocity_error = 0;
	if (g_trajectory_planning_mode == 4) {
		if (g_x_position_delta_command != 0 && g_time_delta != 0){
			g_ramp_trajectory_x_gradient = g_x_position_delta_command/g_time_delta;
		}
		if (g_y_position_delta_command != 0 && g_time_delta != 0){
			g_ramp_trajectory_y_gradient = g_y_position_delta_command/g_time_delta;
		}
		if (g_z_position_delta_command != 0 && g_time_delta != 0){
			g_ramp_trajectory_z_gradient = g_z_position_delta_command/g_time_delta;
		}
	}
	else if (g_trajectory_planning_mode == 5) {
		g_smoothramp_time_speedincrease_endperiod = g_time_initial + g_smoothramp_time_speedincrease_duration;
		g_smoothramp_time_speeddecrease_startperiod = g_time_final - g_smoothramp_time_speeddecrease_duration;
		if (g_x_position_delta_command != 0 && g_time_delta != 0){
			g_smoothramp_x_maxvelocity = g_x_position_delta_command * 2 / (g_time_delta + (g_time_delta - g_smoothramp_time_speedincrease_duration - g_smoothramp_time_speeddecrease_duration));
			g_smoothramp_x_speedgradient = g_smoothramp_x_maxvelocity / g_smoothramp_time_speedincrease_duration;
		}
		if (g_y_position_delta_command != 0 && g_time_delta != 0){
			g_smoothramp_y_maxvelocity = g_y_position_delta_command * 2 / (g_time_delta + (g_time_delta - g_smoothramp_time_speedincrease_duration - g_smoothramp_time_speeddecrease_duration));
			g_smoothramp_y_speedgradient = g_smoothramp_y_maxvelocity / g_smoothramp_time_speedincrease_duration;
		}
		if (g_z_position_delta_command != 0 && g_time_delta != 0){
			g_smoothramp_z_maxvelocity = g_z_position_delta_command * 2 / (g_time_delta + (g_time_delta - g_smoothramp_time_speedincrease_duration - g_smoothramp_time_speeddecrease_duration));
			g_smoothramp_z_speedgradient = g_smoothramp_z_maxvelocity / g_smoothramp_time_speedincrease_duration;
		}
	}
	g_key_any = true;
	resetKeyboardCommand();
	return;
}

void resetKeyboardCommand(){
	g_key_command_received.data = 'o';
	g_key_command_allow = false;
	return;
}

void rampCommandCalculation() {
	g_ramp_trajectory_time_multiplier = g_time_current - g_time_initial;
	if (g_x_position_delta_command != 0 && g_time_delta != 0){
		if (g_time_current <= g_time_final) {
			g_x_position_reference = g_x_position_initial + (g_ramp_trajectory_time_multiplier * g_ramp_trajectory_x_gradient);
		}
		else if (g_time_current > g_time_final) {
			positionCommandFinalValueCalculationX();
		}
	}
	if (g_y_position_delta_command != 0 && g_time_delta != 0){
		if (g_time_current <= g_time_final) {
			g_y_position_reference = g_y_position_initial + (g_ramp_trajectory_time_multiplier * g_ramp_trajectory_y_gradient);
		}
		else if (g_time_current > g_time_final) {
			positionCommandFinalValueCalculationY();
		}
	}
	if (g_z_position_delta_command != 0 && g_time_delta != 0){
		if (g_time_current <= g_time_final) {
			g_z_position_reference = g_z_position_initial + (g_ramp_trajectory_time_multiplier * g_ramp_trajectory_z_gradient);
		}
		else if (g_time_current > g_time_final) {
			positionCommandFinalValueCalculationZ();
		}
	}
	return;
}

void squareCommandCalculation() {
	g_ramp_trajectory_time_multiplier = g_time_current - g_time_initial;
	if (g_x_position_delta_command != 0 && g_time_delta != 0){
		if (g_time_current < (g_time_final-9) && g_time_current > g_time_initial) {
			g_x_position_reference = g_x_position_initial + g_ramp_trajectory_time_multiplier;
		}		
	}
	if (g_y_position_delta_command != 0 && g_time_delta != 0){
		if (g_time_current < (g_time_final-6) && g_time_current > (g_time_final-9)) {
			g_y_position_reference = g_y_position_initial + (g_ramp_trajectory_time_multiplier-3);
		}
	}
	if (g_x_position_delta_command != 0 && g_time_delta != 0){
		if (g_time_current < (g_time_final-3) && g_time_current > (g_time_final-6)) {
			g_x_position_reference = (g_x_position_initial+3) - (g_ramp_trajectory_time_multiplier-6);
		}
		else if (g_time_current > g_time_final) {
			g_x_position_reference = g_x_position_initial;		
			}
	}
	if (g_y_position_delta_command != 0 && g_time_delta != 0){
		if (g_time_current < g_time_final && g_time_current > (g_time_final-1*(g_time_delta/4))) {
			g_y_position_reference = (g_y_position_initial+3) - (g_ramp_trajectory_time_multiplier-9);
		}
		else if (g_time_current > g_time_final) {
			g_y_position_reference = g_y_position_initial;
		}
	}

	return;
}
void sigmoidCommandCalculation(){
	g_time_command = (g_time_current - g_time_initial)/(g_time_final - g_time_initial);
	if (g_time_command < 1) {
		g_delta_command_multiplier = (6 * pow(g_time_command, 5.0)) - (15 * pow(g_time_command, 4.0)) + (10 * pow(g_time_command, 3.0));
		positionCommandCalculation();
	}
	else if (g_time_command >= 1) {
		g_time_command = 1; //limit it to 1
		g_delta_command_multiplier = (6 * pow(g_time_command, 5.0)) - (15 * pow(g_time_command, 4.0)) + (10 * pow(g_time_command, 3.0));
		g_x_command_changed_indicator_linear = false;
		g_y_command_changed_indicator_linear = false;
		g_z_command_changed_indicator_linear = false;
		g_x_command_changed_indicator_wave = false;
		g_y_command_changed_indicator_wave = false;
		g_z_command_changed_indicator_wave = false;
		positionCommandCalculation();
	}
	return;
}

void sinWaveCommandCalculation(){
	g_time_command = (g_time_current - g_time_initial)/(g_time_final - g_time_initial);
	if (g_time_command < 1) {
		waveCurrentTimeCalculation();
		trajectoryDeltaWaveCalculation();
		positionCommandCalculation();
	}
	else if (g_time_command >= 1) {
		g_time_command = 1; //limit it to 1
		waveCurrentTimeCalculation();
		trajectoryDeltaWaveCalculation();
		g_x_command_changed_indicator_linear = false;
		g_y_command_changed_indicator_linear = false;
		g_z_command_changed_indicator_linear = false;
		g_x_command_changed_indicator_wave = false;
		g_y_command_changed_indicator_wave = false;
		g_z_command_changed_indicator_wave = false;
		positionCommandCalculation();
	}
	return;
}

void zigzagCommandCalculation(){
	g_time_command = (g_time_current - g_time_initial)/(g_time_final - g_time_initial);
	if (g_time_command < 1) {
		waveCurrentTimeCalculation();
		trajectoryDeltaWaveCalculation();
		positionCommandCalculation();
	}
	else if (g_time_command >= 1) {
		g_time_command = 1; //limit it to 1
		waveCurrentTimeCalculation();
		trajectoryDeltaWaveCalculation();
		g_x_command_changed_indicator_linear = false;
		g_y_command_changed_indicator_linear = false;
		g_z_command_changed_indicator_linear = false;
		g_x_command_changed_indicator_wave = false;
		g_y_command_changed_indicator_wave = false;
		g_z_command_changed_indicator_wave = false;
		positionCommandCalculation();
	}
	return;
}

void stepCommandCalculation(){
	g_x_command_changed_indicator_linear = false;
	g_y_command_changed_indicator_linear = false;
	g_z_command_changed_indicator_linear = false;
	g_x_command_changed_indicator_wave = false;
	g_y_command_changed_indicator_wave = false;
	g_z_command_changed_indicator_wave = false;
	positionCommandCalculation();
	return;
}

void smoothrampCommandCalculation(){
	if (g_x_position_delta_command != 0 && g_time_delta != 0){
		if (g_time_current <= g_smoothramp_time_speedincrease_endperiod) {
			g_smoothramp_x_velocity = g_smoothramp_x_speedgradient * (g_time_current - g_time_initial);
			g_x_position_reference += g_smoothramp_x_velocity * 0.03;
		}
		else if (g_time_current > g_smoothramp_time_speedincrease_endperiod && g_time_current < g_smoothramp_time_speeddecrease_startperiod) {
			g_smoothramp_x_velocity = g_smoothramp_x_speedgradient * g_smoothramp_time_speedincrease_duration;
			g_x_position_reference += g_smoothramp_x_velocity * 0.03;
		}
		else if (g_time_current >= g_smoothramp_time_speeddecrease_startperiod && g_time_current < g_time_final) {
			g_smoothramp_x_velocity = (g_smoothramp_x_speedgradient * (g_time_final - g_time_current));
			g_x_position_reference += g_smoothramp_x_velocity * 0.03;
		}
		else if (g_time_current >= g_time_final) {
			g_smoothramp_x_velocity = 0;
			positionCommandFinalValueCalculationX();
		}
	}
	if (g_y_position_delta_command != 0 && g_time_delta != 0){
		if (g_time_current <= g_smoothramp_time_speedincrease_endperiod) {
			g_smoothramp_y_velocity = g_smoothramp_y_speedgradient * (g_time_current - g_time_initial);
			g_y_position_reference += g_smoothramp_y_velocity * 0.03;
		}
		else if (g_time_current > g_smoothramp_time_speedincrease_endperiod && g_time_current < g_smoothramp_time_speeddecrease_startperiod) {
			g_smoothramp_y_velocity = g_smoothramp_y_speedgradient * g_smoothramp_time_speedincrease_duration;
			g_y_position_reference += g_smoothramp_y_velocity * 0.03;
		}
		else if (g_time_current >= g_smoothramp_time_speeddecrease_startperiod && g_time_current < g_time_final) {
			g_smoothramp_y_velocity = (g_smoothramp_y_speedgradient * (g_time_final - g_time_current));
			g_y_position_reference += g_smoothramp_y_velocity * 0.03;
		}
		else if (g_time_current >= g_time_final) {
			g_smoothramp_y_velocity = 0;
			positionCommandFinalValueCalculationY();
		}
	}
	if (g_z_position_delta_command != 0 && g_time_delta != 0){
		if (g_time_current <= g_smoothramp_time_speedincrease_endperiod) {
			g_smoothramp_z_velocity = g_smoothramp_z_speedgradient * (g_time_current - g_time_initial);
			g_z_position_reference += g_smoothramp_z_velocity * 0.03;
		}
		else if (g_time_current > g_smoothramp_time_speedincrease_endperiod && g_time_current < g_smoothramp_time_speeddecrease_startperiod) {
			g_smoothramp_z_velocity = g_smoothramp_z_speedgradient * g_smoothramp_time_speedincrease_duration;
			g_z_position_reference += g_smoothramp_z_velocity * 0.03;
		}
		else if (g_time_current >= g_smoothramp_time_speeddecrease_startperiod && g_time_current <= g_time_final) {
			g_smoothramp_z_velocity = (g_smoothramp_z_speedgradient * (g_time_final - g_time_current));
			g_z_position_reference += g_smoothramp_z_velocity * 0.03;
		}
		else if (g_time_current > g_time_final) {
			g_smoothramp_z_velocity = 0;
			positionCommandFinalValueCalculationZ();
		}
	}
	return;
}

void trajectoryPlanning(){
	switch (g_trajectory_planning_mode) {
		case 0:
			sigmoidCommandCalculation();
			break;
		case 1:
			sinWaveCommandCalculation();
			break;
		case 2:
			zigzagCommandCalculation();
			break;
		case 3:
			stepCommandCalculation();
			break;
		case 4:
			rampCommandCalculation();
			break;
		case 5:
			smoothrampCommandCalculation();
			break;
		case 6:
			squareCommandCalculation();
			break;
		
		default:
			break;
	}
	return;
}

void waveCurrentTimeCalculation(){
	if (g_time_command == 1) {
		g_delta_command_multiplier = g_time_command;
		return;
	}
	g_delta_command_multiplier = g_time_command;
	g_time_difference = g_time_current - g_time_initial;
	if (g_time_difference < g_time_delta) {
		g_wave_time = g_time_difference;
	}
	else if (g_time_difference >= g_time_delta) {
		g_wave_time = g_time_delta;
	}
	return;
}

void positionCommandFinalValueCalculationX() {
	g_x_position_reference = g_x_position_initial + g_x_position_delta_command;
	return;
}
void positionCommandFinalValueCalculationY() {
	g_y_position_reference = g_y_position_initial + g_y_position_delta_command;
	return;
}
void positionCommandFinalValueCalculationZ() {
	g_z_position_reference = g_z_position_initial + g_z_position_delta_command;
	if (g_z_position_reference < 0) {//To make use it doesnt go below 0
		g_z_position_reference = 0;
	}
	return;
}
void trajectoryDeltaWaveCalculation(){
	/*
	This function is used to calculate the distance at which the quadcopter move
	up and down in a wave motion.
	*/
	if (g_wave_trajectory_amplitude_x != 0) {
		g_wave_trajectory_amplitude = g_wave_trajectory_amplitude_x;
	}
	else if (g_wave_trajectory_amplitude_y != 0) {
		g_wave_trajectory_amplitude = g_wave_trajectory_amplitude_y;
	}
	else if (g_wave_trajectory_amplitude_z != 0) {
		g_wave_trajectory_amplitude = g_wave_trajectory_amplitude_z;
	}
	else{
		g_wave_trajectory_amplitude = g_wave_trajectory_amplitude_default;//default setting
	}
	switch (g_trajectory_planning_mode) {
		case 1://sin wave
			g_wave_delta = g_wave_trajectory_amplitude * sin(2 * PI * (1 / g_wave_trajectory_period * g_wave_time));
			break;
		case 2://zigzag wave
			g_wave_trajectory_halfperiod = g_wave_trajectory_period / 2;
			g_wave_delta = (((g_wave_trajectory_amplitude / g_wave_trajectory_halfperiod) * (g_wave_trajectory_halfperiod - fabs(fmod(g_wave_time + (g_wave_trajectory_halfperiod / 2) , g_wave_trajectory_period) - g_wave_trajectory_halfperiod))) - (g_wave_trajectory_amplitude / 2)) * 2;
			break;
		default://if no wave motion is indicated by the keyboard command
			break;
	}
	return;
}
void positionCommandCalculation(){
	/*
	This function is used to calculate the final reference value used to do a linear and wave movement
	of the quadcopter for a specific time point.
	*/
	if(g_x_command_changed_indicator_linear && g_x_command_changed_indicator_wave == false){
		g_x_position_reference = g_x_position_initial + (g_delta_command_multiplier * g_x_position_delta_command);
	}
	else if (g_x_command_changed_indicator_wave && g_x_command_changed_indicator_linear == false) {
		g_x_position_reference = g_x_position_initial + g_wave_delta;
	}
	else{
		positionCommandFinalValueCalculationX();
	}

	if(g_y_command_changed_indicator_linear && g_y_command_changed_indicator_wave == false){
		g_y_position_reference = g_y_position_initial + (g_delta_command_multiplier * g_y_position_delta_command);
	}
	else if (g_y_command_changed_indicator_wave && g_y_command_changed_indicator_linear == false) {
		g_y_position_reference = g_y_position_initial + g_wave_delta;
	}
	else{
		positionCommandFinalValueCalculationY();
	}

	if(g_z_command_changed_indicator_linear && g_z_command_changed_indicator_wave == false){
		g_z_position_reference = g_z_position_initial + (g_delta_command_multiplier * g_z_position_delta_command);
	}
	else if (g_z_command_changed_indicator_wave && g_z_command_changed_indicator_linear == false) {
		g_z_position_reference = g_z_position_initial + g_wave_delta;
	}
	else{
		positionCommandFinalValueCalculationZ();
	}
	return;
}

void positionControlLoop(){
	/*
	This function is the main control loop used for the position reference of the Quadcopter
	all of the calculation below used a standard PID controller model, with a sample interval
	of 30 ms
	*/
	//Calculation for PID position controller input
	g_x_position_fcu_current = g_position_current_fcu.pose.position.x;
	g_y_position_fcu_current = g_position_current_fcu.pose.position.y;
	g_z_position_fcu_current = g_position_current_fcu.pose.position.z;
	g_x_position_error_previous = g_x_position_error;
	g_y_position_error_previous = g_y_position_error;
	g_z_position_error_previous = g_z_position_error;
	g_x_position_error = g_x_position_reference - g_x_position_fcu_current;
	g_y_position_error = g_y_position_reference - g_y_position_fcu_current;
	g_z_position_error = g_z_position_reference - g_z_position_fcu_current;
	g_x_position_error_derivative = (g_x_position_error - g_x_position_error_previous) / DT;
	g_y_position_error_derivative = (g_y_position_error - g_y_position_error_previous) / DT;
	g_z_position_error_derivative = (g_z_position_error - g_z_position_error_previous) / DT;
	g_x_position_error_integral += (g_x_position_error) * DT;
	g_y_position_error_integral += (g_y_position_error) * DT;
	g_z_position_error_integral += (g_z_position_error) * DT;

	g_x_position_control_signal_previous = g_x_position_control_signal;
	g_y_position_control_signal_previous = g_y_position_control_signal;
	g_z_position_control_signal_previous = g_z_position_control_signal;

	//PID controller output calculation, producing position control signal
	g_x_position_control_signal = (G_KP_POSITIONCONTROL_X * g_x_position_error) + (G_KD_POSITIONCONTROL_X * g_x_position_error_derivative) + (G_KI_POSITIONCONTROL_X * g_x_position_error_integral);
	g_y_position_control_signal = (G_KP_POSITIONCONTROL_Y * g_y_position_error) + (G_KD_POSITIONCONTROL_Y * g_y_position_error_derivative) + (G_KI_POSITIONCONTROL_Y * g_y_position_error_integral);
	g_z_position_control_signal = (G_KP_POSITIONCONTROL_Z * g_z_position_error) + (G_KD_POSITIONCONTROL_Z * g_z_position_error_derivative) + (G_KI_POSITIONCONTROL_Z * g_z_position_error_integral);

	//Calculating the derivative of the control signal
	g_x_position_control_signal_derivative  = (g_x_position_control_signal - g_x_position_control_signal_previous) / DT;
	g_y_position_control_signal_derivative  = (g_y_position_control_signal - g_y_position_control_signal_previous) / DT;
	g_z_position_control_signal_derivative  = (g_z_position_control_signal - g_z_position_control_signal_previous) / DT;
	return;
}

void velocityControlLoop(){
	//Velocity reference obtained using the control signal produced by position control output
	g_x_velocity_reference = g_x_velocity_manualcount;
	g_y_velocity_reference = g_y_velocity_manualcount;
	g_z_velocity_reference = g_z_velocity_manualcount;

	//Obtaining current velocity
	g_x_velocity_fcu_current = g_velocity_current_fcu.twist.linear.x;
	g_y_velocity_fcu_current = g_velocity_current_fcu.twist.linear.y;
	g_z_velocity_fcu_current = g_velocity_current_fcu.twist.linear.z;

	g_x_velocity_error_previous = g_x_velocity_error;
	g_y_velocity_error_previous = g_y_velocity_error;
	g_z_velocity_error_previous = g_z_velocity_error;

	g_x_velocity_error = g_x_velocity_reference - g_x_velocity_fcu_current;
	g_y_velocity_error = g_y_velocity_reference - g_y_velocity_fcu_current;
	g_z_velocity_error = g_z_velocity_reference - g_z_velocity_fcu_current;

	g_x_velocity_error_derivative = (g_x_velocity_error - g_x_velocity_error_previous) / DT;
	g_y_velocity_error_derivative = (g_y_velocity_error - g_y_velocity_error_previous) / DT;
	g_z_velocity_error_derivative = (g_z_velocity_error - g_z_velocity_error_previous) / DT;

	g_x_velocity_error_integral += (g_x_velocity_error) * DT;
	g_y_velocity_error_integral += (g_y_velocity_error) * DT;
	g_z_velocity_error_integral += (g_z_velocity_error) * DT;

	//PID controller output calculation, producing velocity control signal
	g_x_velocity_control_signal = (G_KP_VELOCITYCONTROL_X * g_x_velocity_error) + (G_KD_VELOCITYCONTROL_X * g_x_velocity_error_derivative) + (G_KI_VELOCITYCONTROL_X * g_x_velocity_error_integral);
	g_y_velocity_control_signal = (G_KP_VELOCITYCONTROL_Y * g_y_velocity_error) + (G_KD_VELOCITYCONTROL_Y * g_y_velocity_error_derivative) + (G_KI_VELOCITYCONTROL_Y * g_y_velocity_error_integral);
	g_z_velocity_control_signal = (G_KP_VELOCITYCONTROL_Z * g_z_velocity_error) + (G_KD_VELOCITYCONTROL_Z * g_z_velocity_error_derivative) + (G_KI_VELOCITYCONTROL_Z * g_z_velocity_error_integral);
	return;
}

void keyboardCommands(){
	/*
	This function is used to received commands from the keyboard_commands node to the quadcopter,
	g_key_command_allow is used to allow the command to be processed. This prevent commands
	to be interrupted by any new commands coming in.
	the variables used are descriptive enough to indicate its use.
	*/
	if (g_key_command_allow) {
		switch(g_key_command){
			//do not use o or p for any command, it is used after the intended command have been published by publisher

			case 'x'://initiate position control
				resetDeltaPositionCommand();
				g_trajectory_planning_mode = 3;
				initiateTrajectoryPlanning();
				break;

			case '='://Emergency disarm, press only when the Quadrotor fall
				disarmQuadrotor();
				break;

			case 'c'://Land the quadrotor
				resetDeltaPositionCommand();
				initializeLanding();
				puts("\nQuadrotor landing...");
				resetKeyboardCommand();
				break;

			case '['://disarm
				if (g_position_current_fcu.pose.position.z >= G_LANDING_Z_POSITION) {
					puts("\nDisarm rejected, Quadrotor must be lowered");
					resetKeyboardCommand();
					break;
				}
				disarmQuadrotor();
				break;

			case 'o':
				g_key_command_received.data = 'p';
				break;

			case ']'://arm and takeoff
				initializeTakeoff();
				resetKeyboardCommand();
				break;

			case 'z'://Node shutdown command
				g_shutdown_indicator = true;
				g_key_command_received.data = 'z';
				break;

			//
			//For sigmoid trajectory control
			// case 'w':
			// 	resetDeltaPositionCommand();
			// 	g_z_position_delta_command = 3.0;
			// 	g_time_delta = 3.0;
			// 	g_trajectory_planning_mode = 0;
			// 	initiateTrajectoryPlanning();
			// 	break;
			case 'a':
				resetDeltaPositionCommand();
				g_x_position_delta_command = 3.0;
				g_time_delta = 3.0;
				g_trajectory_planning_mode = 0;
				initiateTrajectoryPlanning();
				break;
			case 's':
				resetDeltaPositionCommand();
				g_y_position_delta_command = 3.0;
				g_time_delta = 3.0;
				g_trajectory_planning_mode = 0;
				initiateTrajectoryPlanning();
				break;
			case 'd':
				resetDeltaPositionCommand();
				g_z_position_delta_command = 3.0;
				g_time_delta = 3.0;
				g_trajectory_planning_mode = 0;
				initiateTrajectoryPlanning();
				break;
			// case 'q':
			// 	resetDeltaPositionCommand();
			// 	g_y_position_delta_command = 3.0;
			// 	g_time_delta = 3.0;
			// 	g_trajectory_planning_mode = 0;
			// 	initiateTrajectoryPlanning();
			// 	break;
			// case 'e':
			// 	resetDeltaPositionCommand();
			// 	g_y_position_delta_command = 1.5;
			// 	g_time_delta = 3.0;
			// 	g_trajectory_planning_mode = 0;
			// 	initiateTrajectoryPlanning();
			// 	break;

			//for smooth ramp trajectory
			case 'r':
				resetDeltaPositionCommand();
				g_x_position_delta_command = 3.0;
				g_time_delta = 3.0;
				g_smoothramp_time_speedincrease_duration = 1;
				g_smoothramp_time_speeddecrease_duration = 1;
				g_trajectory_planning_mode = 5;
				initiateTrajectoryPlanning();
				break;
			case 't':
				resetDeltaPositionCommand();
				g_y_position_delta_command = 3.0;
				g_time_delta = 3.0;
				g_smoothramp_time_speedincrease_duration = 1;
				g_smoothramp_time_speeddecrease_duration = 1;
				g_trajectory_planning_mode = 5;
				initiateTrajectoryPlanning();
				break;
			case 'y':
				resetDeltaPositionCommand();
				g_z_position_delta_command = 3.0;
				g_time_delta = 3.0;
				g_smoothramp_time_speedincrease_duration = 1;
				g_smoothramp_time_speeddecrease_duration = 1;
				g_trajectory_planning_mode = 5;
				initiateTrajectoryPlanning();
				break;
			case '/':
				resetDeltaPositionCommand();
				g_x_position_delta_command = 1.5;
				g_time_delta = 1.5;
				g_smoothramp_time_speedincrease_duration = 0.375;
				g_smoothramp_time_speeddecrease_duration = 0.375;
				g_trajectory_planning_mode = 5;
				initiateTrajectoryPlanning();
				break;

			// step trajectory
			case 'j':
				resetDeltaPositionCommand();
				g_x_position_delta_command = 1.0;
				g_trajectory_planning_mode = 3;
				initiateTrajectoryPlanning();
				break;
			case 'k':
				resetDeltaPositionCommand();
				g_y_position_delta_command = 1.0;
				g_trajectory_planning_mode = 3;
				initiateTrajectoryPlanning();
				break;
			case 'l':
				resetDeltaPositionCommand();
				g_z_position_delta_command = 1.0;
				g_trajectory_planning_mode = 3;
				initiateTrajectoryPlanning();
				break;

			//ramp trajectory
			case 'f':
				resetDeltaPositionCommand();
				g_x_position_delta_command = 3.0;
				g_time_delta = 3.0;
				g_trajectory_planning_mode = 4;
				initiateTrajectoryPlanning();
				break;
			case 'g':
				resetDeltaPositionCommand();
				g_y_position_delta_command = 3.0;
				g_time_delta = 3.0;
				g_trajectory_planning_mode = 4;
				initiateTrajectoryPlanning();
				break;
			case 'h':
				resetDeltaPositionCommand();
				g_z_position_delta_command = 3.0;
				g_time_delta = 3.0;
				g_trajectory_planning_mode = 4;
				initiateTrajectoryPlanning();
				break;


			//sin wave trajectory
			case 'q':
				resetDeltaPositionCommand();
				g_x_position_delta_command = 1.0;
				g_trajectory_planning_mode = 1;
				g_time_delta = 3.0;
				g_wave_trajectory_period = 3.0;
				g_wave_trajectory_amplitude_y = 1;
				initiateTrajectoryPlanning();
				break;
			case 'w':
				resetDeltaPositionCommand();
				g_y_position_delta_command = 1.0;
				g_trajectory_planning_mode = 1;
				g_time_delta = 3.0;
				g_wave_trajectory_period = 3.0;
				g_wave_trajectory_amplitude_x = 1;
				initiateTrajectoryPlanning();
				break;
			case 'e':
				resetDeltaPositionCommand();
				g_z_position_delta_command = 1.0;
				g_trajectory_planning_mode = 1;
				g_time_delta = 3.0;
				g_wave_trajectory_period = 3.0;
				g_wave_trajectory_amplitude_y = 1;
				initiateTrajectoryPlanning();
				break;
			case 'b':
				resetDeltaPositionCommand();
				g_x_position_delta_command = 2.0;
				g_y_position_delta_command = 2.0;
				g_time_delta = 12.0;
				g_trajectory_planning_mode = 6;
				initiateTrajectoryPlanning();
				break;
			// //zigzag wave trajectory
			// case 'l':
			// 	resetDeltaPositionCommand();
			// 	g_x_position_delta_command = 1.0;
			// 	g_trajectory_planning_mode = 2;
			// 	g_time_delta = 10.0;
			// 	g_wave_trajectory_period = 10.0;
			// 	g_wave_trajectory_amplitude_y = 0.5;
			// 	initiateTrajectoryPlanning();
			// 	break;
			// case 'j':
			// 	resetDeltaPositionCommand();
			// 	g_x_position_delta_command = -1.0;
			// 	g_trajectory_planning_mode = 2;
			// 	g_time_delta = 10.0;
			// 	g_wave_trajectory_period = 10.0;
			// 	g_wave_trajectory_amplitude_y = 0.5;
			// 	initiateTrajectoryPlanning();
			// 	break;
			
			default:
				g_key_command_received.data = 'p';
			break;
		}
	}
	//g_key_command_allow is used so that the program can only run the keyboard command once for each command it sends.
	else if (g_key_command_allow == false) {
		switch (g_key_command) {
			case 'o'://the program will only accept another input if the keyboard_commands node have publish the letter 'o' and not other letters.
				g_key_command_allow = true;
				g_key_command_received.data = 'p';
				break;
			default:
				break;
		}
	}
	return;
}
void stateIndicator(const mavros_msgs::State::ConstPtr& msg){
    g_current_state = *msg;
}
void keyCommandIndicator(const std_msgs::String::ConstPtr& msg){
	g_key_command = *msg->data.c_str();
}
void currentFCUPositionIndicator(const geometry_msgs::PoseStamped::ConstPtr& msg){
	g_position_current_fcu = *msg;
}
void currentFCUVelocityIndicator(const geometry_msgs::TwistStamped::ConstPtr& msg){
	g_velocity_current_fcu = *msg;
}
