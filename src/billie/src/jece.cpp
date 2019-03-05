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
int main(int argc, char **argv)
{
	ros::init(argc, argv,"position_control");//Initialize ros, and also naming the node
	ros::NodeHandle node_handle;//Name the node's handle


	while(ros::ok()){//Main program loop

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
		ros::spinOnce();
		loop_rate.sleep();
	}

}
