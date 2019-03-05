#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <string.h>

//Global Variables
ros::Time g_last_request;
std_msgs::String g_key;

char g_key_command_received = 'p';
char g_key_press;

//Constants
const double KEY_COMMAND_PUBLISH_DURATION = 1;
const double NODE_RUN_RATE = 100.0;//rate in Hz

//Prototype
char keyboardInterrupt();
void keyCommandReceived(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "keyboard_commands");
	ros::NodeHandle node_handle;
  	ros::Publisher key_command_publisher = node_handle.advertise<std_msgs::String>
		("key_commands", 1000);
	ros::Subscriber key_command_received_subscriber = node_handle.subscribe
		("key_command_receive", 1000, keyCommandReceived);
	ros::Rate loop_rate(NODE_RUN_RATE);
	while (ros::ok()){
		if(ros::Time::now() - g_last_request > ros::Duration(KEY_COMMAND_PUBLISH_DURATION)){
			switch (g_key_command_received) {
				case 'o':
					g_key.data = 'o';
					break;
					
				case 'p':
					printf("Enter key command: ");
					g_key_press = keyboardInterrupt();
					printf("\nKey command entered: %c\n\n",g_key_press);
					g_key.data = g_key_press;
					break;
				case 'z':
					ros::shutdown();
					break;
				default:
					g_key.data = 'o';
					break;
			}
			key_command_publisher.publish(g_key);
			g_last_request = ros::Time::now();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return(0);
}

char keyboardInterrupt(){
	static struct termios oldTermios, newTermios;
	newTermios.c_cc[VMIN] = 0;
	newTermios.c_cc[VTIME] = 0;
	tcgetattr(STDIN_FILENO, &oldTermios);           // save old settings
	newTermios = oldTermios;
	newTermios.c_lflag &= ~(ICANON);                 // disable buffering
	tcsetattr(STDIN_FILENO, TCSANOW, &newTermios);  // apply new settings
	g_key_press = getchar();  // read character (non-blocking)
	tcsetattr(STDIN_FILENO, TCSANOW, &oldTermios);  // restore old settings
	return g_key_press;
}
void keyCommandReceived(const std_msgs::String::ConstPtr& msg){
	g_key_command_received = *msg->data.c_str();
}
