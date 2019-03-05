#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h> 
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
void state_callback(const mavros_msgs::State::ConstPtr& msg);

mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;
geometry_msgs::PoseStamped position;
void currentFCUPositionIndicator(const geometry_msgs::PoseStamped::ConstPtr& msg);
geometry_msgs::PoseStamped g_position_current_fcu;
char keyboard = ']';
int main(int argc,char **argv)
{
	ros::init(argc, argv, "velocity_control");
	ros::NodeHandle nh;
	ros::Subscriber state_subscriber = nh.subscribe<mavros_msgs::State>("mavros/state",1000,state_callback);
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::Publisher position_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
	ros::Subscriber current_position_FCU_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, currentFCUPositionIndicator);

	ros::Rate rate(20);
	ros::spinOnce();
	while(ros::ok()){
	keyboard = getchar();
	
	switch(keyboard){
		case ']':
		{
			while(1){
				arm_cmd.request.value = true;
				arming_client.call(arm_cmd);
				ROS_INFO_STREAM(current_state);
				offb_set_mode.request.custom_mode = "OFFBOARD";
				set_mode_client.call(offb_set_mode);
	
				rate.sleep();
				position.pose.position.x = 0;
				position.pose.position.y = 0;
				position.pose.position.z = 3;
				position_publisher.publish(position);
				rate.sleep();
				ros::spinOnce();
		}
		}
		break;
	}

}
}
void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
	ROS_INFO_STREAM(current_state);

}
void currentFCUPositionIndicator(const geometry_msgs::PoseStamped::ConstPtr& msg){
	g_position_current_fcu = *msg;
	ROS_INFO_STREAM("Haloo");
}
