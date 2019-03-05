#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
int main(int argc, char **argv)
{
   ros::init(argc,argv, "aris");
   ros::NodeHandle umar;
   ros::ServiceClient set_mode_client = umar.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
mavros_msgs::SetMode data;
data.request.custom_mode = "AUTO.LAND";

while(1)
{
set_mode_client.call(data);
}
}
