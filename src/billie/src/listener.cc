/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/math/gzmath.hh>
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
geometry_msgs::PoseStamped rotor0;
geometry_msgs::PoseStamped rotor1;
geometry_msgs::PoseStamped rotor2;
geometry_msgs::PoseStamped rotor3;
gazebo::transport::PublisherPtr pub;
gazebo::msgs::Quaternion quaternion;
/////////////////////////////////////////////////
// Function is called everytime a message is received.

//Rotor0 variables
double rotor0_x = 0;
double rotor0_y = 0;
double rotor0_z = 0;
double rotor0_w = 0;

//Rotor1 variables 
double rotor1_x = 0;
double rotor1_y = 0;
double rotor1_z = 0;
double rotor1_w = 0;

//Rotor2 variables 
double rotor2_x = 0;
double rotor2_y = 0;
double rotor2_z = 0;
double rotor2_w = 0;

//Rotor3 variables 
double rotor3_x = 0;
double rotor3_y = 0;
double rotor3_z = 0;
double rotor3_w = 0;


int x = 0;


void cb(ConstPosesStampedPtr &msg_in);
void cb(ConstPosesStampedPtr &msg_in)
{
 rotor0_x = msg_in->pose(3).orientation().x();
 rotor0_y = msg_in->pose(3).orientation().y();
 rotor0_z = msg_in->pose(3).orientation().z();
 rotor0_w = msg_in->pose(3).orientation().w();

 rotor1_x = msg_in->pose(4).orientation().x();
 rotor1_y = msg_in->pose(4).orientation().y();
 rotor1_z = msg_in->pose(4).orientation().z();
 rotor1_w = msg_in->pose(4).orientation().w();

 rotor2_x = msg_in->pose(5).orientation().x();
 rotor2_y = msg_in->pose(5).orientation().y();
 rotor2_z = msg_in->pose(5).orientation().z();
 rotor2_w = msg_in->pose(5).orientation().w();

 rotor3_x = msg_in->pose(6).orientation().x();
 rotor3_y = msg_in->pose(6).orientation().y();
 rotor3_z = msg_in->pose(6).orientation().z();
 rotor3_w = msg_in->pose(6).orientation().w();

//gazebo::msgs::Set(&msg, quaternion)
//pub->Publish(quaternion);

}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);
	ros::init(_argc,_argv, "listener");
	ros::NodeHandle nh;

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/pose/local/info", cb);
  ros::Publisher rotor0_publisher = nh.advertise<geometry_msgs::PoseStamped>("/rotor0",10);
  ros::Publisher rotor1_publisher = nh.advertise<geometry_msgs::PoseStamped>("/rotor1",10);
  ros::Publisher rotor2_publisher = nh.advertise<geometry_msgs::PoseStamped>("/rotor2",10);
  ros::Publisher rotor3_publisher = nh.advertise<geometry_msgs::PoseStamped>("/rotor3",10);

  // Busy wait loop...replace with your own code as needed.
  while (ros::ok()){
	  //Rotor0 orientation data
	  rotor0.pose.orientation.x = rotor0_x;
	  rotor0.pose.orientation.y = rotor0_y;
	  rotor0.pose.orientation.z = rotor0_z;
	  rotor0.pose.orientation.w = rotor0_w;
	  rotor0_publisher.publish(rotor0);
	  //Rotor1 orientation data
	  rotor1.pose.orientation.x = rotor1_x;
	  rotor1.pose.orientation.y = rotor1_y;
	  rotor1.pose.orientation.z = rotor1_z;
	  rotor1.pose.orientation.w = rotor1_w;
	  rotor1_publisher.publish(rotor1);
	  //Rotor2 orientation data
	  rotor2.pose.orientation.x = rotor2_x;
	  rotor2.pose.orientation.y = rotor2_y;
	  rotor2.pose.orientation.z = rotor2_z;
	  rotor2.pose.orientation.w = rotor2_w;
	  rotor2_publisher.publish(rotor2);
	  //Rotor3 orientation data
	  rotor3.pose.orientation.x = rotor3_x;
	  rotor3.pose.orientation.y = rotor3_y;
	  rotor3.pose.orientation.z = rotor3_z;
	  rotor3.pose.orientation.w = rotor3_w;
	  rotor3_publisher.publish(rotor3);
      gazebo::common::Time::MSleep(10);
}
  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
