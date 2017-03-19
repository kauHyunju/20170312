/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*#include "robot_localization/SetPose.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <gtest/gtest.h>
#include <iostream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include "ekf_localization_node_interfaces.h"


//struct nav_msgs_Odometry filtered_;
_Bool stateUpdated_;

//#define M_PI 3.141592

double M_PI = 3.141592;

#pragma pack(4)
typedef struct position{
	double x;
	double y;
	double z;

}position;

#pragma pack(4)
typedef struct orientation{
	double x;
	double y;
	double z;
	double w;
}orientation;

#pragma pack(4)
typedef struct linear{
	double x;
	double y;
	double z;
}linear;

#pragma pack(4)
typedef struct linear_acceleration{
	double x;
	double y;
	double z;
}linear_acceleration;

#pragma pack(4)
typedef struct angular{
	double x;
	double y;
	double z;
}angular;

#pragma pack(4)
typedef struct angular_velocity{
	double x;
	double y;
	double z;
}angular_velocity;


#pragma pack(4)
typedef struct pose{
	struct pose* pose;
	struct position* position;
	struct orientation* orientation;
	double covariance[36];
}pose;

#pragma pack(4)
typedef struct twist{
	struct twist* twist;
	struct linear* linear;
	struct angular* angular;
	double covariance[36];
}twist;

#pragma pack(4)
typedef struct header{
	char* frame_id;
	int stamp;
	int seq;
}header;


#pragma pack(4)
typedef struct nav_msgs_Odometry{

	double pose_position_x;
	double pose_position_y;
	double pose_position_z;//struct position* position;

	double pose_orientation_x;
	double pose_orientation_y;
	double pose_orientation_z;
	double pose_orientation_w;//struct orientation* ori;
	double pose_covariance[36];
	//struct pose* pose


	double twist_linear_x;
	double twist_linear_y;
	double twist_linear_z;

	double twist_angular_x;
	double twist_angular_y;
	double twist_angular_z;
	double twist_covariance[36];
	//struct twist* twist;

	char* header_frame_id;
	int header_stamp;
	int header_seq;

	//	struct header* header;

	char* child_frame_id;
}nav_msgs_Odometry;


#pragma pack(4)
typedef struct geometry_msgs_Pose{

	//struct geometry_msgs* geometry_msgs_Pose;
	double pose_position_x;
	double pose_position_y;
	double pose_position_z;//struct position* position;


	double pose_orientation_x;
	double pose_orientation_y;
	double pose_orientation_z;
	double pose_orientation_w;
	double pose_covariance[36];

	double twist_linear_x;
	double twist_linear_y;
	double twist_linear_z;

	double twist_angular_x;
	double twist_angular_y;
	double twist_angular_z;
	double twist_covariance[36];
	//struct twist* twist;

	char* header_frame_id;
	int header_stamp;
	int header_seq;

	//  struct header* header;
	//	struct pose* pose;
	//	struct twist* twist;
	char* child_frame_id;
}geometry_msgs_Pose;


#pragma pack(4)
typedef struct geometry_msgs_Twist{

	double pose_position_x;
	double pose_position_y;
	double pose_position_z;//struct position* position;


	double pose_orientation_x;
	double pose_orientation_y;
	double pose_orientation_z;
	double pose_orientation_w;
	double pose_covariance[36];

	double twist_linear_x;
	double twist_linear_y;
	double twist_linear_z;

	double twist_angular_x;
	double twist_angular_y;
	double twist_angular_z;
	double twist_covariance[36];
	//struct twist* twist;

	char* header_frame_id;
	int header_stamp;
	int header_seq;


	//	struct geometry_msgs* geometry_msgs_Pose;
	//    struct pose* pose;
	//	struct twist* twist;
	//	struct header* header;
	char* child_frame_id;
}geometry_msgs_Twist;

#pragma pack(4)
typedef struct sensor_msgs_Imu_Pose{
	//struct orientation* orientation;
	double orientation_x;
	double orientation_y;
	double orientation_z;
	double orientation_w;

	double orientation_covariance[9];
	char* header_frame_id;
	int header_stamp;
	int header_seq;
	//	struct header* header;
}sensor_msgs_Imu_Pose;


#pragma pack(4)
typedef struct sensor_msgs_Imu_Twist{
	//	struct angular_velocity* angular_velocity;
	double angular_velocity_x;
	double angular_velocity_y;
	double angular_velocity_z;


	double angular_velocity_covariance[9];
	char* header_frame_id;
	int header_stamp;
	int header_seq;


	//	struct header* header;
}sensor_msgs_Imu_Twist;


#pragma pack(4)
typedef struct sensor_msgs_Imu_Acc{
	//struct linear_acceleration* linear_acceleration;
	double linear_acceleration_x;
	double linear_acceleration_y;
	double linear_acceleration_z;

	double linear_acceleration_covariance[9];
	//struct header* header;
	char* header_frame_id;
	int header_stamp;
	int header_seq;
}sensor_msgs_Imu_Acc;

#pragma pack(4)
typedef struct sensor_msgs_Imu{
	//struct orientation* orientation;

	double orientation_x;
	double orientation_y;
	double orientation_z;
	double orientation_w;

	double orientation_covariance[9];
	double angular_velocity_covariance[9];
	//	struct header* header;
	char* header_frame_id;
	int header_stamp;
	int header_seq;


}sensor_msgs_Imu;

#pragma pack(4)
typedef struct ros_Publisher{
	char* topic;
	int queuesize;
}ros_Publisher;

#pragma pack(4)
typedef struct ros_Subscriber{
	char* topic;
	int queuesize;
}ros_Subscriber;

struct ros_NodeHandle{
};

int mystrcmp(const char *s, const char* t);

void nh_serviceClient();

struct ros_Publisher* nh_advertise(char* topic_name, int queue_size);
struct ros_Subscriber* nh_subscribe(char* topic_name, int queue_size);


void Duration_sleep(double seconds);
void loopRate_sleep(int fq);
void publish(char* meta, int size, int seq);


int mystrcmp(const char *s, const char* t){//*

	int i;

	for(i=0;s[i]==t[i];i++)

		if(s[i]=='\0')

			return 0;

	return s[i]-t[i];
}

void nh_serviceClient(){
}

struct ros_Publisher* nh_advertise(char* topic_name, int queue_size){
	struct ros_Publisher* pub = malloc(sizeof(ros_Publisher));

	pub->topic = topic_name;
	pub->queuesize = queue_size;
	return pub;

}
struct ros_Subscriber* nh_subscribe(char* topic_name, int queue_size){
	struct ros_Subscriber* sub = malloc(sizeof(ros_Subscriber));

	/*  int odom_input1_size =0;
		int odom_input2_size =0;
	 */
	sub->topic = topic_name;
	sub->queuesize = queue_size;
	return sub;
}

void Duration_sleep(double seconds){
	double sec;
	sec = seconds;

}

void loopRate_sleep(int fq){
	int sleep_fq=0;
	sleep_fq = fq;
	/*  time_t timer;
		struct tm *start;
		timer = time(NULL);
		start = localtime(&timer);
		double expected_cycle_time = (1.0 / fq);

		expected_end = start + expected_cycle_time;

		timer = time(NULL);
		end = localtime(&timer);

		double sleep_time;
		sleep_time = expected_end - end;
	 */
	//printf("sleep rate %f\n",fq);
}

void publish(char* meta, int size, int sq){
	char* topic;
//	topic=meta;
	char* topic_name=meta;
	int topic_size=size;
	int pub_seq=sq;

/*	if(mystrcmp(meta,"odom")==0){
		//printf("odom pubsize:%d\n",size);
		odom_size=size;
		odom_seq = seq;
	}
	else if(mystrcmp(meta,"twist")==0){
		//printf("twist pubsize:%d\n",size);
		twist_size=size;
		twist_seq = seq;
	}
	else if(mystrcmp(meta,"imu")==0){
		//printf("imu pubsize:%d\n",size);
		imu_size=size;
		imu_seq = seq;
	}
	else if(mystrcmp(meta,"imuIgnore")==0){
		//printf("imuIgnore pubsize:%d\n",size);
		imuIgnore_size=size;
		imuIgnore_seq = seq;
	}
	else if(mystrcmp(meta,"pose")==0){
		//printf("pose pubsize:%d\n",size);
		pose_size=size;
		pose_seq = seq;
	}else{
	   else_size=size;
	   else_seq = seq;
	   }*/

}

void resetFilter()
{
	/*  ros::NodeHandle nh;
		ros::ServiceClient client = nh.serviceClient<robot_localization::SetPose>("/set_pose");

		robot_localization::SetPose setPose;
		setPose.request.pose.pose.pose.orientation.w = 1;
		setPose.request.pose.header.frame_id = "odom";
		for (size_t ind = 0; ind < 36; ind+=7)
		{
		setPose.request.pose.pose.covariance[ind] = 1e-6;
		}

		setPose.request.pose.header.stamp = ros::Time::now();
		client.call(setPose);
		setPose.request.pose.header.seq++;
		ros::spinOnce();
		ros::Duration(0.01).sleep();
		stateUpdated_ = false;

		double deltaX = 0.0;
		double deltaY = 0.0;
		double deltaZ = 0.0;

		while (!stateUpdated_ || ::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ) > 0.1)
		{
		ros::spinOnce();
		ros::Duration(0.01).sleep();

		deltaX = filtered_.pose.pose.position.x - setPose.request.pose.pose.pose.position.x;
		deltaY = filtered_.pose.pose.position.y - setPose.request.pose.pose.pose.position.y;
		deltaZ = filtered_.pose.pose.position.z - setPose.request.pose.pose.pose.position.z;
		}

		EXPECT_LT(::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ), 0.1);
	 */
	//printf("resetFilter()\n");
}

/*void filterCallback(const nav_msgs::OdometryConstPtr &msg)
  {
  filtered_ = *msg;
  stateUpdated_ = true;
  }*/

//TEST(InterfacesTest, OdomPoseBasicIO)
void OdomPoseBasicIOTEST()
{
	stateUpdated_ = 0;//false;
	int pubsize=0;
	int seq=1;
	//ros::NodeHandle nh;
	struct ros_Publisher* odomPub;
//	odomPub = nh_advertise("/odom_input0",5);//토픽생성하는부분이라..
	//  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odom_input0", 5);
	struct ros_Subscriber* filteredSub;
//	filteredSub = nh_subscribe("/odometry/filtered",1);
	//	filterCallback(filteredSub);//?????????????????????????????????
	//ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

	nav_msgs_Odometry* odom = (struct nav_msgs_Odometry*)malloc(sizeof(nav_msgs_Odometry));

	pubsize+=sizeof(nav_msgs_Odometry);

	/*odom->pose = (struct pose*)malloc(sizeof(pose));
	  odom->pose->pose=(struct pose*)malloc(sizeof(pose));
	  odom->pose->pose->position=(struct position*)malloc(sizeof(position));

	  pubsize+=sizeof(pose);
	  pubsize+=sizeof(pose);
	  pubsize+=sizeof(position);
	 */
	/*odom->pose_position_x = 20.0;
	odom->pose_position_y= 10.0;
	odom->pose_position_z=-40.0;
*/
	/*odom->pose->pose->position->x = 20.0;
	  odom->pose->pose->position->y = 10.0;
	  odom->pose->pose->position->z = -40.0;
	 */
/*	odom->pose_covariance[0] = 2.0;
	odom->pose_covariance[7]= 2.0;
	odom->pose_covariance[14] = 2.0;
*/
	/*	odom->pose->covariance[0] = 2.0;
		odom->pose->covariance[7] = 2.0;
		odom->pose->covariance[14] = 2.0;
	 */

	/*odom->header = (struct header*)malloc(sizeof(header));
	  pubsize+=sizeof(header);
	  odom->header->frame_id = "odom";
	 */
/*	odom->header_frame_id = "odom";

	odom->child_frame_id = "base_link";

	odom->header_seq=1;
*/
	//printf("pubsize:%d\n",pubsize);
	//	loopRate(50);
	//size_t i;
	int i;

	for (i = 0; i < 50; ++i)
	{
		//	printf("sizeof(odom):%d\n",sizeof(odom));
		//		odom.header.stamp = ros::Time::now();
		//		odom->header->stamp = 10;
		//	odomPub.publish(odom);//************
		publish("odom",pubsize,seq);

		//		publish(odom);
		//		ros::spinOnce();

		//		loopRate.sleep();
		loopRate_sleep(50);

//		odom->header_seq++;
		seq++;
	}

	// Now check the values from the callback
	/*  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - odom.pose.pose.position.x), 0.01);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 0.01);  // Configuration for this variable for this sensor is false
		EXPECT_LT(::fabs(filtered_.pose.pose.position.z - odom.pose.pose.position.z), 0.01);

		EXPECT_LT(filtered_.pose.covariance[0], 0.5);
		EXPECT_LT(filtered_.pose.covariance[7], 0.25);  // Configuration for this variable for this sensor is false
		EXPECT_LT(filtered_.pose.covariance[14], 0.5);
	 */
	//	seq = odom->header->seq;

	/*	free(odom->pose->pose->position);
		free(odom->pose->pose);
		free(odom->pose);
		free(odom->header);*/
	free(odom);

	resetFilter();
}

void OdomTwistBasicIOTEST()
{
	//	ros::NodeHandle nh;
	struct ros_Publisher* odomPub;
	//odomPub = nh_advertise("/odom_input2", 5);
	struct ros_Subscriber* filteredSub;
	//filteredSub = nh_subscribe("/odometry/filtered", 1);
	//filterCallback(???)

	int pubsize=0;
	int seq=1;
	//nav_msgs_Odometry* odom = (struct nav_msgs_Odometry*)malloc(sizeof(nav_msgs_Odometry));
	pubsize += sizeof(nav_msgs_Odometry);

	/*odom->twist = (struct twist*)malloc(sizeof(twist));
	  odom->twist->twist = (struct twist*)malloc(sizeof(twist));
	  odom->twist->twist->linear = (struct linear*)malloc(sizeof(linear));
	  odom->twist->twist->angular = (struct angular*)malloc(sizeof(angular));

	  pubsize+=sizeof(twist);
	  pubsize+=sizeof(twist);
	  pubsize+=sizeof(linear);
	  pubsize+=sizeof(angular);
	 

	odom->twist_linear_x=5.0;
	odom->twist_linear_y=0.0;
	odom->twist_linear_z=0.0;
	odom->twist_angular_x=0.0;
	odom->twist_angular_y=0.0;
	odom->twist_angular_z=0.0;

	odom->twist->twist->linear->x = 5.0;
	  odom->twist->twist->linear->y = 0.0;
	  odom->twist->twist->linear->z = 0.0;
	  odom->twist->twist->angular->x = 0.0;
	  odom->twist->twist->angular->y = 0.0;
	  odom->twist->twist->angular->z = 0.0;
	 */
	//size_t ind;
	int ind;
	for (ind = 0; ind < 36; ind+=7)
	{
	//	odom->twist_covariance[ind] = 1e-6;
	}

	//	odom->header = (struct header*)malloc(sizeof(header));
	//	pubsize+=sizeof(header);
	//odom->header_frame_id = "odom";
	//odom->child_frame_id = "base_link";

	//odom->header_seq=1;
	//	ros::Rate loopRate(20);
	//size_t i;
	int i;

	for (i = 0; i < 400; ++i)
	{
		//		odom.header.stamp = ros::Time::now();
		//		odomPub.publish(odom);
		publish("odom",pubsize,seq);
		//		ros::spinOnce();

		//		loopRate.sleep();
		loopRate_sleep(20);

	//	odom->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

	/*	EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x), 0.1);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 100.0), 2.0);
	 */

//	resetFilter();

	//odom->twist_linear_x=0.0;
	//odom->twist_linear_y=5.0;

	//odom->twist->twist->linear->x = 0.0;
	//	odom->twist->twist->linear->y = 5.0;

	//	loopRate = ros::Rate(20);
	i=0;
	for (i = 0; i < 200; ++i)
	{
		//odom.header.stamp = ros::Time::now();
		//odomPub.publish(odom);
		publish("odom",pubsize,seq);
		//		ros::spinOnce();

		//		loopRate.sleep();
		loopRate_sleep(20);

	//	odom->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

	/*	EXPECT_LT(::fabs(filtered_.twist.twist.linear.y - odom.twist.twist.linear.y), 0.1);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 50.0), 1.0);
	 */
	//resetFilter();

	//odom->twist_linear_y = 0.0;
	//odom->twist_linear_z = 5.0;

	//	loopRate = ros::Rate(20);
	i=0;
	for (i = 0; i < 100; ++i)
	{
		//		odom.header.stamp = ros::Time::now();
		//		odomPub.publish(odom);
		publish("odom",pubsize,seq);
		//		ros::spinOnce();

		//		loopRate.sleep();
		loopRate_sleep(20);
	//	odom->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

	/*	EXPECT_LT(::fabs(filtered_.twist.twist.linear.z - odom.twist.twist.linear.z), 0.1);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.z - 25.0), 1.0);
	 */
	//resetFilter();

	//odom->twist_linear_z = 0.0;
	//odom->twist_linear_x = 1.0;
	//odom->twist_angular_z = (M_PI/2) / (100.0 * 0.05);

	//	loopRate = ros::Rate(20);
	i=0;
	for (i = 0; i < 100; ++i)
	{
		//		odom.header.stamp = ros::Time::now();
		//		odomPub.publish(odom);
		publish("odom",pubsize,seq);
		//		ros::spinOnce();

		//		loopRate.sleep();
		loopRate_sleep(20);

	//	odom->header_seq++;
	//	seq++;
	}
	//	ros::spinOnce();

	/*	EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x), 0.1);
		EXPECT_LT(::fabs(filtered_.twist.twist.angular.z - odom.twist.twist.angular.z), 0.1);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.x - filtered_.pose.pose.position.y), 0.5);
	 */
	/*resetFilter();

	odom->twist_linear_x = 0.0;
	odom->twist_angular_z = 0.0;
	odom->twist_angular_x = -(M_PI/2) / (100.0 * 0.05);
*/
	// First, roll the vehicle on its side
	//	loopRate = ros::Rate(20);
	i=0;
	for (i = 0; i < 100; ++i)
	{
		//		odom.header.stamp = ros::Time::now();
		//		odomPub.publish(odom);
		publish("odom",pubsize,seq);
		//		ros::spinOnce();

		//		loopRate.sleep();
		loopRate_sleep(20);

//		odom->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

/*	odom->twist_angular_x = 0.0;
	odom->twist_angular_y = (M_PI/2) / (100.0 * 0.05);
*/
	// Now, pitch it down (positive pitch velocity in vehicle frame)
	//	loopRate = ros::Rate(20);
	i=0;
	for (i = 0; i < 100; ++i)
	{
		//		odom.header.stamp = ros::Time::now();
		//		odomPub.publish(odom);
		publish("odom",pubsize,seq);
		//		ros::spinOnce();

		//		loopRate.sleep();
		loopRate_sleep(20);

//		odom->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

//	odom->twist_angular_y = 0.0;
//	odom->twist_linear_x = 3.0;

	// We should now be on our side and facing -Y. Move forward in
	// the vehicle frame X direction, and make sure Y decreases in
	// the world frame.
	//	loopRate = ros::Rate(20);
	i=0;
	for (i = 0; i < 100; ++i)
	{
		//		odom.header.stamp = ros::Time::now();
		//		odomPub.publish(odom);
		publish("odom",pubsize,seq);
		//		ros::spinOnce();

		//		loopRate.sleep();
		loopRate_sleep(20);

//		odom->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

	/*	EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x), 0.1);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.y + 15), 1.0);
	 */
	/*	free(odom->twist->twist->angular);
		free(odom->twist->twist->linear);
		free(odom->twist->twist);
		free(odom->twist);
		free(odom->header);
		free(odom);*/
//	resetFilter();
}
void PoseBasicIOTEST()
{
	int pubsize=0;
	//	struct ros_Publisher* odomPub = malloc(sizeof(ros_Publisher));
	//	ros::NodeHandle nh;
	//	ros::Publisher posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_input0", 5);

	int seq=1;
	struct ros_Publisher* odomPub;
//	odomPub = nh_advertise("/pose_input0",5);

	struct ros_Subscriber* filteredSub;
//	filteredSub = nh_subscribe("/odometry/filtered", 1);
	//ilterCallback(filteredSub);///

	//	geometry_msgs::PoseWithCovarianceStamped pose;
	struct geometry_msgs_Pose* pose = (struct geometry_msgs_Pose*)malloc(sizeof(geometry_msgs_Pose));
	pubsize+=sizeof(geometry_msgs_Pose);

	/*pose->pose = (struct pose*)malloc(sizeof(pose)+sizeof(double)*36);
	  pose->pose->pose = (struct pose*)malloc(sizeof(pose));
	  pose->pose->pose->position=(struct position*)malloc(sizeof(position));
	  pose->pose->pose->orientation=(struct orientation*)malloc(sizeof(orientation));

	  pubsize+=sizeof(pose);
	  pubsize+=sizeof(pose);
	  pubsize+=sizeof(position);
	  pubsize+=sizeof(orientation);
	 
	pose->pose_position_x = 20.0;
	pose->pose_position_y = 10.0;
	pose->pose_position_z = -40.0;
	pose->pose_orientation_x = 0;
	pose->pose_orientation_y = 0;
	pose->pose_orientation_z = 0;
	pose->pose_orientation_w = 1;
*/
	//size_t ind=0;
	int ind=0;

	for (ind = 0; ind < 36; ind+=7)
	{
//		pose->pose_covariance[ind] = 1e-6;
	}

	//	pose->header = (struct header*)malloc(sizeof(header));
	//	pubsize+=sizeof(header);
//	pose->header_frame_id = "odom";
//	pose->header_seq=1;
	//	ros::Rate loopRate = ros::Rate(50);
	//size_t i=0;
	int i=0;
	for (i = 0; i < 50; ++i)
	{
		//		pose.header.stamp = ros::Time::now();
		//		posePub.publish(pose);
		//		ros::spinOnce();

		publish("pose",pubsize,seq);
		loopRate_sleep(50);

//		pose->header_seq++;
		seq++;
	}
	/*
	// Now check the values from the callback
	EXPECT_LT(::fabs(filtered_.pose.pose.position.x - pose.pose.pose.position.x), 0.1);
	EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 0.1);  // Configuration for this variable for this sensor is false
	EXPECT_LT(::fabs(filtered_.pose.pose.position.z - pose.pose.pose.position.z), 0.1);

	EXPECT_LT(filtered_.pose.covariance[0], 0.5);
	EXPECT_LT(filtered_.pose.covariance[7], 0.25);  // Configuration for this variable for this sensor is false
	EXPECT_LT(filtered_.pose.covariance[14], 0.5);
	 */
	/*	free(pose->pose->pose->orientation);
		free(pose->pose->pose->position);
		free(pose->pose->pose);
		free(pose->pose);//??...
		free(pose->header);*/
//	free(pose);
//	resetFilter();
}

void TwistBasicIOTEST()
{
	int pubsize=0;
	int seq=1;
	//	ros::NodeHandle nh;
	struct ros_Publisher* twistPub;
//	twistPub = nh_advertise("/twist_input0", 5);
	struct ros_Subscriber* filteredSub;
//	filteredSub = nh_subscribe("/odometry/filtered", 1);
	//filterCallback();

	//	geometry_msgs::TwistWithCovarianceStamped twist;
	geometry_msgs_Twist* twist = (struct geometry_msgs_Twist*)malloc(sizeof(geometry_msgs_Twist));
	pubsize+=sizeof(geometry_msgs_Twist);

	/*twist->twist = (struct twist*)malloc(sizeof(twist)+sizeof(double)*36);
	  twist->twist->twist = (struct twist*)malloc(sizeof(twist));
	  twist->twist->twist->linear = (struct linear*)malloc(sizeof(linear));
	  twist->twist->twist->angular = (struct angular*)malloc(sizeof(angular));

	  pubsize+=sizeof(twist);
	  pubsize+=sizeof(twist);
	  pubsize+=sizeof(linear);
	  pubsize+=sizeof(angular);
	 
	twist->twist_linear_x = 5.0;
	twist->twist_linear_y = 0;
	twist->twist_linear_z = 0;
	twist->twist_angular_x = 0;
	twist->twist_angular_y = 0;
	twist->twist_angular_z = 0;
*/
	//size_t ind=0;
	int ind=0;

	for (ind = 0; ind < 36; ind+=7)
	{
//		twist->twist_covariance[ind] = 1e-6;
	}

	/*	twist->header = (struct header*)malloc(sizeof(header));
		pubsize+=sizeof(header);*/
//	twist->header_frame_id = "base_link";
//	twist->header_seq=1;
	//	ros::Rate loopRate = ros::Rate(20);
	//size_t i=0;
	int i=0;
	for (i = 0; i < 400; ++i)
	{
		//		twist.header.stamp = ros::Time::now();
		//		twistPub.publish(twist);
		//		ros::spinOnce();

		publish("twist",pubsize,seq);
		loopRate_sleep(20);

//		twist->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

	//	EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x), 0.1);
	//	EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 100.0), 2.0);

//	resetFilter();

//	twist->twist_linear_x = 0.0;
//	twist->twist_linear_y = 5.0;

	//	loopRate = ros::Rate(20);
	for (i = 0; i < 200; ++i)
	{
		//		twist.header.stamp = ros::Time::now();
		//		twistPub.publish(twist);
		//		ros::spinOnce();
		publish("twist",pubsize,seq);

		loopRate_sleep(20);

//		twist->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

	//   EXPECT_LT(::fabs(filtered_.twist.twist.linear.y - twist.twist.twist.linear.y), 0.1);
	// EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 50.0), 1.0);

//	resetFilter();

//	twist->twist_linear_y = 0.0;
//	twist->twist_linear_z = 5.0;

	//	loopRate = ros::Rate(20);
	for (i = 0; i < 100; ++i)
	{
		//		twist.header.stamp = ros::Time::now();
		//		twistPub.publish(twist);
		//		ros::spinOnce();

		publish("twist",pubsize,seq);
		loopRate_sleep(20);

//		twist->header_seq++;
		seq++;
	}
	//ros::spinOnce();

	//   EXPECT_LT(::fabs(filtered_.twist.twist.linear.z - twist.twist.twist.linear.z), 0.1);
	// EXPECT_LT(::fabs(filtered_.pose.pose.position.z - 25.0), 1.0);

//	resetFilter();

//	twist->twist_linear_z = 0.0;
//	twist->twist_linear_x = 1.0;
//	twist->twist_angular_z = (M_PI/2) / (100.0 * 0.05);

	//	loopRate = ros::Rate(20);
	for (i = 0; i < 100; ++i)
	{
		//		twist.header.stamp = ros::Time::now();
		//		twistPub.publish(twist);
		//		ros::spinOnce();

		publish("twist",pubsize,seq);

		loopRate_sleep(20);

//		twist->header_seq++;
//		seq++;
	}
	//	ros::spinOnce();

	//   EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x), 0.1);
	// EXPECT_LT(::fabs(filtered_.twist.twist.angular.z - twist.twist.twist.angular.z), 0.1);
	// EXPECT_LT(::fabs(filtered_.pose.pose.position.x - filtered_.pose.pose.position.y), 0.5);

//	resetFilter();

//	twist->twist_linear_x = 0.0;
//	twist->twist_angular_z = 0.0;
//	twist->twist_angular_x = -(M_PI/2) / (100.0 * 0.05);

	// First, roll the vehicle on its side
	//	loopRate = ros::Rate(20);
	for (i = 0; i < 100; ++i)
	{
		//	twist->header.stamp = ros::Time::now();
		//	twistPub.publish(twist);
		//	ros::spinOnce();
		publish("twist",pubsize,seq);

		loopRate_sleep(20);

//		twist->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

//	twist->twist_angular_x = 0.0;
//	twist->twist_angular_y = (M_PI/2) / (100.0 * 0.05);

	// Now, pitch it down (positive pitch velocity in vehicle frame)
	//	loopRate = ros::Rate(20);
	for (i = 0; i < 100; ++i)
	{
		//twist.header.stamp = ros::Time::now();
		//twistPub.publish(twist);
		//ros::spinOnce();

		publish("twist",pubsize,seq);
		loopRate_sleep(20);

//		twist->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

//	twist->twist_angular_y = 0.0;
//	twist->twist_linear_x = 3.0;

	// We should now be on our side and facing -Y. Move forward in
	// the vehicle frame X direction, and make sure Y decreases in
	// the world frame.
	//	loopRate = ros::Rate(20);
	for (i = 0; i < 100; ++i)
	{
		//		twist.header.stamp = ros::Time::now();
		//		twistPub.publish(twist);
		//		ros::spinOnce();

		publish("twist",pubsize,seq);
		loopRate_sleep(20);

//		twist->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

	//	EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x), 0.1);
	//	EXPECT_LT(::fabs(filtered_.pose.pose.position.y + 15), 1.0);


	/*	free(twist->twist->twist->linear);
		free(twist->twist->twist->angular);
		free(twist->twist->twist);
		free(twist->twist);
		free(twist->header);*/
//	free(twist);

//	resetFilter();
}

void ImuPoseBasicIOTEST()
{
	//	ros::NodeHandle nh;
	struct ros_Publisher* imuPub;
//	imuPub = nh_advertise("/imu_input0", 5);
	struct ros_Subscriber* filteredSub;
//	filteredSub = nh_subscribe("/odometry/filtered", 1);
	//, &filterCallback);

	int pubsize=0;
	int seq=1;
	struct sensor_msgs_Imu_Pose* imu = (struct sensor_msgs_Imu_Pose*)malloc(sizeof(sensor_msgs_Imu_Pose)+sizeof(double)*9);
	pubsize+=sizeof(sensor_msgs_Imu_Pose);

	//tf2::Quaternion quat;
	//quat.setRPY(M_PI/4, -M_PI/4, M_PI/2);
	/*struct orientation* quat = (struct orientation*)malloc(sizeof(orientation));

	  imu->orientation = (struct orientation*)malloc(sizeof(orientation));
	  pubsize+=sizeof(orientation);

	//	imu->orientation = tf2::toMsg(quat);

	imu->orientation = quat;
	 */
	//size_t ind;
	int ind;
	for (ind = 0; ind < 9; ind+=4)
	{
//		imu->orientation_covariance[ind] = 1e-6;
	}

	//	imu->header = (struct header*)malloc(sizeof(header));
	//	pubsize+=sizeof(header);
//	imu->header_frame_id = "base_link";
//	imu->header_seq=1;
	// Make sure the pose reset worked. Test will timeout
	// if this fails.
	//	ros::Rate loopRate(50);
//	size_t i;
	int i;
	for (i = 0; i < 50; ++i)
	{
		//imu.header.stamp = ros::Time::now();
		// imuPub.publish(imu);
		//ros::spinOnce();

		publish("imu",pubsize,seq);
		loopRate_sleep(50);

//		imu->header_seq++;
		seq++;
	}

	// Now check the values from the callback
	//	tf2::fromMsg(filtered_.pose.pose.orientation, quat);
	//		tf2::Matrix3x3 mat(quat);
	//		double r, p, y;
	//		mat.getRPY(r, p, y);
	//		EXPECT_LT(::fabs(r - M_PI/4), 0.1);
	//		EXPECT_LT(::fabs(p + M_PI/4), 0.1);
	//		EXPECT_LT(::fabs(y - M_PI/2), 0.1);

	//		EXPECT_LT(filtered_.pose.covariance[21], 0.5);
	//		EXPECT_LT(filtered_.pose.covariance[28], 0.25);
	//		EXPECT_LT(filtered_.pose.covariance[35], 0.5);

//	resetFilter();

	// Test to see if the orientation data is ignored when we set the
	// first covariance value to -1
	int imuIgnorepubsize=0;
	int seq2=1;
	struct sensor_msgs_Imu_Pose* imuIgnore = (struct sensor_msgs_Imu_Pose*)malloc(sizeof(sensor_msgs_Imu_Pose)+sizeof(double)*9);
	imuIgnorepubsize+=sizeof(sensor_msgs_Imu_Pose);

	/*	imuIgnore->orientation = (struct orientation*)malloc(sizeof(orientation));
		imuIgnorepubsize+=sizeof(orientation);
	 */
//	imuIgnore->orientation_x = 0.1;
//	imuIgnore->orientation_y = 0.2;
//	imuIgnore->orientation_z = 0.3;
//	imuIgnore->orientation_w = 0.4;
//	imuIgnore->orientation_covariance[0] = -1;

	//	imuIgnore->header = (struct header*)malloc(sizeof(header));
//	imuIgnore->header_seq=1;
	//	loopRate = ros::Rate(50);
	i=0;
	for (i = 0; i < 50; ++i)
	{
		//	imuIgnore.header.stamp = ros::Time::now();
		//		imuPub.publish(imuIgnore);
		publish("imuIgnore",imuIgnorepubsize,seq2);
		//////////////////////////////////????
		loopRate_sleep(50);
		//		ros::spinOnce();

//		imuIgnore->header_seq++;
		seq2++;
	}

	//tf2::fromMsg(filtered_.pose.pose.orientation, quat);
	// mat.setRotation(quat);
	//	  mat.getRPY(r, p, y);
	//	  EXPECT_LT(::fabs(r), 1e-3);
	//	  EXPECT_LT(::fabs(p), 1e-3);
	//	  EXPECT_LT(::fabs(y), 1e-3);


	//free(imu->orientation);
	//free(imu->header);
//	free(imu);
	//free(imuIgnore->orientation);
	//free(imuIgnore->header);
//	free(imuIgnore);

//	resetFilter();
}
void ImuTwistBasicIOTEST()
{
	//	ros::NodeHandle nh;
	struct ros_Publisher* imuPub;
//	imuPub = nh_advertise("/imu_input1", 5);
	struct ros_Subscriber* filteredSub;
//	filteredSub = nh_subscribe("/odometry/filtered", 1);
	//, &filterCallback);

	int pubsize=0;
	int seq=1;
	struct sensor_msgs_Imu_Twist* imu = (struct sensor_msgs_Imu_Twist*)malloc(sizeof(sensor_msgs_Imu_Twist)+sizeof(double)*9);
	pubsize += sizeof(sensor_msgs_Imu_Twist);
	//tf2::Quaternion quat;

	//imu->angular_velocity = (struct angular_velocity*)malloc(sizeof(angular_velocity));
	//pubsize += sizeof(angular_velocity);

//	imu->angular_velocity_x = (M_PI / 2.0);

	//size_t ind=0;
	int ind=0;
	for (ind = 0; ind < 9; ind+=4)
	{
//		imu->angular_velocity_covariance[ind] = 1e-6;
	}

//	imu->header_frame_id = "base_link";
//	imu->header_seq=1;
	//	ros::Rate loopRate(50);
	//size_t i=0;
	int i=0;
	for (i = 0; i < 50; ++i)
	{
		/*		imu.header.stamp = ros::Time::now();
				imuPub.publish(imu);
				loopRate.sleep();
				ros::spinOnce();
		 */
		publish("imu",pubsize,seq);
		loopRate_sleep(50);
//		imu->header_seq++;
		seq++;
	}

	// Now check the values from the callback
	/*	tf2::fromMsg(filtered_.pose.pose.orientation, quat);
		tf2::Matrix3x3 mat(quat);
		double r, p, y;
		mat.getRPY(r, p, y);

	// Tolerances may seem loose, but the initial state covariances
	// are tiny, so the filter is sluggish to accept velocity data
	EXPECT_LT(::fabs(r - M_PI / 2.0), 0.7);
	EXPECT_LT(::fabs(p), 0.1);
	EXPECT_LT(::fabs(y), 0.1);

	EXPECT_LT(filtered_.twist.covariance[21], 1e-3);
	EXPECT_LT(filtered_.pose.covariance[21], 0.1);
	 */
//	resetFilter();

//	imu->angular_velocity_x = 0.0;
//	imu->angular_velocity_y = -(M_PI / 2.0);

	// Make sure the pose reset worked. Test will timeout
	// if this fails.
	//	loopRate = ros::Rate(50);
	for (i = 0; i < 50; ++i)
	{
		/*imu.header.stamp = ros::Time::now();
		  imuPub.publish(imu);
		  loopRate.sleep();
		  ros::spinOnce();
		 */
		publish("imu",pubsize,seq);
		loopRate_sleep(50);
//		imu->header_seq++;
		seq++;
	}

	// Now check the values from the callback
	/*	tf2::fromMsg(filtered_.pose.pose.orientation, quat);
		mat.setRotation(quat);
		mat.getRPY(r, p, y);
		EXPECT_LT(::fabs(r), 0.1);
		EXPECT_LT(::fabs(p + M_PI / 2.0), 0.7);
		EXPECT_LT(::fabs(y), 0.1);

		EXPECT_LT(filtered_.twist.covariance[28], 1e-3);
		EXPECT_LT(filtered_.pose.covariance[28], 0.1);
	 */
//	resetFilter();

//	imu->angular_velocity_y = 0;
//	imu->angular_velocity_z = M_PI / 4.0;

	// Make sure the pose reset worked. Test will timeout
	// if this fails.
	//	loopRate = ros::Rate(50);
	for (i = 0; i < 50; ++i)
	{
		/*		imu.header.stamp = ros::Time::now();
				imuPub.publish(imu);
				loopRate.sleep();
				ros::spinOnce();
		 */
		publish("imu",pubsize,seq);
//		imu->header_seq++;
		seq++;
	}

	// Now check the values from the callback
	/*	tf2::fromMsg(filtered_.pose.pose.orientation, quat);
		mat.setRotation(quat);
		mat.getRPY(r, p, y);
		EXPECT_LT(::fabs(r), 0.1);
		EXPECT_LT(::fabs(p), 0.1);
		EXPECT_LT(::fabs(y - M_PI / 4.0), 0.7);

		EXPECT_LT(filtered_.twist.covariance[35], 1e-3);
		EXPECT_LT(filtered_.pose.covariance[35], 0.1);
	 */
//	resetFilter();

	// Test to see if the angular velocity data is ignored when we set the
	// first covariance value to -1
	int imuIgnorepubsize=0;
	int seq2=1;
	struct sensor_msgs_Imu_Twist* imuIgnore = (struct sensor_msgs_Imu_Twist*)malloc(sizeof(sensor_msgs_Imu_Twist));
	imuIgnorepubsize+=sizeof(sensor_msgs_Imu_Twist);

	//	imuIgnore->angular_velocity = (struct angular_velocity*)malloc(sizeof(angular_velocity));
	//	imuIgnorepubsize+=sizeof(angular_velocity);

//	imuIgnore->angular_velocity_x = 100;
//	imuIgnore->angular_velocity_y = 100;
//	imuIgnore->angular_velocity_z = 100;
//	imuIgnore->angular_velocity_covariance[0] = -1;

	//	loopRate = ros::Rate(50);
	//	imuIgnore->header = (struct header*)malloc(sizeof(header));
	//	imuIgnorepubsize+=sizeof(header);

//	imuIgnore->header_seq=1;
	for (i = 0; i < 50; ++i)
	{
		/*	imuIgnore.header.stamp = ros::Time::now();
			imuPub.publish(imuIgnore);
			loopRate.sleep();
			ros::spinOnce();
		 */
		publish("imuIgnore",imuIgnorepubsize,seq2);
		loopRate_sleep(50);
//		imuIgnore->header_seq++;
		seq2++;
	}

	/*	tf2::fromMsg(filtered_.pose.pose.orientation, quat);
		mat.setRotation(quat);
		mat.getRPY(r, p, y);
		EXPECT_LT(::fabs(r), 1e-3);
		EXPECT_LT(::fabs(p), 1e-3);
		EXPECT_LT(::fabs(y), 1e-3);
	 */
//	resetFilter();

	//	free(imu->header);
	//	free(imu->angular_velocity);
//	free(imu);
	//	free(imuIgnore->angular_velocity);
	//	free(imuIgnore->header);
//	free(imuIgnore);
}
//TEST(InterfacesTest, ImuAccBasicIO)
void ImuAccBasicIOTEST()
{
	//	ros::NodeHandle nh;
	struct ros_Publisher* imuPub;
//	imuPub = nh_advertise("/imu_input2", 5);
	struct ros_Subscriber* filteredSub;
//	filteredSub = nh_subscribe("/odometry/filtered", 1);
	//, &filterCallback);

	int pubsize=0;
	int seq=1;
	struct sensor_msgs_Imu_Acc* imu = (struct sensor_msgs_Imu_Acc*)malloc(sizeof(sensor_msgs_Imu_Acc)+sizeof(double)*9);
	pubsize+=sizeof(sensor_msgs_Imu_Acc);

	//imu->header = (struct header*)malloc(sizeof(header));
	//pubsize+=sizeof(header);

//	imu->header_frame_id = "base_link";
//	imu->linear_acceleration_covariance[0] = 1e-6;
//	imu->linear_acceleration_covariance[4] = 1e-6;
//	imu->linear_acceleration_covariance[8] = 1e-6;

	//	imu->linear_acceleration = (struct linear_acceleration*)malloc(sizeof(linear_acceleration));
	//	pubsize+=sizeof(linear_acceleration);

//	imu->linear_acceleration_x = 1;
//	imu->linear_acceleration_y = -1;
//	imu->linear_acceleration_z = 1;

//	imu->header_seq=1;
	// Move with constant acceleration for 1 second,
	// then check our velocity.
	//	ros::Rate loopRate(50);
	//size_t i=0;

	int i=0;
	for (i = 0; i < 50; ++i)
	{
		/*		imu.header.stamp = ros::Time::now();
				imuPub.publish(imu);
				loopRate.sleep();
				ros::spinOnce();
		 */
		publish("imu",pubsize,seq);
		loopRate_sleep(50);
//		imu->header_seq++;
		seq++;
	}
	/*
	   EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - 1.0), 0.4);
	   EXPECT_LT(::fabs(filtered_.twist.twist.linear.y + 1.0), 0.4);
	   EXPECT_LT(::fabs(filtered_.twist.twist.linear.z - 1.0), 0.4);
	 */

//	imu->linear_acceleration_x = 0.0;
//	imu->linear_acceleration_y = 0.0;
//	imu->linear_acceleration_z = 0.0;

	// Now move for another second, and see where we
	// end up
	//	loopRate = ros::Rate(50);
	for (i = 0; i < 50; ++i)
	{
		/*imu.header.stamp = ros::Time::now();
		  imuPub.publish(imu);
		  loopRate.sleep();
		  ros::spinOnce();
		 */
		publish("imu",pubsize,seq);
		loopRate_sleep(50);
//		imu->header_seq++;
		seq++;
	}

	/*	EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 1.2), 0.4);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.y + 1.2), 0.4);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.z - 1.2), 0.4);
	 */
//	resetFilter();

	// Test to see if the linear acceleration data is ignored when we set the
	// first covariance value to -1
	int imuIgnorepubsize=0; 
	int seq2=1;
	struct sensor_msgs_Imu_Acc* imuIgnore = (struct sensor_msgs_Imu_Acc*)malloc(sizeof(sensor_msgs_Imu_Acc)+sizeof(double)*9);
	imuIgnorepubsize+=sizeof(sensor_msgs_Imu_Acc);

	//	imuIgnore->linear_acceleration = (struct linear_acceleration*)malloc(sizeof(linear_acceleration));
	//	imuIgnorepubsize+=sizeof(linear_acceleration);

//	imuIgnore->linear_acceleration_x = 1000;
//	imuIgnore->linear_acceleration_y = 1000;
//	imuIgnore->linear_acceleration_z = 1000;
//	imuIgnore->linear_acceleration_covariance[0] = -1;

	//	loopRate = ros::Rate(50);
	//	imuIgnore->header = (struct header*)malloc(sizeof(header));
	//	imuIgnorepubsize+=sizeof(header);
//	imuIgnore->header_seq=1;
	for (i = 0; i < 50; ++i)
	{
		/*		imuIgnore.header.stamp = ros::Time::now();
				imuPub.publish(imuIgnore);
				loopRate.sleep();
				ros::spinOnce();
		 */
		publish("imuIgnore",imuIgnorepubsize,seq2);
		loopRate_sleep(50);
//		imuIgnore->header_seq++;
		seq2++;
	}
	/*
	   EXPECT_LT(::fabs(filtered_.pose.pose.position.x), 1e-3);
	   EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 1e-3);
	   EXPECT_LT(::fabs(filtered_.pose.pose.position.z), 1e-3);
	 */
//	resetFilter();

//	free(imu);
//	free(imuIgnore);


}
//////ddd

void OdomDifferentialIOTEST()
{
	//	ros::NodeHandle nh;
	struct ros_Publisher* odomPub;
//	odomPub = nh_advertise("/odom_input1", 5);
	struct ros_Subscriber* filteredSub;
//	filteredSub = nh_subscribe("/odometry/filtered", 1);
	//, &filterCallback);

	int pubsize=0;
	int seq=1;
	struct nav_msgs_Odometry* odom = (struct nav_msgs_Odometry*)malloc(sizeof(nav_msgs_Odometry));;
	pubsize+=sizeof(nav_msgs_Odometry);

//	odom->pose_position_x = 20.0;
//	odom->pose_position_y = 10.0;
//	odom->pose_position_z = -40.0;

	//	odom->pose->pose->orientation=(struct orientation*)malloc(sizeof(orientation));
	//	pubsize+=sizeof(orientation);

//	odom->pose_orientation_w = 1;

//	odom->pose_covariance[0] = 2.0;
//	odom->pose_covariance[7] = 2.0;
//	odom->pose_covariance[14] = 2.0;
//	odom->pose_covariance[21] = 0.2;
//	odom->pose_covariance[28] = 0.2;
//	odom->pose_covariance[35] = 0.2;

	//	odom->header = (struct header*)malloc(sizeof(header));
	//	pubsize+=sizeof(header);

//	odom->header_frame_id = "odom";
//	odom->child_frame_id = "base_link";
//	odom->header_seq=1;
	// No guaranteeing that the zero state
	// we're expecting to see here isn't just
	// a result of zeroing it out previously,
	// so check 10 times in succession.
	//size_t zeroCount = 0;
	int zeroCount=0;
	while (zeroCount++ < 10)
	{
		//odom.header.stamp = ros::Time::now();
		//odomPub.publish(odom);
		publish("odom",pubsize,seq);
		//ros::spinOnce();

		/*		EXPECT_LT(::fabs(filtered_.pose.pose.position.x), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.position.z), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.orientation.x), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.orientation.y), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.orientation.z), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.orientation.w - 1), 0.01);
		 */
		//		ros::Duration(0.1).sleep();
		Duration_sleep(0.1);
//		odom->header_seq++;
		seq++;
	}

	//size_t ind=0;
	int ind=0;
	for (ind = 0; ind < 36; ind+=7)
	{
//		odom->pose_covariance[ind] = 1e-6;
	}

	// Slowly move the position, and hope that the
	// differential position keeps up
	//	ros::Rate loopRate(20);
	//size_t i=0;

	int i=0;
	for (i = 0; i < 100; ++i)
	{
//		odom->pose_position_x += 0.01;
//		odom->pose_position_y += 0.02;
//		odom->pose_position_z -= 0.03;

		//		odom.header.stamp = ros::Time::now();
		//		odomPub.publish(odom);
		publish("odom",pubsize,seq);
		//		ros::spinOnce();

		loopRate_sleep(20);

//		odom->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

	/*	EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 1), 0.2);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 2), 0.4);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.z + 3), 0.6);
	 */
//	resetFilter();

//	free(odom);

}

//TEST(InterfacesTest, PoseDifferentialIO)
void PoseDifferentialIOTEST()
{
	//ros::NodeHandle nh;
	int pubsize=0;
	int seq=1; 

	struct ros_Publisher* posePub;
//	posePub = nh_advertise("/pose_input1", 5);
	struct ros_Subscriber* filteredSub;
//	filteredSub = nh_subscribe("/odometry/filtered", 1);
	//, &filterCallback);

	//geometry_msgs::PoseWithCovarianceStamped pose;
	struct geometry_msgs_Pose* pose = (struct geometry_msgs_Pose*)malloc(sizeof(geometry_msgs_Pose));
	pubsize+=sizeof(geometry_msgs_Pose);



//	pose->pose_position_x = 20.0;
//	pose->pose_position_y = 10.0;
//	pose->pose_position_z = -40.0;

//	pose->pose_orientation_w = 1;

//	pose->pose_covariance[0] = 2.0;
//	pose->pose_covariance[7] = 2.0;
//	pose->pose_covariance[14] = 2.0;
//	pose->pose_covariance[21] = 0.2;
//	pose->pose_covariance[28] = 0.2;
//	pose->pose_covariance[35] = 0.2;


//	pose->header_frame_id = "odom";
//	pose->header_seq=1;
	// No guaranteeing that the zero state
	// we're expecting to see here isn't just
	// a result of zeroing it out previously,
	// so check 10 times in succession.
	//size_t zeroCount = 0;
	int zeroCount=0;
	while (zeroCount++ < 10)
	{
		/*		pose.header.stamp = ros::Time::now();
				posePub.publish(pose);
				ros::spinOnce();

				EXPECT_LT(::fabs(filtered_.pose.pose.position.x), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.position.z), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.orientation.x), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.orientation.y), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.orientation.z), 0.01);
				EXPECT_LT(::fabs(filtered_.pose.pose.orientation.w - 1), 0.01);

				ros::Duration(0.1).sleep();//sec

		 */
		publish("pose",pubsize,seq);
		Duration_sleep(0.1);
//		pose->header_seq++;
		seq++;
	}

	// ...but only if we give the measurement a tiny covariance
	//size_t ind=0;
	int ind=0;
	for (ind = 0; ind < 36; ind+=7)
	{
//		pose->pose_covariance[ind] = 1e-6;
	}

	// Issue this location repeatedly, and see if we get
	// a final reported position of (1, 2, -3)
	//ros::Rate loopRate(20);
	//size_t i=0;
	int i=0;
	for (i = 0; i < 100; ++i)
	{
//		pose->pose_position_x += 0.01;
//		pose->pose_position_y += 0.02;
//		pose->pose_position_z -= 0.03;

		/*		pose.header.stamp = ros::Time::now();
				posePub.publish(pose);
				ros::spinOnce();
		 */
		publish("pose",pubsize,seq);
		loopRate_sleep(20);

//		pose->header_seq++;
		seq++;
	}
	/*	ros::spinOnce();

		EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 1), 0.2);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 2), 0.4);
		EXPECT_LT(::fabs(filtered_.pose.pose.position.z + 3), 0.6);
	 */

//	free(pose);

//	resetFilter();
}

//TEST(InterfacesTest, ImuDifferentialIO)
void ImuDifferentialIOTEST()
{
	//	ros::NodeHandle nh;
	struct ros_Publisher* imu0Pub;
//	imu0Pub = nh_advertise("/imu_input0", 5);
	struct ros_Publisher* imu1Pub;
//	imu1Pub = nh_advertise("/imu_input1", 5);
	struct ros_Publisher* imuPub;
//	imuPub = nh_advertise("/imu_input3", 5);
	struct ros_Subscriber* filteredSub;
//	filteredSub = nh_subscribe("/odometry/filtered", 1);
	//, &filterCallback);

	int pubsize=0;
	int seq=1;

	struct sensor_msgs_Imu* imu = (struct sensor_msgs_Imu*)malloc(sizeof(sensor_msgs_Imu)+sizeof(double)*9);
	pubsize+=sizeof(sensor_msgs_Imu);

//	imu->header_frame_id = "base_link";

	//tf2::Quaternion quat;
	const double roll = M_PI/2.0;
	const double pitch = -M_PI;
	const double yaw = -M_PI/4.0;
	//quat.setRPY(roll, pitch, yaw);

//	imu->orientation_covariance[0] = 0.02;
//	imu->orientation_covariance[4] = 0.02;
//	imu->orientation_covariance[8] = 0.02;

//	imu->angular_velocity_covariance[0] = 0.02;
//	imu->angular_velocity_covariance[4] = 0.02;
//	imu->angular_velocity_covariance[8] = 0.02;
//	imu->header_seq=1;
	//size_t setCount = 0;
	int setCount=0;
	while (setCount++ < 10)
	{
		/*	imu.header.stamp = ros::Time::now();
			imu0Pub.publish(imu);  // Use this to move the absolute orientation
			imu1Pub.publish(imu);  // Use this to keep velocities at 0
			ros::spinOnce();

			ros::Duration(0.1).sleep();
		 */
		publish("imu",pubsize,seq);
		publish("imu",pubsize,seq);
		Duration_sleep(0.1);
//		imu->header_seq++;
		seq++;
	}

	//size_t zeroCount = 0;
	int zeroCount=0;
	while (zeroCount++ < 10)
	{
		/*		imu.header.stamp = ros::Time::now();
				imuPub.publish(imu);
				ros::spinOnce();

				ros::Duration(0.1).sleep();
		 */
		publish("imu",pubsize,seq);
		Duration_sleep(0.1);
//		imu->header_seq++;
		seq++;
	}

	double rollFinal = roll;
	double pitchFinal = pitch;
	double yawFinal = yaw;

	// Move the orientation slowly, and see if we
	// can push it to 0
	//ros::Rate loopRate(20);
	//size_t i=0;
	int i=0;
	for (i = 0; i < 100; ++i)
	{
		yawFinal -= 0.01 * (3.0 * M_PI/4.0);

		//quat.setRPY(rollFinal, pitchFinal, yawFinal);

		//		imu->orientation = quat;
		//		imu.header.stamp = ros::Time::now();
		//		imuPub.publish(imu);
		//		ros::spinOnce();

		publish("imu",pubsize,seq);
		loopRate_sleep(20);

//		imu->header_seq++;
		seq++;
	}
	//	ros::spinOnce();

	// Move the orientation slowly, and see if we
	// can push it to 0
	//	loopRate = ros::Rate(20);
	for (i = 0; i < 100; ++i)
	{
		rollFinal += 0.01 * (M_PI/2.0);

		//		quat.setRPY(rollFinal, pitchFinal, yawFinal);

		//		imu->orientation = quat;
		//		imu.header.stamp = ros::Time::now();
		//		imuPub.publish(imu);
		//		ros::spinOnce();
		publish("imu",pubsize,seq);

		loopRate_sleep(20);

//		imu->header_seq++;
		seq++;
	}
	//	ros::spinOnce();
	/*
	   tf2::fromMsg(filtered_.pose.pose.orientation, quat);
	   tf2::Matrix3x3 mat(quat);
	   mat.getRPY(rollFinal, pitchFinal, yawFinal);
	   EXPECT_LT(::fabs(rollFinal), 0.2);
	   EXPECT_LT(::fabs(pitchFinal), 0.2);
	   EXPECT_LT(::fabs(yawFinal), 0.2);
	 */

//	free(imu);

//	resetFilter();
}


void main(int argc, char **argv)
{
	//  testing::InitGoogleTest(&argc, argv);

	//  ros::init(argc, argv, "ekf_navigation_node-test-interfaces");
	//  ros::Time::init();

	// Give ekf_localization_node time to initialize
	//  ros::Duration(2.0).sleep();

	//  return RUN_ALL_TESTS();
//	while(1){

		OdomPoseBasicIOTEST();
		OdomTwistBasicIOTEST();
		PoseBasicIOTEST();
		TwistBasicIOTEST();
		ImuPoseBasicIOTEST();
		ImuTwistBasicIOTEST();
		ImuAccBasicIOTEST();
		OdomDifferentialIOTEST();
		PoseDifferentialIOTEST();
		ImuDifferentialIOTEST();
//	}
	//sleep(2);//sleep 함수추가
	//}
}
