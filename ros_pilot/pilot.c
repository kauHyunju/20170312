/*
 *  Copyright (C) 2005-2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id: 10ae05a1a298db38ff73edddce4094751776500a $
 */

/**  @file

  ROS node for controlling direction and speed of the ART
  autonomous vehicle.

  @todo (optionally) stop if no commands received recently.
  @todo shift to Park, when appropriate
  @todo distinguish device failures before and after initialization
  @todo deprecate old CarCommand message interface

  @author Jack O'Quin

 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>


//reference
//http://docs.ros.org/diamondback/api/art_msgs/html/files.html

#include <stdint.h>
#include <stdbool.h>

#pragma pack(4)
typedef struct header{
	char* frame_id;
	int stamp;
	int seq;
}header;

//reference
//http://docs.ros.org/diamondback/api/art_pilot/html/classart__pilot_1_1PilotConfig.html#ade88ecb2bf4827dadbc8e9ebe0293fe9

typedef struct Config{
	int acceleration_controller;
	double brake_hold;
	double brake_kd;
	double brake_ki;
	double brake_kp;
	_Bool human_steering;
	double limit_forward;
	double limit_reverse;
	double throttle_kd;
	double throttle_ki;
	double throttle_kp;
	double timeout;
}Config;

typedef struct Epsilonst{
	float brake_position;
	float distance;
	float float_value;
	float speed;
	float steering_angle;
	float throttle_position;
}Epsilonst;

typedef struct driver_state{
	int state;
#define CLOSED = 0;
#define OPENED = 1;
#define RUNNING = 2;
}driver_state;

typedef struct gear{
	int value;
#define Naught = 0;
#define	Park = 1;
#define Reverse = 2;
#define Neutral = 3;
#define Drive = 4;
#define N_gears = 5;
}gear;

typedef struct behavior{
	int value;
#define Abort = 0;
#define Quit = 1;
#define Pause = 2;
#define Run = 3;
#define Suspend = 4;
#define Initialize = 5;
#define Go = 6;
#define	NONE = 7;
#define	N_behaviors = 8;

}behavior;

typedef struct car_drive{//target{
	float speed;
	float acceleration;
	float jerk;
	float steering_angle;
	struct gear* gear;
	struct behavior* behavior;
}car_drive;

typedef struct PilotState{
	struct header* header;
	//struct driver_state* pilot;
	int pilot_state;
#define CLOSED = 0;
#define OPENED = 1;
#define RUNNING = 2;
	//struct driver_state* brake;
	int brake_state;
	//struct driver_state* imu;
	int imu_state;
	//struct driver_state* odom;
	int odom_state;
	//struct driver_state* shifter;
	int shifter_state;
	//struct driver_state* steering;
	int steering_state;
	//struct driver_state* throttle;
	int throttle_state;

	int preempted;
	//struct car_drive* target;
	float target_speed;
	float target_acceleration;
	float target_jerk;
	float target_steering_angle;
	int target_gear_value;
#define target_Naught = 0;
#define target_Park = 1;
#define target_Reverse = 2;
#define target_Neutral = 3;
#define target_Drive = 4;
#define target_N_gears = 5;

	//struct gear* gear;
	int target_behavior_value;
	//struct behavior* behavior;

	//struct car_drive* plan;
	float plan_speed;
	float plan_acceleration;
	float plan_jerk;
	float plan_steering_angle;

	int plan_gear_value;

	//struct gear* gear;
	int plan_behavior_value;

	//struct car_drive* current;
	float current_speed;
	float current_acceleration;
	float current_jerk;
	float current_steering_angle;
	//	struct gear* gear;
	//	struct behavior* behavior;

	int current_gear_value;

	//struct gear* gear;
	int current_behavior_value;

}PilotState;


#pragma pack(4)
typedef struct position{
	double x;


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
typedef struct nav_msgs_Odometry{

	struct nav_msgs_Odometry* nav_msgs_Odometry;
	struct pose* pose;
	struct twist* twist;
	struct header* header;
	char* child_frame_id;
}nav_msgs_Odometry;


#pragma pack(4)
typedef struct geometry_msgs_Pose{

	struct geometry_msgs* geometry_msgs_Pose;
	struct pose* pose;
	//	struct twist* twist;
	struct header* header;
	char* child_frame_id;
}geometry_msgs_Pose;


#pragma pack(4)
typedef struct geometry_msgs_Twist{

	struct geometry_msgs* geometry_msgs_Pose;
	//    struct pose* pose;
	struct twist* twist;
	struct header* header;
	char* child_frame_id;
}geometry_msgs_Twist;

#pragma pack(4)
typedef struct sensor_msgs_Imu_Pose{
	struct orientation* orientation;
	double orientation_covariance[9];
	struct header* header;
}sensor_msgs_Imu_Pose;


#pragma pack(4)
typedef struct sensor_msgs_Imu_Twist{
	struct angular_velocity* angular_velocity;
	double angular_velocity_covariance[9];
	struct header* header;
}sensor_msgs_Imu_Twist;


#pragma pack(4)
typedef struct sensor_msgs_Imu_Acc{
	struct linear_acceleration* linear_acceleration;
	double linear_acceleration_covariance[9];
	struct header* header;
}sensor_msgs_Imu_Acc;

#pragma pack(4)
typedef struct sensor_msgs_Imu{
	struct orientation* orientation;
	double orientation_covariance[9];
	double angular_velocity_covariance[9];
	struct header* header;
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

typedef struct brake_state{
	//	struct header* header;
	char* header_frame_id;
	int header_stamp;
	int header_seq;

	float position;
	float potentiometer;
	float encoder;
	float pressure;
}brake_state;

typedef struct brake_command{
	//	struct header* header;
	char* header_frame_id;
	int header_stamp;
	int header_seq;
	int request;
	float position;
}brake_command;

typedef struct devicebrake{
	char* header_frame_id;
	int header_stamp;
	int header_seq;
	int brake_command_request;
	float brake_command_position;

	float brake_state_position;
	float brake_state_potentiometer;
	float brake_state_encoder;
	float brake_state_pressure;

	//	struct brake_command* cmd;
	//	struct brake_state* msg;
}devicebrake;

typedef struct deviceimu{
	struct sensor_msgs_Imu* msg;
}deviceimu;

typedef struct deviceodom{
	struct nav_msgs_Odometry* msg;
}deviceodom;

typedef struct shifter{
	//	struct header* header;
	char* header_frame_id;
	int header_stamp;
	int header_seq;
	int gear;
	int relays;
#define Reset = 0;
#define Park = 1;
#define Reverse = 2;
#define Neutral = 3;
#define Drive = 4;
}shifter;

typedef struct deviceshifter{
	struct shifter* cmd;
	struct shifter* msg;
	struct ros_Publisher* pub;
	float shift_duration;//time
	float shift_time;//time
	_Bool is_reset;//
	_Bool busy;
}deviceshifter;

typedef struct steering_command{
	struct header* header;
	int request;
	float angle;
}steering_command;

typedef struct steering_state{
	struct header* header;
	struct driver_state*  driver;
	float angle;
	float sensor;
}steering_state;


typedef struct devicesteering{
	struct steering_command* cmd;
	struct steering_state* msg;
}devicesteering;

typedef struct throttle_command{
	struct header* header;
	int request;
	float position;
}throttle_command;

typedef struct throttle_state{
	struct header* header;
	float position;
	float rpms;
	int estop;
	float pwm;
	float dstate;
	float istate;
}throttle_state;

typedef struct devicethrottle{
	struct throttle_command* cmd;
	struct throttle_state* msg;
}devicethrottle;


int mystrcmp(const char *s, const char* t);


void nh_serviceClient();

struct ros_Publisher* nh_advertise(char* topic_name, int queue_size);
struct ros_Subscriber* nh_subscribe(char* topic_name, int queue_size);


void Duration_sleep(double seconds);
void loopRate_sleep(int fq);
void publish(char* meta, int size, int t);

//typedef art_msgs::DriverState DriverState;
struct driver_state* DriverState;
struct Epsilonst* Epsilon;
//using art_msgs::Epsilon;


/**
  @brief controls the ART vehicle brake, throttle, steering and transmission

  The pilot receives CarDriveStamped messages from the navigator, then
  translates them into commands to the servo motor actuators for
  controlling the speed and direction of the vehicle.  It gets odometry
  information from a separate node.

Subscribes:

- @b pilot/drive [art_msgs::CarDriveStamped] driving command
- @b pilot/cmd [art_msgs::CarCommand] velocity and steering angle command
- @b imu [sensor_msgs::Imu] estimate of robot accelerations
- @b odom [nav_msgs::Odometry] estimate of robot position and velocity.

- @b brake/state [art_msgs::BrakeState] brake status.
- @b shifter/state [art_msgs::Shifter] shifter relays status.
- @b steering/state [art_msgs::SteeringState] steering status.
- @b throttle/state [art_msgs::ThrottleState] throttle status.

Publishes:

- @b pilot/state [art_msgs::PilotState] current pilot state information.
- @b brake/cmd [art_msgs::BrakeCommand] brake commands.
- @b shifter/cmd [art_msgs::Shifter] shifter commands.
- @b steering/cmd [art_msgs::SteeringCommand] steering commands.
- @b throttle/cmd [art_msgs::ThrottleCommand] throttle commands.

 */

/** Pilot node class. */
//class PilotNode
//{

_Bool is_shifting_;                    // is transmission active?

/*typedef dynamic_reconfigure::Server<Config> ReconfigServer;
  boost::shared_ptr<ReconfigServer> reconfig_server_;
 */
// ROS topics used by this node
//ros::Subscriber accel_cmd_;           // CarDriveStamped command
struct ros_Subscriber* accel_cmd_;

//ros::Subscriber car_cmd_;             // CarCommand
struct ros_Subscriber* car_cmd_;

// Device interfaces used by pilot
struct devicebrake* brake_;
struct deviceimu* imu_;
struct deviceodom* odom_;
struct deviceshifter* shifter_;
struct devicesteering* steering_;
struct devicethrottle* throttle_;

//ros::Subscriber learning_cmd_;
struct ros_Subscriber* learning_cmd_;

struct ros_Publisher* pilot_state_;          // pilot state

// configuration
struct Config* config_;                       // dynamic configuration
int current_time_;

/*ros::Duration timeout_;               // device timeout (sec)

  ros::Time current_time_;              // time current cycle began

// times when messages received
ros::Time goal_time_;                 // latest goal command
 */
//struct PilotState* pstate_msg_ = (struct PilotState*)malloc(sizeof(PilotState));     // pilot state message

//boost::shared_ptr<pilot::AccelBase> accel_;  // acceleration controller
//};


//////////////////////////////////////////////////////////////////
// public methods
//////////////////////////////////////////////////////////////////

/** constructor */
//PilotNode::PilotNode(ros::NodeHandle node):
//  reconfig_server_(new dynamic_reconfigure::Server<Config>)

//#include <stdlib.h>
//#include "vehicle.h"


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

void publish(char* meta, int size, int seq){
	char* topic;
	topic=meta;

	int topic_size;
	topic_size=size;
	/*int shifter_size=0;
	  int brake_size=0;
	  int throttle_size=0;
	  int steering_size=0;
	  int pilot_state_size=0;

	  if(mystrcmp(meta,"shifter")==0){
	//printf("odom pubsize:%d\n",size);
	shifter_size=size;
	//		odom_seq = seq;
	}
	else if(mystrcmp(meta,"brake")==0){
	//printf("twist pubsize:%d\n",size);
	brake_size=size;
	//		twist_seq = seq;
	}
	else if(mystrcmp(meta,"throttle")==0){
	//printf("imu pubsize:%d\n",size);
	throttle_size=size;
	//		imu_seq = seq;
	}
	else if(mystrcmp(meta,"steering")==0){
	//printf("imuIgnore pubsize:%d\n",size);
	steering_size=size;
	//		imuIgnore_seq = seq;
	}
	else if(mystrcmp(meta,"pilot_state")==0){
	//printf("pose pubsize:%d\n",size);
	pilot_state_size=size;
	//		pose_seq = seq;
	}
	 */
}

static inline double to_degrees(double radians)
{
	return radians * 180.0 / 3.14;//M_PI;
}

/** Adjust steering angle.
 *  
 *  We do not use PID control, because the odometry does not provide
 *  accurate yaw speed feedback.  Instead, we directly compute the
 *  corresponding steering angle.  We can use open loop control at this
 *  level, because navigator monitors our actual course and will
 *  request any steering changes needed to reach its goal.
 *
 *  @todo Limit angle actually requested based on current velocity to
 *        avoid unsafe high speed turns.
 */
//void PilotNode::adjustSteering(void)
void adjustSteering(PilotState* pstate_msg_)
{
	//if (config_->human_steering)           // pilot not steering?
	//	return;

	//	if (fabs(pstate_msg_->target_steering_angle - pstate_msg_->current_steering_angle)
	//			> Epsilon->steering_angle)
	//	{
	// Set the steering angle in degrees.
	float steer_degrees;
	//		float steer_degrees = to_degrees(pstate_msg_->target_steering_angle);
	//ROS_DEBUG("requesting steering angle = %.1f (degrees)", steer_degrees);
	//		steering_->publish(steer_degrees, current_time_);
	publish("steering",sizeof(steer_degrees), current_time_);
	//	}
}

/** halt -- soft version of hardware E-Stop.
 *  
 *   The goal is to bring the vehicle to a halt as quickly as possible,
 *   while remaining safely under control.  Normally, navigator sends
 *   gradually declining speed requests when bringing the vehicle to a
 *   controlled stop.  The only abrupt requests we see are in
 *   "emergency" stop situations, when there was a pause request, or no
 *   clear path around an obstacle.
 *
 *   @note The target goal velocity may not always be zero, due to the
 *         gear shifting requirements.
 *
 *   @post Since this function bypasses the normal acceleration
 *         controller, it is reset on exit.
 */

void accel_reset(void){
}

void accel_adjust(struct PilotState* p, struct devicebrake* b, struct devicethrottle* t){

}

void halt(void)
{
	// absolute value of current velocity in m/sec
	//float abs_speed = pstate_msg_->current_speed;
	//if (abs_speed < Epsilon->speed)
	//{
	// Already stopped.  Ease up on the brake to reduce strain on
	// the actuator.  Brake hold position *must* be adequate to
	// prevent motion, even on a hill.
	publish("brake",sizeof(config_->brake_hold), current_time_);
	//}
	//else
	//	{
	// Stop the moving vehicle very quickly.
	//
	//  Apply min throttle and max brake at the same time.  This is
	//  an exception to the general rule of never applying brake and
	//  throttle together.  There seems to be enough inertia in the
	//  brake mechanism for this to be safe.
	publish("throttle",sizeof(0.0), current_time_);
	publish("brake",sizeof(1.0), current_time_);
	//	}
	//	accel_reset();
}

/** monitor hardware status based on current inputs
 *
 *  @post pstate_msg_ updated to reflect current control hardware
 *        status and time of this cycle
 */
//void PilotNode::monitorHardware(void)
void monitorHardware(void)
{}
/*  // update current pilot state
//  current_time_ = ros::Time::now();
pstate_msg_.header.stamp = current_time_;
pstate_msg_.current.acceleration = fabs(imu_->value());
pstate_msg_.current.speed = fabs(odom_->value());
pstate_msg_.current.steering_angle =
angles::from_degrees(steering_->value());
pstate_msg_.current.gear.value = shifter_->value();

// Stage time should not ever start at zero, but there seems to be a
// bug.  In any case it could be < timeout_ (unlike wall time).
//  ros::Time recently;              // minimum time for recent messages
//  if (current_time_ > (recently + timeout_))
//    {
//      recently = current_time_ - timeout_;
//    }

// accumulate new pilot state based on device states
pstate_msg_.brake.state = brake_->state(recently);
pstate_msg_.imu.state = imu_->state(recently);
pstate_msg_.odom.state = odom_->state(recently);
pstate_msg_.shifter.state = shifter_->state(recently);
pstate_msg_.steering.state = steering_->state(recently);
pstate_msg_.throttle.state = throttle_->state(recently);

/// @todo Optionally check if no commands received recently.

pstate_msg_.pilot.state = DriverState::RUNNING;
if (pstate_msg_.brake.state != DriverState::RUNNING
|| pstate_msg_.imu.state != DriverState::RUNNING
|| pstate_msg_.odom.state != DriverState::RUNNING
|| (!config_.human_steering
&& pstate_msg_.steering.state != DriverState::RUNNING)
|| pstate_msg_.throttle.state != DriverState::RUNNING)
{
// pilot is not running
pstate_msg_.pilot.state = DriverState::OPENED;
ROS_WARN_THROTTLE(40, "critical component failure, pilot not running");
// reset latest target request
pstate_msg_.target = art_msgs::CarDrive();
}
}*/

//void PilotNode::validateTarget(void)
void validateTarget(PilotState* pstate_msg_)
{
	// Warn if negative speed, acceleration and jerk.
	if (pstate_msg_->target_speed < 0.0
			|| pstate_msg_->target_acceleration < 0.0
			|| pstate_msg_->target_jerk < 0.0)
	{
		//	ROS_WARN_THROTTLE(100, "Negative speed, acceleration and jerk "
		//			"are DEPRECATED (using absolute value).");
		pstate_msg_->target_speed = fabs(pstate_msg_->target_speed);
		pstate_msg_->target_acceleration = fabs(pstate_msg_->target_acceleration);
		pstate_msg_->target_jerk = fabs(pstate_msg_->target_jerk);
	}

	if (pstate_msg_->target_gear_value == 2)//art_msgs::Gear::Reverse)
	{
		if (pstate_msg_->target_speed > config_->limit_reverse)
		{
			/*ROS_WARN_STREAM_THROTTLE(100, "Requested speed ("
			  << pstate_msg_.target.speed
			  << ") exceeds reverse limit of "
			  << config_.limit_reverse
			  << " m/s");*/
			pstate_msg_->target_speed = config_->limit_reverse;
		}
	}
	else
	{
		if (pstate_msg_->target_speed > config_->limit_forward)
		{
			/*          ROS_WARN_STREAM_THROTTLE(100, "Requested speed ("
						<< pstate_msg_.target.speed
						<< ") exceeds limit of "
						<< config_.limit_forward
						<< " m/s");*/
			pstate_msg_->target_speed = config_->limit_forward;
		}
	}

	// limit steering angle to permitted range
	//  using namespace pilot;
	//  using namespace art_msgs;
	/*  pstate_msg_->target->steering_angle = clamp(-ArtVehicle::max_steer_radians,
		pstate_msg_->target->steering_angle,
		ArtVehicle::max_steer_radians);*/
}

/** CarDriveStamped message callback */
void processCarDrive(PilotState* pstate_msg_)//(const art_msgs::CarDriveStamped::ConstPtr &msg)
{
	//goal_time_ = msg->header->stamp;
	//pstate_msg_->target = msg->control;
	//	validateTarget();
}

/** CarCommand message callback (now DEPRECATED) */
void processCarCommand(PilotState* pstate_msg_)//(const art_msgs::CarCommand::ConstPtr &msg)
{
	//	ROS_WARN_THROTTLE(100, "CarCommand deprecated: use CarDriveStamped.");

	//goal_time_ = msg->header->stamp;
	pstate_msg_->target_steering_angle = 1;//angles::from_degrees(msg->control.angle);
	pstate_msg_->target_behavior_value = 1;//art_msgs::PilotBehavior::Run;

	pstate_msg_->target_jerk = 0.0;
	pstate_msg_->target_acceleration = 0.0;
	//	pstate_msg_->target_speed = msg->control->velocity;////////////////////////
	if (pstate_msg_->target_speed > 0.0)
	{
		//		struct gear* g;
		pstate_msg_->target_gear_value = 4;//g->Drive;

	}
	else if (pstate_msg_->target_speed < 0.0)
	{
		// in reverse: make speed positive
		pstate_msg_->target_speed = 1;//-msg->control->velocity;
		pstate_msg_->target_gear_value = 1;//(struct gear* gear)->Reverse;
	}
	else
	{
		pstate_msg_->target_gear_value = 0;//gear->Naught;
	}

	//validateTarget();
}

/** LearningCommand message callback (DEPRECATED) */
/*void processLearning(const art_msgs::LearningCommand::ConstPtr &learningIn)
  {
//ROS_WARN_THROTTLE(100, "LearningCommand deprecated: use CarDriveStamped.");
pstate_msg_->preempted = (learningIn->pilotActive == 0);
ROS_INFO_STREAM("Pilot is "
<< (pstate_msg_->preempted? "preempted": "active"));
}*/

/** handle dynamic reconfigure service request
 *
 * @param newconfig new configuration from dynamic reconfigure client,
 *        becomes the service reply message as updated here.
 * @param level SensorLevels value (0xffffffff on initial call)
 *
 * This is done without any locking because it is called in the same
 * thread as ros::spinOnce() and all the topic subscription callbacks.
 */

void PilotNodeConstructor(PilotState* pstate_msg_)
{
	// Must declare dynamic reconfigure callback before initializing
	// devices or subscribing to topics.
	//  reconfig_server_->setCallback(boost::bind(&PilotNode::reconfig,
	//              this, _1, _2));

	// allocate and initialize device interfaces
	//  brake_.reset(new device_interface::DeviceBrake(node));
	//  imu_.reset(new device_interface::DeviceImu(node));
	//  odom_.reset(new device_interface::DeviceOdom(node));
	//  shifter_.reset(new device_interface::DeviceShifter(node));
	//  steering_.reset(new device_interface::DeviceSteering(node));
	//  throttle_.reset(new device_interface::DeviceThrottle(node));

	// no delay: we always want the most recent data
	//  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
	//is_shifting_ = false;
	int qDepth = 1;

	// command topics (car_cmd will be deprecated)
	//accel_cmd_ = nh_subscribe("pilot/drive", qDepth);//,            &PilotNode::processCarDrive, this, noDelay);
	processCarDrive(pstate_msg_);
	//car_cmd_ = nh_subscribe("pilot/cmd", qDepth);//,            &PilotNode::processCarCommand, this , noDelay);
	//processCarCommand(pstate_msg_);
	//learning_cmd_ = nh_subscribe("pilot/learningCmd", qDepth);//,           &PilotNode::processLearning, this, noDelay);
	//processLearning();

	// topic for publishing pilot state
	//pilot_state_ = nh_advertise("pilot/state", qDepth);
	////    pstate_msg_.header.frame_id = art_msgs::ArtVehicle:a:frame_id;
}

/*void PilotNode::reconfig(Config &newconfig, uint32_t level)
  {
  ROS_INFO("pilot dynamic reconfigure, level 0x%08x", level);

  if (level & driver_base::SensorLevels::RECONFIGURE_CLOSE)
  {
// reallocate acceleration controller using new configuration
//accel_ = pilot::allocAccel(newconfig);
}
else
{
// pass any other parameters to existing acceleration controller
//accel_->reconfigure(newconfig);
}

config_ = newconfig;
timeout_ = ros::Duration(config_.timeout);
}
 */

/** Speed control
 *  
 *  Manage the shifter.  Inputs are the current and target states.  If
 *  a gear shift is requested, the vehicle must first be brought to a
 *  stop, then one of the transmission shift relays set for one second
 *  (then reset), before the vehicle can begin moving in the opposite
 *  direction.
 */
void speedControl(PilotState* pstate_msg_)
{
	// do nothing while pilot preempted for learning speed control (no
	// servo commands permitted)
	//if (pstate_msg_->preempted)
	//	return;

	//	float dt = 1.0 / art_msgs::ArtHertz::PILOT;
	//float dt = 1.0;
	//float abs_current_speed = pstate_msg_->current_speed;
	//float abs_target_speed = pstate_msg_->target_speed;

	//if (is_shifting_)
	//{
	//	if (shifter_->is_reset)
	//	{
	// all relays are reset now
	//is_shifting_ = false;
	//	}
	//	else if (!shifter_->busy)
	//	{
	// original operation complete, reset shifter relays
	publish("shifter", sizeof(int)/*sizeof(shifter->Reset)*/, current_time_);//
	//	}
	//}

	//if (pstate_msg_->current_gear_value == pstate_msg_->target_gear_value
	//		|| pstate_msg_->target_gear_value == 0)//gear->Naught)
	//		{
	// no shift required
	//			if ((pstate_msg_->pilot_state != 0)//DriverState::RUNNING)
	//				|| (pstate_msg_->target_gear_value == 1)//gear->Park)
	//					|| (pstate_msg_->target_gear_value == 3)//art_msgs::Gear::Neutral)
	//					|| shifter_->busy)
	//					{
	// unable to proceed
	halt();
	//					}
	//			else
	//			{
	// driving in the correct gear
	//				if ((abs_target_speed < Epsilon->speed)
	//						&& ((pstate_msg_->target_acceleration == 0.0)
	//							|| (pstate_msg_->target_acceleration * dt
	//								> abs_current_speed)))
	//				{
	// stop requested and acceleration not specified, or
	// almost stopped
	halt();
	//				}
	//				else
	//				{
	// have acceleration controller adjust speed
	//					accel_adjust(pstate_msg_, brake_, throttle_);
	//				}
	//			}
	//		}
	//else
	//{
	// not in desired gear, shift (still) needed
	//	is_shifting_ = true;
	//if (!shifter_->busy)
	//{
	// request shift until driver reports success
	publish("shifter",sizeof(/*uint8_t*/pstate_msg_->target_gear_value), current_time_);//
	//}

	halt();                           // never move while shifting
	//}
}


/** main loop */
void PilotNode_spin(PilotState* p)
{
	// Main loop
	//  ros::Rate cycle(art_msgs::ArtHertz::PILOT); // set driver cycle rate
	//  while(ros::ok())
	//    {
	//      ros::spinOnce();                  // handle incoming messages

	monitorHardware();                // monitor device status

	// issue control commands
	speedControl(p);
	adjustSteering(p);

	publish("pilot_state",sizeof(PilotState),1); // publish updated state message///////////////

	//     cycle.sleep();                    // sleep until next cycle
	//    }
}



/** main entry point */
int main(int argc, char** argv)
{

	//  ros::init(argc, argv, "pilot");
	//  ros::NodeHandle node;

	//  PilotNode pilot(node);
	//PilotNodeConstructor();
	struct PilotState* pstate_msg_ = (struct PilotState*)malloc(sizeof(PilotState));
	PilotNodeConstructor(pstate_msg_);
	PilotNode_spin(pstate_msg_);
	//PilotState*
	return 0;
}
