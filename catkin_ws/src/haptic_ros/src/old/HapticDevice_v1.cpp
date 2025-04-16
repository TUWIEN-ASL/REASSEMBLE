#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// #include <dhdc.h>
#include "haptic_ros/dhdc.h"

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include "eigen3/Eigen/Geometry"

#include "haptic_ros/OneEuroFilter.h"

// Fixed parameters
#define REFRESH_INTERVAL        0.1     // rate for cheching keyboard [seconds]
#define CTRL_RATE				1000 	// Hz
#define _USE_MATH_DEFINES
#define MAX_FORCE				10		// N
#define FILTER_DCUTOFF			1.0		// DO NOT CHANGE !!! (=1)

// Parameters form config
double MAX_FEEDBACK_FORCE;
double SCALE_FEEDBACK_FORCE;
double MAX_FRICTION_FORCE;
double FRICTION_SAT_REGION;
double ANGULAR_VISCOSITY;
double KP_CENTERING;
double CENTERING_SAT_REGION;
double FILTER_MINCUTOFF;
double FILTER_BETA;
double K_LOWPASS;
int SERIAL_NUMBER_OMEGA3;
int SERIAL_NUMBER_OMEGA6;

class HapticDevice
{
public:
	ros::NodeHandlePtr node_;

	// Publishers
	ros::Publisher master_dt_pub_;
	ros::Publisher master_position_cmd_pub_;
	ros::Publisher master_velocity_cmd_pub_;
	ros::Publisher master_button_cmd_pub_;
	// Subscribers
	ros::Subscriber force_feedback_sub_;	

	double ForceFeedback_[3];

	// Force feedback callback
	void ForceFeedbackCallback(const geometry_msgs::WrenchStampedConstPtr& msg){

		static double Dt = 0.0;
		static ros::Time old_t = msg->header.stamp;
		Dt = (msg->header.stamp - old_t).toSec();
		old_t = msg->header.stamp;
		if(!(Dt>0)) return;
        
		// Dt = 1/CTRL_RATE;     // ??? why manually setting dt and not taking previous calculation

		double Fs[3] = {msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z}; // Force from Slave

		// Force Saturation
		for(unsigned int i=0;i<3;i++){
			Fs[i] = Fs[i]*SCALE_FEEDBACK_FORCE;
			if(Fs[i] > MAX_FEEDBACK_FORCE){
				Fs[i] = MAX_FEEDBACK_FORCE;
			}
			else if(Fs[i] < -MAX_FEEDBACK_FORCE){
				Fs[i] = -MAX_FEEDBACK_FORCE;
			}
			ForceFeedback_[i] = Fs[i];
		}
	}

	double LowPassFilter(const double measured_value, const double filtered_value_old, const float alpha){
	  double filtered_value;
      filtered_value = filtered_value_old + alpha * (measured_value - filtered_value_old);
	  return filtered_value;
	}

	// Load parameters etc
	int init()
	{
		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));

		// Publisher
	    master_dt_pub_ 				= node_->advertise<std_msgs::Float64>("/Master/dt",10);
	    master_position_cmd_pub_ 	= node_->advertise<geometry_msgs::PoseStamped>("/Master/command/pose", 10);
	    master_velocity_cmd_pub_ 	= node_->advertise<geometry_msgs::TwistStamped>("/Master/command/twist",10);
		master_button_cmd_pub_ 		= node_->advertise<std_msgs::Bool>("/Master/command/button", 10);
		//Subscriber
	    force_feedback_sub_ 		= node_->subscribe<geometry_msgs::WrenchStamped>("/Master/FeedbackForce", 10, &HapticDevice::ForceFeedbackCallback, this);
		
		// Parameter
		// Viscous dmaping force
		node_->getParam("MAX_FRICTION_FORCE", MAX_FRICTION_FORCE);
        node_->getParam("FRICTION_SAT_REGION", FRICTION_SAT_REGION);
		node_->getParam("ANGULAR_VISCOSITY", ANGULAR_VISCOSITY);
		// Centering force
        node_->getParam("CENTERING_SAT_REGION", CENTERING_SAT_REGION);
		node_->getParam("KP_CENTERING", KP_CENTERING);
		// Feedback force
        node_->getParam("MAX_FEEDBACK_FORCE", MAX_FEEDBACK_FORCE);
		node_->getParam("SCALE_FEEDBACK_FORCE", SCALE_FEEDBACK_FORCE);
		// Control force filter
        node_->getParam("FILTER_MINCUTOFF", FILTER_MINCUTOFF);
		node_->getParam("FILTER_BETA", FILTER_BETA);
		// Twist filter
        node_->getParam("K_LOWPASS", K_LOWPASS);
		// Devices
		node_->getParam("SERIAL_NUMBER_OMEGA3", SERIAL_NUMBER_OMEGA3);
        node_->getParam("SERIAL_NUMBER_OMEGA6", SERIAL_NUMBER_OMEGA6);

		return 0;
	}
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "HapticDevice");

  HapticDevice haptic_device;
  if (haptic_device.init())
	{
    	ROS_FATAL("haptic device initialization failed");
		return -1;
	}
	
	// 1€ Filter
	OneEuroFilter filterX(CTRL_RATE, FILTER_MINCUTOFF, FILTER_BETA, FILTER_DCUTOFF);
	OneEuroFilter filterY(CTRL_RATE, FILTER_MINCUTOFF, FILTER_BETA, FILTER_DCUTOFF);
	OneEuroFilter filterZ(CTRL_RATE, FILTER_MINCUTOFF, FILTER_BETA, FILTER_DCUTOFF);

	// Initialize variables
	std_msgs::Bool userButton;
	std_msgs::Float64 master_dt;
	double px, py, pz;
	double roll, pitch, yaw;
	double current_rot_matrix[3][3] = {};
	double orientation_[4];
	double vx, vy, vz;
	double wx, wy, wz;
	double fx, fy, fz;
	double tx, ty, tz;
	
	int    done  = 0;
	double t0    = dhdGetTime ();
	double t1    = t0;

	double final_force[DHD_MAX_DOF];
		
	haptic_device.ForceFeedback_[0] = 0.0;
	haptic_device.ForceFeedback_[1] = 0.0;
	haptic_device.ForceFeedback_[2] = 0.0;

  	// message
	int major, minor, release, revision;
	dhdGetSDKVersion(&major, &minor, &release, &revision);
	printf("\n");
	printf("Force Dimension - Viscosity Example %d.%d.%d.%d\n", major, minor, release, revision);
	printf("(C) 2014 Force Dimension\n");
	printf("All Rights Reserved.\n\n");

	// required to change asynchronous operation mode
	// dhdEnableExpertMode();

	// // open device specified by serial number
	// if (dhdOpenSerial(SERIAL_NUMBER_OMEGA6) < 0) {
	// 		printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
	// 		dhdSleep (2.0);
	// 		return -1;
	// 	}
	// open the first available device
    if (dhdOpenID(0) < 0) {
      printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
      dhdSleep(2.0);
      return -1;
    }
    printf("device ID : %d\n",dhdGetDeviceID());

	// identify device
    printf ("%s device detected\n\n", dhdGetSystemName());

	// display instructions
	printf ("press 'q' to quit\n");

	// enable force
	dhdEnableForce (DHD_ON);

  	ros::Rate loop_rate(CTRL_RATE);
	double Dt = 1/CTRL_RATE;
	// loop while the button is not pushed
	while (!done) {
		ros::spinOnce();

		ros::Time t_now = ros::Time::now();
		static ros::Time t_old = t_now;
		double dt = (t_now - t_old).toSec();
		master_dt.data = dt;
		t_old = t_now;
		if(dt == 0.0) continue;

		//---Get Omega State
		dhdGetPosition(&px,&py,&pz);
		dhdGetOrientationRad(&roll, &pitch, &yaw);
		dhdGetLinearVelocity (&vx, &vy, &vz);
		dhdGetAngularVelocityRad (&wx, &wy, &wz);
		userButton.data = (dhdGetButton(0) != DHD_OFF);
		dhdGetOrientationFrame(current_rot_matrix);
		// create an Eigen matrix from the double array
		Eigen::Matrix3d rot_matrix;
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				rot_matrix(i,j) = current_rot_matrix[i][j];
			}
		}
		// convert the rotation matrix to quaternion
		Eigen::Quaterniond quat(rot_matrix);
		orientation_[0] = quat.w();
		orientation_[1] = quat.x();
		orientation_[2] = quat.y();
		orientation_[3] = quat.z();

		//--- Force Init
		final_force[0] = 0.0;
		final_force[1] = 0.0;
		final_force[2] = 0.0;

		//---1. Centering Force
		double centering_force[3];
		double p[3];
		p[0] = px;
		p[1] = py;
		p[2] = pz;

		// saturation of centering force
		for(unsigned int i=0;i<3;i++){
			if(p[i] > CENTERING_SAT_REGION){
				p[i] = CENTERING_SAT_REGION;
			}
			else if(p[i] < -CENTERING_SAT_REGION){
				p[i] = -CENTERING_SAT_REGION;
			}
			centering_force[i] = -p[i]*KP_CENTERING;
			final_force[i] += centering_force[i];
		}

		//---2. Viscosity Damping Force
		double damping_force[3];
		tx = -ANGULAR_VISCOSITY * wx;
		ty = -ANGULAR_VISCOSITY * wy;
		tz = -ANGULAR_VISCOSITY * wz;

		double velocity = sqrt(pow(vx,2)+pow(vy,2)+pow(vz,2));
		double friction_force = 0.0;

		friction_force = -MAX_FRICTION_FORCE/FRICTION_SAT_REGION * velocity;

		if(friction_force > MAX_FRICTION_FORCE) friction_force = MAX_FRICTION_FORCE;
		else if(friction_force < -MAX_FRICTION_FORCE) friction_force = -MAX_FRICTION_FORCE;

		if(fabs(velocity)>0) {
			damping_force[0] = vx / velocity * friction_force;
			damping_force[1] = vy / velocity * friction_force;
			damping_force[2] = vz / velocity * friction_force;
		}
		else{
			damping_force[0] = 0.0;
			damping_force[1] = 0.0;
			damping_force[2] = 0.0;
		}
		// printf("Damping Force: x=%f, y=%f, z=%f\n", damping_force[0], damping_force[1], damping_force[2]);

		final_force[0] += damping_force[0];
		final_force[1] += damping_force[1];
		final_force[2] += damping_force[2];

		// Force Feedback reflect to the omega6
		// final_force[0] += haptic_device.ForceFeedback_[0];
		// final_force[1] += haptic_device.ForceFeedback_[1];
		// final_force[2] += haptic_device.ForceFeedback_[2];
		final_force[0] -= haptic_device.ForceFeedback_[0];
		final_force[1] -= haptic_device.ForceFeedback_[1];
		final_force[2] -= haptic_device.ForceFeedback_[2];

		// Anti-resonance force filtering
		final_force[0] = filterX.filter(final_force[0]);
		final_force[1] = filterY.filter(final_force[1]);
		final_force[2] = filterZ.filter(final_force[2]);

		// Limit max final force
		for(unsigned int i=0;i<3;i++){
			if(final_force[i] > MAX_FORCE){
				final_force[i] = MAX_FORCE;
			}
			else if(final_force[i] < -MAX_FORCE){
				final_force[i] = -MAX_FORCE;
			}
		}
		
		geometry_msgs::PoseStamped MasterPositionCmd;
		MasterPositionCmd.header.frame_id = "haptic_device";
		MasterPositionCmd.header.stamp = ros::Time::now();

		geometry_msgs::TwistStamped MasterVelocityCmd;
		MasterVelocityCmd.header.frame_id = "haptic_device";
		MasterVelocityCmd.header.stamp = ros::Time::now();

		// Update pose
		MasterPositionCmd.pose.position.x 	= px;
		MasterPositionCmd.pose.position.y 	= py;
		MasterPositionCmd.pose.position.z 	= pz;
		MasterPositionCmd.pose.orientation.w 	= orientation_[0];
		MasterPositionCmd.pose.orientation.x 	= orientation_[1];
		MasterPositionCmd.pose.orientation.y 	= orientation_[2];
		MasterPositionCmd.pose.orientation.z 	= orientation_[3];
		
		// Update velocity
		MasterVelocityCmd.twist.linear.x 	= vx;
		MasterVelocityCmd.twist.linear.y 	= vy;
		MasterVelocityCmd.twist.linear.z 	= vz;
		MasterVelocityCmd.twist.angular.x 	= wx;
		MasterVelocityCmd.twist.angular.y 	= wy;
		MasterVelocityCmd.twist.angular.z 	= wz;

		// Low Pass Filtering
		static geometry_msgs::TwistStamped prevMasterVelocityCmd = MasterVelocityCmd;
		MasterVelocityCmd.twist.linear.x = haptic_device.LowPassFilter(MasterVelocityCmd.twist.linear.x, prevMasterVelocityCmd.twist.linear.x, K_LOWPASS);
		MasterVelocityCmd.twist.linear.y = haptic_device.LowPassFilter(MasterVelocityCmd.twist.linear.y, prevMasterVelocityCmd.twist.linear.y, K_LOWPASS);
		MasterVelocityCmd.twist.linear.z = haptic_device.LowPassFilter(MasterVelocityCmd.twist.linear.z, prevMasterVelocityCmd.twist.linear.z, K_LOWPASS);
		prevMasterVelocityCmd = MasterVelocityCmd;		

		// Publish ROS topics
		haptic_device.master_position_cmd_pub_.publish(MasterPositionCmd);
		haptic_device.master_velocity_cmd_pub_.publish(MasterVelocityCmd);
		haptic_device.master_dt_pub_.publish(master_dt);
		haptic_device.master_button_cmd_pub_.publish(userButton);

		//---Force Set To Master Device
		if (dhdSetForceAndTorque(final_force[0], final_force[1], final_force[2], tx, ty, tz) < DHD_NO_ERROR) {
			printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
			done = 1;
		}

		// display refresh rate and position at 10Hz
		t1 = dhdGetTime();
		if ((t1-t0) > REFRESH_INTERVAL) {
			t0 = t1;

			if (dhdKbHit ()) {
				switch (dhdKbGet()) {
					case 'q': done = 1; break;
					}
				}
			}
			loop_rate.sleep();
	}

	// close the connection
  	dhdClose ();

  	// happily exit
  	printf ("\ndone.\n");
  	return 0;
}
