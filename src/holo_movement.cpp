#include "ros/ros.h"
#include "geometry_msgs/Twist.h"	// Message containing velocity
#include "nav_msgs/Odometry.h"		// Message containing estimated position
#include "holo_control/TargetPos.h"	// Service for target position
#include "holo_control/TargetVel.h"	// Service for target velocity

#include <sstream>
#include <iostream>
#include <math.h>


/* This is the node that receives a position and drives the robot towards it */


/*********************************************//**
* Function to compute norm of vector
* 
* This function computes the euclidean norm of the 
* vector identified by starting and ending points 
* passed as arguments
*
* \param x (float):	
* 			starting point X coordinate;
* \param y (float):	
* 			starting point Y coordinate;
* \param tx (float):	
* 			ending point X coordinate;
* \param ty (float):	
* 			ending point y coordinate;
*
* \retval norm2 (float):
* 			euclidean norm of the vector
* 			going from start(x,y) to end(x,y);
************************************************/
float norm2(float x, float y, float tx, float ty);


ros::Publisher pub;					/**< Publisher used to publish velocities */
ros::ServiceClient client_target;	/**< Service client used for obtaining target positions */
ros::ServiceClient client_vel;		/**< Service client used for obtaining target velocity */
geometry_msgs::Point target_pos;	/**< Target position to reach, defined as a Point */
float dist_th = 0.1; 				/**< Distance threshold used for determining if a destination is reached */
		
int vel_algorithm = 0;				/**< Flag determining the criteria used for evaluating the velocity\n
									*	- 0: 	velocity is directly proportional to current distance from 
									*				target position;\n
									*	- 1 :		velocity is the weighted sum between one component, being
									*				proportional to current distance from target position and
									*				the previous velocity
									*/

/*********************************************//**
* Callback publishing the velocity after reading a
* position
* 
* This function computes the velocity published on
* the topic "/cmd_vel" based on the estimated
* position retrieved from the messages in the 
* "/odom" odometry topic. When the target position
* is reached a call to the Service "/target_pos"
* is made in order to retrieve a new target
* position.
* 
* \param pose_msg (/nav_msgs::Odometry::ConstPtr&):
* 			pointer to a message read from topic
* 			"/odom", used to obtain estimated
* 			current position and, if the flag
* 			'pseudo_inertia' = 'true', 
* 			estimated current linear velocity; 
************************************************/
void subscriberCallback(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
	geometry_msgs::Twist vel;
	float posx = pose_msg->pose.pose.position.x;
	float posy = pose_msg->pose.pose.position.y;
	
	if (norm2(posx, posy, target_pos.x, target_pos.y) <= dist_th){
	/* If the robot reached its target position request a new one */
		holo_control::TargetPos t_pos;
		client_target.call(t_pos);	// the request is empty, data is passed in the reply
		target_pos.x = t_pos.response.target_pos.x;
		target_pos.y = t_pos.response.target_pos.y;

		ROS_INFO("HOLO_MOVEMENT new target @[%f, %f]\n", target_pos.x, target_pos.y);
	}
	/* 	move towards the target 
		*note that it will move even right after a new target is received, by design choice* 
	*/
	/* Velocity to publish is not computed inside this node, but inside a the Service Server for TargetVel
		which requires current and target point, plus a parameter defining in whic way to compute such velocity
		which is returned in the response field.
	 */
	holo_control::TargetVel t_vel;
	t_vel.request.current_pos.x 	= posx;
	t_vel.request.current_pos.y 	= posy;
	t_vel.request.target_pos.x	 	= target_pos.x;
	t_vel.request.target_pos.y	 	= target_pos.y;
	t_vel.request.target_mode		= vel_algorithm;	// if the flag 'pseudo_inertia' is set call the service with mode 1, 0 otherwise
	client_vel.call(t_vel);

	vel.linear.x = t_vel.response.twist.linear.x;
	vel.linear.y = t_vel.response.twist.linear.y;

 	pub.publish(vel);
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "holo_control");
  	ros::NodeHandle n;

	vel_algorithm = atoi(argv[1]);
	
  	client_target =	n.serviceClient<holo_control::TargetPos>("/target_position");
	/* 	bootstrap the first target position. Note that all the other targets will be 
		called from inside the subscriberCallback  */ 
		holo_control::TargetPos t_pos;
		client_target.call(t_pos);	// the request is empty, data is passed in the reply
		target_pos.x = t_pos.response.target_pos.x;
		target_pos.y = t_pos.response.target_pos.y;
		ROS_INFO("HOLO_MOVEMENT new target @[%f, %f]\n", target_pos.x, target_pos.y);


  	client_vel =	n.serviceClient<holo_control::TargetVel>("/target_velocity");

  	ros::Subscriber pose_sub = n.subscribe("/odom", 1000, subscriberCallback); 	/* subscriber to the topic relative to
																					robot odometry, setting the callback
																					computing velocity
																				 */
	pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	ros::spin();
	return 0;
}

float norm2(float x, float y, float tx, float ty){
	return sqrt((tx-x)*(tx-x)+(ty-y)*(ty-y));
}




