#include "ros/ros.h"

#include "geometry_msgs/Twist.h"	// Message containing velocity
#include "nav_msgs/Odometry.h"		// Message containing estimated position
#include "holo_control/TargetVel.h"	// Service for target position

geometry_msgs::Twist current_vel;	/**< Twist used to contain the current estimated value of the robot velocities */

int k = 1;							/**< Coefficient used in the linear computation of instantaneous velocity */
float alfa = 0.1;					/**< Weight in the sum in case the mode of the service
										is set to 1, simulating inertia
									*/

/*********************************************//**
* Callback reading current estimated velocity
* and saving it for retrieval by the Service
* call
* 
* This function updates the values saved inside the
* global variable 'current_vel' each time a new
* state estimation is published on the topic
* "/odom"
* 
* \param pose_msg (const nav_msgs::Odometry::ConstPtr&):
* 			the current estimated state of the 
* 			robot, as published in the topic
* 			"/odom"
************************************************/
void subscriberCallback(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
	current_vel = pose_msg->twist.twist;
}

/*********************************************//**
* Server response returning target velocity
* 
* Each time a call request is made to the Service
* "/TargetVel" a velocity value is computed based
* on the current and target point coordinates, 
* passed in the request field of the Service,
* and on a "target_mode" defining which of the
* implemented algorithm to use for the computation.
* Currently two such algorithms are implemented:\n
* - 0: velocity is directly proportional to current
* 		distance from target position;\n
* - 1: velocity is the weighted sum between 
* 	one component being being proportional to 
* 	current distance from target position,
* 	and the previous velocity;
*
* \param req (holo_control::TargetVel::Request &):
* 			request field of the TargetVel
* 			service of package holo_control,
* 			composed of two Point(s) called
* 			'current_pos' and 'target_pos'
* 			and one integer called 
* 			'target_mode';
* \param res (holo_control::TargetPos::Response &):
* 			response field of the TargetVel
* 			service of package holo_control,
* 			is a Twist (defined in 
* 			"geometry_msgs") named 'twist'
* \retval flag (bool):
* 			flag unused, set to 'true';
************************************************/
bool target_vel(holo_control::TargetVel::Request &req, holo_control::TargetVel::Response &res){
	
	char mode = req.target_mode;
	float cx, cy, tx, ty;
	float prev_velx, prev_vely;
	cx = req.current_pos.x;
	cy = req.current_pos.y;
	tx = req.target_pos.x;
	ty = req.target_pos.y;

	switch (mode){
	case 0:		/* purely linearly dependant */
		res.twist.linear.x = k*(tx - cx);	// speed linearly depends from distance
		res.twist.linear.y = k*(ty - cy);
		break;
	case 1:
		prev_velx = current_vel.linear.x;	
		prev_vely = current_vel.linear.y;
		/* weighted sum between desired and previous value, to simulate inertia */
		res.twist.linear.x = alfa*k*(tx - cx) + (1-alfa)*prev_velx;
		res.twist.linear.y = alfa*k*(ty - cy) + (1-alfa)*prev_vely;
		break;
	default:
		res.twist.linear.x = 0;
		res.twist.linear.y = 0;
		break;
	}
	return true;
}


int main(int argc, char* argv[]){
	ros::init(argc, argv, "target_velocity_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("/target_velocity", target_vel);	 /* Server for the Service /target_position */
											/* note that the same topic could be used to hold target position generated in more "meaningful" way */

  	ros::Subscriber vel_sub = n.subscribe("/odom", 1000, subscriberCallback); 	/* subscriber to the topic relative to robot odometry, saving 
																					the current estimated velocity */
	ros::spin();

	return 0;

}
