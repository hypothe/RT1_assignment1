#include "ros/ros.h"
#include <cstdlib>	// rand()
#include <ctime>	// rand() seed
#include <math.h>

#include "holo_control/TargetPos.h"	// Service for target position

/*********************************************//**
* Server response returning two random values
* 
* This function inserts two random values
* between -6.0 and 6.0 each inside the 
* response field of a service.
*
* \param req (holo_control::TargetPos::Request &):
* 			request field of the TargetPos
* 			service of package holo_control,
* 			not utilized (being it acutally
* 			empty);
* \param res (holo_control::TargetPos::Response &):
* 			response field of the TargetPos
* 			service of package holo_control,
* 			is a Point (defined in 
* 			"geometry_msgs") named 'target_pos'
* 			its fields x and y are set to a
* 			float random value between -6.0
* 			and 6.0;
*
* \retval flag (bool):
* 			flag unused, set to 'true';
************************************************/
bool rand_pos(holo_control::TargetPos::Request &req, holo_control::TargetPos::Response &res){
	res.target_pos.x = 12.0 * ((float) rand())/ ((float)RAND_MAX) - 6.0;
	res.target_pos.y = 12.0 * ((float) rand())/ ((float)RAND_MAX) - 6.0;
	return true;
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "random_position_server");
	ros::NodeHandle n;
	srand(time(0));	/* random seed initialization */
	ros::ServiceServer service = n.advertiseService("/target_position", rand_pos);	 /* Server for the Service /target_position */
										/* note that the same topic could be used to hold target position generated in more "meaningful" way */
	ros::spin();

	return 0;
}
