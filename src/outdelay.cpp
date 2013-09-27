// src/outdelay.cp

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>


//This node captures all incomming mesages on a topic and then tries to 
//re-publish them at a later time, after a set delay.

//All messgaes are captured into a storage array along with a received 
//time. Each time the node loops/iterates, the array of received times 
//is checked to see if any messages have been stored for long enough 
//(i.e. receivedTime+delay > timeNow). If this condition is met the
//message is repulished. The messgaes are checked in order to make sure 
//that if multiple messgaes are found to need republishing then the 
//latest one is published last.


//=========================
//Global variables
//=========================		
//Storage array for many cmd_vel message pointers and received times. 
//An array of size 1000 is used and will cover most delays. 
//At 100Hz this gives a 10s delay.
geometry_msgs::TwistConstPtr dataArray[1000];
ros::Time timeArray[1000];

//Array size
int arraySize=1000;
//Current input location in the array. This is where the last received data was put.
int lastInIdx=0;
//Current output location in the array. This is where the last data was 
//re-sent from in the array
int lastOutIdx=0;

//The time delay required in seconds.
double delayTime = 0.2;



//=========================
//Incomming message callback
//=========================	
//Function to capture arriving messages into the storage array. 
//They are stored as pointers to save memory and cpu time.
void twistCallback(const geometry_msgs::TwistConstPtr& msg)
{
	//Move along one index location, if at the end then loop.
	lastInIdx++;
	if (lastInIdx==arraySize)
	{
		lastInIdx=0;
	}
		
	//Store the pointer to the data and the current time
	dataArray[lastInIdx] = msg;
	timeArray[lastInIdx] = ros::Time::now();
}



int main(int argc, char **argv)
{
	ROS_INFO("outdelay::Start delay script.");
	//Setup ROS node (register with roscore etc.)
	ros::init(argc, argv, "outdelay");
	ros::NodeHandle n;
	
	//Subscribe to the incomming twist messages
	ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel", 1, twistCallback);
	
	//Publisher for outgoing, delayed, messages
	ros::Publisher  pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel_delay", 1);



	//=========================
	//Timings
	//=========================						
	double loopRate = 1000.0;                         
	ros::Rate rateLimiter(loopRate);
	//Time to monitor the node loop rate
	ros::Time lastLoopTime;



	//=========================
	//Main loop
	//=========================
	//Start the loop after a 2 second delay. To allow all ROS init. 
	//processes to complete.
	lastLoopTime = ros::Time::now();
	ros::Duration sleepone = ros::Duration(1,0);
	sleepone.sleep();
	sleepone.sleep();
	ROS_INFO("outdelay::Start loop at %5.2fHz",loopRate);		
	
	//Main loop. Loop forever until node is killed.
	while (ros::ok())
	{	
		//Get latest image from input stream (if one is available).
		ros::spinOnce();
		
		//Each loop check through the stored values to see if any need 
		//to be republished. Move from lastOutDataIdx++ location to the 
		//current lastInDataIdx.
		int ii = lastOutIdx;
		int outputOn = 1;
		while ( (ii!=lastInIdx) && (outputOn == 1) )
		{
			//Move along one index (since the lastOutIdx represents the 
			//data that has been outputted already)
			ii++;
			if (ii==arraySize)
			{
				ii=0;
			}
			
			//Check if the data should be output, and if so then output it.
			ros::Time storedTime = timeArray[ii];
			if ( (storedTime.toSec() + delayTime) < (ros::Time::now()).toSec() )
			{
				pub_cmd_vel.publish(dataArray[ii]);
				lastOutIdx = ii;
				double dealyStats = (ros::Time::now() - timeArray[ii]).toSec();
				ROS_INFO("outdelay::Delay time: %6.4fs",dealyStats);
			}
			else
			{
				//The first data value that fails, because not enough 
				//time has elapsed, marks the point at which the outputs
				//should stop trying. This saves searching though a large 
				//number of data entries.
				outputOn = 0;
			}
		}
		
		
	
		//Check for new ros messages
		ros::spinOnce();
		rateLimiter.sleep();
		ros::spinOnce();	
		
		if (0)
		{
			//Find and publish the node rate.
			double loopRate = (1.0)/((ros::Time::now() - lastLoopTime).toSec());
			lastLoopTime = ros::Time::now();
			ROS_INFO("outdelay::Node rate: %6.3fs",loopRate);
		}
	}
	
	ROS_INFO("outdelay::End delay script.");
  	return 0;
}
//eof
