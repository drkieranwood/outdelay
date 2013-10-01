// src/outdelay.cp

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <fstream>
#include <string>
#include <outdelay/outdelay.h>


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

//Debug messages flag.
int showDebugMsgs = 0;

//Storage array for many cmd_vel message pointers and received times. 
//An array of size 1000 is used and will cover most delays. 
//At 100Hz this gives a 10s delay.
geometry_msgs::TwistConstPtr dataArray[1000];
ros::Time timeArray[1000];

//The size of the above storage arrays
int arraySize=1000;
//Current input location in the array. This is where the last received data was put.
int lastInIdx=0;
//Current output location in the array. This is where the last data was 
//re-sent from in the array.
int lastOutIdx=0;

//Time to monitor the node loop rate
ros::Time lastLoopTime;


//======================
//Incomming message callback
//for a new twist message
//======================
void twistCallback(const geometry_msgs::TwistConstPtr& msg)
{
	//The latest message is put into the storage array as a pointer. The
	//when this occured is also stored.
	
	//Move along one index location, if at the end, then loop.
	lastInIdx++;
	if (lastInIdx==arraySize)
	{
		lastInIdx=0;
	}
		
	//Store the pointer to the data and the current time
	dataArray[lastInIdx] = msg;
	timeArray[lastInIdx] = ros::Time::now();
}


//======================
//Main function
//======================
//1)subscribes to the input stream
//2)captures every frame from the input stream
//3)check the storage arrays and republishes messages if the delay has expired.
int main(int argc, char **argv)
{
	//Setup ROS node (register with roscore etc.)
	ROS_INFO("outdelay::Start script.");
	ros::init(argc, argv, "outdelay");
	ros::NodeHandle n;
	
	
	//=========================
	//Check the arguments
	//=========================
	double delayTime;
	std::string inputTopic;
	std::string outputTopic;
	
	//If there are missing arguments then use the default values
	if (argc!=4)
	{
		ROS_WARN("outdelay::%d arguments given. Expected 3 -> [delay inTopic outTopic]",argc-1);
		delayTime = 0.0;
		inputTopic  = std::string("/cmd_vel_pre");
		outputTopic = std::string("/cmd_vel");
		ROS_INFO("outdelay::Using default delay: %6.3fHz",delayTime);
		ROS_INFO("outdelay::Using default input topic:  %s",inputTopic.c_str());
		ROS_INFO("outdelay::Using default output topic: %s",outputTopic.c_str());
	}
	else
	{
		//If all arguments have been provided then extract the information.	
		delayTime = atof(argv[1]);
		inputTopic  = argv[2];
		outputTopic = argv[3];
		ROS_INFO("outdelay::Using given delay: %6.3fHz",delayTime);
		ROS_INFO("outdelay::Using given input topic:  %s",inputTopic.c_str());
		ROS_INFO("outdelay::Using given output topic: %s",outputTopic.c_str());
	}
	
	
	//=========================
	//Publishers and subscibers
	//=========================
	
	//Subscribe to the incomming twist messages
	ros::Subscriber sub_cmd_vel = n.subscribe(inputTopic.c_str(), 1, twistCallback);
	//Publisher for outgoing, delayed, messages
	ros::Publisher  pub_cmd_vel = n.advertise<geometry_msgs::Twist>(outputTopic.c_str(), 1);


	//=========================
	//Other variables
	//=========================



	//=========================
	//Timings
	//=========================						
	double loopRate = 1000.0;                         
	ros::Rate rateLimiter(loopRate);


	//=========================
	//Main loop
	//=========================
	//Start the loop after a 2 second delay. To allow all ROS init. 
	//processes to complete.
	lastLoopTime = ros::Time::now();
	ros::Duration sleepone = ros::Duration(1,0);
	sleepone.sleep();
	sleepone.sleep();
	ROS_INFO("outdelay::Start loop at %6.3fHz",loopRate);		
	
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
				ROS_INFO("outdelay::Delay time: %6.5fs",dealyStats);
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
		
		if (showDebugMsgs)
		{
			//Find and publish the node rate (this is different to the output rate).
			double loopRate = (1.0)/((ros::Time::now() - lastLoopTime).toSec());
			lastLoopTime = ros::Time::now();
			ROS_INFO("outdelay::Node rate: %6.3fs",loopRate);
		}
		
	} //end main loop
	
	ROS_INFO("outdelay::End delay script.");
  	return 0;
}
//eof
