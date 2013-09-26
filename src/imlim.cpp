// src/imlim.cp

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <math.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>



//The current image to publish each loop. Even if no new image has 
//arrived the lastest captured image will be published repeatedly.
//It is stored as a pointer rather than an entire image.
sensor_msgs::ImageConstPtr currentImg;

//Time the last image was received and time the last image was sent.
ros::Time lastImageTime_rec;
ros::Time lastImageTime_pub;

//Flag to indicate at least one image has been received. This allows the
//node to start even when the input image stream is not operating.
int oneImgFlag = 0;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//The msg object is a pointer. As such it must be used with the 
	//pointer operator -> to access its children and any functions it is
	//passed to must recognise it as a pointer, NOT an object.
	
	//Store the pointer to the latest image.
	currentImg = msg;

	//Set the time this image arrived
	lastImageTime_rec = ros::Time::now();
	
	if (oneImgFlag==0)
	{
		oneImgFlag=1;
	}
}



//Main limit loop. This,
//1)subscribes to an image stream
//2)captures every frame from the image stream
//3)only republishes the latest received image at a specified rate
int main(int argc, char **argv)
{
	ROS_INFO("imlim::Start script.");
	//Setup ROS node (register with roscore etc.)
	ros::init(argc, argv, "imlim");
	ros::NodeHandle n;
	

	//=========================
	//Publishers and subscibers
	//=========================
	
	//Subscribe to the image stream
	ros::Subscriber sub_image = n.subscribe<sensor_msgs::Image>("/ardrone/image_mono", 1, imageCallback);
	
	//Setup output image stream
	ros::Publisher  pub_image = n.advertise<sensor_msgs::Image>("/ardrone/image_mono_limited", 1);


	//=========================
	//Other variables
	//=========================
	//A counter to show a warning if no new images arrive, and to make 
	//the node publish an image every 10 loops.
	int warnCount = 0;
	int pubCount = 0;

	//Tolerance in seconds for a drop out warning.
	double dropOutTol = 3.0;


	//=========================
	//Timings
	//=========================

	//The timing is controlled by a ros::rate object. The argument is 
	//the desired loop rate in Hz. Note this must be slower than the 
	//native image stream rate else this node will do nothing and will
	//just republish at the incomming image rate.
	//Although the output of this node is published at the desired rate 
	//the actual internal loop runs 10x faster to ensure the latest 
	//image is always published.
	double imageRate = 1.0;                                             /*The output image rate*/
	double loopRate = imageRate*10.0;                                   /*The node loop rate (10x output rate)*/
	ros::Rate rateLimiter(loopRate);


	//=========================
	//Main loop
	//=========================
	//Start the loop after a 2 second delay. To allow all ROS init. 
	//processes to complete.
	ros::Duration sleepone = ros::Duration(1,0);
	sleepone.sleep();
	sleepone.sleep();
	ROS_INFO("imlim::Start loop at %5.2fHz",loopRate);		
	
	//Main loop. Loop forever until node is killed.
	while (ros::ok())
	{	
		//Get latest image from input stream (if one is available).
		ros::spinOnce();


		//Publish the image. This happens regardless if the image has 
		//been updated. The node loop runs 10x the desired rate. Hence 
		//only publish every 10 iteraions.
		if (oneImgFlag==1)
		{
			if (pubCount==0)
			{
				pub_image.publish(currentImg);
				
				//Find the current loop rate and display. Also find the 
				//delay imposed by this node.
				double curLoopRate = (1.0)/((ros::Time::now() - lastImageTime_pub).toSec());
				double delayTime   = (ros::Time::now() - lastImageTime_rec).toSec();
				ROS_INFO("imlim::Current rate: %6.3fHz, delay: %6.5fs",curLoopRate,delayTime);
				lastImageTime_pub = ros::Time::now();
			}
			pubCount++;
			if (pubCount==10)
			{
				pubCount=0;
			}
		}
		else
		{
			ROS_INFO("imlim::No images published.");
		}


		//Check to see if no new images have arrived for a while. 
		//If so then show a warning. Only show this warning every 50 
		//iterations, or every time a new dropout happens.
		double timeSinceLast = (ros::Time::now() - lastImageTime_rec).toSec();
		if (timeSinceLast > dropOutTol)
		{
			if (warnCount==0)
			{
				ROS_INFO("imlim::Warning no new images received for %5.2fs",dropOutTol);
			}
			warnCount++;
			if (warnCount==50)
			{
				warnCount=0;
			}
		}
		//If a new image was received recently (less that 0.5s) then 
		//reset the warning counter.
		if ((timeSinceLast < 0.5) && (warnCount!=0))
		{
			warnCount=0;
		}	


		//Check for new ros messages
		ros::spinOnce();
		rateLimiter.sleep();
		ros::spinOnce();	
	}

	ROS_INFO("imlim::End script.");
  	return 0;
}
//eof
