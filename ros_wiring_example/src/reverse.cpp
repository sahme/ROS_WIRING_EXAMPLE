#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Bool.h"
#include <stdio.h>
#include <iostream>
#include "wiringPi.h"
const int pwmrf = 19, pwmlf = 18, rr= 16, lr=17;	
void pwm (short speed_index1, short speed_index2);
void pwm (short speed_index2, short speed_index1) // was index1, and index2, saw 1000,-1000 for left turn for ELSE return
{
	//speed_index2 = speed_index2/2;
	//speed_index1 = speed_index1/2;
	
	ROS_INFO("exe'in pwm with %d, %d", speed_index1, speed_index2);	
	short ip = 0;
	if (speed_index1 > 1023)
	{  speed_index1 = 1023;}
	if (speed_index2 > 1023)
	{	speed_index2 = 1023;}
	if (speed_index1 < -1023)
	{  speed_index1 = -1023;}
	if (speed_index2 < -1023)
	{	speed_index2 = -1023;}  //truncates indices at 1023 and -1023
	
	if(speed_index1 >= 0){
		pwmWrite(pwmrf, speed_index1);
		digitalWrite(rr, LOW);
		ROS_INFO("exe'in pwm right_forward");	}
	else{
		ip = (1023+speed_index1);
		pwmWrite(pwmrf, (ip));
		digitalWrite(rr, HIGH);
		ROS_INFO("exe'in pwm rr");}
		
	if(speed_index2 >= 0){
		pwmWrite(pwmlf, speed_index2);
		digitalWrite(lr, LOW);
		ROS_INFO("exe'in pwm left_forward");}
	else{
		ip = (1023+speed_index2);
		pwmWrite(pwmlf, (ip));
		digitalWrite(lr, HIGH);
		ROS_INFO("exe'in pwm lr");}
} 


int main(int argc,char** argv)
{
	wiringPiSetupGpio();
	unsigned  char pwmVal = 0;
 	ros::init(argc, argv, "blink_led1");
													ROS_INFO("Started Blink Node");
 	//wiringPiSetup ();
	pinMode(pwmrf,PWM_OUTPUT);
	pinMode(pwmlf,PWM_OUTPUT);
	pinMode(lr, OUTPUT);
	pinMode(rr, OUTPUT);
	digitalWrite(lr, LOW);
	digitalWrite(rr, LOW); 
	unsigned char count = 0;
	for (count = 0; count < 5; count++)
	{
		pinMode(uso[count], OUTPUT);
		pinMode(usi[count], INPUT);
		digitalWrite(uso[count],LOW);
	}
	unsigned char cti = 0;

	pwm(0,0);
	delay(100);	
	
	ros::NodeHandle n;
		
		//ros::Subscriber width = n.subscribe("objects",1,centroid_callback);
		ROS_INFO("Reversing");	
 		pwm(-1000,-1000);
		delay(1000);
		pwm(0,0);


	ros::spin();

return 0;
}
