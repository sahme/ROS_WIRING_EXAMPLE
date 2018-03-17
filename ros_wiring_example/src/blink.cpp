#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include <stdio.h>

#include <iostream>
#include "wiringPi.h"
#include <sys/time.h>

#define thresh1 210
#define thresh2 420
#define finding_delay 500						
#define right_turn_delay 800
#define neg_buffer 5
#define corr_fwd_del 500
#define w8_tym_for_echo 900

//motor pins
const int pwmrf = 19, pwmlf = 18, rr= 16, lr=17;	


//polling variables
unsigned char obs1=0, obs2=0, obs3=0, obs4=0, obs5=0;
unsigned char echo = 0;
//polling pins
int uso [6] = {0,23, 21,25,27,4};			
int usi [6] = {0,22, 20,24, 26,5};

void pwm (short speed_index1, short speed_index2);
void poll(unsigned char i);
void Move(void);


const short alignindexsmall = -1000, alignindexlarge = 1000,aligndelay =200;				
													
	
													
const short rightturnlarge = 1000,rightturnsmall = -1000;
float width_of_object =0;

int avoiding_obstacle = 0;

unsigned char obs=0;

unsigned  char cip =0, false_negative_buffer = 1;
short centroid =0, x2 = 100, y2 = 100;
float id = 0.00;


void centroid_callback(const std_msgs::Float32MultiArray::ConstPtr &  msg) 
{
//align() variables:
 	unsigned char cil = 0, cir = 0, cit = 0;
	unsigned char count = 0;
//call_back variables
	const std::vector<float> & homo = msg->data;           
	ROS_INFO("in centroid call_back");
	
	
//if avoiding obstacle, stay here till past the obstacle	
	while(avoiding_obstacle == 1){	
	ROS_INFO("Poling the sensrs while avoiding obstacles");
	obs =0;									// go in with an open mind
	for(count = 1; count<=5; count++)		//check for obstacles
	{
	poll(count);
	}
	ROS_INFO("Polling Output: %s", obs?"obs detected":"Path is clear");
	Move();
//return to allow fresh input from the camera... the var should turn zero when no obs are detected in corr_fwd state and it then moves left
	if (avoiding_obstacle == 0){return;}
//otw stay in the loop
	}
//go forward only when the obstacle has been avoided


	if (homo.size())
	{ 
	false_negative_buffer = 1;
	ROS_INFO("homo_Zero: %d", (int)homo[0]);
	cip =1;
	int id = (int)homo[0];
	width_of_object = homo[1];
	float  h1 = homo[3], h2 = homo[4], h3 = homo[5],h4 = homo[6], h5 = homo[7], h6 = homo[8], h7 = homo[9], h8 = homo[10], h9 = homo[11];
	float x2= 0.0, y2 = 0.0, x1 = 0.0, y1 = 0.0;
	x2 = 320 - (homo[1]/2); y2 = 200 - (homo[2]/2);
	x1 = ((h1*x2) + (h2*y2) + h3);
	y1 = ((h4*x2) + (h5*y2) + h6);
	centroid = short(ceil(x1 + (homo[1]/2)));
	

	centroid = homo[9] + (homo[1]/2);
	

	ROS_INFO("homo[0] [2]: %f, %f, %f", homo[0], homo[1], homo[2]);
 	ROS_INFO("homo[3] [5]: %f, %f, %f", homo[3],homo[4], homo[5]);
 	ROS_INFO("homo[6] [8]: %f, %f, %f",homo[6],homo[7],homo[8]);
 	ROS_INFO("homo[9] [11]: %f, %f, %f",homo[9],homo[10],homo[11]);
	ROS_INFO("CENTROID: %d", centroid);
	if(width_of_object >= 270) {pwm(0,0); return;}
		
	if ((centroid<=thresh2)&&(centroid>=thresh1)){
		//centroid in the picture, we can move forward
		ROS_INFO("Ready to move frwrd, Poling the sensrs");
		obs =0;									// go in with an open mind
		for(count = 1; count<=5; count++)		//check for obstacles
		{
			poll(count);
		}
		ROS_INFO("Polling Output: %s", obs?"obs detected":"Path is clear");
		//then move
		Move();
	}
	else if (centroid<=thresh1)
		{
		ROS_INFO("What  kreft! now its too left...= %d", centroid);
		pwm(alignindexsmall,alignindexlarge);
		delay(aligndelay);
		pwm(0,0);
		}
	else if (centroid >= thresh2){
		ROS_INFO("GOAL IN SIGHT, but too right centroid centroid = %d", centroid);
		pwm(alignindexlarge,alignindexsmall);
		delay(aligndelay);
		pwm(0,0);
		}
	
	
		
	
		
	
	}
	else{
		
		//ROS_INFO("Object not detected, Poling the sensrs");
		//obs =0;						// go in with an open mind
		//for(count = 1; count<=5; count++)		//check for obstacles
		//{
			//poll(count);
		//}
		//ROS_INFO("Polling Output: %s", obs?"obs detected":"Path is clear");
		false_negative_buffer++;
		pwm(0,0);
		if(false_negative_buffer >=neg_buffer){
		cip= 0;
		ROS_INFO("ELSE return from centr_callback");
		pwm(alignindexsmall,alignindexlarge);				
		delay(finding_delay);
		pwm(0,0);
		false_negative_buffer = 0;	
		}
		}
}







void poll(unsigned char i)
{	long ind =0;
	echo = 0;
	ROS_INFO("poll called with i=%d", i);
	 ROS_INFO("op pin: %d ip pin: %d", uso[i], usi[i]);
	digitalWrite(uso[i],HIGH);
	delayMicroseconds(10);
	digitalWrite(uso[i],LOW);

	while(digitalRead(usi[i]) == 0)
	{ ind++;
	 if (ind >= 100000){ROS_INFO("Ignored sensor %d", i);
	goto TF;}
	}	
	delayMicroseconds(w8_tym_for_echo);
	echo = !(digitalRead(usi[i]));
	TF:
	//ROS_INFO("echo: pin read: %d,   i: %d", echo, i);	
        if(echo != 0)
	{
	ROS_INFO("echo = true");
	obs = 1;
	switch (i)
	{
		case 1:
		obs1 = 1;		break;
		case 2:
		obs2 =1;		break;
		case 3:
		obs3 =1;		break;
		case 4:
		obs4 =1;		break;
		case 5:
		obs5 =1;		break;
	}
	}
	else{
	ROS_INFO("echo = false");
		switch(i)
		{
			case 1:
			obs1 = 0;		break;
			case 2:
			obs2 =0;		break;
			case 3:
			obs3 =0;		break;
			case 4:
			obs4 =0;		break;
			case 5:
			obs5 =0;		break;
		}
	}
	return;
}






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





//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//.......................................................................................................................................................................................................................


//int sass =0;
			
//.......................................................................................................................................................................................................................
int object = int(id);
enum Move_State {initi, movfwd, movright, movleft, correctionalfwd} state;
void Move(void)
{
	ROS_INFO("MOVE with centroid = %d", centroid);
	switch(state)
	{
		case initi:
		ROS_INFO("Tansition:  State: initi, obs =%d", obs);
		if (!obs){state = movfwd;}
		if(!obs2 && !obs3 && !obs4 ){state = movfwd;}
		else{state = movright;}
		break;
		
		case movfwd:
		ROS_INFO("Tansition:  State: movfwd");
		if(!obs1 && !obs2 && !obs3)
		{			state = movfwd;			}
		else if(obs2 || obs3)
		{			state = movright;		}
		break;
		
		case movright:
		ROS_INFO("Tansition:  State: movright");
		avoiding_obstacle = 1;
		if(obs2 || obs3 )//|| !obs4)		//and not obs5
		{			state = movright;	
		//sass = 1;
			}
		if(!obs2 && !obs3)// && !obs4)
		{		state = correctionalfwd;	}
		break;
		
		case correctionalfwd:
		ROS_INFO("Tansition:  State: corr_fwd");
		if(!obs1 && !obs2 && !obs3)
		{
			state = movleft;
			avoiding_obstacle = 0;}
		if(obs1 || obs2)
		{		state = correctionalfwd;	
		
		}
		break;
		
		case movleft:
		ROS_INFO("Tansition:  State: movleft");
		avoiding_obstacle = 0;
		if(centroid > thresh1)
		{			state = movfwd;			}
		if(centroid <= thresh1)
		{			state = movleft;			}	

		break;
		default:
		break;
	}
	switch(state)
	{
		case movfwd:
		ROS_INFO("Action:  State: movfwd");			
			if(id == 1){pwm(0,0);}
			else if((id == 1)||(obs3 == 1)||width_of_object >=360){pwm(0,0);}
			else if((id == 3)||(id==2)){pwm(800,800);}
				else{pwm(1023,1023);}
			delay(1000);
			pwm(0,0);
			break;

		case movright:						
		ROS_INFO("action:  State: movr8");
			//if(!obs3){
			pwm(rightturnlarge,rightturnsmall);
			delay(right_turn_delay);
			pwm(0,0);
			//}
			//else if(obs3 && sass){pwm(-800, -1023);delay(right_turn_delay);pwm(0,0);sass = 0;}
			avoiding_obstacle = 1;
			ROS_INFO("av_obs flag raised");
			break;	
		
		case correctionalfwd:
		ROS_INFO("Action:  State: corr_fwd");
			pwm(700, 700);
			delay(corr_fwd_del);
			pwm(0,0);
			break;
		
		case movleft:
		ROS_INFO("Action:  State: movleft");
			pwm(alignindexsmall,alignindexlarge);
			delay(400);		//right_turn_delay
			pwm(0,0);
			state = initi;
			break;

		default:
		break;
	}	
}
	




	
int main(int argc,char** argv)
{
	wiringPiSetupGpio();
	unsigned  char pwmVal = 0;
 	ros::init(argc, argv, "blink_led");
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

			ROS_INFO("Poling the sensrs outside the call_back");
		obs =0;									// go in with an open mind
		for(count = 1; count<=5; count++)		//check for obstacles
		{
			poll(count);
		}
		ROS_INFO("Polling Output: %s", obs?"obs detected":"Path is clear");
		
		//while(avoiding_obstacle == 1){
			//ROS_INFO("Poling the sensrs while avoiding obstacles");
			//obs =0;									// go in with an open mind
			//for(count = 1; count<=5; count++)		//check for obstacles
			//{
				//poll(count);
			//}
			//ROS_INFO("Polling Output: %s", obs?"obs detected":"Path is clear");
			//Move();
		//}
		
		ros::Subscriber width = n.subscribe("objects",1,centroid_callback);
		//ROS_INFO("Done running the callback from main");	
 		//pwm(-1000,-1000);
		//	delay(right_turn_delay);
		//	pwm(0,0);


	ros::spin();

return 0;
}
