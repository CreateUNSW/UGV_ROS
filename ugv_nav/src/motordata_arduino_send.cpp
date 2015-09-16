#include <stdlib.h>
#include <bitset>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/rate.h>
#include <cmath>
#include <cstlib>
#include "shared/shared.hpp"

#define MOTORDRIVER_RANGE_MAX 255
#define MOTORDRIVER_COMBUFFER_MSGLENGTH 6
#define MOTORDRIVER_COMBUFFER_CHKSUMDISTFROMEND 3
#define MOTORDRIVER_COMBUFFER_CHKSUMLENGTH 2
#define MOTORDRIVER_COMBUFFER_LENGTH 11

using namespace std;


/* This program will read values from /heading and /magnitude,
 * convert them to motor driver input and send them to
 * comport /dev/ttyACM0. It does this 100 times per second */

int main(int argc, char** argv){
   // Initialising the ros node
   ros::init(argc, argv, "motornav_com");
   ros::NodeHandle n;

   // Set up the rate of 100 Hz
   ros::Rate rate(10); // I haven't tested this past 10 Hz, 30 Hz causes it to fail

   // Set up the Com Port
   /*fstream comPort;
   if (argc > 1)
      comPort.open(argv[1]);
   else {
      ROS_INFO("Usage: please give the comport to communicate on as the first argument (i.e. /dev/ttyACM0)");
      return 1;
   }*/
   system("stty -F /dev/ttyACM0 cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts"); //Activates the tty connection with the Arduino
   ofstream comPort("/dev/ttyACM0");
   long int t = time(NULL);
   while (time(NULL)-t < 5) {};

//	comPort.write("[hales]", 7);
      comPort.flush();

   // Loop until this node is stopped (using ctrl-c)
   while(ros::ok()){
      float theta;
      float r; 
      double theta_d;
      double r_d; 

      // Get the data
      ros::param::getCached("/heading", theta_d);
      ros::param::getCached("/magnitude", r_d);
      ROS_INFO_STREAM("theta " << theta_d << " mag " << r_d);
      
      int leftMotorVal, rightMotorVal;
      char leftMotorSign, rightMotorSign;
	// new driving maths coming from old UGV_sketch
	
	if(theta_d<=M_PI/4&&theta_d>=-3*M_PI/4){
		rightMotorSign = '+';
	} else {
		rightMotorSign = '-';
	}
	if(theta_d<=3*M_PI/4&&theta_d>=-M_PI/4){
		leftMotorSign = '+';
	} else {
		leftMotorSign = '-';
	}
	if(theta_d>=0){
		leftMotorVal = fmin(M_PI/4, fabs(3*M_PI/4-theta_d))*255/(M_PI/4);
		rightMotorVal = fmin(M_PI/4, fabs(M_PI/4-theta_d))*255/(M_PI/4);
	} else {
		leftMotorVal = fmin(M_PI/4, fabs(M_PI/4+theta_d))*255/(M_PI/4);
		rightMotorVal = fmin(M_PI/4, fabs(3*M_PI/4+theta_d))*255/(M_PI/4);
	}
	leftMotorVal = fmin(leftMotorVal*r_d,255);
	rightMotorVal = fmin(rightMotorVal*r_d,255);

      char buffer[MOTORDRIVER_COMBUFFER_LENGTH+1];
      char message[MOTORDRIVER_COMBUFFER_MSGLENGTH+1];
      snprintf(message, MOTORDRIVER_COMBUFFER_MSGLENGTH+1, "%c%2.2X%c%2.2X", leftMotorSign, leftMotorVal, rightMotorSign, rightMotorVal);
      unsigned char checksum = calculateChecksum(message, MOTORDRIVER_COMBUFFER_MSGLENGTH);
      snprintf(buffer,MOTORDRIVER_COMBUFFER_LENGTH+1,"[%s:%2.2X]",message,checksum);
      printf("%.*s\n", MOTORDRIVER_COMBUFFER_LENGTH, buffer);

      // Write these bytes to the Com Port
	if(r_d>0.15){
		comPort.write(buffer, MOTORDRIVER_COMBUFFER_LENGTH);
		comPort.flush();
	}

      //cout << "theta =  " theta << " magnitude = " << r << endl;

      // Sleep until the next iteration of this loop
      rate.sleep();
      ros::spinOnce();
   }

   // All done
   return 0;

}
