#include <stdlib.h>
#include <bitset>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/rate.h>
#include <cmath>
#include "shared/shared.hpp"
#include <message_filters/subscriber.h>
#include "ugv_nav/Movement.h"

#define MOTORDRIVER_RANGE_MAX 255
#define MOTORDRIVER_COMBUFFER_MSGLENGTH 6
#define MOTORDRIVER_COMBUFFER_CHKSUMDISTFROMEND 3
#define MOTORDRIVER_COMBUFFER_CHKSUMLENGTH 2
#define MOTORDRIVER_COMBUFFER_LENGTH 11

using namespace std;


/* This program will read values from /heading and /magnitude,
 * convert them to motor driver input and send them to
 * comport /dev/ttyACM0. It does this 100 times per second */

class Motornav_Com {
public:
    Motornav_Com(ros::NodeHandle n);
    void sendMovement();
private:
    ros::NodeHandle n;
    ros::Subscriber movement_sub;

    ofstream comPort;

    float theta;
    float r;
    double theta_d = 0;
    double r_d = 0;

    int leftMotorVal, rightMotorVal;
    char leftMotorSign, rightMotorSign;

    void movement_callback(const ugv_nav::Movement::ConstPtr& msg);
};

//Motornav_Com::Motornav_Com(ros::NodeHandle n, char** argv) : n{n} {
Motornav_Com::Motornav_Com(ros::NodeHandle n) : n{n} {

   movement_sub = n.subscribe("ugv_nav/movement", 1, &Motornav_Com::movement_callback, this);

   char setupString[200];
   strcpy(setupString, "stty -F ");
   //strcat(setupString, argv[1]);
   strcat(setupString, "/dev/ttyACM0");
   strcat(setupString, " cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts");
   system(setupString); //Activates the tty connection with the Arduino
   //comPort.open(argv[1]);
   comPort.open("/dev/ttyACM0");
   long int t = time(NULL);
   while (time(NULL)-t < 5) {};

   //	comPort.write("[hales]", 7);
   comPort.flush();
}

void Motornav_Com::movement_callback(const ugv_nav::Movement::ConstPtr& msg) {
   // Get the data
   theta_d = msg->heading;
   r_d = msg->magnitude;
	r_d = fmin(r_d,1);
	r_d *= r_d;

   // Send movement to bot
   sendMovement();
}

void Motornav_Com::sendMovement() {

	if(r_d<0.1)
        return;
    // new driving maths coming from old UGV_sketch
    if(theta_d<=M_PI_4&&theta_d>=-3*M_PI_4){
        rightMotorSign = '+';
    } else {
        rightMotorSign = '-';
    }
    if(theta_d<=3*M_PI_4&&theta_d>=-M_PI_4){
        leftMotorSign = '+';
    } else {
        leftMotorSign = '-';
    }
    leftMotorVal = fmin(M_PI_4,fmin(fabs(theta_d+M_PI_4), fabs(theta_d-3*M_PI_4)))*255/M_PI_4;
    rightMotorVal = fmin(M_PI_4,fmin(fabs(theta_d+3*M_PI_4), fabs(theta_d-M_PI_4)))*255/M_PI_4;

    leftMotorVal = fmin(leftMotorVal*r_d,255);
    rightMotorVal = fmin(rightMotorVal*r_d,255);

    char buffer[MOTORDRIVER_COMBUFFER_LENGTH+1];
    char message[MOTORDRIVER_COMBUFFER_MSGLENGTH+1];
    snprintf(message, MOTORDRIVER_COMBUFFER_MSGLENGTH+1, "%c%2.2X%c%2.2X", leftMotorSign, leftMotorVal, rightMotorSign, rightMotorVal);
    unsigned char checksum = calculateChecksum(message, MOTORDRIVER_COMBUFFER_MSGLENGTH);
    snprintf(buffer,MOTORDRIVER_COMBUFFER_LENGTH+1,"[%s:%2.2X]",message,checksum);
    printf("%.*s\n", MOTORDRIVER_COMBUFFER_LENGTH, buffer);

  // Write these bytes to the Com Port

    //ROS_INFO_STREAM("theta " << theta_d << " mag " << r_d);
    comPort.write(buffer, MOTORDRIVER_COMBUFFER_LENGTH);
    comPort.flush();
}

int main(int argc, char** argv){
   // Initialising the ros node
   ros::init(argc, argv, "motornav_com");
   ros::NodeHandle n;

   // Set up the rate of 100 Hz
   ros::Rate rate(10); // I haven't tested this past 10 Hz, 30 Hz causes it to fail

   /*
   if (argc <= 1) {
      ROS_INFO("Usage: please give the comport to communicate on as the first argument (i.e. /dev/ttyACM0)");
      return 1;
   }
   */

   Motornav_Com mnc(n);
   //Motornav_Com mnc(n, argv);
   ROS_INFO("motornav_com setup successfully");
   /*
   while (ros::ok()) {
      mnc.sendMovement();
      ros::spinOnce();
   }
   */
   ros::spin();
   return 0;
}
