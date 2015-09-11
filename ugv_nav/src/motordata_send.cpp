#include <stdlib.h>
#include <bitset>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/rate.h>
#include <cmath>
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
   fstream comPort;
   if (argc > 1)
      comPort.open(argv[1]);
   else {
      ROS_INFO("Usage: please give the comport to communicate on as the first argument (i.e. /dev/ttyACM0)");
      return 1;
   }

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
      
      // Convert to motor coordinates
      //  * Half the range is dedicated to forward motion
      //  * Half the range is dedicated to turning
      int forward = floor(r_d * cos(theta_d) * MOTORDRIVER_RANGE_MAX/2.0);
      int turn = floor(r_d * sin(theta_d) * MOTORDRIVER_RANGE_MAX/2.0);
      int leftMotorVal = forward + ((turn>=0)?turn:-1*turn);
      int rightMotorVal = forward + ((turn>=0)?-1*turn:turn);
      char leftMotorSign = (leftMotorVal >= 0) ? '+' : '-';
      char rightMotorSign = (leftMotorVal >= 0) ? '+' : '-';
      leftMotorVal = abs(leftMotorVal);
      rightMotorVal = abs(rightMotorVal);

      char buffer[MOTORDRIVER_COMBUFFER_LENGTH];
      snprintf(buffer, MOTORDRIVER_COMBUFFER_LENGTH, "[%c%2x%c%2x:00]", leftMotorSign, leftMotorVal, rightMotorSign, rightMotorVal);
      char checksum = calculateChecksum(&buffer[1], MOTORDRIVER_COMBUFFER_MSGLENGTH);
      snprintf(&buffer[(MOTORDRIVER_COMBUFFER_LENGTH - MOTORDRIVER_COMBUFFER_CHKSUMDISTFROMEND)], MOTORDRIVER_COMBUFFER_CHKSUMLENGTH, "%2x", checksum);

      // Write these bytes to the Com Port
      comPort.write(buffer, MOTORDRIVER_COMBUFFER_LENGTH);
      comPort.flush();

      //cout << "theta =  " theta << " magnitude = " << r << endl;

      // Sleep until the next iteration of this loop
      rate.sleep();
      ros::spinOnce();
   }

   // All done
   return 0;

}
