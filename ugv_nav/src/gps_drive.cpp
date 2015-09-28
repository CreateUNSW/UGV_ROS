#include <stdlib.h>
#include <bitset>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/rate.h>
#include <cmath>
#include "shared/shared.hpp"
#include <message_filters/subscriber.h>
#include <sensor_msgs/NavSatFix.h>
#include "ugv_nav/Movement.h"

using namespace std;


/* This program will read values from /heading and /magnitude,
 * convert them to motor driver input and send them to
 * comport /dev/ttyACM0. It does this 100 times per second */

class GPS_Drive {
public:
   GPS_Drive(ros::NodeHandle);
private:
   ros::NodeHandle n;
   ros::Subscriber phone_gps_sub;

   float source_latitude;
   float source_longitude;
   float destination_latitude = -33.916284;
   float destination_longitude = 151.229490;

   void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
};

GPS_Drive::GPS_Drive(ros::NodeHandle n) : n{n} {
   phone_gps_sub = n.subscribe("/phone1/android/fix", 1, &GPS_Drive::gps_callback, this);
}

void GPS_Drive::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  source_latitude = msg->latitude; 
  source_longitude = msg->longitude;

  ROS_INFO_STREAM("Current GPS fix,  lat: " << source_latitude << " long: " << source_longitude);
}

int main(int argc, char** argv) {
   // Initialising the ros node
   ros::init(argc, argv, "gps_drive");
   ros::NodeHandle n;

   // Set up the rate of 100 Hz
   ros::Rate rate(10); // I haven't tested this past 10 Hz, 30 Hz causes it to fail
  
   GPS_Drive driver(n);
   ros::spin();

   return 0;
}
