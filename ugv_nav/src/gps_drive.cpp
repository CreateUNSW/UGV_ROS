#include <stdlib.h>
#include <bitset>
#include <cmath>
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

   double source_latitude;
   double source_longitude;
   // Village green
   double destination_latitude = -33.918172;
   double destination_longitude = 151.227975;
   void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
};

GPS_Drive::GPS_Drive(ros::NodeHandle n) : n{n} {
   phone_gps_sub = n.subscribe("/phone1/android/fix", 1, &GPS_Drive::gps_callback, this);
}

void GPS_Drive::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  source_latitude = msg->latitude; 
  source_longitude = msg->longitude;

  printf("Current GPS fix,  lat: %lf, long: %lf\n", source_latitude, source_longitude);

   double difference_lat = destination_latitude - source_latitude;
   double difference_long =  destination_longitude - source_longitude;
   
   double difference_angle = atan2(difference_long, difference_lat);
   double difference_angle_deg = difference_angle*180.0/M_PI;
   double distance = sqrt(difference_lat*difference_lat+difference_long*difference_long)*111000.0;
   printf("Global angle to our destination %lf", difference_angle_deg);
   printf("Distance to our destination %lf", distance);
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
