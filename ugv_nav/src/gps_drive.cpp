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
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include "ugv_nav/Movement.h"

using namespace std;

#define PHONE_OFFSET_ANGLE 90   // Angle of phone in comparison to robot


/* This program will read values from /heading and /magnitude,
 * convert them to motor driver input and send them to
 * comport /dev/ttyACM0. It does this 100 times per second */

class GPS_Drive {
public:
   GPS_Drive(ros::NodeHandle);
private:
   ros::NodeHandle n;
   ros::Subscriber phone_gps_sub;
   ros::Subscriber phone_mag_sub;
   ros::Subscriber waypoint_sub;
   ros::Subscriber safe_sub;
   ros::Publisher movement_pub;
   ros::Publisher arrived_pub;

   bool safe;

   double source_latitude;
   double source_longitude;
   double distance_to_go;

   double current_heading;
   double desired_heading = 0; // Global heading
   double diff_heading; // Local heading
   // Village green
   // double destination_latitude = -33.918172;
   // double destination_longitude = 151.227975;

   // Middle of College lawn
   //  double destination_latitude = -33.916247;
   // double destination_longitude = 151.228818;

   // Outside MCIC
   // double destination_latitude = -33.916381;
   // double destination_longitude = 151.228768;
   // double destination_latitude = -33.916247;
   // double destination_longitude = 151.228818;

   double destination_latitude;
   double destination_longitude;

   void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void mag_callback(const sensor_msgs::MagneticField::ConstPtr& msg);
   void waypoint_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
   void safe_callback(const std_msgs::Bool::ConstPtr& msg);
};

GPS_Drive::GPS_Drive(ros::NodeHandle n) : n{n} {
   safe = false;
   phone_gps_sub = n.subscribe("/phone1/android/fix", 1, &GPS_Drive::gps_callback, this);
   phone_mag_sub = n.subscribe("/phone1/android/magnetic_field", 1, &GPS_Drive::mag_callback, this);
   waypoint_sub = n.subscribe("/ugv_nav/waypoints", 1, &GPS_Drive::waypoint_callback, this);
   safe_sub = n.subscribe("/ugv_nav/safe", 1, &GPS_Drive::safe_callback, this);
   movement_pub = n.advertise<ugv_nav::Movement>("/ugv_nav/movement", 1);
   arrived_pub = n.advertise<std_msgs::Bool>("/ugv_nav/arrived", 1);
}

void GPS_Drive::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
   source_latitude = msg->latitude;
   source_longitude = msg->longitude;

   printf("Current GPS fix,  lat: %lf, long: %lf\n", source_latitude, source_longitude);
   printf("We want to go to destination,  lat: %lf, long: %lf\n", destination_latitude, destination_longitude);

   double difference_lat = destination_latitude - source_latitude;
   double difference_long =  destination_longitude - source_longitude;

   printf("Difference in lat: %lf, long: %lf\n", difference_lat, difference_long);

   distance_to_go = sqrt(difference_lat*difference_lat+difference_long*difference_long)*111000.0;
   // note: clockwise is positive direction
   desired_heading = atan2(difference_long, difference_lat)*180.0/M_PI;

   printf("Global angle to our destination %lf degrees\n", desired_heading);
   printf("Distance to our destination %lf\n", distance_to_go);


   printf("Our current heading %lf degrees\n", current_heading);
   printf("We need to turn %lf degrees to reach our goal\n", diff_heading);
}

void GPS_Drive::mag_callback(const sensor_msgs::MagneticField::ConstPtr& msg){
   double x_mag = msg->magnetic_field.x;
   double y_mag = msg->magnetic_field.y;
   current_heading = -atan2(x_mag,y_mag)*180.0/M_PI; // negative because phone seems to invert compass, i think
   current_heading -= PHONE_OFFSET_ANGLE;
    if(current_heading<=-180){
	current_heading+=360;
   }	
   diff_heading = desired_heading - current_heading;
   // normalize to range of -180 to +180 with clockwise as positive
   if(diff_heading>180){
	   diff_heading-=360;
   } else if (diff_heading<=-180){
   	diff_heading+=360;
   }

   double diff_heading_rad = diff_heading * M_PI / 180;

   // Publish a movement only if distance is greater than 5m
   // If we are within 5m of our destination, stop driving
   if (distance_to_go > 5) {
      ugv_nav::Movement movement_msg;
      movement_msg.heading = diff_heading_rad/2;
      movement_msg.magnitude = 0.7;
      if (safe) {
         // Publish movement message only if it is safe to move
         movement_pub.publish(movement_msg);
      }
   } else {
      // Publish message to signal that we have arrived at this waypoint
      std_msgs::Bool arrived_at_dest;
      arrived_at_dest.data = true;
      arrived_pub.publish(arrived_at_dest);
   }
}

void GPS_Drive::waypoint_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
   // Update destination
   destination_latitude = msg->latitude;
   destination_longitude = msg->longitude;
}

void GPS_Drive::safe_callback(const std_msgs::Bool::ConstPtr& msg) {
   // Update whether it is safe to drive
   safe = msg->data;
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
