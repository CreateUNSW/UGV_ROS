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
#include <std_msgs/Float32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>

using namespace std;

#define PHONE_OFFSET_ANGLE 90   // Angle of phone in comparison to robot
#define RTK_BUFFER_SIZE 5
#define RTK_TIMEOUT 5

class GPS_Drive {
public:
   GPS_Drive(ros::NodeHandle);
private:
   ros::NodeHandle n;
   ros::Subscriber phone_gps_sub;
   ros::Subscriber phone_mag_sub;
   ros::Subscriber rtk_gps_sub;
   ros::Subscriber waypoint_sub;
   ros::Publisher desired_heading_pub;
   ros::Publisher arrived_pub;

   double source_latitude;
   double source_longitude;
   double distance_to_go;

   double current_heading;
   double desired_heading = 0; // Global heading
   double diff_heading; // Local heading

   double destination_latitude;
   double destination_longitude;

   double start_latitude;
   double start_longitude;

   double start_to_dest_heading;

   bool waypointReceived = false;
   bool fixAttained = false;
   bool rtkFixAttained = false;
   int rtkTimeoutCount = 0;
   int rtkBuffCount = 0;
   double rtkBuffer[RTK_BUFFER_SIZE][2] = {{0},{0}};

   void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
   void rtk_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
   void mag_callback(const sensor_msgs::MagneticField::ConstPtr& msg);
   void waypoint_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

   void update_coordinates(double newLatitude, double newLongitude);
};

GPS_Drive::GPS_Drive(ros::NodeHandle n) : n{n} {
   phone_gps_sub = n.subscribe("/phone1/android/fix", 1, &GPS_Drive::gps_callback, this);
   phone_mag_sub = n.subscribe("/phone1/android/magnetic_field", 1, &GPS_Drive::mag_callback, this);
   rtk_gps_sub = n.subscribe("/ugv_nav/rtk_fix", 1, &GPS_Drive::rtk_gps_callback, this);
   waypoint_sub = n.subscribe("/ugv_nav/waypoints", 1, &GPS_Drive::waypoint_callback, this);
   desired_heading_pub = n.advertise<std_msgs::Float32>("/ugv_nav/desired_heading", 1);
   arrived_pub = n.advertise<std_msgs::Bool>("/ugv_nav/arrived", 1);
}

void GPS_Drive::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
   rtkTimeoutCount++;

   if (rtkTimeoutCount < RTK_TIMEOUT) {
      return;
   } else {
      rtkFixAttained = false;
   }

   fixAttained = true;
   update_coordinates(msg->latitude,msg->longitude);
}

void GPS_Drive::rtk_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
   rtkBuffer[rtkBuffCount][0] = msg->latitude;
   rtkBuffer[rtkBuffCount][1] = msg->longitude;
   rtkBuffCount = (rtkBuffCount + 1) % RTK_BUFFER_SIZE;

   double meanRTK[2] = {0,0};
   int i; double d, difference_lat, difference_long;

   for (i = 0; i < RTK_BUFFER_SIZE; i++) {
      meanRTK[0] += rtkBuffer[i][0] / RTK_BUFFER_SIZE;
      meanRTK[1] += rtkBuffer[i][1] / RTK_BUFFER_SIZE;
   }

   for (i = 0; i < RTK_BUFFER_SIZE; i++) {
      difference_lat = meanRTK[0] - rtkBuffer[i][0];
      difference_long = meanRTK[1] - rtkBuffer[i][1];
      d = sqrt(difference_lat * difference_lat + difference_long * difference_long);
      if (d > 8) {
         return;
      }
   }

   rtkTimeoutCount = 0;
   rtkFixAttained = true;
   update_coordinates(msg->latitude,msg->longitude);
}

void GPS_Drive::update_coordinates(double newLatitude, double newLongitude){
   source_latitude = newLatitude;
   source_longitude = newLongitude;
   fixAttained = true;
   printf("Current GPS fix,  lat: %lf, long: %lf\n", source_latitude, source_longitude);
   printf("We want to go to destination,  lat: %lf, long: %lf\n", destination_latitude, destination_longitude);

   if (!waypointReceived) {
      return;
   }

   double difference_lat = destination_latitude - source_latitude;
   double difference_long =  destination_longitude - source_longitude;

   printf("Difference in lat: %lf, long: %lf\n", difference_lat, difference_long);

   const double precision = 111000.0;
   distance_to_go = sqrt(difference_lat * difference_lat + difference_long * difference_long) * precision;
   if (distance_to_go < 5) {
      std_msgs::Bool arrived_at_dest;
      arrived_at_dest.data = true;
      arrived_pub.publish(arrived_at_dest);
   }

   // note: clockwise is positive direction
   desired_heading = atan2(difference_long, difference_lat)*180.0/M_PI;

   // add bias back to straight line path
   double scale_offset = 2.0;
   desired_heading += scale_offset * distance_to_go * sin((desired_heading-start_to_dest_heading)*M_PI/180);

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

   if (current_heading <= -180) {
	   current_heading += 360;
   }

   if (!(fixAttained || rtkFixAttained) || !waypointReceived) {
      return;
   }

   diff_heading = desired_heading - current_heading;

   // normalize to range of -180 to +180 with clockwise as positive
   if (diff_heading > 180) {
      diff_heading -= 360;
   } else if (diff_heading <= -180){
      diff_heading += 360;
   }

   // Publish a movement only if distance is greater than 5m
   // If we are within 5m of our destination, stop driving
   if (distance_to_go > 5) {
      std_msgs::Float32 new_heading;
      new_heading.data = diff_heading;
      desired_heading_pub.publish(new_heading);
   }
}

void GPS_Drive::waypoint_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
   // Update destination
   if (!fixAttained) {
      // do nothing
      return;
   } else if (!waypointReceived) {
      start_latitude = source_latitude;
      start_longitude = source_longitude;
      waypointReceived = true;
   } else if (destination_latitude != msg->latitude && destination_longitude != msg->longitude) {
      start_latitude = destination_latitude;
      start_longitude = destination_longitude;
   } else {
      return;
   }
   destination_latitude = msg->latitude;
   destination_longitude = msg->longitude;
   double difference_lat = destination_latitude - start_latitude;
   double difference_long =  destination_longitude - start_longitude;
   start_to_dest_heading = atan2(difference_long, difference_lat) * 180.0/M_PI;
}

int main(int argc, char** argv) {
   // Initialising the ros node
   ros::init(argc, argv, "gps_drive");
   ros::NodeHandle n;

   // Set up the rate of 100 Hz
   ros::Rate rate(10); // I haven't tested this past 10 Hz, 30 Hz causes it to fail

   GPS_Drive driver(n);
   ROS_INFO("gps_drive setup successfully");
   ros::spin();

   return 0;
}
