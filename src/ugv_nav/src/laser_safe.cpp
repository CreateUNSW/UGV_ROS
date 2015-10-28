#include <stdlib.h>
#include <bitset>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/rate.h>
#include <cmath>
#include <string>
#include "shared/shared.hpp"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <vector>
#include <algorithm>

#include "ugv_nav/Movement.h"

#define RANGES 271
#define MIDDLE_RANGE 135
#define MIN_RANGE 60
#define MAX_RANGE 210

#define SAFE_DISTANCE 0.7
#define DETECT_RANGE 3
#define MAX_TURNING_ANGLE 30

typedef enum {TOO_CLOSE,IN_RANGE,CLEAR} obstacle_type;

using namespace std;

class Laser {
public:
   Laser(ros::NodeHandle);
private:
   ros::NodeHandle n;
   ros::Publisher safe_pub;
   ros::Publisher movement_pub;
   ros::Subscriber heading_sub;
   ros::Subscriber laser_sub;
   ros::Subscriber arrived_sub;
   void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
   void heading_callback(const std_msgs::Float32::ConstPtr& msg);
   void arrived_callback(const std_msgs::Bool::ConstPtr& msg);
   obstacle_type inRange(vector<float> ranges) const;
   int getClosestSafePoint(vector<float> ranges) const;
   int getBestHeading(vector<float> ranges, int desired_heading) const;
   int desired_heading; //degrees
   bool continue_driving = false;
};

Laser::Laser(ros::NodeHandle n) : n{n} {
   laser_sub = n.subscribe("/scan", 1, &Laser::laser_callback, this);
   heading_sub = n.subscribe("/ugv_nav/desired_heading", 1, &Laser::heading_callback, this);
   arrived_sub = n.subscribe("/ugv_nav/arrived", 1, &Laser::arrived_callback, this);
   movement_pub = n.advertise<ugv_nav::Movement>("/ugv_nav/movement", 1);

   ros::Rate rate(10);
   ros::spin();
}

void Laser::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
   obstacle_type currentView = inRange(msg->ranges);
   if (currentView!=TOO_CLOSE&&continue_driving) {
      ugv_nav::Movement movement_msg;
      double heading_deg;
      if(currentView==IN_RANGE){
         int desired_heading_transform = MIDDLE_RANGE - desired_heading;
         int safeIndex = getBestHeading(msg->ranges,desired_heading_transform);
	 //printf("Obstacle detected!\n");
         if(safeIndex>=0){
            // Calculate bearing to this safe index
            // Heading in local coordinates
            heading_deg = (MIDDLE_RANGE-safeIndex)/2;
            // -180 < heading_deg <= 180
	   //printf("Obstacle avoided.\n");
         } else {
            // no way around
	    printf("Obstacle unavoidable!\n");
            return;
         }
      } else {
         heading_deg = desired_heading/2;
	 //printf("No obstacles!\n");
      }

      if (heading_deg > 180){
         heading_deg -= 360;
      } else if (heading_deg <= -180){
         heading_deg += 360;
      }
      
      heading_deg = fmin(heading_deg,MAX_TURNING_ANGLE);
      heading_deg = fmax(heading_deg,-MAX_TURNING_ANGLE);
      // Turn to radians
      double heading_rad = heading_deg*M_PI/180;

      // Publish a movement message to swerve around the obstacle

      movement_msg.heading = heading_rad;
      movement_msg.magnitude = 0.7;
      movement_pub.publish(movement_msg);
   } else if(currentView==TOO_CLOSE){
      printf("Too close!\n");
   }
}

void Laser::heading_callback(const std_msgs::Float32::ConstPtr & msg){
   desired_heading = (int)msg->data;
   continue_driving = true;
}

void Laser::arrived_callback(const std_msgs::Bool::ConstPtr& msg){
   continue_driving = false;
}

obstacle_type Laser::inRange(vector<float> ranges) const {
   obstacle_type ret = CLEAR;
   for (int i = MIN_RANGE; i <= MAX_RANGE; ++i) {
      if (ranges[i] != 0.0) {
         if( ranges[i] < SAFE_DISTANCE) {
            return TOO_CLOSE;
         } else if (ranges[i]<DETECT_RANGE) {
             ret = IN_RANGE;
         }
      }
   }
   return ret;
}

int Laser::getBestHeading(vector<float> ranges, int desired_heading_transform) const {
   // note: no value in ranges can be less than SAFE_DISTANCE for this function to work
   int obstacles[RANGES] = {0};
   int best_heading = -1;
   double angle_buffer_rad;
   int angle_buffer_deg = 0;
   // populate obstacles array by padding each obstacle reading on both sides
   // first, sweep one way, right to left
   for (int i = MIN_RANGE; i <= MAX_RANGE; ++i) {
      if (ranges[i] != 0.0 && ranges[i] < DETECT_RANGE) {
         angle_buffer_rad = asin(SAFE_DISTANCE/ranges[i]);
         angle_buffer_deg = fmax(angle_buffer_deg,floor(angle_buffer_rad * 180/M_PI));
      }
      if(angle_buffer_deg>0){
         obstacles[i] = 1;
         angle_buffer_deg--;
      }
   }
   // then, sweep the other way
   angle_buffer_deg = 0;
   for (int i = MAX_RANGE; i>=MIN_RANGE; --i) {
      if (ranges[i] != 0.0 && ranges[i] < DETECT_RANGE) {
         angle_buffer_rad = asin(SAFE_DISTANCE/ranges[i]);
         angle_buffer_deg = fmax(angle_buffer_deg,floor(angle_buffer_rad * 180/M_PI));
      }
      if(angle_buffer_deg>0){
         obstacles[i] = 1;
         angle_buffer_deg--;
      }
      //printf("%d",obstacles[i]);
   }
   //printf("\n");
   //then, get the viewable angle free of obstacles closest to desired heading
   for (int i = MIN_RANGE,abs_diff = 1000; i <= MAX_RANGE; ++i) {
      if (obstacles[i]==0){
         if(fabs(i-desired_heading_transform)<abs_diff){
            best_heading = i;
            abs_diff = fabs(i-desired_heading_transform);
         }
      }
   }
   //printf("best heading = %d\n",best_heading);
   return best_heading;
}

int main (int argc, char *argv[]) {
   // Initialising the ros node
   ros::init(argc, argv, "laser_safe");
   ros::NodeHandle n;

   Laser laser(n);
   ROS_INFO("laser_safe setup successfully");
   return 0;
}
