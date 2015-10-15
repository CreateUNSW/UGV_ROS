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
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>
#include <vector>
#include <algorithm>


#include "ugv_nav/Movement.h"

// #define MIN_RANGE 60
#define MIN_RANGE 90 
//#define MAX_RANGE 210
#define MAX_RANGE 180
#define SAFE_DISTANCE 1.5
#define SAFETY_OFFSET_DISTANCE 0.5 

using namespace std;

class Laser {
public:
   Laser(ros::NodeHandle);
private:
   ros::NodeHandle n;
   ros::Publisher safe_pub;
   ros::Publisher movement_pub;
   ros::Subscriber laser_sub;
   void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
   bool tooClose(vector<float> ranges) const;
   int getClosestSafePoint(vector<float> ranges) const;
};

Laser::Laser(ros::NodeHandle n) : n{n} {
   laser_sub = n.subscribe("/scan", 1, &Laser::laser_callback, this);
   safe_pub = n.advertise<std_msgs::Bool>("/ugv_nav/safe", 1);
   movement_pub = n.advertise<ugv_nav::Movement>("/ugv_nav/movement", 1);

   ros::Rate rate(10); 
   ros::spin();
}

void Laser::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
   std_msgs::Bool safe;
   if (tooClose(msg->ranges)) {
      safe.data = false;
      safe_pub.publish(safe);
      int safeIndex = getClosestSafePoint(msg->ranges);

      // Calculate bearing to this safe index
      // Heading in local coordinates
      double heading_deg = safeIndex - 135;
      // -180 < heading_deg <= 180
      if (heading_deg > 180){
         heading_deg -= 360;
      } else if (heading_deg <= -180){
         heading_deg += 360;
      }

      // Turn to radians
      double heading_rad = heading_deg*M_PI/180;

      // Publish a movement message to swerve around the obstacle
      ugv_nav::Movement movement_msg;
      // TODO: does this need to be halved?
      movement_msg.heading = heading_rad;
      movement_msg.magnitude = 0.7;
      movement_pub.publish(movement_msg);
   } else {
      safe.data = true;
      safe_pub.publish(safe);
   }
}

bool Laser::tooClose(vector<float> ranges) const {
   for (int i = MIN_RANGE; i <= MAX_RANGE; ++i) {
      if (ranges[i] != 0.0 && ranges[i] < SAFE_DISTANCE) {
         return true;
      }
   }
   return false;
}

int Laser::getClosestSafePoint(vector<float> ranges) const {
   int safeIndex = 0;
   double safetyOffset_rad, safetyOffset_deg;

   // If our obstacle is getting cut out of our laser's range on the left hand side
   // take the closest safe point on its right and drive there.
   if (ranges[MIN_RANGE] != 0.0 && ranges[MIN_RANGE] < SAFE_DISTANCE) {
      for (int i = MIN_RANGE; i <= MAX_RANGE; ++i) {
         if (ranges[i] == 0.0 || ranges[i] > SAFE_DISTANCE) {
            //safeIndex = i + SAFETY_OFFSET;
            safetyOffset_rad = atan(SAFETY_OFFSET_DISTANCE/ranges[i]);
            safetyOffset_deg = safetyOffset_rad * 180/M_PI;
            safeIndex = i + safetyOffset_deg;
            break;
         }
      }
   } else {
      // We take the closest safe point on its left and drive there
      for (int i = MIN_RANGE; i <= MAX_RANGE; ++i) {
         if (ranges[i] != 0.0 && ranges[i] < SAFE_DISTANCE) {
            //safeIndex = i - SAFETY_OFFSET;
            safetyOffset_rad = atan(SAFETY_OFFSET_DISTANCE/ranges[i]);
            safetyOffset_deg = safetyOffset_rad * 180/M_PI;
            safeIndex = i - safetyOffset_deg;
            break;
         }
      }
   }

   return safeIndex;
}

int main (int argc, char *argv[]) {
   // Initialising the ros node
   ros::init(argc, argv, "laser_safe");
   ros::NodeHandle n;

   Laser laser(n);
   ROS_INFO("laser_safe setup successfully");
   return 0;
}
