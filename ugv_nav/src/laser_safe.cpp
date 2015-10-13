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

using namespace std;

class Laser {
public:
   Laser(ros::NodeHandle);
private:
   ros::NodeHandle n;
   ros::Publisher safe_pub;
   ros::Subscriber laser_sub;
   void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
   bool tooClose(vector<float> ranges) const;
};

Laser::Laser(ros::NodeHandle n) : n{n} {
   laser_sub = n.subscribe("/scan", 1, &Laser::laser_callback, this);
   safe_pub = n.advertise<std_msgs::Bool>("/ugv_nav/safe", 1);

   ros::Rate rate(10); 
   ros::spin();
}

void Laser::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
   std_msgs::Bool safe;
   if (tooClose(msg->ranges)) {
      safe.data = false;
   } else {
      safe.data = true;
   }
   safe_pub.publish(safe);
}

bool Laser::tooClose(vector<float> ranges) const {
   int min_range = 60;
   int max_range = 210;
   float safe_distance = 1.0;
   for (int i = min_range; i <= max_range; ++i) {
      if (ranges[i] != 0.0 && ranges[i] < safe_distance) {
         return true;
      }
   }
   return false;
}

int main (int argc, char *argv[]) {
   // Initialising the ros node
   ros::init(argc, argv, "laser");
   ros::NodeHandle n;

   Laser laser(n);
   return 0;
}
