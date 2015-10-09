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
#include <message_filters/subscriber.h>
#include <vector>
#include <algorithm>

using namespace std;

class Laser {
public:
   Laser(ros::NodeHandle);
private:
   ros::NodeHandle n;
   ros::Subscriber laser_sub;
   void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
   bool tooClose(vector<float> ranges) const;
};

Laser::Laser(ros::NodeHandle n) : n{n} {
   laser_sub = n.subscribe("/scan", 1, &Laser::laser_callback, this);

   ros::Rate rate(10); 
   ros::spin();
}

void Laser::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
   if (tooClose(msg->ranges)) {
      cout << "we're too close!" << endl;
   } else {
      cout << "clear" << endl;
   }
}

bool Laser::tooClose(vector<float> ranges) const {
   for (int i = 60; i <= 210; ++i) {
      if (ranges[i] != 0.0 && ranges[i] < 1.0) {
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
