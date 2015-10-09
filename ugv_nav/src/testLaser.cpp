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
#include <sensor_msgs/NavSatFix.h>
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
};

Laser::Laser(ros::NodeHandle n) : n{n} {
   laser_sub = n.subscribe("/scan", 1, &Laser::laser_callback, this);

   ros::Rate rate(10); 
   ros::spinOnce();
}

Laser::laser_callback(const std_msgs::Scan::ConstPtr& msg) {

}

int main (int argc, char *argv[]) {
   // Initialising the ros node
   ros::init(argc, argv, "laser");
   ros::NodeHandle n;

   Laser laser(n);
   return 0;
}
