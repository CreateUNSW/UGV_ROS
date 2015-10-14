#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <ros/ros.h>
#include <cmath>
#include <sstream>

#include <sensor_msgs/NavSatFix.h>

using namespace std;

int main(int argc, char** argv){
   // Initialising the ros node
   ros::init(argc, argv, "rtk_get");
   ros::NodeHandle n;

   string line;

   // Message to publish
    sensor_msgs::NavSatFix rtk_gps_msg;
    ros::Publisher rtk_gps_pub = n.advertise<sensor_msgs::NavSatFix>("/ugv_nav/rtk_fix", 1);

   string gps_words[3];
   int i;
   while(ros::ok()){
      std::getline (std::cin,line);
      stringstream ssin(line);
      i = 0;
      while (ssin.good() && i < 3){
        ssin >> gps_words[i];
        ++i;
      }
      rtk_gps_msg.latitude = atof(gps_words[1].c_str());
      rtk_gps_msg.longitude = atof(gps_words[2].c_str());
      // For debug
      printf("%lf %lf\n", rtk_gps_msg.latitude, rtk_gps_msg.longitude);

      rtk_gps_pub.publish(rtk_gps_msg);
      ros::spinOnce();
   }
   return (0);

}

