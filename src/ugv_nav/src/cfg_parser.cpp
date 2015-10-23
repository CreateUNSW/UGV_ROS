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

class Cfg_Parser {
public:
   Cfg_Parser(ros::NodeHandle);
   bool hasNextDestination() const;
private:
   ros::NodeHandle n;
   ros::Publisher waypoint_pub;
   ros::Subscriber arrived_sub;
   list<sensor_msgs::NavSatFix> waypoints;
   void parseFile();

   void arrived_callback(const std_msgs::Bool::ConstPtr& msg);
};

Cfg_Parser::Cfg_Parser(ros::NodeHandle n) : n{n} {
   waypoint_pub = n.advertise<sensor_msgs::NavSatFix>("/ugv_nav/waypoints", 100);
   arrived_sub = n.subscribe("/ugv_nav/arrived", 1, &Cfg_Parser::arrived_callback, this);
   parseFile();

   ros::Rate rate(2); 

   while (ros::ok()) {
      // Spin once to get any arrived messages
      ros::spinOnce();
      // If there is a next destination to go to:
      if (hasNextDestination()) {
         // Publish the location to go to!
         waypoint_pub.publish(waypoints.front());
      }
      rate.sleep();
      // TOOD: else: shutdown
   }
}

void Cfg_Parser::parseFile() {
   vector<string> list;
   string word;
   ifstream file("gps_cfg",ifstream::in);

   if (!file) {
      cerr << "Unable to open file" << endl;
      exit(1);
   }

   while(!file.eof()) {
      file >> word;
      // If the word has a digit as 1st or 2nd char
      // then it is a GPS coord
      // We care about the 2nd digit if the number is negative e.g. -1.3928
      if (isdigit(word[0]) || isdigit(word[1])) {
         list.push_back(word);
      }
   }
   
   sensor_msgs::NavSatFix location;
   for (int i = 0; i < list.size(); i+=2) {
      // Create message with list[i] [i+1]
      location.latitude = atof(list[i].c_str());
      location.longitude = atof(list[i+1].c_str());
      // For debug
      printf("%lf %lf\n", location.latitude, location.longitude);

      waypoints.push_back(location);
   }
   file.close();
   printf("End of file\n");
}

bool Cfg_Parser::hasNextDestination() const {
   return !waypoints.empty();
}

void Cfg_Parser::arrived_callback(const std_msgs::Bool::ConstPtr& msg) {
   // Update list
   waypoints.pop_front();
}

int main (int argc, char *argv[]) {
   // Initialising the ros node
   ros::init(argc, argv, "cfg_parser");
   ros::NodeHandle n;

   Cfg_Parser parser(n);
   ROS_INFO("cfg_parser setup successfully");

   return 0;
}
