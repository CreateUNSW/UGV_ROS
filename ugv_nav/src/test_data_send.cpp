#include <stdlib.h>
#include <bitset>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/rate.h>

using namespace std;


/* This program will read values from /heading and /magnitude and send
   them to comport /dev/ttyACM0. It does this 100 times per second
   */

int main(int argc, char** argv){
   // Initialising the ros node
   ros::init(argc, argv, "nav_com");
   ros::NodeHandle n;

   // Set up the rate of 100 Hz
   ros::Rate rate(10); // I haven't tested this past 10 Hz, 30 Hz causes it to fail

   // Set up the Com Port
   fstream comPort;
   comPort.open("/dev/ttyACM1");

   // Loop until this node is stopped (using ctrl-c)
   while(ros::ok()){
      float theta;
      float r; 
      double theta_d;
      double r_d; 

      // Get the data
      ros::param::getCached("/heading", theta_d);
      ros::param::getCached("/magnitude", r_d);
      
      // Data is stored as doubles, convert it to floats
      theta = (float) theta_d;
      r = (float) r_d;

      // Now convert floats into bytes
      char* theta_b = (char*) &theta;
      char* r_b= (char*) &r;

      // Write these bytes to the Com Port
      comPort.write(theta_b,4);
      comPort.write(r_b,4);
      comPort.flush();

      //cout << "theta =  " theta << " magnitude = " << r << endl;

      // Sleep until the next iteration of this loop
      ros::spinOnce();
      rate.sleep();
   }

   // All done
   return 0;

}
