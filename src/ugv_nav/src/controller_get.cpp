#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <ros/ros.h>
#include <cmath>
//#include <ctgmath>
#include <ctime>
#include <std_msgs/Float32.h>

#include "ugv_nav/Movement.h"

#define KILL_TIMER 0.0

using namespace std;

int main(int argc, char** argv){
   // Initialising the ros node
   ros::init(argc, argv, "con_get");
   ros::NodeHandle n;

   time_t startTime;
   time(&startTime);
   time_t curTime;

   string line;

   // Message to publish

   // FOR DIRECT TO MOTORS
    ugv_nav::Movement movement_msg;
   
    // FOR VIA LASER SCANNER
    //struct movement_MSG {
	//	 std_msgs::Float32 heading;
	//	 std_msgs::Float32 magnitude;
    //} movement_msg;

    // FOR DIRECT TO MOTORS
    ros::Publisher movement_publisher = n.advertise<ugv_nav::Movement>("/ugv_nav/movement", 1);
    
    // FOR VIA LASER SCANNER
    //ros::Publisher desired_heading_pub = n.advertise<std_msgs::Float32>("/ugv_nav/desired_heading", 1);
    //ros::Publisher desired_speed_pub = n.advertise<std_msgs::Float32>("/ugv_nav/desired_speed", 1);

   while(ros::ok()){
      time(&curTime);
      if (difftime(curTime, startTime) > KILL_TIMER && KILL_TIMER > 0.0) {
          cerr << "Your " << KILL_TIMER << " seconds are up. Killing node!" << endl;
          cerr << "Change KILL_TIMER to 0.0 to disable this behaviour" << endl;
          return (1);
      }
      int num[21];
      cin.ignore(100,':');
      for (int i = 0; i < 21; i++){
         string str;
         if (i != 20)
            getline(cin,str,':');
         else
            getline(cin,str);
         stringstream ss;
         ss.str (str);
         ss >> num[i];
         cout << num[i];
      }
      //cout << endl;
      // Put this data into /heading and /magnitude
      // X1 and Y1 are the ones we want
      if (num[1] >= 0){
         //movement_msg.heading.data = atan(num[0]/(float)num[1]);
         
	 // FOR DIRECT TO MOTORS
	 movement_msg.heading = atan(num[0]/(float)num[1]);
      } else if (num[0] >= 0) {
         // FOR VIA LASER SCANNER
         //movement_msg.heading.data = 3.14159 + atan(num[0]/(float)num[1]);
         
         // FOR DIRECT TO MOTORS
         movement_msg.heading = 3.14159 + atan(num[0]/(float)num[1]);
      } else {

	 // FOR DIRECT TO MOTORS
         movement_msg.heading = -3.14159 + atan(num[0]/(float)num[1]);
         
	 // FOR VIA LASER SCANNER
	 //movement_msg.heading.data = -3.14159 + atan(num[0]/(float)num[1]);
      }

      //movement_msg.magnitude.data = sqrt(num[0]*num[0] + num[1]*num[1])/32768.0;
	   movement_msg.magnitude = sqrt(num[0]*num[0] + num[1]*num[1])/32768.0;
           //std::cout << movement_msg.magnitude << std::endl;
      // THE FOLLOWING LINE OF CODE IS TO SLOW UGV FOR ENG NIGHT
      movement_msg.magnitude*=0.7 ;

      //n.setParam("magnitude",sqrt(num[0]*num[0] + num[1]*num[1])/32768.0);
	//	movement_msg.heading.data *= 180/M_PI;
      
      // FOR DIRECT TO MOTORS
      movement_publisher.publish(movement_msg);

      // FOR VIA LASER SCANNER
      //desired_heading_pub.publish(movement_msg.heading);
      //desired_speed_pub.publish(movement_msg.magnitude);
      ros::spinOnce();
   }
   return (0);

}

