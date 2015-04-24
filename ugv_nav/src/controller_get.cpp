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

using namespace std;

int main(int argc, char** argv){
   // Initialising the ros node
   ros::init(argc, argv, "con_get");
   ros::NodeHandle n;

   string line;

   while(1){
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
         //cout << num[i];
      }
      //cout << endl;
      // Put this data into /heading and /magnitude
      // X1 and Y1 are the ones we want
      if (num[1] >= 0){
         n.setParam("heading",atan(num[0]/(float)num[1]));
      } else if (num[0] >= 0) {
         n.setParam("heading",3.14159 + atan(num[0]/(float)num[1]));
      } else {
         n.setParam("heading", -3.14159 + atan(num[0]/(float)num[1]));
      }

      n.setParam("magnitude",sqrt(num[0]*num[0] + num[1]*num[1])/32768.0);
      ros::spinOnce();
   }
   return (0);

} 

