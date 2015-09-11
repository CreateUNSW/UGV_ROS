#include <stdlib.h>
#include <bitset>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/rate.h>
#include <cmath>

#define MOTORDRIVER_RANGE_MAX 255
#define MOTORDRIVER_COMBUFFER_LENGTH 11

using namespace std;
int createChecksum (char * message, int len);

/* This program will read values from /heading and /magnitude,
 * convert them to motor driver input and send them to
 * comport /dev/ttyACM0. It does this 100 times per second */

int main(int argc, char** argv){
   // Initialising the ros node
   ros::init(argc, argv, "motornav_com");
   ros::NodeHandle n;

   // Set up the rate of 100 Hz
   ros::Rate rate(10); // I haven't tested this past 10 Hz, 30 Hz causes it to fail

   // Set up the Com Port
   fstream comPort;
   if (argc > 1)
      comPort.open(argv[1]);
   else {
      ROS_INFO("Usage: please give the comport to communicate on as the first argument (i.e. /dev/ttyACM0)");
      return 1;
   }

   // Loop until this node is stopped (using ctrl-c)
   while(ros::ok()){
      float theta;
      float r;
      double theta_d;
      double r_d;

      // Get the data
      ros::param::getCached("/heading", theta_d);
      ros::param::getCached("/magnitude", r_d);
      ROS_INFO_STREAM("theta " << theta_d << " mag " << r_d);

      // Convert to motor coordinates
      //  * Half the range is dedicated to forward motion
      //  * Half the range is dedicated to turning
      int forward = floor(r_d * cos(theta_d) * MOTORDRIVER_RANGE_MAX/2.0);
      int turn = floor(r_d * sin(theta_d) * MOTORDRIVER_RANGE_MAX/2.0);
      int leftMotorVal = forward + ((turn>=0)?turn:-1*turn);
      int rightMotorVal = forward + ((turn>=0)?-1*turn:turn);
      char leftMotorSign = (leftMotorVal >= 0) ? '+' : '-';
      char rightMotorSign = (leftMotorVal >= 0) ? '+' : '-';
      leftMotorVal = abs(leftMotorVal);
      rightMotorVal = abs(rightMotorVal);

      char createsum[MOTORDRIVER_COMBUFFER_LENGTH];
      snprintf(buffer, MOTORDRIVER_COMBUFFER_LENGTH, "[%c%2x%c%2x:]", leftMotorSign, leftMotorVal, rightMotorSign, rightMotorVal);
      int checksum = createChecksum(createsum,MOTORDRIVER_COMBUFFER_LENGTH);

      char buffer[MOTORDRIVER_COMBUFFER_LENGTH];
      snprintf(buffer, MOTORDRIVER_COMBUFFER_LENGTH, "[%c%2x%c%2x:%2x]", leftMotorSign, leftMotorVal, rightMotorSign, rightMotorVal, checksum);

      // Write these bytes to the Com Port
      comPort.write(buffer, MOTORDRIVER_COMBUFFER_LENGTH);
      comPort.flush();

      //cout << "theta =  " theta << " magnitude = " << r << endl;

      // Sleep until the next iteration of this loop
      rate.sleep();
      ros::spinOnce();
   }

   // All done
   return 0;

}

int createChecksum (char * message, int len){
   unsigned char primes[] = {  2,   3,   5,   7,  11,  13,  17,  19,  23,  29,
                           31,  37,  41,  43,  47,  53,  59,  61,  67,  71,
                             73,  79,  83,  89,  97, 101, 103, 107, 109, 113,
                            127, 131, 137, 139, 149, 151, 157, 163, 167, 173 };
  int checksum = 0;
  for (unsigned char i=0; i < len; i++ )
  {
    if ( message[i] == ':' ) break; // Seperates message body from checksum
    checksum += message[i] * primes[i];
  }
  return checksum % 256;
}

/* // This does the background checks in the motor driver for the checksum
bool numberAsign = ( command->text[0] == '-' );
unsigned char numberA  = hexToNum( command->text[1] ) * 16 + hexToNum( command->text[2] );
bool numberBsign = ( command->text[3] == '-' );
unsigned char numberB  = hexToNum( command->text[4] ) * 16 + hexToNum( command->text[5] );
unsigned char claimedChecksum = hexToNum( command->text[7] ) * 16 + hexToNum( command->text[8] );
*/




/* This is the check sum calculator for the motor driver
unsigned char calculateChecksum( char * message, int len)
{
  unsigned char primes[] = {  2,   3,   5,   7,  11,  13,  17,  19,  23,  29,
                             31,  37,  41,  43,  47,  53,  59,  61,  67,  71,
                             73,  79,  83,  89,  97, 101, 103, 107, 109, 113,
                            127, 131, 137, 139, 149, 151, 157, 163, 167, 173 };
  int checksum = 0;
  for (unsigned char i=0; i < len; i++ )
  {
    if ( message[i] == ':' ) break; // Seperates message body from checksum
    checksum += message[i] * primes[i];
  }
  return checksum % 256;
}
*/
