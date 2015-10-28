#include <cstdlib>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <ros/ros.h>
#include <cmath>
#include <sstream>

#define MY_PORT 9999
#define SERVER_IP "25.55.128.91"

using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "rtk_get");
   ros::NodeHandle n;
   // Message to publish
   sensor_msgs::NavSatFix rtk_gps_msg;
   ros::Publisher rtk_gps_pub = n.advertise<sensor_msgs::NavSatFix>("/ugv_nav/rtk_fix", 1);

   int sockfd, portno, n;
   struct sockaddr_in serv_addr;
   struct hostent *server;

   char buffer[256];
   string buffer2;
   string gps_words[4];

   portno = MY_PORT;
   sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if (sockfd < 0)
      error("ERROR opening socket");
   server = gethostbyname(SERVER_IP);
   if (server == NULL) {
      fprintf(stderr,"ERROR, no such host\n");
      exit(0);
   }
   bzero((char *) &serv_addr, sizeof(serv_addr));
   serv_addr.sin_family = AF_INET;
   bcopy((char *)server->h_addr,
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
   serv_addr.sin_port = htons(portno);
   if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
      error("ERROR connecting");


   int i,j;
   while(ros::ok()){
      bzero(buffer,256);
      n = read(sockfd,buffer,255);
      if (n < 0) {
         //error("ERROR reading from socket");
      } else {
         for(i = 0; i<n; i++){
            buffer2 += buffer[i];
            if(buffer[i] = '\n'){
               stringstream ssin(buffer2);
               //cout << ssin.str() << endl;
               j = 0;
               while (ssin.good() && j < 4){
                  ssin >> gps_words[j];
                  ++j;
               }
               rtk_gps_msg.latitude = atof(gps_words[2].c_str());
               rtk_gps_msg.longitude = atof(gps_words[3].c_str());
               // For debug
               printf("%lf %lf\n", rtk_gps_msg.latitude, rtk_gps_msg.longitude);
               rtk_gps_pub.publish(rtk_gps_msg);
               ros::spinOnce();
               buffer2.clear();
            }
         }
      }
   }
   close(sockfd);
   return (0);
}
