#include <iostream>
#include <string>
#include <fstream>

//#include <ros/ros.h>
#include <string.h>
#include <stdio.h>	/* for fprintf */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
using namespace std;

#define PORT 2112
#define BUFSIZE 2048

int main(void){

   int s = socket(AF_INET, SOCK_DGRAM, 0); // UDP connection, https://www.cs.rutgers.edu/~pxk/417/notes/sockets/udp.html

   if (s < 0){
      // socket creation has failed
      cout << "Socket creation failed" << endl;
      return 0;
   }



   struct sockaddr_in myaddr;

   /* bind to an arbitrary return address */
   /* because this is the client side, we don't care about the address */
   /* since no application will initiate communication here - it will */
   /* just send responses */
   /* INADDR_ANY is the IP address and 0 is the socket */
   /* htonl converts a long integer (e.g. address) to a network representation */
   /* htons converts a short integer (e.g. port) to a network representation */

   int myport = PORT; // special port for communication

   memset((char *)&myaddr, 0, sizeof(myaddr));
   myaddr.sin_port = htons(myport);
   myaddr.sin_family = AF_INET;
   myaddr.sin_addr.s_addr = htonl(INADDR_ANY);

   if (bind(s, (const struct sockaddr *)&myaddr, sizeof(myaddr)) < 0){
      // binding has failed
      cout << "Socket binding has failed" << endl;
      return 0;
   }

   /* Receiving the DATA */
   struct sockaddr_in remaddr; // remote address: for sending msg back - not needed
   socklen_t addrlen = sizeof(remaddr); // length of addresses
   int recvlen; // # bytes received
   unsigned char buf[BUFSIZE];

   /* now loop, receiving data and printing what we received */
   for (;;) {
       printf("waiting on port %d\n", myport);
       recvlen = recvfrom(s, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
       printf("received %d bytes\n", recvlen);
       if (recvlen > 0) {
           buf[recvlen] = 0;
           printf("received message: \"%s\"\n", buf);
       }
   }
   /* never exits */

   return 0;
}
