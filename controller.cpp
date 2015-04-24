#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <cstring>

using namespace std;

int main(void){
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
         cout << num[i];
      }
      cout << endl;
   }
   return (0);

} 

