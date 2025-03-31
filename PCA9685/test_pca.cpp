#include <iostream>
#include <string>
#include <sstream>
#include "PCA9685.h"

using std::cout, std::endl, std::string;
#define ADDRESS 0x40

int main(int argc, char* argv[])
{

  if (argc <2) {
    cout << "please specify device path" << endl;
  }
  PCA9685 pca(argv[1], ADDRESS);

  string command;
  while (std::cin >>command) {
    string led, value;
    std::stringstream ss(command);
    
    std::getline(ss, led, ','); 
    std::getline(ss, value, ','); 

    if (value.size() ) {
      cout<< "setting pwm: " << led << " to: " << value << endl;
      pca.setPWM(std::stoi(led), std::stoi(value));
    }
    else {
      
      cout<< "pwm for led: " << led << " is: " << pca.getPWM(std::stoi(led)) << endl;
    }


  }

}
