#include <iostream>
#include <string>
#include <sstream>
#include "PCA9685.h"
#include "inverseKinematics.h"

using std::cout, std::endl, std::string;
#define ADDRESS 0x40

int main(int argc, char* argv[])
{

  if (argc <2) {
    cout << "please specify device path" << endl;
  }
  bool angles = false;
  if (argc == 3) {
    angles = true;
    cout << "using inverse kinematics (deg)" << endl;
  }
    double servoAngles[] = {0,0,0};
    PCA9685 pca(argv[1], ADDRESS);
  
  string command;
  while (std::cin >>command) {
    string led, value, value2;
    std::stringstream ss(command);
    
    std::getline(ss, led, ','); 
    std::getline(ss, value, ','); 
    std::getline(ss, value2, ','); 

    if (value.size() ) {
      if (angles) {
        inverseKinematics(std::stoi(value), std::stoi(value2), servoAngles);
        cout << "setting pwm " << anglesToPwm(servoAngles[0]) << " " << anglesToPwm(servoAngles[1]) << endl;
        pca.setPWM(1, anglesToPwm(servoAngles[0]));
        pca.setPWM(2, anglesToPwm(servoAngles[1]));
        pca.setPWM(3, anglesToPwm(servoAngles[2]));
      }else {
        cout<< "setting pwm: " << led << " to: " << value << endl;
        pca.setPWM(std::stoi(led), std::stoi(value));
      }
    }
    else {
      
      cout<< "pwm for led: " << led << " is: " << pca.getPWM(std::stoi(led)) << endl;
    }


  }

}

