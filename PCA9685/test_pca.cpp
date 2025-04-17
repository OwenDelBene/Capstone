#include <iostream>
#include <string>
#include <sstream>
#include "PCA9685.h"
#include "inverseKinematics.h"

#include <thread>
#include <chrono>
#include "driveConfig.h"


void pcaDrive(PCA9685* pca, uint32_t L1, uint32_t L2, uint32_t R1, uint32_t R2, uint32_t ms)
{
  pca->setPWM(DC_L1, L1);
  pca->setPWM(DC_L2, L2);
  pca->setPWM(DC_R1, R1);
  pca->setPWM(DC_R2, R2);
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  pca->setPWM(DC_L1, 0);
  pca->setPWM(DC_L2, 0);
  pca->setPWM(DC_R1, 0);
  pca->setPWM(DC_R2, 0);

}

using std::cout, std::endl, std::string;
#define ADDRESS 0x40

int main(int argc, char* argv[])
{

  if (argc <2) {
    cout << "please specify device path" << endl;
  }
  bool angles = false;
  bool drive = false;
  if (argc == 3) {
    angles = argv[2]=="a";
    drive = argv[2]=="d";
    cout << "using inverse kinematics (deg)" << endl;
  }
    double servoAngles[] = {0,0,0};
    PCA9685 pca(argv[1], ADDRESS);
  
  string command;
  while (std::cin >>command) {
    string led, value, value2, value3, value4, time;
    std::stringstream ss(command);
    
    std::getline(ss, led, ','); 
    std::getline(ss, value, ','); 
    std::getline(ss, value2, ','); 
    std::getline(ss, value3, ','); 
    std::getline(ss, value4, ','); 
    std::getline(ss, time, ','); 

    if (value.size() ) {
      if (drive) {
        cout << "driving" << endl;
        pcaDrive(&pca, std::stoi(value), std::stoi(value2), std::stoi(value3), std::stoi(value4), std::stoi(time));

      }
      
      else if (angles) {
        inverseKinematics(std::stoi(value), std::stoi(value2), servoAngles);
        cout << "setting pwm " << anglesToPwm(servoAngles[0]) << " " << anglesToPwm(servoAngles[1]) << " " << anglesToPwm(servoAngles[2]) << endl;
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

