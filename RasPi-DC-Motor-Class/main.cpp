#include <cstdlib>
#include <stdio.h>
#include "PiMotor.h"

#include <iostream>
#include <sstream>
#include <string>
using namespace std;

//int m1_A = 20; //Motor 1 Forward
//int m1_B = 21; //Motor 1 Reverse

int main(int argc, char** argv) {
    //Create a new instance for our Motor.

    if (argc < 3) {
      cout << "specify forward and reverse pins" << endl;
      exit(0);
    }
    int m1_A = std::stoi(argv[1]);
    int m1_B = std::stoi(argv[2]);

    PiMotor motor1(m1_A, m1_B);
    
    motor1.setDebug(true); //Turn on Debug message output (see console window!)
    if (argc < 4) {
    motor1.run(0, 255); //Set PWM value for direction (0 = reverse, 1 = forwards)
    motor1.stop(); //Stop the motor  
    motor1.runForMS(0, 100, 4000); //Run for 4 seconds.
    }
    string command;
    while (std::cin  >> command) {
      int speed = std::stoi(command);
      if (speed ==0) motor1.stop();
      if (speed>0) motor1.run(1, speed);
      if (speed<0) motor1.run(0, speed);

    }
    return 0;
}

