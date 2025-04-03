#pragma once
#include <cmath>
#include <iostream>

using std::cout, std::endl;


void inverseKinematics(double roll, double pitch, double servoAngles[3]);
uint32_t anglesToPwm(double angle);


