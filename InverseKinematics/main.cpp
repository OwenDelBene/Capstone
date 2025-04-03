#include <iostream>

#include "inverseKinematics.h"
using namespace std;


int main() {

  double servoAngles[3];
  inverseKinematics(-2, 7, servoAngles);

  cout << "inverse kinematics for -2, 7 " << servoAngles[0] << " , " << servoAngles[1] << " , " << servoAngles[2] << endl;
}
