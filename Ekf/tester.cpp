#include "Ekf.h"
#include <iostream>
#include <string>
#include <vector>


using std::cout, std::endl;



int main() {
  ekfData ekf;
  double x[7];
  double P[7][7];
  double measure[6];
  double dt = .1;
  ekf_init(&ekf, x, P);
  
  ekf_step(x, P, &ekf, measure, dt, x, P);  

  return 0;
}







