#include "Ekf.h"
#include <iostream>
#include <string>
#include <vector>
#include "Ekf.h"
#include "HIL.h"
#include "string.h"

#include <fstream>

using std::cout, std::endl;
using std::vector;
using std::string;






int main() {
  //EKF parameters
  ekfData ekf;
  vector<double> x(7);
  double P[7][7];
  ekf_init(&ekf, x.data(), P);
  double y[6];
  double w[3];


  vector<double> true_x(7);
  double init_eul[] = {-M_PI/6.0, M_PI/8.0, M_PI/12.0};
  double w0[] = {-M_PI/180.0, 5*M_PI/180.0, 0};
  eul2Quat(init_eul[0], init_eul[1], init_eul[2], true_x.data());
  true_x[4] = w0[0];
  true_x[5] = w0[1];
  true_x[6] = w0[2];
  double declination = 0;
  double t = 0;
  double tf = 60 * .5;
  double dt = .1;
  size_t num_points = (tf-t)/dt;


  HIL hil(ekf, true_x);

  vector<double> eul(3);
  vector<double> eul2(3);

  vector<double> eule(3);

  vector<double> cov_vec(3);

    

  while (t < tf) {

    hil.propogate(dt, y, w); 
    x[4] = w[0];
    x[5] = w[1];
    x[6] = w[2];
    ekf_step(x.data(), P, &ekf, y, dt, x.data(), P);  
    quat2Eul(x.data(), eule.data());
    hil.writeVector(hil.ekf_eul,eule);
    t += dt;



  }
  
  
  

  return 0;
}







