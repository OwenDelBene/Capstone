#include "Ekf.h"
#include <iostream>
#include <string>
#include <vector>
#include "Ekf.h"
#include "string.h"

#include <fstream>

using std::cout, std::endl;
using std::vector;
using std::string;



void writeVector(std::fstream& f, vector<double>& vec)
{
    for (const auto& x: vec) {
      f << std::to_string(x) << ",";
    }
    f << '\n';
}
void printVector( vector<double>vec)
{
    for (const auto& x: vec) {
      cout << std::to_string(x) << ",";
    }
    cout << '\n';
}


double getRandom(int low, int high)
{
  return low + ( std::rand() % ( high - low + 1 ) );
}

int main() {
  ekfData ekf;
  vector<double> x(7);
  double P[7][7];
  double measure[6];
 
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


  std::fstream ekf_state("ekf_state.out", std::ios_base::out);
  std::fstream ekf_eul("ekf_eul.out", std::ios_base::out);
  std::fstream true_state("true_state.out", std::ios_base::out);
  std::fstream true_eul("true_eul.out", std::ios_base::out);
  std::fstream noise_eul("noise_eul.out", std::ios_base::out);
  std::fstream ekf_cov("ekf_cov.out", std::ios_base::out);
  std::fstream sensor("sensor.out", std::ios_base::out);
  std::fstream qerr("qerr.out", std::ios_base::out);


  vector<double> eul(3);
  vector<double> eul2(3);

  vector<double> eule(3);

  vector<double> cov_vec(3);

  ekf_init(&ekf, x.data(), P);
  size_t i=0;
  for (auto j: true_x) {

    //x[i++] = j;
  }  

  while (t < tf) {
    
    writeVector(true_state, true_x);

    rk4(stateTransition, &ekf, true_x.data(), 7, t, dt, true_x.data() );
    vNormalize(true_x.data(), 4, true_x.data());
    
    quat2Eul(true_x.data(), eul.data());
    //add noise
    double r = getRandom(-100, 100) * 2e-3 ;
    v3Set(r, r, r, eul2.data()); 
    v3Add(eul.data(), eul2.data(), eul2.data());  
    double noiseq[4];
    eul2Quat(eul2[0], eul2[1], eul2[2], noiseq);
    
    double y[6];
    accelModel(true_x.data(), y);
    magModel(declination, noiseq, y+3);
    x[4] = true_x[4];
    x[5] = true_x[5];
    x[6] = true_x[6];

    ekf_step(x.data(), P, &ekf, y, dt, x.data(), P);  
    quat2Eul(x.data(), eule.data());
  
    cov_vec[0] = P[0][0];
    cov_vec[1] = P[1][1];
    cov_vec[2] = P[2][2];

    double conj[4];
    vector<double> quat_err(4);
    quatConj(true_x.data(), conj);
    quatMultiply(conj, x.data(), quat_err.data());

    writeVector(qerr, quat_err);


    writeVector(ekf_state, x);
    writeVector(true_eul, eul);
    writeVector(noise_eul, eul2);
    writeVector(ekf_eul,eule);
    writeVector(ekf_cov, cov_vec);
    vector<double> vec(y, y+6);
    writeVector(sensor, vec);
    t += dt;
    i++;
  }
  
  
  

  return 0;
}







