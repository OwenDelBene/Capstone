#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include "Ekf.h"

using std::vector, std::cout, std::endl;


class HIL {


  double declination = 0;

  public:
    vector<double> true_x;
    vector<double> eul;
    vector<double> eul2;
    vector<double> eule;
    vector<double> cov_vec;
    std::fstream ekf_state;
    std::fstream ekf_eul;
    std::fstream true_state;
    std::fstream true_eul;
    std::fstream noise_eul;
    std::fstream ekf_cov;
    std::fstream sensor;
    std::fstream qerr;

   ekfData ekf;
   
    HIL(ekfData& ekf, const vector<double>&
        init_x = {}) {
      ekf_state.open("ekf_state.out", std::ios_base::out);
      ekf_eul.open("ekf_eul.out", std::ios_base::out);
      true_state.open("true_state.out", std::ios_base::out);
      true_eul.open("true_eul.out", std::ios_base::out);
      noise_eul.open("noise_eul.out", std::ios_base::out);
      ekf_cov.open("ekf_cov.out", std::ios_base::out);
      sensor.open("sensor.out", std::ios_base::out);
      qerr.open("qerr.out", std::ios_base::out);

      this->ekf = ekf;
      if (init_x.size()) {
        this->true_x = init_x;
      }
      else {
        true_x = {1, 0, 0, 0, 0, 0, 0};
      }
    eul.resize(3);
    eul2.resize(3);
    eule.resize(3);
    cov_vec.resize(3);

    } 

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


void propogate(double dt, double y[6], double w[3])
{
    double t = 0;
    rk4(stateTransition, &ekf, true_x.data(), 7, t, dt, true_x.data() );
    vNormalize(true_x.data(), 4, true_x.data());
    
    quat2Eul(true_x.data(), eul.data());
    //add noise
    double r = getRandom(-100, 100) * 2e-3 ;
    v3Set(r, r, r, eul2.data()); 
    v3Add(eul.data(), eul2.data(), eul2.data());  
    double noiseq[4];
    eul2Quat(eul2[0], eul2[1], eul2[2], noiseq);
    
    accelModel(true_x.data(), y);
    magModel(declination, noiseq, y+3);
    w[0] = true_x[4];
    w[1] = true_x[5];
    w[2] = true_x[6];

    double conj[4];
    vector<double> quat_err(4);
    quatConj(true_x.data(), conj);



    writeVector(true_state, true_x);
    writeVector(true_eul, eul);
    writeVector(noise_eul, eul2);
    t += dt;

}


 ~HIL() {
      ekf_state.close();
      ekf_eul.close();
      
      true_eul.close();
      noise_eul.close();
      ekf_cov.close();
      sensor.close();
      qerr.close();
    
 }



};
