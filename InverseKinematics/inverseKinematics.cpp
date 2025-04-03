#include "inverseKinematics.h"
#include "linearAlgebra.h"


void L_bai(double rpp[3], double rop[3][3], double ra[3], double rb[3], double res[3])
{

  mMultV(rop,3,3, ra, res);
  v3Add(rpp, res, res);
  v3Subtract(res, rb, res);

}

double getAlpha(double lbc, double lca, double lbai[3])
{
  double n = v3Norm(lbai);
    return std::acos( (lbc*lbc + n*n - lca*lca)  / (2 * lbc * n));
}

double getBeta(double lbai[3], double yp[3]) {

  double d = v3Dot(lbai, yp);
  double n = v3Norm(lbai);
  return std::acos(d/n);

}

void inverseKinematics(double roll, double pitch, double servoAngles[3]) {
  
  double deg2rad = M_PI/180.0f;
  roll = roll * deg2rad;
  pitch = pitch * deg2rad;
  double R1[3][3], R2[3][3], R3[3][3]; 
  
  oneRot(R1, -roll);
  twoRot(R2, -pitch);
  mSetIdentity(R3, 3, 3);
  
  double Rop[3][3];
  mMultM(R3,3,3, R2,3,3, Rop);
  mMultM(Rop,3,3, R1,3,3, Rop);
  double rp[3] = {0, 0, 112.28};


  double ra_nY[3], ra_pY[3], ra_X[3];
  double RA[3][3];
  v3Set(294.74, 0, 0, ra_X);

  threeRot(RA, -4 * M_PI/3.0f);
  mMultV(RA, 3, 3, ra_X, ra_nY);

  threeRot(RA, -2 * M_PI/3.0f);
  mMultV(RA, 3, 3, ra_X, ra_pY);

  double rb_nY[3], rb_pY[3], rb_X[3];

  v3Set(224.4, 0, 0, rb_X);

  threeRot(RA, -4 * M_PI/3.0f);
  mMultV(RA, 3, 3, rb_X, rb_nY);

  threeRot(RA, -2 * M_PI/3.0f);
  mMultV(RA, 3, 3, rb_X, rb_pY);


  //link vectors
  double L_banY[3], L_bapY[3], L_baX[3];
  L_bai(rp, Rop, ra_nY, rb_nY, L_banY);
  L_bai(rp, Rop, ra_pY, rb_pY, L_bapY);
  L_bai(rp, Rop, ra_X, rb_X, L_baX);
  double L_bc = 48.26;
  double L_ca = 114.43;

  //Yaw-pitch trans
  double yp_bnY[3], yp_bpY[3], yp_bX[3];
  double nx[3];
  v3Set( -1, 0, 0, nx);
  threeRot(RA, -4*M_PI/3);
  mMultV(RA, 3, 3, nx,  yp_bnY);

  threeRot(RA, -2*M_PI/3);
  mMultV(RA,3,3, nx,  yp_bpY);

  v3Set(-1, 0, 0, yp_bX);

  //angle calculations

  double alpha_ny = getAlpha(L_bc, L_ca, L_banY);
  double alpha_py = getAlpha(L_bc, L_ca, L_bapY);
  double alpha_X  = getAlpha(L_bc, L_ca, L_baX);

  double beta_ny = getBeta(L_banY, yp_bnY);
  double beta_py = getBeta(L_bapY, yp_bpY);
  double beta_X  = getBeta(L_baX, yp_bX);

  double rad2deg = 180.0f /M_PI;
  servoAngles[0]= 90 - (alpha_ny + beta_ny - M_PI/2) * rad2deg; 
  servoAngles[1] =90 - (alpha_py + beta_py - M_PI/2) * rad2deg;
  servoAngles[2] =90 -  (alpha_X + beta_X - M_PI/2) * rad2deg;

}




int main() {

  double servoAngles[3];
  inverseKinematics(-2, 7, servoAngles);

  cout << "inverse kinematics for -2, 7 " << servoAngles[0] << " , " << servoAngles[1] << " , " << servoAngles[2] << endl;
}
