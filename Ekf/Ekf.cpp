#include "linearAlgebra.h"
#include "Ekf.h"
#include "cmath"
void accelModel(double x[4], double dest[3])
{
  dest[0] = -2 * (x[1]*x[3]-x[0]*x[2]);
  dest[1] = -2 * (x[0]*x[1]+x[2]*x[3]);
  dest[2] = -(1-2)*(x[1]*x[1] + x[2]*x[2]);

}

void magModel(double declination, double x[4], double dest[3])
{

  double s = sin(declination);
  double c = cos(declination);

  v3Set(s * (2*x[0]*x[3] + 2*x[1]*x[2]) - c*(2*x[2]*x[2] + x[3]*x[3] -1),
        -c*(2*x[0]*x[3] -2*x[1]*x[2] - s*(2*x[1]*x[1] + 2*x[3]*x[3]-1)),
       c*(2*x[0]*x[2]+2*x[1]*x[3]) - s*(2*x[0]*x[1] -2*x[2]*x[3]) ,dest);

}



static void stateTransition(ekfData* info, double* x, size_t n, double t, double* res)
{
  double qdot[4];
  double H[3];
  double cx[3];
  double pqrdot[3];

  quatDerivative(x, x+4, res);

  m33MultV3(info->MOI, x+4, H);

  v3Cross(x+4, H, cx);
  //subtract Tctrl here if using above imu
  v3Scale(-1, cx, cx);

  m33MultV3(info->MOI, cx, res + 4);
}




void dwdotdw(double w[3], ekfData* ekf, double dw[][3])
{
  double J1 = ekf->MOI[0][0];
  double J2 = ekf->MOI[1][1];
  double J3 = ekf->MOI[2][2];

  m33Set(0, w[2]*(J2-J3)/J1, w[1]*(J2-J3)/J1,
         w[2]*(J3-J1)/J2, 0, w[0]*(J3-J1)/J2,
         w[1]*(J1-J2)/J3, w[0]*(J1-J2)/J3, 0, dw);

}


void OmegaQ(double q[4], double dest[4][3])
{
  dest[0][0] = -q[1];
  dest[0][1] = -q[2];
  dest[0][2] = -q[3];

  dest[1][0] =  q[1];
  dest[1][1] = -q[3];
  dest[1][2] =  q[2];
   
  dest[2][0] =  q[3];
  dest[2][1] =  q[0];
  dest[2][2] = -q[1];

  dest[3][0] = -q[2];
  dest[3][1] =  q[1];
  dest[3][2] =  q[0];
}

void OmegaW(double w[3], double dest[4][4])
{

  dest[0][0] = 0;
  dest[0][1] = -w[0];
  dest[0][2] = -w[1];
  dest[0][3] = -w[2];

  dest[1][0] =  w[0];
  dest[1][1] =  0;
  dest[1][2] =  w[2];
  dest[1][3] = -w[1];

  dest[2][0] =  w[1];
  dest[2][1] = -w[3];
  dest[2][2] =  0;
  dest[2][3] =  w[0];
  
  dest[3][0] =  w[2];
  dest[3][1] =  w[1];
  dest[3][2] = -w[0];
  dest[3][3] =  0;


}


void observation(double x[7], double declination, double dest[6][7])
{
  double s = sin(declination);
  double c = cos(declination);

  dest[0][0] =  2*x[2];
  dest[0][1] = -2*x[3];
  dest[0][2] =  2*x[0];
  dest[0][3] = -2*x[1];
  dest[0][4] =  0;
  dest[0][5] =  0;
  dest[0][6] =  0;

  dest[1][0] = -2*x[1];
  dest[1][1] = -2*x[0];
  dest[1][2] = -2*x[3];
  dest[1][3] = -2*x[2];
  dest[1][4] =  0;
  dest[1][5] =  0;
  dest[1][6] =  0;


  dest[2][0] =  0;
  dest[2][1] =  4*x[1];
  dest[2][2] =  4*x[2];
  dest[2][3] =  0;
  dest[2][4] =  0;
  dest[2][5] =  0;
  dest[2][6] =  0;


  dest[3][0] =  2*x[3]*s;
  dest[3][1] = -2*x[2]*s;
  dest[3][2] =  2*x[1]*s-4*x[2]*c;
  dest[3][3] = -2*x[0]*s-4*x[3]*c;
  dest[3][4] =  0;
  dest[3][5] =  0;
  dest[3][6] =  0;


  dest[4][0] = -2*x[3]*c;
  dest[4][1] =  2*x[2]*c-4*x[1]*s;
  dest[4][2] =  2*x[1]*c;
  dest[4][3] = -2*x[0]*c-4*x[3]*s;
  dest[4][4] =  0;
  dest[4][5] =  0;
  dest[4][6] =  0;


  dest[0][0] =  2*x[2]*c-2*x[1]*s;
  dest[0][1] =  2*x[3]*c-2*x[0]*s;
  dest[0][2] =  2*x[0]*c+2*x[3]*s;
  dest[0][3] =  2*x[1]*c+2*x[2]*s;
  dest[0][4] =  0;
  dest[0][5] =  0;
  dest[0][6] =  0;

}


void rk4(void (*fun_ptr)(ekfData*, double*, size_t, double, double*),ekfData* info,  double* x, size_t n,double t, double dt, double* out)
{
	double k1[7] /*= (double*) malloc(n*sizeof(double))*/;
	double k2[7] /*= (double*) malloc(n*sizeof(double))*/;
	double k3[7] /*= (double*) malloc(n*sizeof(double))*/;
	double k4[7] /*= (double*) malloc(n*sizeof(double))*/;

	
	
	fun_ptr(info, x, n, t, k1);

	vScale(.5*dt, k1, n, k2);
	vAdd(k1, n, x, k2);
	fun_ptr(info, k2, n, t + .5 * dt, k2);
	
	vScale(.5*dt, k2, n, k3);
	vAdd(k3, n, x, k2);
	fun_ptr(info, k3, n, t, k3);


	vScale(.5*dt, k3, n, k4);
	vAdd(k4, n, x, k4);
	fun_ptr(info, k4, n, t, k4);

	vScale(2, k2, n, k2);
	vScale(2, k3, n, k3);

	vAdd(k1, n, k2, k2);
	vAdd(k2, n, k3, k3);
	vAdd(k3, n, k4, k4);

	vScale(dt / 6, k4, n, k4);

	vAdd(x, n, k4, out);
/*
	free(k1);
	free(k2);
	free(k3);
	free(k4);
*/
}

void covarianceTransition(double t, double Pp[][7], double x[7], ekfData* ekf, double Pdot[7][7])
{
  double Ow[4][4];
  double Oq[4][3];
  double dw[3][3];
  OmegaQ(x, Oq);
  OmegaW(x+4, Ow);
  dwdotdw(x+4, ekf, dw);

  
  double F[7][7];
  double Finv[7][7];
  double P1[7][7];

  mMultM(F, 7, 7, Pp, 7, 7, P1);
  mInverse(F, 7, Finv);
  mMultM(Pp, 7, 7, Finv, 7, 7, Pp);
  mAdd(Pp, 7, 7, P1, Pp);
  mAdd(Pp, 7, 7, ekf->Q, Pdot);
  
  

}

void predict(double xp[7], double Pp[][7], ekfData* ekf, double dt, double x[7], double P[7][7])
{
  rk4(stateTransition, ekf, xp, 7, 0, dt, x );

  vNormalize(xp, 4, xp);

  covarianceTransition(0, Pp, xp, ekf, P);
  mAdd(Pp, 7, 7, P, P);
}



void update(double xp[7], double Pp[7][7], double y[6], ekfData* ekf, double x[7], double P[7][7])
{

  double declination = 0;
  double H[6][7];
  double Ht[7][6];
  double K[7][6];
  double K1[7][6];
  double HP[6][7];
  double inv[6][6];
  double K2[6][6];
  double PI[7][7];
  double KH[7][7];
  double z[6];
  double x1[7];
  double residual[6];
  mSetIdentity(PI, 7, 7);
  
  //get H
  observation(xp, declination, H);
  //get K
  mTranspose(H, 6, 7, Ht);
  mMultM(Pp, 7, 7, Ht, 7, 6, K1);
  mMultM(H, 6, 7, Pp, 7, 7, HP);
  mMultM(HP, 6, 7, Ht, 7, 6, inv);
  mAdd(inv, 6, 6, ekf->R, inv);
  mInverse(inv, 6, K2);
  mMultM(K1, 7, 6, K2, 6, 6, K);

  //get P
  mMultM(K, 7, 6, H, 6, 7, KH);
  mSubtract(PI, 7, 7, KH, KH);
  mAdd(KH, 7, 7, Pp, P);


  //get x
  magModel(declination, xp, z);
  accelModel(xp, z+3);


  vSubtract(y, 6, z, residual);
  mMultV(K, 7, 6, residual, x1);

  vAdd(xp, 7, x1, x);
  //check this
  vNormalize(x, 4, x);




}




void ekf_step(double xp[7], double Pp[7][7], ekfData* data, double measurement[6], double dt, double x[7], double P[7][7])
{
  predict(xp, Pp, data, dt, x, P);

  update(x, P, measurement, data, x, P); 


}


void ekf_init(ekfData* ekf, double x[7], double P[7][7])
{
  double Qv[] = {1e-6, 1e-6, 1e-6, 1e-5, 1e-5, 1e-5, 1e-5};
  double Rv[] = {.045, .045, .045, .015, .015, .015};
  mDiag(Qv, 7, ekf->Q);
  mDiag(Rv, 6, ekf->R);
  double a = (.5 * M_PI/180);
  a*=a;
  double b = (.3 * M_PI/180);
  b*=b;
  double Pv[] = {3*a, 3*a, b, b, b, b, b, b};
  mDiag(Pv, 7, P);

  vSetZero(x, 7);
  x[0] = 1;

}

