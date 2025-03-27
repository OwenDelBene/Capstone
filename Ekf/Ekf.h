#pragma once

#include "linearAlgebra.h"
#include "cmath"

struct ekfData {

  double MOI[3][3];
  double MOI_inv[3][3];
  double Q[7][7];
  double R[6][6];
};

void accelModel(double x[4], double dest[3]);
void magModel(double declination, double x[4], double dest[3]);
void rk4(void (*fun_ptr)(ekfData*, double*, size_t, double, double*),ekfData* info,  double* x, size_t n,double t, double dt, double* out);
void stateTransition(ekfData* info, double* x, size_t n, double t, double* res);
void ekf_init(ekfData* ekf, double x[7], double P[7][7]);

void ekf_step(double xp[7], double Pp[7][7], ekfData* data, double measurement[6], double dt, double x[7], double P[7][7]);
