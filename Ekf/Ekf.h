


struct ekfData {

  double MOI[3][3];
  double MOI_inv[3][3];
  double Q[7][7];
  double R[6][6];
};

void ekf_init(ekfData* ekf, double x[7], double P[7][7]);

void ekf_step(double xp[7], double Pp[7][7], ekfData* data, double measurement[6], double dt, double x[7], double P[7][7]);
