#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4151595018305427990) {
   out_4151595018305427990[0] = delta_x[0] + nom_x[0];
   out_4151595018305427990[1] = delta_x[1] + nom_x[1];
   out_4151595018305427990[2] = delta_x[2] + nom_x[2];
   out_4151595018305427990[3] = delta_x[3] + nom_x[3];
   out_4151595018305427990[4] = delta_x[4] + nom_x[4];
   out_4151595018305427990[5] = delta_x[5] + nom_x[5];
   out_4151595018305427990[6] = delta_x[6] + nom_x[6];
   out_4151595018305427990[7] = delta_x[7] + nom_x[7];
   out_4151595018305427990[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2529941714916061667) {
   out_2529941714916061667[0] = -nom_x[0] + true_x[0];
   out_2529941714916061667[1] = -nom_x[1] + true_x[1];
   out_2529941714916061667[2] = -nom_x[2] + true_x[2];
   out_2529941714916061667[3] = -nom_x[3] + true_x[3];
   out_2529941714916061667[4] = -nom_x[4] + true_x[4];
   out_2529941714916061667[5] = -nom_x[5] + true_x[5];
   out_2529941714916061667[6] = -nom_x[6] + true_x[6];
   out_2529941714916061667[7] = -nom_x[7] + true_x[7];
   out_2529941714916061667[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8044457524073253726) {
   out_8044457524073253726[0] = 1.0;
   out_8044457524073253726[1] = 0;
   out_8044457524073253726[2] = 0;
   out_8044457524073253726[3] = 0;
   out_8044457524073253726[4] = 0;
   out_8044457524073253726[5] = 0;
   out_8044457524073253726[6] = 0;
   out_8044457524073253726[7] = 0;
   out_8044457524073253726[8] = 0;
   out_8044457524073253726[9] = 0;
   out_8044457524073253726[10] = 1.0;
   out_8044457524073253726[11] = 0;
   out_8044457524073253726[12] = 0;
   out_8044457524073253726[13] = 0;
   out_8044457524073253726[14] = 0;
   out_8044457524073253726[15] = 0;
   out_8044457524073253726[16] = 0;
   out_8044457524073253726[17] = 0;
   out_8044457524073253726[18] = 0;
   out_8044457524073253726[19] = 0;
   out_8044457524073253726[20] = 1.0;
   out_8044457524073253726[21] = 0;
   out_8044457524073253726[22] = 0;
   out_8044457524073253726[23] = 0;
   out_8044457524073253726[24] = 0;
   out_8044457524073253726[25] = 0;
   out_8044457524073253726[26] = 0;
   out_8044457524073253726[27] = 0;
   out_8044457524073253726[28] = 0;
   out_8044457524073253726[29] = 0;
   out_8044457524073253726[30] = 1.0;
   out_8044457524073253726[31] = 0;
   out_8044457524073253726[32] = 0;
   out_8044457524073253726[33] = 0;
   out_8044457524073253726[34] = 0;
   out_8044457524073253726[35] = 0;
   out_8044457524073253726[36] = 0;
   out_8044457524073253726[37] = 0;
   out_8044457524073253726[38] = 0;
   out_8044457524073253726[39] = 0;
   out_8044457524073253726[40] = 1.0;
   out_8044457524073253726[41] = 0;
   out_8044457524073253726[42] = 0;
   out_8044457524073253726[43] = 0;
   out_8044457524073253726[44] = 0;
   out_8044457524073253726[45] = 0;
   out_8044457524073253726[46] = 0;
   out_8044457524073253726[47] = 0;
   out_8044457524073253726[48] = 0;
   out_8044457524073253726[49] = 0;
   out_8044457524073253726[50] = 1.0;
   out_8044457524073253726[51] = 0;
   out_8044457524073253726[52] = 0;
   out_8044457524073253726[53] = 0;
   out_8044457524073253726[54] = 0;
   out_8044457524073253726[55] = 0;
   out_8044457524073253726[56] = 0;
   out_8044457524073253726[57] = 0;
   out_8044457524073253726[58] = 0;
   out_8044457524073253726[59] = 0;
   out_8044457524073253726[60] = 1.0;
   out_8044457524073253726[61] = 0;
   out_8044457524073253726[62] = 0;
   out_8044457524073253726[63] = 0;
   out_8044457524073253726[64] = 0;
   out_8044457524073253726[65] = 0;
   out_8044457524073253726[66] = 0;
   out_8044457524073253726[67] = 0;
   out_8044457524073253726[68] = 0;
   out_8044457524073253726[69] = 0;
   out_8044457524073253726[70] = 1.0;
   out_8044457524073253726[71] = 0;
   out_8044457524073253726[72] = 0;
   out_8044457524073253726[73] = 0;
   out_8044457524073253726[74] = 0;
   out_8044457524073253726[75] = 0;
   out_8044457524073253726[76] = 0;
   out_8044457524073253726[77] = 0;
   out_8044457524073253726[78] = 0;
   out_8044457524073253726[79] = 0;
   out_8044457524073253726[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1104205458474419215) {
   out_1104205458474419215[0] = state[0];
   out_1104205458474419215[1] = state[1];
   out_1104205458474419215[2] = state[2];
   out_1104205458474419215[3] = state[3];
   out_1104205458474419215[4] = state[4];
   out_1104205458474419215[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1104205458474419215[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1104205458474419215[7] = state[7];
   out_1104205458474419215[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6465597992951491732) {
   out_6465597992951491732[0] = 1;
   out_6465597992951491732[1] = 0;
   out_6465597992951491732[2] = 0;
   out_6465597992951491732[3] = 0;
   out_6465597992951491732[4] = 0;
   out_6465597992951491732[5] = 0;
   out_6465597992951491732[6] = 0;
   out_6465597992951491732[7] = 0;
   out_6465597992951491732[8] = 0;
   out_6465597992951491732[9] = 0;
   out_6465597992951491732[10] = 1;
   out_6465597992951491732[11] = 0;
   out_6465597992951491732[12] = 0;
   out_6465597992951491732[13] = 0;
   out_6465597992951491732[14] = 0;
   out_6465597992951491732[15] = 0;
   out_6465597992951491732[16] = 0;
   out_6465597992951491732[17] = 0;
   out_6465597992951491732[18] = 0;
   out_6465597992951491732[19] = 0;
   out_6465597992951491732[20] = 1;
   out_6465597992951491732[21] = 0;
   out_6465597992951491732[22] = 0;
   out_6465597992951491732[23] = 0;
   out_6465597992951491732[24] = 0;
   out_6465597992951491732[25] = 0;
   out_6465597992951491732[26] = 0;
   out_6465597992951491732[27] = 0;
   out_6465597992951491732[28] = 0;
   out_6465597992951491732[29] = 0;
   out_6465597992951491732[30] = 1;
   out_6465597992951491732[31] = 0;
   out_6465597992951491732[32] = 0;
   out_6465597992951491732[33] = 0;
   out_6465597992951491732[34] = 0;
   out_6465597992951491732[35] = 0;
   out_6465597992951491732[36] = 0;
   out_6465597992951491732[37] = 0;
   out_6465597992951491732[38] = 0;
   out_6465597992951491732[39] = 0;
   out_6465597992951491732[40] = 1;
   out_6465597992951491732[41] = 0;
   out_6465597992951491732[42] = 0;
   out_6465597992951491732[43] = 0;
   out_6465597992951491732[44] = 0;
   out_6465597992951491732[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6465597992951491732[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6465597992951491732[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6465597992951491732[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6465597992951491732[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6465597992951491732[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6465597992951491732[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6465597992951491732[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6465597992951491732[53] = -9.8000000000000007*dt;
   out_6465597992951491732[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6465597992951491732[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6465597992951491732[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6465597992951491732[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6465597992951491732[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6465597992951491732[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6465597992951491732[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6465597992951491732[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6465597992951491732[62] = 0;
   out_6465597992951491732[63] = 0;
   out_6465597992951491732[64] = 0;
   out_6465597992951491732[65] = 0;
   out_6465597992951491732[66] = 0;
   out_6465597992951491732[67] = 0;
   out_6465597992951491732[68] = 0;
   out_6465597992951491732[69] = 0;
   out_6465597992951491732[70] = 1;
   out_6465597992951491732[71] = 0;
   out_6465597992951491732[72] = 0;
   out_6465597992951491732[73] = 0;
   out_6465597992951491732[74] = 0;
   out_6465597992951491732[75] = 0;
   out_6465597992951491732[76] = 0;
   out_6465597992951491732[77] = 0;
   out_6465597992951491732[78] = 0;
   out_6465597992951491732[79] = 0;
   out_6465597992951491732[80] = 1;
}
void h_25(double *state, double *unused, double *out_7605568598911739569) {
   out_7605568598911739569[0] = state[6];
}
void H_25(double *state, double *unused, double *out_54502815036703862) {
   out_54502815036703862[0] = 0;
   out_54502815036703862[1] = 0;
   out_54502815036703862[2] = 0;
   out_54502815036703862[3] = 0;
   out_54502815036703862[4] = 0;
   out_54502815036703862[5] = 0;
   out_54502815036703862[6] = 1;
   out_54502815036703862[7] = 0;
   out_54502815036703862[8] = 0;
}
void h_24(double *state, double *unused, double *out_47739560005025743) {
   out_47739560005025743[0] = state[4];
   out_47739560005025743[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8330742543947082571) {
   out_8330742543947082571[0] = 0;
   out_8330742543947082571[1] = 0;
   out_8330742543947082571[2] = 0;
   out_8330742543947082571[3] = 0;
   out_8330742543947082571[4] = 1;
   out_8330742543947082571[5] = 0;
   out_8330742543947082571[6] = 0;
   out_8330742543947082571[7] = 0;
   out_8330742543947082571[8] = 0;
   out_8330742543947082571[9] = 0;
   out_8330742543947082571[10] = 0;
   out_8330742543947082571[11] = 0;
   out_8330742543947082571[12] = 0;
   out_8330742543947082571[13] = 0;
   out_8330742543947082571[14] = 1;
   out_8330742543947082571[15] = 0;
   out_8330742543947082571[16] = 0;
   out_8330742543947082571[17] = 0;
}
void h_30(double *state, double *unused, double *out_7448381930560610424) {
   out_7448381930560610424[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6862187526454912893) {
   out_6862187526454912893[0] = 0;
   out_6862187526454912893[1] = 0;
   out_6862187526454912893[2] = 0;
   out_6862187526454912893[3] = 0;
   out_6862187526454912893[4] = 1;
   out_6862187526454912893[5] = 0;
   out_6862187526454912893[6] = 0;
   out_6862187526454912893[7] = 0;
   out_6862187526454912893[8] = 0;
}
void h_26(double *state, double *unused, double *out_8391891783462272859) {
   out_8391891783462272859[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3250023154724096739) {
   out_3250023154724096739[0] = 0;
   out_3250023154724096739[1] = 0;
   out_3250023154724096739[2] = 0;
   out_3250023154724096739[3] = 0;
   out_3250023154724096739[4] = 0;
   out_3250023154724096739[5] = 0;
   out_3250023154724096739[6] = 0;
   out_3250023154724096739[7] = 1;
   out_3250023154724096739[8] = 0;
}
void h_27(double *state, double *unused, double *out_170425164483038756) {
   out_170425164483038756[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4687424214654487982) {
   out_4687424214654487982[0] = 0;
   out_4687424214654487982[1] = 0;
   out_4687424214654487982[2] = 0;
   out_4687424214654487982[3] = 1;
   out_4687424214654487982[4] = 0;
   out_4687424214654487982[5] = 0;
   out_4687424214654487982[6] = 0;
   out_4687424214654487982[7] = 0;
   out_4687424214654487982[8] = 0;
}
void h_29(double *state, double *unused, double *out_6600410061867312180) {
   out_6600410061867312180[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2974061487784936949) {
   out_2974061487784936949[0] = 0;
   out_2974061487784936949[1] = 1;
   out_2974061487784936949[2] = 0;
   out_2974061487784936949[3] = 0;
   out_2974061487784936949[4] = 0;
   out_2974061487784936949[5] = 0;
   out_2974061487784936949[6] = 0;
   out_2974061487784936949[7] = 0;
   out_2974061487784936949[8] = 0;
}
void h_28(double *state, double *unused, double *out_5172098432143884401) {
   out_5172098432143884401[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2108337529284593625) {
   out_2108337529284593625[0] = 1;
   out_2108337529284593625[1] = 0;
   out_2108337529284593625[2] = 0;
   out_2108337529284593625[3] = 0;
   out_2108337529284593625[4] = 0;
   out_2108337529284593625[5] = 0;
   out_2108337529284593625[6] = 0;
   out_2108337529284593625[7] = 0;
   out_2108337529284593625[8] = 0;
}
void h_31(double *state, double *unused, double *out_3380616710548923722) {
   out_3380616710548923722[0] = state[8];
}
void H_31(double *state, double *unused, double *out_23856853159743434) {
   out_23856853159743434[0] = 0;
   out_23856853159743434[1] = 0;
   out_23856853159743434[2] = 0;
   out_23856853159743434[3] = 0;
   out_23856853159743434[4] = 0;
   out_23856853159743434[5] = 0;
   out_23856853159743434[6] = 0;
   out_23856853159743434[7] = 0;
   out_23856853159743434[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_4151595018305427990) {
  err_fun(nom_x, delta_x, out_4151595018305427990);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2529941714916061667) {
  inv_err_fun(nom_x, true_x, out_2529941714916061667);
}
void car_H_mod_fun(double *state, double *out_8044457524073253726) {
  H_mod_fun(state, out_8044457524073253726);
}
void car_f_fun(double *state, double dt, double *out_1104205458474419215) {
  f_fun(state,  dt, out_1104205458474419215);
}
void car_F_fun(double *state, double dt, double *out_6465597992951491732) {
  F_fun(state,  dt, out_6465597992951491732);
}
void car_h_25(double *state, double *unused, double *out_7605568598911739569) {
  h_25(state, unused, out_7605568598911739569);
}
void car_H_25(double *state, double *unused, double *out_54502815036703862) {
  H_25(state, unused, out_54502815036703862);
}
void car_h_24(double *state, double *unused, double *out_47739560005025743) {
  h_24(state, unused, out_47739560005025743);
}
void car_H_24(double *state, double *unused, double *out_8330742543947082571) {
  H_24(state, unused, out_8330742543947082571);
}
void car_h_30(double *state, double *unused, double *out_7448381930560610424) {
  h_30(state, unused, out_7448381930560610424);
}
void car_H_30(double *state, double *unused, double *out_6862187526454912893) {
  H_30(state, unused, out_6862187526454912893);
}
void car_h_26(double *state, double *unused, double *out_8391891783462272859) {
  h_26(state, unused, out_8391891783462272859);
}
void car_H_26(double *state, double *unused, double *out_3250023154724096739) {
  H_26(state, unused, out_3250023154724096739);
}
void car_h_27(double *state, double *unused, double *out_170425164483038756) {
  h_27(state, unused, out_170425164483038756);
}
void car_H_27(double *state, double *unused, double *out_4687424214654487982) {
  H_27(state, unused, out_4687424214654487982);
}
void car_h_29(double *state, double *unused, double *out_6600410061867312180) {
  h_29(state, unused, out_6600410061867312180);
}
void car_H_29(double *state, double *unused, double *out_2974061487784936949) {
  H_29(state, unused, out_2974061487784936949);
}
void car_h_28(double *state, double *unused, double *out_5172098432143884401) {
  h_28(state, unused, out_5172098432143884401);
}
void car_H_28(double *state, double *unused, double *out_2108337529284593625) {
  H_28(state, unused, out_2108337529284593625);
}
void car_h_31(double *state, double *unused, double *out_3380616710548923722) {
  h_31(state, unused, out_3380616710548923722);
}
void car_H_31(double *state, double *unused, double *out_23856853159743434) {
  H_31(state, unused, out_23856853159743434);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
