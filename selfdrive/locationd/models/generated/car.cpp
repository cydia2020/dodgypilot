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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6517639478485368159) {
   out_6517639478485368159[0] = delta_x[0] + nom_x[0];
   out_6517639478485368159[1] = delta_x[1] + nom_x[1];
   out_6517639478485368159[2] = delta_x[2] + nom_x[2];
   out_6517639478485368159[3] = delta_x[3] + nom_x[3];
   out_6517639478485368159[4] = delta_x[4] + nom_x[4];
   out_6517639478485368159[5] = delta_x[5] + nom_x[5];
   out_6517639478485368159[6] = delta_x[6] + nom_x[6];
   out_6517639478485368159[7] = delta_x[7] + nom_x[7];
   out_6517639478485368159[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8466010599859308048) {
   out_8466010599859308048[0] = -nom_x[0] + true_x[0];
   out_8466010599859308048[1] = -nom_x[1] + true_x[1];
   out_8466010599859308048[2] = -nom_x[2] + true_x[2];
   out_8466010599859308048[3] = -nom_x[3] + true_x[3];
   out_8466010599859308048[4] = -nom_x[4] + true_x[4];
   out_8466010599859308048[5] = -nom_x[5] + true_x[5];
   out_8466010599859308048[6] = -nom_x[6] + true_x[6];
   out_8466010599859308048[7] = -nom_x[7] + true_x[7];
   out_8466010599859308048[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7097569489402568281) {
   out_7097569489402568281[0] = 1.0;
   out_7097569489402568281[1] = 0;
   out_7097569489402568281[2] = 0;
   out_7097569489402568281[3] = 0;
   out_7097569489402568281[4] = 0;
   out_7097569489402568281[5] = 0;
   out_7097569489402568281[6] = 0;
   out_7097569489402568281[7] = 0;
   out_7097569489402568281[8] = 0;
   out_7097569489402568281[9] = 0;
   out_7097569489402568281[10] = 1.0;
   out_7097569489402568281[11] = 0;
   out_7097569489402568281[12] = 0;
   out_7097569489402568281[13] = 0;
   out_7097569489402568281[14] = 0;
   out_7097569489402568281[15] = 0;
   out_7097569489402568281[16] = 0;
   out_7097569489402568281[17] = 0;
   out_7097569489402568281[18] = 0;
   out_7097569489402568281[19] = 0;
   out_7097569489402568281[20] = 1.0;
   out_7097569489402568281[21] = 0;
   out_7097569489402568281[22] = 0;
   out_7097569489402568281[23] = 0;
   out_7097569489402568281[24] = 0;
   out_7097569489402568281[25] = 0;
   out_7097569489402568281[26] = 0;
   out_7097569489402568281[27] = 0;
   out_7097569489402568281[28] = 0;
   out_7097569489402568281[29] = 0;
   out_7097569489402568281[30] = 1.0;
   out_7097569489402568281[31] = 0;
   out_7097569489402568281[32] = 0;
   out_7097569489402568281[33] = 0;
   out_7097569489402568281[34] = 0;
   out_7097569489402568281[35] = 0;
   out_7097569489402568281[36] = 0;
   out_7097569489402568281[37] = 0;
   out_7097569489402568281[38] = 0;
   out_7097569489402568281[39] = 0;
   out_7097569489402568281[40] = 1.0;
   out_7097569489402568281[41] = 0;
   out_7097569489402568281[42] = 0;
   out_7097569489402568281[43] = 0;
   out_7097569489402568281[44] = 0;
   out_7097569489402568281[45] = 0;
   out_7097569489402568281[46] = 0;
   out_7097569489402568281[47] = 0;
   out_7097569489402568281[48] = 0;
   out_7097569489402568281[49] = 0;
   out_7097569489402568281[50] = 1.0;
   out_7097569489402568281[51] = 0;
   out_7097569489402568281[52] = 0;
   out_7097569489402568281[53] = 0;
   out_7097569489402568281[54] = 0;
   out_7097569489402568281[55] = 0;
   out_7097569489402568281[56] = 0;
   out_7097569489402568281[57] = 0;
   out_7097569489402568281[58] = 0;
   out_7097569489402568281[59] = 0;
   out_7097569489402568281[60] = 1.0;
   out_7097569489402568281[61] = 0;
   out_7097569489402568281[62] = 0;
   out_7097569489402568281[63] = 0;
   out_7097569489402568281[64] = 0;
   out_7097569489402568281[65] = 0;
   out_7097569489402568281[66] = 0;
   out_7097569489402568281[67] = 0;
   out_7097569489402568281[68] = 0;
   out_7097569489402568281[69] = 0;
   out_7097569489402568281[70] = 1.0;
   out_7097569489402568281[71] = 0;
   out_7097569489402568281[72] = 0;
   out_7097569489402568281[73] = 0;
   out_7097569489402568281[74] = 0;
   out_7097569489402568281[75] = 0;
   out_7097569489402568281[76] = 0;
   out_7097569489402568281[77] = 0;
   out_7097569489402568281[78] = 0;
   out_7097569489402568281[79] = 0;
   out_7097569489402568281[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7173384785288303420) {
   out_7173384785288303420[0] = state[0];
   out_7173384785288303420[1] = state[1];
   out_7173384785288303420[2] = state[2];
   out_7173384785288303420[3] = state[3];
   out_7173384785288303420[4] = state[4];
   out_7173384785288303420[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7173384785288303420[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7173384785288303420[7] = state[7];
   out_7173384785288303420[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2722811709474537536) {
   out_2722811709474537536[0] = 1;
   out_2722811709474537536[1] = 0;
   out_2722811709474537536[2] = 0;
   out_2722811709474537536[3] = 0;
   out_2722811709474537536[4] = 0;
   out_2722811709474537536[5] = 0;
   out_2722811709474537536[6] = 0;
   out_2722811709474537536[7] = 0;
   out_2722811709474537536[8] = 0;
   out_2722811709474537536[9] = 0;
   out_2722811709474537536[10] = 1;
   out_2722811709474537536[11] = 0;
   out_2722811709474537536[12] = 0;
   out_2722811709474537536[13] = 0;
   out_2722811709474537536[14] = 0;
   out_2722811709474537536[15] = 0;
   out_2722811709474537536[16] = 0;
   out_2722811709474537536[17] = 0;
   out_2722811709474537536[18] = 0;
   out_2722811709474537536[19] = 0;
   out_2722811709474537536[20] = 1;
   out_2722811709474537536[21] = 0;
   out_2722811709474537536[22] = 0;
   out_2722811709474537536[23] = 0;
   out_2722811709474537536[24] = 0;
   out_2722811709474537536[25] = 0;
   out_2722811709474537536[26] = 0;
   out_2722811709474537536[27] = 0;
   out_2722811709474537536[28] = 0;
   out_2722811709474537536[29] = 0;
   out_2722811709474537536[30] = 1;
   out_2722811709474537536[31] = 0;
   out_2722811709474537536[32] = 0;
   out_2722811709474537536[33] = 0;
   out_2722811709474537536[34] = 0;
   out_2722811709474537536[35] = 0;
   out_2722811709474537536[36] = 0;
   out_2722811709474537536[37] = 0;
   out_2722811709474537536[38] = 0;
   out_2722811709474537536[39] = 0;
   out_2722811709474537536[40] = 1;
   out_2722811709474537536[41] = 0;
   out_2722811709474537536[42] = 0;
   out_2722811709474537536[43] = 0;
   out_2722811709474537536[44] = 0;
   out_2722811709474537536[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2722811709474537536[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2722811709474537536[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2722811709474537536[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2722811709474537536[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2722811709474537536[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2722811709474537536[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2722811709474537536[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2722811709474537536[53] = -9.8000000000000007*dt;
   out_2722811709474537536[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2722811709474537536[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2722811709474537536[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2722811709474537536[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2722811709474537536[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2722811709474537536[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2722811709474537536[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2722811709474537536[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2722811709474537536[62] = 0;
   out_2722811709474537536[63] = 0;
   out_2722811709474537536[64] = 0;
   out_2722811709474537536[65] = 0;
   out_2722811709474537536[66] = 0;
   out_2722811709474537536[67] = 0;
   out_2722811709474537536[68] = 0;
   out_2722811709474537536[69] = 0;
   out_2722811709474537536[70] = 1;
   out_2722811709474537536[71] = 0;
   out_2722811709474537536[72] = 0;
   out_2722811709474537536[73] = 0;
   out_2722811709474537536[74] = 0;
   out_2722811709474537536[75] = 0;
   out_2722811709474537536[76] = 0;
   out_2722811709474537536[77] = 0;
   out_2722811709474537536[78] = 0;
   out_2722811709474537536[79] = 0;
   out_2722811709474537536[80] = 1;
}
void h_25(double *state, double *unused, double *out_2128546509610862050) {
   out_2128546509610862050[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1681571997751559573) {
   out_1681571997751559573[0] = 0;
   out_1681571997751559573[1] = 0;
   out_1681571997751559573[2] = 0;
   out_1681571997751559573[3] = 0;
   out_1681571997751559573[4] = 0;
   out_1681571997751559573[5] = 0;
   out_1681571997751559573[6] = 1;
   out_1681571997751559573[7] = 0;
   out_1681571997751559573[8] = 0;
}
void h_24(double *state, double *unused, double *out_6189992263984546162) {
   out_6189992263984546162[0] = state[4];
   out_6189992263984546162[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3854221596757059139) {
   out_3854221596757059139[0] = 0;
   out_3854221596757059139[1] = 0;
   out_3854221596757059139[2] = 0;
   out_3854221596757059139[3] = 0;
   out_3854221596757059139[4] = 1;
   out_3854221596757059139[5] = 0;
   out_3854221596757059139[6] = 0;
   out_3854221596757059139[7] = 0;
   out_3854221596757059139[8] = 0;
   out_3854221596757059139[9] = 0;
   out_3854221596757059139[10] = 0;
   out_3854221596757059139[11] = 0;
   out_3854221596757059139[12] = 0;
   out_3854221596757059139[13] = 0;
   out_3854221596757059139[14] = 1;
   out_3854221596757059139[15] = 0;
   out_3854221596757059139[16] = 0;
   out_3854221596757059139[17] = 0;
}
void h_30(double *state, double *unused, double *out_6368114171652877942) {
   out_6368114171652877942[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5235118343740057182) {
   out_5235118343740057182[0] = 0;
   out_5235118343740057182[1] = 0;
   out_5235118343740057182[2] = 0;
   out_5235118343740057182[3] = 0;
   out_5235118343740057182[4] = 1;
   out_5235118343740057182[5] = 0;
   out_5235118343740057182[6] = 0;
   out_5235118343740057182[7] = 0;
   out_5235118343740057182[8] = 0;
}
void h_26(double *state, double *unused, double *out_5424604318751215507) {
   out_5424604318751215507[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5423075316625615797) {
   out_5423075316625615797[0] = 0;
   out_5423075316625615797[1] = 0;
   out_5423075316625615797[2] = 0;
   out_5423075316625615797[3] = 0;
   out_5423075316625615797[4] = 0;
   out_5423075316625615797[5] = 0;
   out_5423075316625615797[6] = 0;
   out_5423075316625615797[7] = 1;
   out_5423075316625615797[8] = 0;
}
void h_27(double *state, double *unused, double *out_5404394322358318639) {
   out_5404394322358318639[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3060355031939632271) {
   out_3060355031939632271[0] = 0;
   out_3060355031939632271[1] = 0;
   out_3060355031939632271[2] = 0;
   out_3060355031939632271[3] = 1;
   out_3060355031939632271[4] = 0;
   out_3060355031939632271[5] = 0;
   out_3060355031939632271[6] = 0;
   out_3060355031939632271[7] = 0;
   out_3060355031939632271[8] = 0;
}
void h_29(double *state, double *unused, double *out_1366440903992032297) {
   out_1366440903992032297[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1346992305070081238) {
   out_1346992305070081238[0] = 0;
   out_1346992305070081238[1] = 1;
   out_1346992305070081238[2] = 0;
   out_1346992305070081238[3] = 0;
   out_1346992305070081238[4] = 0;
   out_1346992305070081238[5] = 0;
   out_1346992305070081238[6] = 0;
   out_1346992305070081238[7] = 0;
   out_1346992305070081238[8] = 0;
}
void h_28(double *state, double *unused, double *out_2799989001549620988) {
   out_2799989001549620988[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3735406711999449336) {
   out_3735406711999449336[0] = 1;
   out_3735406711999449336[1] = 0;
   out_3735406711999449336[2] = 0;
   out_3735406711999449336[3] = 0;
   out_3735406711999449336[4] = 0;
   out_3735406711999449336[5] = 0;
   out_3735406711999449336[6] = 0;
   out_3735406711999449336[7] = 0;
   out_3735406711999449336[8] = 0;
}
void h_31(double *state, double *unused, double *out_1853352447326356161) {
   out_1853352447326356161[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1650926035874599145) {
   out_1650926035874599145[0] = 0;
   out_1650926035874599145[1] = 0;
   out_1650926035874599145[2] = 0;
   out_1650926035874599145[3] = 0;
   out_1650926035874599145[4] = 0;
   out_1650926035874599145[5] = 0;
   out_1650926035874599145[6] = 0;
   out_1650926035874599145[7] = 0;
   out_1650926035874599145[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6517639478485368159) {
  err_fun(nom_x, delta_x, out_6517639478485368159);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8466010599859308048) {
  inv_err_fun(nom_x, true_x, out_8466010599859308048);
}
void car_H_mod_fun(double *state, double *out_7097569489402568281) {
  H_mod_fun(state, out_7097569489402568281);
}
void car_f_fun(double *state, double dt, double *out_7173384785288303420) {
  f_fun(state,  dt, out_7173384785288303420);
}
void car_F_fun(double *state, double dt, double *out_2722811709474537536) {
  F_fun(state,  dt, out_2722811709474537536);
}
void car_h_25(double *state, double *unused, double *out_2128546509610862050) {
  h_25(state, unused, out_2128546509610862050);
}
void car_H_25(double *state, double *unused, double *out_1681571997751559573) {
  H_25(state, unused, out_1681571997751559573);
}
void car_h_24(double *state, double *unused, double *out_6189992263984546162) {
  h_24(state, unused, out_6189992263984546162);
}
void car_H_24(double *state, double *unused, double *out_3854221596757059139) {
  H_24(state, unused, out_3854221596757059139);
}
void car_h_30(double *state, double *unused, double *out_6368114171652877942) {
  h_30(state, unused, out_6368114171652877942);
}
void car_H_30(double *state, double *unused, double *out_5235118343740057182) {
  H_30(state, unused, out_5235118343740057182);
}
void car_h_26(double *state, double *unused, double *out_5424604318751215507) {
  h_26(state, unused, out_5424604318751215507);
}
void car_H_26(double *state, double *unused, double *out_5423075316625615797) {
  H_26(state, unused, out_5423075316625615797);
}
void car_h_27(double *state, double *unused, double *out_5404394322358318639) {
  h_27(state, unused, out_5404394322358318639);
}
void car_H_27(double *state, double *unused, double *out_3060355031939632271) {
  H_27(state, unused, out_3060355031939632271);
}
void car_h_29(double *state, double *unused, double *out_1366440903992032297) {
  h_29(state, unused, out_1366440903992032297);
}
void car_H_29(double *state, double *unused, double *out_1346992305070081238) {
  H_29(state, unused, out_1346992305070081238);
}
void car_h_28(double *state, double *unused, double *out_2799989001549620988) {
  h_28(state, unused, out_2799989001549620988);
}
void car_H_28(double *state, double *unused, double *out_3735406711999449336) {
  H_28(state, unused, out_3735406711999449336);
}
void car_h_31(double *state, double *unused, double *out_1853352447326356161) {
  h_31(state, unused, out_1853352447326356161);
}
void car_H_31(double *state, double *unused, double *out_1650926035874599145) {
  H_31(state, unused, out_1650926035874599145);
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

ekf_lib_init(car)
