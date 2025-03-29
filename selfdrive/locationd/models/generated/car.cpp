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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3892942576152068540) {
   out_3892942576152068540[0] = delta_x[0] + nom_x[0];
   out_3892942576152068540[1] = delta_x[1] + nom_x[1];
   out_3892942576152068540[2] = delta_x[2] + nom_x[2];
   out_3892942576152068540[3] = delta_x[3] + nom_x[3];
   out_3892942576152068540[4] = delta_x[4] + nom_x[4];
   out_3892942576152068540[5] = delta_x[5] + nom_x[5];
   out_3892942576152068540[6] = delta_x[6] + nom_x[6];
   out_3892942576152068540[7] = delta_x[7] + nom_x[7];
   out_3892942576152068540[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3513777533127972372) {
   out_3513777533127972372[0] = -nom_x[0] + true_x[0];
   out_3513777533127972372[1] = -nom_x[1] + true_x[1];
   out_3513777533127972372[2] = -nom_x[2] + true_x[2];
   out_3513777533127972372[3] = -nom_x[3] + true_x[3];
   out_3513777533127972372[4] = -nom_x[4] + true_x[4];
   out_3513777533127972372[5] = -nom_x[5] + true_x[5];
   out_3513777533127972372[6] = -nom_x[6] + true_x[6];
   out_3513777533127972372[7] = -nom_x[7] + true_x[7];
   out_3513777533127972372[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_9069499613101771426) {
   out_9069499613101771426[0] = 1.0;
   out_9069499613101771426[1] = 0.0;
   out_9069499613101771426[2] = 0.0;
   out_9069499613101771426[3] = 0.0;
   out_9069499613101771426[4] = 0.0;
   out_9069499613101771426[5] = 0.0;
   out_9069499613101771426[6] = 0.0;
   out_9069499613101771426[7] = 0.0;
   out_9069499613101771426[8] = 0.0;
   out_9069499613101771426[9] = 0.0;
   out_9069499613101771426[10] = 1.0;
   out_9069499613101771426[11] = 0.0;
   out_9069499613101771426[12] = 0.0;
   out_9069499613101771426[13] = 0.0;
   out_9069499613101771426[14] = 0.0;
   out_9069499613101771426[15] = 0.0;
   out_9069499613101771426[16] = 0.0;
   out_9069499613101771426[17] = 0.0;
   out_9069499613101771426[18] = 0.0;
   out_9069499613101771426[19] = 0.0;
   out_9069499613101771426[20] = 1.0;
   out_9069499613101771426[21] = 0.0;
   out_9069499613101771426[22] = 0.0;
   out_9069499613101771426[23] = 0.0;
   out_9069499613101771426[24] = 0.0;
   out_9069499613101771426[25] = 0.0;
   out_9069499613101771426[26] = 0.0;
   out_9069499613101771426[27] = 0.0;
   out_9069499613101771426[28] = 0.0;
   out_9069499613101771426[29] = 0.0;
   out_9069499613101771426[30] = 1.0;
   out_9069499613101771426[31] = 0.0;
   out_9069499613101771426[32] = 0.0;
   out_9069499613101771426[33] = 0.0;
   out_9069499613101771426[34] = 0.0;
   out_9069499613101771426[35] = 0.0;
   out_9069499613101771426[36] = 0.0;
   out_9069499613101771426[37] = 0.0;
   out_9069499613101771426[38] = 0.0;
   out_9069499613101771426[39] = 0.0;
   out_9069499613101771426[40] = 1.0;
   out_9069499613101771426[41] = 0.0;
   out_9069499613101771426[42] = 0.0;
   out_9069499613101771426[43] = 0.0;
   out_9069499613101771426[44] = 0.0;
   out_9069499613101771426[45] = 0.0;
   out_9069499613101771426[46] = 0.0;
   out_9069499613101771426[47] = 0.0;
   out_9069499613101771426[48] = 0.0;
   out_9069499613101771426[49] = 0.0;
   out_9069499613101771426[50] = 1.0;
   out_9069499613101771426[51] = 0.0;
   out_9069499613101771426[52] = 0.0;
   out_9069499613101771426[53] = 0.0;
   out_9069499613101771426[54] = 0.0;
   out_9069499613101771426[55] = 0.0;
   out_9069499613101771426[56] = 0.0;
   out_9069499613101771426[57] = 0.0;
   out_9069499613101771426[58] = 0.0;
   out_9069499613101771426[59] = 0.0;
   out_9069499613101771426[60] = 1.0;
   out_9069499613101771426[61] = 0.0;
   out_9069499613101771426[62] = 0.0;
   out_9069499613101771426[63] = 0.0;
   out_9069499613101771426[64] = 0.0;
   out_9069499613101771426[65] = 0.0;
   out_9069499613101771426[66] = 0.0;
   out_9069499613101771426[67] = 0.0;
   out_9069499613101771426[68] = 0.0;
   out_9069499613101771426[69] = 0.0;
   out_9069499613101771426[70] = 1.0;
   out_9069499613101771426[71] = 0.0;
   out_9069499613101771426[72] = 0.0;
   out_9069499613101771426[73] = 0.0;
   out_9069499613101771426[74] = 0.0;
   out_9069499613101771426[75] = 0.0;
   out_9069499613101771426[76] = 0.0;
   out_9069499613101771426[77] = 0.0;
   out_9069499613101771426[78] = 0.0;
   out_9069499613101771426[79] = 0.0;
   out_9069499613101771426[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6662999426820259662) {
   out_6662999426820259662[0] = state[0];
   out_6662999426820259662[1] = state[1];
   out_6662999426820259662[2] = state[2];
   out_6662999426820259662[3] = state[3];
   out_6662999426820259662[4] = state[4];
   out_6662999426820259662[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6662999426820259662[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6662999426820259662[7] = state[7];
   out_6662999426820259662[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4926417437503748301) {
   out_4926417437503748301[0] = 1;
   out_4926417437503748301[1] = 0;
   out_4926417437503748301[2] = 0;
   out_4926417437503748301[3] = 0;
   out_4926417437503748301[4] = 0;
   out_4926417437503748301[5] = 0;
   out_4926417437503748301[6] = 0;
   out_4926417437503748301[7] = 0;
   out_4926417437503748301[8] = 0;
   out_4926417437503748301[9] = 0;
   out_4926417437503748301[10] = 1;
   out_4926417437503748301[11] = 0;
   out_4926417437503748301[12] = 0;
   out_4926417437503748301[13] = 0;
   out_4926417437503748301[14] = 0;
   out_4926417437503748301[15] = 0;
   out_4926417437503748301[16] = 0;
   out_4926417437503748301[17] = 0;
   out_4926417437503748301[18] = 0;
   out_4926417437503748301[19] = 0;
   out_4926417437503748301[20] = 1;
   out_4926417437503748301[21] = 0;
   out_4926417437503748301[22] = 0;
   out_4926417437503748301[23] = 0;
   out_4926417437503748301[24] = 0;
   out_4926417437503748301[25] = 0;
   out_4926417437503748301[26] = 0;
   out_4926417437503748301[27] = 0;
   out_4926417437503748301[28] = 0;
   out_4926417437503748301[29] = 0;
   out_4926417437503748301[30] = 1;
   out_4926417437503748301[31] = 0;
   out_4926417437503748301[32] = 0;
   out_4926417437503748301[33] = 0;
   out_4926417437503748301[34] = 0;
   out_4926417437503748301[35] = 0;
   out_4926417437503748301[36] = 0;
   out_4926417437503748301[37] = 0;
   out_4926417437503748301[38] = 0;
   out_4926417437503748301[39] = 0;
   out_4926417437503748301[40] = 1;
   out_4926417437503748301[41] = 0;
   out_4926417437503748301[42] = 0;
   out_4926417437503748301[43] = 0;
   out_4926417437503748301[44] = 0;
   out_4926417437503748301[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4926417437503748301[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4926417437503748301[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4926417437503748301[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4926417437503748301[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4926417437503748301[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4926417437503748301[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4926417437503748301[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4926417437503748301[53] = -9.8000000000000007*dt;
   out_4926417437503748301[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4926417437503748301[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4926417437503748301[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4926417437503748301[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4926417437503748301[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4926417437503748301[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4926417437503748301[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4926417437503748301[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4926417437503748301[62] = 0;
   out_4926417437503748301[63] = 0;
   out_4926417437503748301[64] = 0;
   out_4926417437503748301[65] = 0;
   out_4926417437503748301[66] = 0;
   out_4926417437503748301[67] = 0;
   out_4926417437503748301[68] = 0;
   out_4926417437503748301[69] = 0;
   out_4926417437503748301[70] = 1;
   out_4926417437503748301[71] = 0;
   out_4926417437503748301[72] = 0;
   out_4926417437503748301[73] = 0;
   out_4926417437503748301[74] = 0;
   out_4926417437503748301[75] = 0;
   out_4926417437503748301[76] = 0;
   out_4926417437503748301[77] = 0;
   out_4926417437503748301[78] = 0;
   out_4926417437503748301[79] = 0;
   out_4926417437503748301[80] = 1;
}
void h_25(double *state, double *unused, double *out_7130383517015797079) {
   out_7130383517015797079[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4165807787917106586) {
   out_4165807787917106586[0] = 0;
   out_4165807787917106586[1] = 0;
   out_4165807787917106586[2] = 0;
   out_4165807787917106586[3] = 0;
   out_4165807787917106586[4] = 0;
   out_4165807787917106586[5] = 0;
   out_4165807787917106586[6] = 1;
   out_4165807787917106586[7] = 0;
   out_4165807787917106586[8] = 0;
}
void h_24(double *state, double *unused, double *out_8376628945260103704) {
   out_8376628945260103704[0] = state[4];
   out_8376628945260103704[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2409764018674411515) {
   out_2409764018674411515[0] = 0;
   out_2409764018674411515[1] = 0;
   out_2409764018674411515[2] = 0;
   out_2409764018674411515[3] = 0;
   out_2409764018674411515[4] = 1;
   out_2409764018674411515[5] = 0;
   out_2409764018674411515[6] = 0;
   out_2409764018674411515[7] = 0;
   out_2409764018674411515[8] = 0;
   out_2409764018674411515[9] = 0;
   out_2409764018674411515[10] = 0;
   out_2409764018674411515[11] = 0;
   out_2409764018674411515[12] = 0;
   out_2409764018674411515[13] = 0;
   out_2409764018674411515[14] = 1;
   out_2409764018674411515[15] = 0;
   out_2409764018674411515[16] = 0;
   out_2409764018674411515[17] = 0;
}
void h_30(double *state, double *unused, double *out_3423421061809231178) {
   out_3423421061809231178[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2750882553574510169) {
   out_2750882553574510169[0] = 0;
   out_2750882553574510169[1] = 0;
   out_2750882553574510169[2] = 0;
   out_2750882553574510169[3] = 0;
   out_2750882553574510169[4] = 1;
   out_2750882553574510169[5] = 0;
   out_2750882553574510169[6] = 0;
   out_2750882553574510169[7] = 0;
   out_2750882553574510169[8] = 0;
}
void h_26(double *state, double *unused, double *out_6460207113253563863) {
   out_6460207113253563863[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7907311106791162810) {
   out_7907311106791162810[0] = 0;
   out_7907311106791162810[1] = 0;
   out_7907311106791162810[2] = 0;
   out_7907311106791162810[3] = 0;
   out_7907311106791162810[4] = 0;
   out_7907311106791162810[5] = 0;
   out_7907311106791162810[6] = 0;
   out_7907311106791162810[7] = 1;
   out_7907311106791162810[8] = 0;
}
void h_27(double *state, double *unused, double *out_1457779164789044329) {
   out_1457779164789044329[0] = state[3];
}
void H_27(double *state, double *unused, double *out_576119241774085258) {
   out_576119241774085258[0] = 0;
   out_576119241774085258[1] = 0;
   out_576119241774085258[2] = 0;
   out_576119241774085258[3] = 1;
   out_576119241774085258[4] = 0;
   out_576119241774085258[5] = 0;
   out_576119241774085258[6] = 0;
   out_576119241774085258[7] = 0;
   out_576119241774085258[8] = 0;
}
void h_29(double *state, double *unused, double *out_3579341641983834601) {
   out_3579341641983834601[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3261113897888902353) {
   out_3261113897888902353[0] = 0;
   out_3261113897888902353[1] = 1;
   out_3261113897888902353[2] = 0;
   out_3261113897888902353[3] = 0;
   out_3261113897888902353[4] = 0;
   out_3261113897888902353[5] = 0;
   out_3261113897888902353[6] = 0;
   out_3261113897888902353[7] = 0;
   out_3261113897888902353[8] = 0;
}
void h_28(double *state, double *unused, double *out_7336064652347759417) {
   out_7336064652347759417[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6219642502164996349) {
   out_6219642502164996349[0] = 1;
   out_6219642502164996349[1] = 0;
   out_6219642502164996349[2] = 0;
   out_6219642502164996349[3] = 0;
   out_6219642502164996349[4] = 0;
   out_6219642502164996349[5] = 0;
   out_6219642502164996349[6] = 0;
   out_6219642502164996349[7] = 0;
   out_6219642502164996349[8] = 0;
}
void h_31(double *state, double *unused, double *out_7405577579300302968) {
   out_7405577579300302968[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4135161826040146158) {
   out_4135161826040146158[0] = 0;
   out_4135161826040146158[1] = 0;
   out_4135161826040146158[2] = 0;
   out_4135161826040146158[3] = 0;
   out_4135161826040146158[4] = 0;
   out_4135161826040146158[5] = 0;
   out_4135161826040146158[6] = 0;
   out_4135161826040146158[7] = 0;
   out_4135161826040146158[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3892942576152068540) {
  err_fun(nom_x, delta_x, out_3892942576152068540);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3513777533127972372) {
  inv_err_fun(nom_x, true_x, out_3513777533127972372);
}
void car_H_mod_fun(double *state, double *out_9069499613101771426) {
  H_mod_fun(state, out_9069499613101771426);
}
void car_f_fun(double *state, double dt, double *out_6662999426820259662) {
  f_fun(state,  dt, out_6662999426820259662);
}
void car_F_fun(double *state, double dt, double *out_4926417437503748301) {
  F_fun(state,  dt, out_4926417437503748301);
}
void car_h_25(double *state, double *unused, double *out_7130383517015797079) {
  h_25(state, unused, out_7130383517015797079);
}
void car_H_25(double *state, double *unused, double *out_4165807787917106586) {
  H_25(state, unused, out_4165807787917106586);
}
void car_h_24(double *state, double *unused, double *out_8376628945260103704) {
  h_24(state, unused, out_8376628945260103704);
}
void car_H_24(double *state, double *unused, double *out_2409764018674411515) {
  H_24(state, unused, out_2409764018674411515);
}
void car_h_30(double *state, double *unused, double *out_3423421061809231178) {
  h_30(state, unused, out_3423421061809231178);
}
void car_H_30(double *state, double *unused, double *out_2750882553574510169) {
  H_30(state, unused, out_2750882553574510169);
}
void car_h_26(double *state, double *unused, double *out_6460207113253563863) {
  h_26(state, unused, out_6460207113253563863);
}
void car_H_26(double *state, double *unused, double *out_7907311106791162810) {
  H_26(state, unused, out_7907311106791162810);
}
void car_h_27(double *state, double *unused, double *out_1457779164789044329) {
  h_27(state, unused, out_1457779164789044329);
}
void car_H_27(double *state, double *unused, double *out_576119241774085258) {
  H_27(state, unused, out_576119241774085258);
}
void car_h_29(double *state, double *unused, double *out_3579341641983834601) {
  h_29(state, unused, out_3579341641983834601);
}
void car_H_29(double *state, double *unused, double *out_3261113897888902353) {
  H_29(state, unused, out_3261113897888902353);
}
void car_h_28(double *state, double *unused, double *out_7336064652347759417) {
  h_28(state, unused, out_7336064652347759417);
}
void car_H_28(double *state, double *unused, double *out_6219642502164996349) {
  H_28(state, unused, out_6219642502164996349);
}
void car_h_31(double *state, double *unused, double *out_7405577579300302968) {
  h_31(state, unused, out_7405577579300302968);
}
void car_H_31(double *state, double *unused, double *out_4135161826040146158) {
  H_31(state, unused, out_4135161826040146158);
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
