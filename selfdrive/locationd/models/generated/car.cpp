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
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5726561356979763240) {
   out_5726561356979763240[0] = delta_x[0] + nom_x[0];
   out_5726561356979763240[1] = delta_x[1] + nom_x[1];
   out_5726561356979763240[2] = delta_x[2] + nom_x[2];
   out_5726561356979763240[3] = delta_x[3] + nom_x[3];
   out_5726561356979763240[4] = delta_x[4] + nom_x[4];
   out_5726561356979763240[5] = delta_x[5] + nom_x[5];
   out_5726561356979763240[6] = delta_x[6] + nom_x[6];
   out_5726561356979763240[7] = delta_x[7] + nom_x[7];
   out_5726561356979763240[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8243349756363810926) {
   out_8243349756363810926[0] = -nom_x[0] + true_x[0];
   out_8243349756363810926[1] = -nom_x[1] + true_x[1];
   out_8243349756363810926[2] = -nom_x[2] + true_x[2];
   out_8243349756363810926[3] = -nom_x[3] + true_x[3];
   out_8243349756363810926[4] = -nom_x[4] + true_x[4];
   out_8243349756363810926[5] = -nom_x[5] + true_x[5];
   out_8243349756363810926[6] = -nom_x[6] + true_x[6];
   out_8243349756363810926[7] = -nom_x[7] + true_x[7];
   out_8243349756363810926[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_9213820877321560871) {
   out_9213820877321560871[0] = 1.0;
   out_9213820877321560871[1] = 0.0;
   out_9213820877321560871[2] = 0.0;
   out_9213820877321560871[3] = 0.0;
   out_9213820877321560871[4] = 0.0;
   out_9213820877321560871[5] = 0.0;
   out_9213820877321560871[6] = 0.0;
   out_9213820877321560871[7] = 0.0;
   out_9213820877321560871[8] = 0.0;
   out_9213820877321560871[9] = 0.0;
   out_9213820877321560871[10] = 1.0;
   out_9213820877321560871[11] = 0.0;
   out_9213820877321560871[12] = 0.0;
   out_9213820877321560871[13] = 0.0;
   out_9213820877321560871[14] = 0.0;
   out_9213820877321560871[15] = 0.0;
   out_9213820877321560871[16] = 0.0;
   out_9213820877321560871[17] = 0.0;
   out_9213820877321560871[18] = 0.0;
   out_9213820877321560871[19] = 0.0;
   out_9213820877321560871[20] = 1.0;
   out_9213820877321560871[21] = 0.0;
   out_9213820877321560871[22] = 0.0;
   out_9213820877321560871[23] = 0.0;
   out_9213820877321560871[24] = 0.0;
   out_9213820877321560871[25] = 0.0;
   out_9213820877321560871[26] = 0.0;
   out_9213820877321560871[27] = 0.0;
   out_9213820877321560871[28] = 0.0;
   out_9213820877321560871[29] = 0.0;
   out_9213820877321560871[30] = 1.0;
   out_9213820877321560871[31] = 0.0;
   out_9213820877321560871[32] = 0.0;
   out_9213820877321560871[33] = 0.0;
   out_9213820877321560871[34] = 0.0;
   out_9213820877321560871[35] = 0.0;
   out_9213820877321560871[36] = 0.0;
   out_9213820877321560871[37] = 0.0;
   out_9213820877321560871[38] = 0.0;
   out_9213820877321560871[39] = 0.0;
   out_9213820877321560871[40] = 1.0;
   out_9213820877321560871[41] = 0.0;
   out_9213820877321560871[42] = 0.0;
   out_9213820877321560871[43] = 0.0;
   out_9213820877321560871[44] = 0.0;
   out_9213820877321560871[45] = 0.0;
   out_9213820877321560871[46] = 0.0;
   out_9213820877321560871[47] = 0.0;
   out_9213820877321560871[48] = 0.0;
   out_9213820877321560871[49] = 0.0;
   out_9213820877321560871[50] = 1.0;
   out_9213820877321560871[51] = 0.0;
   out_9213820877321560871[52] = 0.0;
   out_9213820877321560871[53] = 0.0;
   out_9213820877321560871[54] = 0.0;
   out_9213820877321560871[55] = 0.0;
   out_9213820877321560871[56] = 0.0;
   out_9213820877321560871[57] = 0.0;
   out_9213820877321560871[58] = 0.0;
   out_9213820877321560871[59] = 0.0;
   out_9213820877321560871[60] = 1.0;
   out_9213820877321560871[61] = 0.0;
   out_9213820877321560871[62] = 0.0;
   out_9213820877321560871[63] = 0.0;
   out_9213820877321560871[64] = 0.0;
   out_9213820877321560871[65] = 0.0;
   out_9213820877321560871[66] = 0.0;
   out_9213820877321560871[67] = 0.0;
   out_9213820877321560871[68] = 0.0;
   out_9213820877321560871[69] = 0.0;
   out_9213820877321560871[70] = 1.0;
   out_9213820877321560871[71] = 0.0;
   out_9213820877321560871[72] = 0.0;
   out_9213820877321560871[73] = 0.0;
   out_9213820877321560871[74] = 0.0;
   out_9213820877321560871[75] = 0.0;
   out_9213820877321560871[76] = 0.0;
   out_9213820877321560871[77] = 0.0;
   out_9213820877321560871[78] = 0.0;
   out_9213820877321560871[79] = 0.0;
   out_9213820877321560871[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4022821575240968378) {
   out_4022821575240968378[0] = state[0];
   out_4022821575240968378[1] = state[1];
   out_4022821575240968378[2] = state[2];
   out_4022821575240968378[3] = state[3];
   out_4022821575240968378[4] = state[4];
   out_4022821575240968378[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4022821575240968378[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4022821575240968378[7] = state[7];
   out_4022821575240968378[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2833976534117093236) {
   out_2833976534117093236[0] = 1;
   out_2833976534117093236[1] = 0;
   out_2833976534117093236[2] = 0;
   out_2833976534117093236[3] = 0;
   out_2833976534117093236[4] = 0;
   out_2833976534117093236[5] = 0;
   out_2833976534117093236[6] = 0;
   out_2833976534117093236[7] = 0;
   out_2833976534117093236[8] = 0;
   out_2833976534117093236[9] = 0;
   out_2833976534117093236[10] = 1;
   out_2833976534117093236[11] = 0;
   out_2833976534117093236[12] = 0;
   out_2833976534117093236[13] = 0;
   out_2833976534117093236[14] = 0;
   out_2833976534117093236[15] = 0;
   out_2833976534117093236[16] = 0;
   out_2833976534117093236[17] = 0;
   out_2833976534117093236[18] = 0;
   out_2833976534117093236[19] = 0;
   out_2833976534117093236[20] = 1;
   out_2833976534117093236[21] = 0;
   out_2833976534117093236[22] = 0;
   out_2833976534117093236[23] = 0;
   out_2833976534117093236[24] = 0;
   out_2833976534117093236[25] = 0;
   out_2833976534117093236[26] = 0;
   out_2833976534117093236[27] = 0;
   out_2833976534117093236[28] = 0;
   out_2833976534117093236[29] = 0;
   out_2833976534117093236[30] = 1;
   out_2833976534117093236[31] = 0;
   out_2833976534117093236[32] = 0;
   out_2833976534117093236[33] = 0;
   out_2833976534117093236[34] = 0;
   out_2833976534117093236[35] = 0;
   out_2833976534117093236[36] = 0;
   out_2833976534117093236[37] = 0;
   out_2833976534117093236[38] = 0;
   out_2833976534117093236[39] = 0;
   out_2833976534117093236[40] = 1;
   out_2833976534117093236[41] = 0;
   out_2833976534117093236[42] = 0;
   out_2833976534117093236[43] = 0;
   out_2833976534117093236[44] = 0;
   out_2833976534117093236[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2833976534117093236[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2833976534117093236[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2833976534117093236[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2833976534117093236[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2833976534117093236[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2833976534117093236[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2833976534117093236[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2833976534117093236[53] = -9.8100000000000005*dt;
   out_2833976534117093236[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2833976534117093236[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2833976534117093236[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2833976534117093236[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2833976534117093236[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2833976534117093236[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2833976534117093236[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2833976534117093236[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2833976534117093236[62] = 0;
   out_2833976534117093236[63] = 0;
   out_2833976534117093236[64] = 0;
   out_2833976534117093236[65] = 0;
   out_2833976534117093236[66] = 0;
   out_2833976534117093236[67] = 0;
   out_2833976534117093236[68] = 0;
   out_2833976534117093236[69] = 0;
   out_2833976534117093236[70] = 1;
   out_2833976534117093236[71] = 0;
   out_2833976534117093236[72] = 0;
   out_2833976534117093236[73] = 0;
   out_2833976534117093236[74] = 0;
   out_2833976534117093236[75] = 0;
   out_2833976534117093236[76] = 0;
   out_2833976534117093236[77] = 0;
   out_2833976534117093236[78] = 0;
   out_2833976534117093236[79] = 0;
   out_2833976534117093236[80] = 1;
}
void h_25(double *state, double *unused, double *out_940129192681146697) {
   out_940129192681146697[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8895437811502878665) {
   out_8895437811502878665[0] = 0;
   out_8895437811502878665[1] = 0;
   out_8895437811502878665[2] = 0;
   out_8895437811502878665[3] = 0;
   out_8895437811502878665[4] = 0;
   out_8895437811502878665[5] = 0;
   out_8895437811502878665[6] = 1;
   out_8895437811502878665[7] = 0;
   out_8895437811502878665[8] = 0;
}
void h_24(double *state, double *unused, double *out_2894786402544485994) {
   out_2894786402544485994[0] = state[4];
   out_2894786402544485994[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2132914628529746058) {
   out_2132914628529746058[0] = 0;
   out_2132914628529746058[1] = 0;
   out_2132914628529746058[2] = 0;
   out_2132914628529746058[3] = 0;
   out_2132914628529746058[4] = 1;
   out_2132914628529746058[5] = 0;
   out_2132914628529746058[6] = 0;
   out_2132914628529746058[7] = 0;
   out_2132914628529746058[8] = 0;
   out_2132914628529746058[9] = 0;
   out_2132914628529746058[10] = 0;
   out_2132914628529746058[11] = 0;
   out_2132914628529746058[12] = 0;
   out_2132914628529746058[13] = 0;
   out_2132914628529746058[14] = 1;
   out_2132914628529746058[15] = 0;
   out_2132914628529746058[16] = 0;
   out_2132914628529746058[17] = 0;
}
void h_30(double *state, double *unused, double *out_6952810302203376662) {
   out_6952810302203376662[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4367741481375270467) {
   out_4367741481375270467[0] = 0;
   out_4367741481375270467[1] = 0;
   out_4367741481375270467[2] = 0;
   out_4367741481375270467[3] = 0;
   out_4367741481375270467[4] = 1;
   out_4367741481375270467[5] = 0;
   out_4367741481375270467[6] = 0;
   out_4367741481375270467[7] = 0;
   out_4367741481375270467[8] = 0;
}
void h_26(double *state, double *unused, double *out_8303064168713872704) {
   out_8303064168713872704[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5153934492628822441) {
   out_5153934492628822441[0] = 0;
   out_5153934492628822441[1] = 0;
   out_5153934492628822441[2] = 0;
   out_5153934492628822441[3] = 0;
   out_5153934492628822441[4] = 0;
   out_5153934492628822441[5] = 0;
   out_5153934492628822441[6] = 0;
   out_5153934492628822441[7] = 1;
   out_5153934492628822441[8] = 0;
}
void h_27(double *state, double *unused, double *out_8918452199223563511) {
   out_8918452199223563511[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6591335552559213684) {
   out_6591335552559213684[0] = 0;
   out_6591335552559213684[1] = 0;
   out_6591335552559213684[2] = 0;
   out_6591335552559213684[3] = 1;
   out_6591335552559213684[4] = 0;
   out_6591335552559213684[5] = 0;
   out_6591335552559213684[6] = 0;
   out_6591335552559213684[7] = 0;
   out_6591335552559213684[8] = 0;
}
void h_29(double *state, double *unused, double *out_8889528450697477303) {
   out_8889528450697477303[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4877972825689662651) {
   out_4877972825689662651[0] = 0;
   out_4877972825689662651[1] = 1;
   out_4877972825689662651[2] = 0;
   out_4877972825689662651[3] = 0;
   out_4877972825689662651[4] = 0;
   out_4877972825689662651[5] = 0;
   out_4877972825689662651[6] = 0;
   out_4877972825689662651[7] = 0;
   out_4877972825689662651[8] = 0;
}
void h_28(double *state, double *unused, double *out_3129250357072138959) {
   out_3129250357072138959[0] = state[0];
}
void H_28(double *state, double *unused, double *out_204426191379867923) {
   out_204426191379867923[0] = 1;
   out_204426191379867923[1] = 0;
   out_204426191379867923[2] = 0;
   out_204426191379867923[3] = 0;
   out_204426191379867923[4] = 0;
   out_204426191379867923[5] = 0;
   out_204426191379867923[6] = 0;
   out_204426191379867923[7] = 0;
   out_204426191379867923[8] = 0;
}
void h_31(double *state, double *unused, double *out_664935130396640808) {
   out_664935130396640808[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8926083773379839093) {
   out_8926083773379839093[0] = 0;
   out_8926083773379839093[1] = 0;
   out_8926083773379839093[2] = 0;
   out_8926083773379839093[3] = 0;
   out_8926083773379839093[4] = 0;
   out_8926083773379839093[5] = 0;
   out_8926083773379839093[6] = 0;
   out_8926083773379839093[7] = 0;
   out_8926083773379839093[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5726561356979763240) {
  err_fun(nom_x, delta_x, out_5726561356979763240);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8243349756363810926) {
  inv_err_fun(nom_x, true_x, out_8243349756363810926);
}
void car_H_mod_fun(double *state, double *out_9213820877321560871) {
  H_mod_fun(state, out_9213820877321560871);
}
void car_f_fun(double *state, double dt, double *out_4022821575240968378) {
  f_fun(state,  dt, out_4022821575240968378);
}
void car_F_fun(double *state, double dt, double *out_2833976534117093236) {
  F_fun(state,  dt, out_2833976534117093236);
}
void car_h_25(double *state, double *unused, double *out_940129192681146697) {
  h_25(state, unused, out_940129192681146697);
}
void car_H_25(double *state, double *unused, double *out_8895437811502878665) {
  H_25(state, unused, out_8895437811502878665);
}
void car_h_24(double *state, double *unused, double *out_2894786402544485994) {
  h_24(state, unused, out_2894786402544485994);
}
void car_H_24(double *state, double *unused, double *out_2132914628529746058) {
  H_24(state, unused, out_2132914628529746058);
}
void car_h_30(double *state, double *unused, double *out_6952810302203376662) {
  h_30(state, unused, out_6952810302203376662);
}
void car_H_30(double *state, double *unused, double *out_4367741481375270467) {
  H_30(state, unused, out_4367741481375270467);
}
void car_h_26(double *state, double *unused, double *out_8303064168713872704) {
  h_26(state, unused, out_8303064168713872704);
}
void car_H_26(double *state, double *unused, double *out_5153934492628822441) {
  H_26(state, unused, out_5153934492628822441);
}
void car_h_27(double *state, double *unused, double *out_8918452199223563511) {
  h_27(state, unused, out_8918452199223563511);
}
void car_H_27(double *state, double *unused, double *out_6591335552559213684) {
  H_27(state, unused, out_6591335552559213684);
}
void car_h_29(double *state, double *unused, double *out_8889528450697477303) {
  h_29(state, unused, out_8889528450697477303);
}
void car_H_29(double *state, double *unused, double *out_4877972825689662651) {
  H_29(state, unused, out_4877972825689662651);
}
void car_h_28(double *state, double *unused, double *out_3129250357072138959) {
  h_28(state, unused, out_3129250357072138959);
}
void car_H_28(double *state, double *unused, double *out_204426191379867923) {
  H_28(state, unused, out_204426191379867923);
}
void car_h_31(double *state, double *unused, double *out_664935130396640808) {
  h_31(state, unused, out_664935130396640808);
}
void car_H_31(double *state, double *unused, double *out_8926083773379839093) {
  H_31(state, unused, out_8926083773379839093);
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
