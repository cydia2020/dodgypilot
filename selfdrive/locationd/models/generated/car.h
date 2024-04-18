#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6517639478485368159);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8466010599859308048);
void car_H_mod_fun(double *state, double *out_7097569489402568281);
void car_f_fun(double *state, double dt, double *out_7173384785288303420);
void car_F_fun(double *state, double dt, double *out_2722811709474537536);
void car_h_25(double *state, double *unused, double *out_2128546509610862050);
void car_H_25(double *state, double *unused, double *out_1681571997751559573);
void car_h_24(double *state, double *unused, double *out_6189992263984546162);
void car_H_24(double *state, double *unused, double *out_3854221596757059139);
void car_h_30(double *state, double *unused, double *out_6368114171652877942);
void car_H_30(double *state, double *unused, double *out_5235118343740057182);
void car_h_26(double *state, double *unused, double *out_5424604318751215507);
void car_H_26(double *state, double *unused, double *out_5423075316625615797);
void car_h_27(double *state, double *unused, double *out_5404394322358318639);
void car_H_27(double *state, double *unused, double *out_3060355031939632271);
void car_h_29(double *state, double *unused, double *out_1366440903992032297);
void car_H_29(double *state, double *unused, double *out_1346992305070081238);
void car_h_28(double *state, double *unused, double *out_2799989001549620988);
void car_H_28(double *state, double *unused, double *out_3735406711999449336);
void car_h_31(double *state, double *unused, double *out_1853352447326356161);
void car_H_31(double *state, double *unused, double *out_1650926035874599145);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}