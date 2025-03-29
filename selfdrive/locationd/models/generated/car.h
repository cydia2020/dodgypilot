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
void car_err_fun(double *nom_x, double *delta_x, double *out_3892942576152068540);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3513777533127972372);
void car_H_mod_fun(double *state, double *out_9069499613101771426);
void car_f_fun(double *state, double dt, double *out_6662999426820259662);
void car_F_fun(double *state, double dt, double *out_4926417437503748301);
void car_h_25(double *state, double *unused, double *out_7130383517015797079);
void car_H_25(double *state, double *unused, double *out_4165807787917106586);
void car_h_24(double *state, double *unused, double *out_8376628945260103704);
void car_H_24(double *state, double *unused, double *out_2409764018674411515);
void car_h_30(double *state, double *unused, double *out_3423421061809231178);
void car_H_30(double *state, double *unused, double *out_2750882553574510169);
void car_h_26(double *state, double *unused, double *out_6460207113253563863);
void car_H_26(double *state, double *unused, double *out_7907311106791162810);
void car_h_27(double *state, double *unused, double *out_1457779164789044329);
void car_H_27(double *state, double *unused, double *out_576119241774085258);
void car_h_29(double *state, double *unused, double *out_3579341641983834601);
void car_H_29(double *state, double *unused, double *out_3261113897888902353);
void car_h_28(double *state, double *unused, double *out_7336064652347759417);
void car_H_28(double *state, double *unused, double *out_6219642502164996349);
void car_h_31(double *state, double *unused, double *out_7405577579300302968);
void car_H_31(double *state, double *unused, double *out_4135161826040146158);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}