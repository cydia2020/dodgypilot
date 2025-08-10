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
void car_err_fun(double *nom_x, double *delta_x, double *out_5726561356979763240);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8243349756363810926);
void car_H_mod_fun(double *state, double *out_9213820877321560871);
void car_f_fun(double *state, double dt, double *out_4022821575240968378);
void car_F_fun(double *state, double dt, double *out_2833976534117093236);
void car_h_25(double *state, double *unused, double *out_940129192681146697);
void car_H_25(double *state, double *unused, double *out_8895437811502878665);
void car_h_24(double *state, double *unused, double *out_2894786402544485994);
void car_H_24(double *state, double *unused, double *out_2132914628529746058);
void car_h_30(double *state, double *unused, double *out_6952810302203376662);
void car_H_30(double *state, double *unused, double *out_4367741481375270467);
void car_h_26(double *state, double *unused, double *out_8303064168713872704);
void car_H_26(double *state, double *unused, double *out_5153934492628822441);
void car_h_27(double *state, double *unused, double *out_8918452199223563511);
void car_H_27(double *state, double *unused, double *out_6591335552559213684);
void car_h_29(double *state, double *unused, double *out_8889528450697477303);
void car_H_29(double *state, double *unused, double *out_4877972825689662651);
void car_h_28(double *state, double *unused, double *out_3129250357072138959);
void car_H_28(double *state, double *unused, double *out_204426191379867923);
void car_h_31(double *state, double *unused, double *out_664935130396640808);
void car_H_31(double *state, double *unused, double *out_8926083773379839093);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}