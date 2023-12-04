#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4151595018305427990);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2529941714916061667);
void car_H_mod_fun(double *state, double *out_8044457524073253726);
void car_f_fun(double *state, double dt, double *out_1104205458474419215);
void car_F_fun(double *state, double dt, double *out_6465597992951491732);
void car_h_25(double *state, double *unused, double *out_7605568598911739569);
void car_H_25(double *state, double *unused, double *out_54502815036703862);
void car_h_24(double *state, double *unused, double *out_47739560005025743);
void car_H_24(double *state, double *unused, double *out_8330742543947082571);
void car_h_30(double *state, double *unused, double *out_7448381930560610424);
void car_H_30(double *state, double *unused, double *out_6862187526454912893);
void car_h_26(double *state, double *unused, double *out_8391891783462272859);
void car_H_26(double *state, double *unused, double *out_3250023154724096739);
void car_h_27(double *state, double *unused, double *out_170425164483038756);
void car_H_27(double *state, double *unused, double *out_4687424214654487982);
void car_h_29(double *state, double *unused, double *out_6600410061867312180);
void car_H_29(double *state, double *unused, double *out_2974061487784936949);
void car_h_28(double *state, double *unused, double *out_5172098432143884401);
void car_H_28(double *state, double *unused, double *out_2108337529284593625);
void car_h_31(double *state, double *unused, double *out_3380616710548923722);
void car_H_31(double *state, double *unused, double *out_23856853159743434);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}