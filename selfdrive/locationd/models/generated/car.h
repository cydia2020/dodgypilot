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
void car_err_fun(double *nom_x, double *delta_x, double *out_30338062319998159);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_504080388801288227);
void car_H_mod_fun(double *state, double *out_8698487051627588745);
void car_f_fun(double *state, double dt, double *out_4117521611353438574);
void car_F_fun(double *state, double dt, double *out_4585009845263771629);
void car_h_25(double *state, double *unused, double *out_6646063448133992886);
void car_H_25(double *state, double *unused, double *out_7111700097066536677);
void car_h_24(double *state, double *unused, double *out_3392394255775213972);
void car_H_24(double *state, double *unused, double *out_4939050498061037111);
void car_h_30(double *state, double *unused, double *out_58235521657697723);
void car_H_30(double *state, double *unused, double *out_8816711018135766312);
void car_h_26(double *state, double *unused, double *out_5059908789318543368);
void car_H_26(double *state, double *unused, double *out_3370196778192480453);
void car_h_27(double *state, double *unused, double *out_4941901395385166624);
void car_H_27(double *state, double *unused, double *out_7455269743773360393);
void car_h_29(double *state, double *unused, double *out_4943437746003147922);
void car_H_29(double *state, double *unused, double *out_8306479673821374128);
void car_h_28(double *state, double *unused, double *out_59771872275679021);
void car_H_28(double *state, double *unused, double *out_5057865382818646914);
void car_h_31(double *state, double *unused, double *out_2218047976759028300);
void car_H_31(double *state, double *unused, double *out_2743988675959128977);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}