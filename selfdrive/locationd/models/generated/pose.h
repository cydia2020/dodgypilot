#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8209044570432856386);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4042472597088507909);
void pose_H_mod_fun(double *state, double *out_3109392813306738693);
void pose_f_fun(double *state, double dt, double *out_2650242605500977184);
void pose_F_fun(double *state, double dt, double *out_4407203996993581678);
void pose_h_4(double *state, double *unused, double *out_992674251783283689);
void pose_H_4(double *state, double *unused, double *out_2043125571732342042);
void pose_h_10(double *state, double *unused, double *out_7169769923752438266);
void pose_H_10(double *state, double *unused, double *out_3475252269168967167);
void pose_h_13(double *state, double *unused, double *out_7070279399632189089);
void pose_H_13(double *state, double *unused, double *out_5255399397064674843);
void pose_h_14(double *state, double *unused, double *out_1916798049807563076);
void pose_H_14(double *state, double *unused, double *out_6006366428071826571);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}