#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_5046380216439218526);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2577348512298300834);
void pose_H_mod_fun(double *state, double *out_490162499866022473);
void pose_f_fun(double *state, double dt, double *out_1869185704993173059);
void pose_F_fun(double *state, double dt, double *out_1284215992985263392);
void pose_h_4(double *state, double *unused, double *out_3952000395503486674);
void pose_H_4(double *state, double *unused, double *out_9012219907785707801);
void pose_h_10(double *state, double *unused, double *out_4901115569812966272);
void pose_H_10(double *state, double *unused, double *out_5949843926999388058);
void pose_h_13(double *state, double *unused, double *out_4932748021563902320);
void pose_H_13(double *state, double *unused, double *out_6222250340591511014);
void pose_h_14(double *state, double *unused, double *out_2761179743815230965);
void pose_H_14(double *state, double *unused, double *out_5471283309584359286);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}