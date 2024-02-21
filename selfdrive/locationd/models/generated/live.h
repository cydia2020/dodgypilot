#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_7316337814942835842);
void live_err_fun(double *nom_x, double *delta_x, double *out_4513862475936664212);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4172672385577685767);
void live_H_mod_fun(double *state, double *out_2356253274402645423);
void live_f_fun(double *state, double dt, double *out_8019757858739511986);
void live_F_fun(double *state, double dt, double *out_3738833127034685730);
void live_h_4(double *state, double *unused, double *out_7929832534546602181);
void live_H_4(double *state, double *unused, double *out_4733316144775420525);
void live_h_9(double *state, double *unused, double *out_1092301682363318328);
void live_H_9(double *state, double *unused, double *out_4492126498145829880);
void live_h_10(double *state, double *unused, double *out_8878905249087906540);
void live_H_10(double *state, double *unused, double *out_7730980201798325637);
void live_h_12(double *state, double *unused, double *out_4506279052763526560);
void live_H_12(double *state, double *unused, double *out_286140263256541270);
void live_h_31(double *state, double *unused, double *out_748830701842749504);
void live_H_31(double *state, double *unused, double *out_1366654087402813149);
void live_h_32(double *state, double *unused, double *out_7302114582756987825);
void live_H_32(double *state, double *unused, double *out_8281518591096929505);
void live_h_13(double *state, double *unused, double *out_3980670143312308048);
void live_H_13(double *state, double *unused, double *out_5249559627443720105);
void live_h_14(double *state, double *unused, double *out_1092301682363318328);
void live_H_14(double *state, double *unused, double *out_4492126498145829880);
void live_h_33(double *state, double *unused, double *out_262105134184184712);
void live_H_33(double *state, double *unused, double *out_1783902917236044455);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}