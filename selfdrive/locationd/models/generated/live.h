#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_8177860881423721097);
void live_err_fun(double *nom_x, double *delta_x, double *out_8353194234871527983);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3669708330239154866);
void live_H_mod_fun(double *state, double *out_1766872349777911748);
void live_f_fun(double *state, double dt, double *out_7988801304375705399);
void live_F_fun(double *state, double dt, double *out_106973650267764100);
void live_h_4(double *state, double *unused, double *out_8032299892119530523);
void live_H_4(double *state, double *unused, double *out_2044598066312698566);
void live_h_9(double *state, double *unused, double *out_8388823788043738380);
void live_H_9(double *state, double *unused, double *out_9114927072132405580);
void live_h_10(double *state, double *unused, double *out_9080029181622432000);
void live_H_10(double *state, double *unused, double *out_1558288581001861970);
void live_h_12(double *state, double *unused, double *out_438529044314930813);
void live_H_12(double *state, double *unused, double *out_4336660310730034430);
void live_h_35(double *state, double *unused, double *out_5365953328651537646);
void live_H_35(double *state, double *unused, double *out_5411260123685305942);
void live_h_32(double *state, double *unused, double *out_6351893806347753406);
void live_H_32(double *state, double *unused, double *out_3766420613974406340);
void live_h_13(double *state, double *unused, double *out_6772400558873417036);
void live_H_13(double *state, double *unused, double *out_1365703255155124589);
void live_h_14(double *state, double *unused, double *out_8388823788043738380);
void live_H_14(double *state, double *unused, double *out_9114927072132405580);
void live_h_33(double *state, double *unused, double *out_8248718677295485822);
void live_H_33(double *state, double *unused, double *out_2838897656750531245);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}