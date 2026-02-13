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
void live_H(double *in_vec, double *out_7626513390256512324);
void live_err_fun(double *nom_x, double *delta_x, double *out_1209619150828279981);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7167274454258554278);
void live_H_mod_fun(double *state, double *out_6003836584634482836);
void live_f_fun(double *state, double dt, double *out_65927313982015855);
void live_F_fun(double *state, double dt, double *out_5108471478026490385);
void live_h_4(double *state, double *unused, double *out_2591526226458708109);
void live_H_4(double *state, double *unused, double *out_8554860113000918144);
void live_h_9(double *state, double *unused, double *out_8442778593654952759);
void live_H_9(double *state, double *unused, double *out_1267641177736470674);
void live_h_10(double *state, double *unused, double *out_2748243495236917575);
void live_H_10(double *state, double *unused, double *out_1935059459640152423);
void live_h_12(double *state, double *unused, double *out_4752631964098191386);
void live_H_12(double *state, double *unused, double *out_7933761087953324477);
void live_h_35(double *state, double *unused, double *out_635156831572008645);
void live_H_35(double *state, double *unused, double *out_5188198055628310768);
void live_h_32(double *state, double *unused, double *out_1711646351426286710);
void live_H_32(double *state, double *unused, double *out_5057033270687570299);
void live_h_13(double *state, double *unused, double *out_8329630983821488406);
void live_H_13(double *state, double *unused, double *out_8256569793686904771);
void live_h_14(double *state, double *unused, double *out_8442778593654952759);
void live_H_14(double *state, double *unused, double *out_1267641177736470674);
void live_h_33(double *state, double *unused, double *out_2129386500870919595);
void live_H_33(double *state, double *unused, double *out_2037641050989453164);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}