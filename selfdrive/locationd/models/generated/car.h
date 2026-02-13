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
void car_err_fun(double *nom_x, double *delta_x, double *out_2838428601242523711);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1569222267276235328);
void car_H_mod_fun(double *state, double *out_6664598636645111356);
void car_f_fun(double *state, double dt, double *out_1048064928891473471);
void car_F_fun(double *state, double dt, double *out_1901252344738326427);
void car_h_25(double *state, double *unused, double *out_3036090623932210559);
void car_H_25(double *state, double *unused, double *out_8054967264237643249);
void car_h_24(double *state, double *unused, double *out_810131366787653052);
void car_H_24(double *state, double *unused, double *out_1479395457646125148);
void car_h_30(double *state, double *unused, double *out_9203204663903344862);
void car_H_30(double *state, double *unused, double *out_5536634305730394622);
void car_h_26(double *state, double *unused, double *out_4240824666222051630);
void car_H_26(double *state, double *unused, double *out_6650273490597852143);
void car_h_27(double *state, double *unused, double *out_1965582643728635086);
void car_H_27(double *state, double *unused, double *out_3313040234546451405);
void car_h_29(double *state, double *unused, double *out_2240776706013140975);
void car_H_29(double *state, double *unused, double *out_5026402961416002438);
void car_h_28(double *state, double *unused, double *out_3519501387612197369);
void car_H_28(double *state, double *unused, double *out_8337942095224018604);
void car_h_31(double *state, double *unused, double *out_6673540152186232909);
void car_H_31(double *state, double *unused, double *out_8024321302360682821);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}