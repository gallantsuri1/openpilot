#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2838428601242523711) {
   out_2838428601242523711[0] = delta_x[0] + nom_x[0];
   out_2838428601242523711[1] = delta_x[1] + nom_x[1];
   out_2838428601242523711[2] = delta_x[2] + nom_x[2];
   out_2838428601242523711[3] = delta_x[3] + nom_x[3];
   out_2838428601242523711[4] = delta_x[4] + nom_x[4];
   out_2838428601242523711[5] = delta_x[5] + nom_x[5];
   out_2838428601242523711[6] = delta_x[6] + nom_x[6];
   out_2838428601242523711[7] = delta_x[7] + nom_x[7];
   out_2838428601242523711[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1569222267276235328) {
   out_1569222267276235328[0] = -nom_x[0] + true_x[0];
   out_1569222267276235328[1] = -nom_x[1] + true_x[1];
   out_1569222267276235328[2] = -nom_x[2] + true_x[2];
   out_1569222267276235328[3] = -nom_x[3] + true_x[3];
   out_1569222267276235328[4] = -nom_x[4] + true_x[4];
   out_1569222267276235328[5] = -nom_x[5] + true_x[5];
   out_1569222267276235328[6] = -nom_x[6] + true_x[6];
   out_1569222267276235328[7] = -nom_x[7] + true_x[7];
   out_1569222267276235328[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6664598636645111356) {
   out_6664598636645111356[0] = 1.0;
   out_6664598636645111356[1] = 0;
   out_6664598636645111356[2] = 0;
   out_6664598636645111356[3] = 0;
   out_6664598636645111356[4] = 0;
   out_6664598636645111356[5] = 0;
   out_6664598636645111356[6] = 0;
   out_6664598636645111356[7] = 0;
   out_6664598636645111356[8] = 0;
   out_6664598636645111356[9] = 0;
   out_6664598636645111356[10] = 1.0;
   out_6664598636645111356[11] = 0;
   out_6664598636645111356[12] = 0;
   out_6664598636645111356[13] = 0;
   out_6664598636645111356[14] = 0;
   out_6664598636645111356[15] = 0;
   out_6664598636645111356[16] = 0;
   out_6664598636645111356[17] = 0;
   out_6664598636645111356[18] = 0;
   out_6664598636645111356[19] = 0;
   out_6664598636645111356[20] = 1.0;
   out_6664598636645111356[21] = 0;
   out_6664598636645111356[22] = 0;
   out_6664598636645111356[23] = 0;
   out_6664598636645111356[24] = 0;
   out_6664598636645111356[25] = 0;
   out_6664598636645111356[26] = 0;
   out_6664598636645111356[27] = 0;
   out_6664598636645111356[28] = 0;
   out_6664598636645111356[29] = 0;
   out_6664598636645111356[30] = 1.0;
   out_6664598636645111356[31] = 0;
   out_6664598636645111356[32] = 0;
   out_6664598636645111356[33] = 0;
   out_6664598636645111356[34] = 0;
   out_6664598636645111356[35] = 0;
   out_6664598636645111356[36] = 0;
   out_6664598636645111356[37] = 0;
   out_6664598636645111356[38] = 0;
   out_6664598636645111356[39] = 0;
   out_6664598636645111356[40] = 1.0;
   out_6664598636645111356[41] = 0;
   out_6664598636645111356[42] = 0;
   out_6664598636645111356[43] = 0;
   out_6664598636645111356[44] = 0;
   out_6664598636645111356[45] = 0;
   out_6664598636645111356[46] = 0;
   out_6664598636645111356[47] = 0;
   out_6664598636645111356[48] = 0;
   out_6664598636645111356[49] = 0;
   out_6664598636645111356[50] = 1.0;
   out_6664598636645111356[51] = 0;
   out_6664598636645111356[52] = 0;
   out_6664598636645111356[53] = 0;
   out_6664598636645111356[54] = 0;
   out_6664598636645111356[55] = 0;
   out_6664598636645111356[56] = 0;
   out_6664598636645111356[57] = 0;
   out_6664598636645111356[58] = 0;
   out_6664598636645111356[59] = 0;
   out_6664598636645111356[60] = 1.0;
   out_6664598636645111356[61] = 0;
   out_6664598636645111356[62] = 0;
   out_6664598636645111356[63] = 0;
   out_6664598636645111356[64] = 0;
   out_6664598636645111356[65] = 0;
   out_6664598636645111356[66] = 0;
   out_6664598636645111356[67] = 0;
   out_6664598636645111356[68] = 0;
   out_6664598636645111356[69] = 0;
   out_6664598636645111356[70] = 1.0;
   out_6664598636645111356[71] = 0;
   out_6664598636645111356[72] = 0;
   out_6664598636645111356[73] = 0;
   out_6664598636645111356[74] = 0;
   out_6664598636645111356[75] = 0;
   out_6664598636645111356[76] = 0;
   out_6664598636645111356[77] = 0;
   out_6664598636645111356[78] = 0;
   out_6664598636645111356[79] = 0;
   out_6664598636645111356[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1048064928891473471) {
   out_1048064928891473471[0] = state[0];
   out_1048064928891473471[1] = state[1];
   out_1048064928891473471[2] = state[2];
   out_1048064928891473471[3] = state[3];
   out_1048064928891473471[4] = state[4];
   out_1048064928891473471[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1048064928891473471[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1048064928891473471[7] = state[7];
   out_1048064928891473471[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1901252344738326427) {
   out_1901252344738326427[0] = 1;
   out_1901252344738326427[1] = 0;
   out_1901252344738326427[2] = 0;
   out_1901252344738326427[3] = 0;
   out_1901252344738326427[4] = 0;
   out_1901252344738326427[5] = 0;
   out_1901252344738326427[6] = 0;
   out_1901252344738326427[7] = 0;
   out_1901252344738326427[8] = 0;
   out_1901252344738326427[9] = 0;
   out_1901252344738326427[10] = 1;
   out_1901252344738326427[11] = 0;
   out_1901252344738326427[12] = 0;
   out_1901252344738326427[13] = 0;
   out_1901252344738326427[14] = 0;
   out_1901252344738326427[15] = 0;
   out_1901252344738326427[16] = 0;
   out_1901252344738326427[17] = 0;
   out_1901252344738326427[18] = 0;
   out_1901252344738326427[19] = 0;
   out_1901252344738326427[20] = 1;
   out_1901252344738326427[21] = 0;
   out_1901252344738326427[22] = 0;
   out_1901252344738326427[23] = 0;
   out_1901252344738326427[24] = 0;
   out_1901252344738326427[25] = 0;
   out_1901252344738326427[26] = 0;
   out_1901252344738326427[27] = 0;
   out_1901252344738326427[28] = 0;
   out_1901252344738326427[29] = 0;
   out_1901252344738326427[30] = 1;
   out_1901252344738326427[31] = 0;
   out_1901252344738326427[32] = 0;
   out_1901252344738326427[33] = 0;
   out_1901252344738326427[34] = 0;
   out_1901252344738326427[35] = 0;
   out_1901252344738326427[36] = 0;
   out_1901252344738326427[37] = 0;
   out_1901252344738326427[38] = 0;
   out_1901252344738326427[39] = 0;
   out_1901252344738326427[40] = 1;
   out_1901252344738326427[41] = 0;
   out_1901252344738326427[42] = 0;
   out_1901252344738326427[43] = 0;
   out_1901252344738326427[44] = 0;
   out_1901252344738326427[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1901252344738326427[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1901252344738326427[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1901252344738326427[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1901252344738326427[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1901252344738326427[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1901252344738326427[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1901252344738326427[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1901252344738326427[53] = -9.8000000000000007*dt;
   out_1901252344738326427[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1901252344738326427[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1901252344738326427[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1901252344738326427[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1901252344738326427[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1901252344738326427[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1901252344738326427[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1901252344738326427[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1901252344738326427[62] = 0;
   out_1901252344738326427[63] = 0;
   out_1901252344738326427[64] = 0;
   out_1901252344738326427[65] = 0;
   out_1901252344738326427[66] = 0;
   out_1901252344738326427[67] = 0;
   out_1901252344738326427[68] = 0;
   out_1901252344738326427[69] = 0;
   out_1901252344738326427[70] = 1;
   out_1901252344738326427[71] = 0;
   out_1901252344738326427[72] = 0;
   out_1901252344738326427[73] = 0;
   out_1901252344738326427[74] = 0;
   out_1901252344738326427[75] = 0;
   out_1901252344738326427[76] = 0;
   out_1901252344738326427[77] = 0;
   out_1901252344738326427[78] = 0;
   out_1901252344738326427[79] = 0;
   out_1901252344738326427[80] = 1;
}
void h_25(double *state, double *unused, double *out_3036090623932210559) {
   out_3036090623932210559[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8054967264237643249) {
   out_8054967264237643249[0] = 0;
   out_8054967264237643249[1] = 0;
   out_8054967264237643249[2] = 0;
   out_8054967264237643249[3] = 0;
   out_8054967264237643249[4] = 0;
   out_8054967264237643249[5] = 0;
   out_8054967264237643249[6] = 1;
   out_8054967264237643249[7] = 0;
   out_8054967264237643249[8] = 0;
}
void h_24(double *state, double *unused, double *out_810131366787653052) {
   out_810131366787653052[0] = state[4];
   out_810131366787653052[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1479395457646125148) {
   out_1479395457646125148[0] = 0;
   out_1479395457646125148[1] = 0;
   out_1479395457646125148[2] = 0;
   out_1479395457646125148[3] = 0;
   out_1479395457646125148[4] = 1;
   out_1479395457646125148[5] = 0;
   out_1479395457646125148[6] = 0;
   out_1479395457646125148[7] = 0;
   out_1479395457646125148[8] = 0;
   out_1479395457646125148[9] = 0;
   out_1479395457646125148[10] = 0;
   out_1479395457646125148[11] = 0;
   out_1479395457646125148[12] = 0;
   out_1479395457646125148[13] = 0;
   out_1479395457646125148[14] = 1;
   out_1479395457646125148[15] = 0;
   out_1479395457646125148[16] = 0;
   out_1479395457646125148[17] = 0;
}
void h_30(double *state, double *unused, double *out_9203204663903344862) {
   out_9203204663903344862[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5536634305730394622) {
   out_5536634305730394622[0] = 0;
   out_5536634305730394622[1] = 0;
   out_5536634305730394622[2] = 0;
   out_5536634305730394622[3] = 0;
   out_5536634305730394622[4] = 1;
   out_5536634305730394622[5] = 0;
   out_5536634305730394622[6] = 0;
   out_5536634305730394622[7] = 0;
   out_5536634305730394622[8] = 0;
}
void h_26(double *state, double *unused, double *out_4240824666222051630) {
   out_4240824666222051630[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6650273490597852143) {
   out_6650273490597852143[0] = 0;
   out_6650273490597852143[1] = 0;
   out_6650273490597852143[2] = 0;
   out_6650273490597852143[3] = 0;
   out_6650273490597852143[4] = 0;
   out_6650273490597852143[5] = 0;
   out_6650273490597852143[6] = 0;
   out_6650273490597852143[7] = 1;
   out_6650273490597852143[8] = 0;
}
void h_27(double *state, double *unused, double *out_1965582643728635086) {
   out_1965582643728635086[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3313040234546451405) {
   out_3313040234546451405[0] = 0;
   out_3313040234546451405[1] = 0;
   out_3313040234546451405[2] = 0;
   out_3313040234546451405[3] = 1;
   out_3313040234546451405[4] = 0;
   out_3313040234546451405[5] = 0;
   out_3313040234546451405[6] = 0;
   out_3313040234546451405[7] = 0;
   out_3313040234546451405[8] = 0;
}
void h_29(double *state, double *unused, double *out_2240776706013140975) {
   out_2240776706013140975[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5026402961416002438) {
   out_5026402961416002438[0] = 0;
   out_5026402961416002438[1] = 1;
   out_5026402961416002438[2] = 0;
   out_5026402961416002438[3] = 0;
   out_5026402961416002438[4] = 0;
   out_5026402961416002438[5] = 0;
   out_5026402961416002438[6] = 0;
   out_5026402961416002438[7] = 0;
   out_5026402961416002438[8] = 0;
}
void h_28(double *state, double *unused, double *out_3519501387612197369) {
   out_3519501387612197369[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8337942095224018604) {
   out_8337942095224018604[0] = 1;
   out_8337942095224018604[1] = 0;
   out_8337942095224018604[2] = 0;
   out_8337942095224018604[3] = 0;
   out_8337942095224018604[4] = 0;
   out_8337942095224018604[5] = 0;
   out_8337942095224018604[6] = 0;
   out_8337942095224018604[7] = 0;
   out_8337942095224018604[8] = 0;
}
void h_31(double *state, double *unused, double *out_6673540152186232909) {
   out_6673540152186232909[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8024321302360682821) {
   out_8024321302360682821[0] = 0;
   out_8024321302360682821[1] = 0;
   out_8024321302360682821[2] = 0;
   out_8024321302360682821[3] = 0;
   out_8024321302360682821[4] = 0;
   out_8024321302360682821[5] = 0;
   out_8024321302360682821[6] = 0;
   out_8024321302360682821[7] = 0;
   out_8024321302360682821[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_2838428601242523711) {
  err_fun(nom_x, delta_x, out_2838428601242523711);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1569222267276235328) {
  inv_err_fun(nom_x, true_x, out_1569222267276235328);
}
void car_H_mod_fun(double *state, double *out_6664598636645111356) {
  H_mod_fun(state, out_6664598636645111356);
}
void car_f_fun(double *state, double dt, double *out_1048064928891473471) {
  f_fun(state,  dt, out_1048064928891473471);
}
void car_F_fun(double *state, double dt, double *out_1901252344738326427) {
  F_fun(state,  dt, out_1901252344738326427);
}
void car_h_25(double *state, double *unused, double *out_3036090623932210559) {
  h_25(state, unused, out_3036090623932210559);
}
void car_H_25(double *state, double *unused, double *out_8054967264237643249) {
  H_25(state, unused, out_8054967264237643249);
}
void car_h_24(double *state, double *unused, double *out_810131366787653052) {
  h_24(state, unused, out_810131366787653052);
}
void car_H_24(double *state, double *unused, double *out_1479395457646125148) {
  H_24(state, unused, out_1479395457646125148);
}
void car_h_30(double *state, double *unused, double *out_9203204663903344862) {
  h_30(state, unused, out_9203204663903344862);
}
void car_H_30(double *state, double *unused, double *out_5536634305730394622) {
  H_30(state, unused, out_5536634305730394622);
}
void car_h_26(double *state, double *unused, double *out_4240824666222051630) {
  h_26(state, unused, out_4240824666222051630);
}
void car_H_26(double *state, double *unused, double *out_6650273490597852143) {
  H_26(state, unused, out_6650273490597852143);
}
void car_h_27(double *state, double *unused, double *out_1965582643728635086) {
  h_27(state, unused, out_1965582643728635086);
}
void car_H_27(double *state, double *unused, double *out_3313040234546451405) {
  H_27(state, unused, out_3313040234546451405);
}
void car_h_29(double *state, double *unused, double *out_2240776706013140975) {
  h_29(state, unused, out_2240776706013140975);
}
void car_H_29(double *state, double *unused, double *out_5026402961416002438) {
  H_29(state, unused, out_5026402961416002438);
}
void car_h_28(double *state, double *unused, double *out_3519501387612197369) {
  h_28(state, unused, out_3519501387612197369);
}
void car_H_28(double *state, double *unused, double *out_8337942095224018604) {
  H_28(state, unused, out_8337942095224018604);
}
void car_h_31(double *state, double *unused, double *out_6673540152186232909) {
  h_31(state, unused, out_6673540152186232909);
}
void car_H_31(double *state, double *unused, double *out_8024321302360682821) {
  H_31(state, unused, out_8024321302360682821);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
