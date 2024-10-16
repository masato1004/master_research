/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpc_config__stateFcn.c
 *
 * Code generation for function 'nlmpc_config__stateFcn'
 *
 */

/* Include files */
#include "nlmpc_config__stateFcn.h"
#include "expm.h"
#include "makima.h"
#include "mtimes.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_emxutil.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "pinv.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_c_struct_T
#define typedef_c_struct_T
typedef struct {
  real_T A[196];
} c_struct_T;
#endif /* typedef_c_struct_T */

#ifndef typedef_anonymous_function
#define typedef_anonymous_function
typedef struct {
  c_struct_T workspace;
} anonymous_function;
#endif /* typedef_anonymous_function */

/* Variable Definitions */
static boolean_T A_not_empty;

static boolean_T last_param_not_empty;

/* Function Definitions */
void nlmpc_config__stateFcn(const real_T x[14], const real_T u[4],
                            const real_T p[109], real_T dxdt[14])
{
  static const real_T b_dv[196] = {0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   -2056.1875,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   -57.291666666666664,
                                   750.0,
                                   625.0,
                                   3.4013605442176864,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   31.25,
                                   -7500.0,
                                   0.0,
                                   21.335807050092761,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   26.041666666666668,
                                   0.0,
                                   -7375.0,
                                   -24.737167594310449,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   5.104166666666667,
                                   768.409090909091,
                                   -890.909090909091,
                                   -57.12121212121211,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   -9.3270833333333325,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   -13.541666666666666,
                                   175.0,
                                   150.0,
                                   0.95856524427952983,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   7.291666666666667,
                                   -200.0,
                                   0.0,
                                   4.9783549783549779,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   6.25,
                                   0.0,
                                   -175.0,
                                   -5.9369202226345079,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.4384469696969697,
                                   179.29545454545456,
                                   -213.81818181818184,
                                   -13.563360881542698,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0};
  static const real_T dv2[112] = {0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    1233.7083333333333,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    822.47916666666663,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    6750.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  6750.0, 0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    4.916666666666667,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    4.4104166666666664,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    25.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  0.0,    0.0,
                                  25.0,   0.0,
                                  0.0,    0.0};
  static const real_T dv1[56] = {0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.66115702479338834,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.66115702479338834,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0010416666666666667,
                                 -0.025,
                                 0.0,
                                 0.00071119356833642539,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0010416666666666667,
                                 0.0,
                                 -0.025,
                                 -0.00098948670377241787,
                                 0.0,
                                 0.0};
  static real_T A[196];
  static real_T E[112];
  static real_T B[56];
  static real_T b_current_dis[20];
  static real_T b_current_mileage_f[20];
  static real_T b_current_mileage_r[20];
  static real_T b_current_wheel_traj_f[20];
  static real_T b_current_wheel_traj_r[20];
  static real_T b_first_states[14];
  static real_T b_first_d[8];
  static real_T b_param_flag;
  __m128d r;
  __m128d r1;
  emxArray_boolean_T *b_x;
  real_T a[196];
  real_T y[196];
  real_T c_varargout_1[112];
  real_T d_varargout_1[112];
  real_T b_varargout_1[56];
  real_T varargout_1[56];
  real_T delta_x[14];
  real_T b_current_d[8];
  real_T c_y[3];
  real_T d_y[2];
  real_T temp_gradient_f[2];
  real_T temp_gradient_r[2];
  real_T duration_length_f;
  real_T temp_duration_f_tmp;
  int32_T i;
  int32_T k;
  boolean_T *x_data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  Copyright 2023 The MathWorks, Inc. */
  /*     %% Vehicle parameter */
  /*  control period */
  /*  [kg]      body mass */
  /*  [kg]      wheel mass */
  /*  [N/m]     front spring stiffness */
  /*  [N/m]     rear spring stiffness */
  /*  [N/m]     tire stiffness */
  /*  [N/(m/s)] front damping */
  /*  [N/(m/s)] rear damping */
  /*  [N/(m/s)] tire damping */
  /*  wheel base */
  /*  [m]       front length */
  /*  [m]       rear length */
  /*  [m]   radius of wheel */
  /*  [kgm^2]   body inertia moment */
  /*  [kgm^2]   wheel inertia moment */
  if (!A_not_empty) {
    anonymous_function disc_func;
    real_T b[196];
    memcpy(&A[0], &b_dv[0], 196U * sizeof(real_T));
    A_not_empty = true;
    memcpy(&B[0], &dv1[0], 56U * sizeof(real_T));
    memcpy(&E[0], &dv2[0], 112U * sizeof(real_T));
    /*  discretization */
    for (k = 0; k <= 194; k += 2) {
      r = _mm_loadu_pd(&A[k]);
      _mm_storeu_pd(&disc_func.workspace.A[k], r);
      _mm_storeu_pd(&y[k], _mm_mul_pd(r, _mm_set1_pd(0.01)));
    }
    expm(y, A);
    pinv(disc_func.workspace.A, a);
    for (k = 0; k <= 194; k += 2) {
      r = _mm_loadu_pd(&a[k]);
      _mm_storeu_pd(&a[k], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
      r = _mm_loadu_pd(&disc_func.workspace.A[k]);
      _mm_storeu_pd(&y[k], _mm_mul_pd(r, _mm_set1_pd(0.0)));
    }
    expm(y, b);
    mtimes(a, b, y);
    b_mtimes(y, B, varargout_1);
    pinv(disc_func.workspace.A, a);
    for (k = 0; k <= 194; k += 2) {
      r = _mm_loadu_pd(&a[k]);
      _mm_storeu_pd(&a[k], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
      r = _mm_loadu_pd(&disc_func.workspace.A[k]);
      _mm_storeu_pd(&y[k], _mm_mul_pd(r, _mm_set1_pd(0.01)));
    }
    expm(y, b);
    mtimes(a, b, y);
    b_mtimes(y, B, b_varargout_1);
    for (k = 0; k <= 54; k += 2) {
      r = _mm_loadu_pd(&varargout_1[k]);
      r1 = _mm_loadu_pd(&b_varargout_1[k]);
      _mm_storeu_pd(&B[k], _mm_sub_pd(r, r1));
    }
    pinv(disc_func.workspace.A, a);
    for (k = 0; k <= 194; k += 2) {
      r = _mm_loadu_pd(&a[k]);
      _mm_storeu_pd(&a[k], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
      r = _mm_loadu_pd(&disc_func.workspace.A[k]);
      _mm_storeu_pd(&y[k], _mm_mul_pd(r, _mm_set1_pd(0.0)));
    }
    expm(y, b);
    mtimes(a, b, y);
    c_mtimes(y, E, c_varargout_1);
    pinv(disc_func.workspace.A, a);
    for (k = 0; k <= 194; k += 2) {
      r = _mm_loadu_pd(&a[k]);
      _mm_storeu_pd(&a[k], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
      r = _mm_loadu_pd(&disc_func.workspace.A[k]);
      _mm_storeu_pd(&y[k], _mm_mul_pd(r, _mm_set1_pd(0.01)));
    }
    expm(y, b);
    mtimes(a, b, y);
    c_mtimes(y, E, d_varargout_1);
    for (k = 0; k <= 110; k += 2) {
      r = _mm_loadu_pd(&c_varargout_1[k]);
      r1 = _mm_loadu_pd(&d_varargout_1[k]);
      _mm_storeu_pd(&E[k], _mm_sub_pd(r, r1));
    }
  }
  /*     %% set parameters through horizon */
  memcpy(&b_current_d[0], &p[0], 8U * sizeof(real_T));
  if (!last_param_not_empty) {
    b_param_flag = p[108];
    /*  number of iteration */
    last_param_not_empty = true;
    memcpy(&b_current_d[0], &p[0], 8U * sizeof(real_T));
    /*  current_disturbance */
    /*  wheel trajectory constraints */
    memcpy(&b_current_dis[0], &p[8], 20U * sizeof(real_T));
    memcpy(&b_current_mileage_f[0], &p[28], 20U * sizeof(real_T));
    memcpy(&b_current_mileage_r[0], &p[48], 20U * sizeof(real_T));
    memcpy(&b_current_wheel_traj_f[0], &p[68], 20U * sizeof(real_T));
    memcpy(&b_current_wheel_traj_r[0], &p[88], 20U * sizeof(real_T));
    /*  wheel trajectory constraints */
    memcpy(&b_first_states[0], &x[0], 14U * sizeof(real_T));
    memcpy(&b_first_d[0], &b_current_d[0], 8U * sizeof(real_T));
  } else if (b_param_flag != p[108]) {
    b_param_flag = p[108];
    memcpy(&b_current_d[0], &p[0], 8U * sizeof(real_T));
    memcpy(&b_current_dis[0], &p[8], 20U * sizeof(real_T));
    memcpy(&b_current_mileage_f[0], &p[28], 20U * sizeof(real_T));
    memcpy(&b_current_mileage_r[0], &p[48], 20U * sizeof(real_T));
    memcpy(&b_current_wheel_traj_f[0], &p[68], 20U * sizeof(real_T));
    memcpy(&b_current_wheel_traj_r[0], &p[88], 20U * sizeof(real_T));
    memcpy(&b_first_states[0], &x[0], 14U * sizeof(real_T));
    memcpy(&b_first_d[0], &b_current_d[0], 8U * sizeof(real_T));
  } else {
    boolean_T b_y;
    boolean_T exitg1;
    emxInit_boolean_T(&b_x);
    k = b_x->size[0];
    b_x->size[0] = 14;
    emxEnsureCapacity_boolean_T(b_x, k);
    x_data = b_x->data;
    for (k = 0; k < 14; k++) {
      x_data[k] = (muDoubleScalarRound(x[k] * 1.0E+10) !=
                   muDoubleScalarRound(b_first_states[k] * 1.0E+10));
    }
    b_y = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= 13)) {
      if (x_data[k]) {
        b_y = true;
        exitg1 = true;
      } else {
        k++;
      }
    }
    emxFree_boolean_T(&b_x);
    if (b_y) {
      for (k = 0; k <= 12; k += 2) {
        r = _mm_loadu_pd(&b_first_states[k]);
        _mm_storeu_pd(&delta_x[k], _mm_sub_pd(_mm_loadu_pd(&x[k]), r));
      }
      real_T temp_duration_f[3];
      real_T e_y[2];
      b_current_d[0] = b_first_d[0] + makima(b_current_mileage_f, b_current_dis,
                                             delta_x[5] * 0.275);
      /*  x_disf */
      b_current_d[1] = b_first_d[1] + makima(b_current_mileage_r, b_current_dis,
                                             delta_x[6] * 0.275);
      /*  x_disr */
      b_current_d[2] =
          muDoubleScalarRound(makima(b_current_dis, b_current_wheel_traj_f,
                                     b_current_d[0] - b_first_d[0]) *
                              100000.0) *
          1.0E-5;
      b_current_d[3] =
          muDoubleScalarRound(makima(b_current_dis, b_current_wheel_traj_r,
                                     b_current_d[1] - b_first_d[1]) *
                              100000.0) *
          1.0E-5;
      duration_length_f = 0.01 * x[12] * 0.275 / 2.0;
      temp_duration_f_tmp = b_current_d[0] - b_first_d[0];
      temp_duration_f[0] = temp_duration_f_tmp - duration_length_f;
      temp_duration_f[1] = temp_duration_f_tmp;
      temp_duration_f[2] = temp_duration_f_tmp + duration_length_f;
      b_makima(b_current_dis, b_current_wheel_traj_f, temp_duration_f, c_y);
      c_y[0] = muDoubleScalarRound(c_y[0] * 100000.0);
      c_y[1] = muDoubleScalarRound(c_y[1] * 100000.0);
      c_y[2] = muDoubleScalarRound(c_y[2] * 100000.0);
      d_y[0] = temp_duration_f_tmp - temp_duration_f[0];
      d_y[1] = temp_duration_f[2] - temp_duration_f_tmp;
      r = _mm_loadu_pd(&c_y[0]);
      r1 = _mm_set1_pd(1.0E-5);
      _mm_storeu_pd(&c_y[0], _mm_mul_pd(r, r1));
      c_y[2] *= 1.0E-5;
      e_y[0] = c_y[1] - c_y[0];
      e_y[1] = c_y[2] - c_y[1];
      temp_gradient_f[0] = sumColumnB(d_y) / 2.0;
      temp_gradient_f[1] = sumColumnB(e_y) / 2.0;
      duration_length_f = 0.01 * x[13] * 0.275 / 2.0;
      temp_duration_f_tmp = b_current_d[1] - b_first_d[1];
      temp_duration_f[0] = temp_duration_f_tmp - duration_length_f;
      temp_duration_f[1] = temp_duration_f_tmp;
      temp_duration_f[2] = temp_duration_f_tmp + duration_length_f;
      b_makima(b_current_dis, b_current_wheel_traj_r, temp_duration_f, c_y);
      c_y[0] = muDoubleScalarRound(c_y[0] * 100000.0);
      c_y[1] = muDoubleScalarRound(c_y[1] * 100000.0);
      c_y[2] = muDoubleScalarRound(c_y[2] * 100000.0);
      d_y[0] = temp_duration_f_tmp - temp_duration_f[0];
      d_y[1] = temp_duration_f[2] - temp_duration_f_tmp;
      r = _mm_loadu_pd(&c_y[0]);
      _mm_storeu_pd(&c_y[0], _mm_mul_pd(r, r1));
      c_y[2] *= 1.0E-5;
      e_y[0] = c_y[1] - c_y[0];
      e_y[1] = c_y[2] - c_y[1];
      temp_gradient_r[0] = sumColumnB(d_y) / 2.0;
      temp_gradient_r[1] = sumColumnB(e_y) / 2.0;
      r = _mm_loadu_pd(&temp_gradient_f[0]);
      _mm_storeu_pd(&d_y[0], _mm_mul_pd(r, r));
      duration_length_f = sumColumnB(d_y);
      b_current_d[4] =
          x[12] * 0.275 *
          (temp_gradient_f[0] * temp_gradient_f[0] / duration_length_f);
      r = _mm_loadu_pd(&temp_gradient_r[0]);
      _mm_storeu_pd(&d_y[0], _mm_mul_pd(r, r));
      temp_duration_f_tmp = sumColumnB(d_y);
      b_current_d[5] =
          x[13] * 0.275 *
          (temp_gradient_r[0] * temp_gradient_r[0] / temp_duration_f_tmp);
      b_current_d[6] =
          x[12] * 0.275 *
          (temp_gradient_f[1] * temp_gradient_f[1] / duration_length_f);
      b_current_d[7] =
          x[13] * 0.275 *
          (temp_gradient_r[1] * temp_gradient_r[1] / temp_duration_f_tmp);
      /*  current_d(5) */
      /*  current_d(6) */
      /*  current_d(7) */
      /*  current_d(8) */
    }
  }
  /*  avoid nan */
  for (k = 0; k < 14; k++) {
    duration_length_f = 0.0;
    for (i = 0; i < 14; i++) {
      duration_length_f += A[k + 14 * i] * x[i];
    }
    temp_duration_f_tmp = 0.0;
    for (i = 0; i < 8; i++) {
      temp_duration_f_tmp += E[k + 14 * i] * b_current_d[i];
    }
    dxdt[k] = (duration_length_f +
               (((B[k] * u[0] + B[k + 14] * u[1]) + B[k + 28] * u[2]) +
                B[k + 42] * u[3])) +
              temp_duration_f_tmp;
  }
  /*  + G*g; */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

void nlmpc_config__stateFcn_init(void)
{
  last_param_not_empty = false;
  A_not_empty = false;
}

/* End of code generation (nlmpc_config__stateFcn.c) */
