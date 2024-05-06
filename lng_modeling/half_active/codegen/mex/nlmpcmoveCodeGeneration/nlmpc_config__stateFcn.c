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
#include "makima.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_emxutil.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static boolean_T A_not_empty;

static boolean_T last_param_not_empty;

/* Function Definitions */
void nlmpc_config__stateFcn(const real_T x[14], const real_T u[4],
                            const real_T p[309], real_T dxdt[14])
{
  static real_T A[196];
  static real_T E[112];
  static real_T b_current_dis[60];
  static real_T b_current_mileage_f[60];
  static real_T b_current_mileage_r[60];
  static real_T b_current_wheel_traj_f[60];
  static real_T b_current_wheel_traj_r[60];
  static real_T B[56];
  static real_T b_first_states[14];
  static real_T b_first_d[8];
  static real_T last_d[8];
  static real_T b_param_flag;
  emxArray_boolean_T *b_x;
  real_T check_x[14];
  real_T delta_x[14];
  real_T b_current_d[8];
  real_T b_y[3];
  real_T c_y[2];
  real_T temp_gradient_f[2];
  real_T temp_gradient_r[2];
  real_T duration_length_f;
  real_T temp_duration_f_tmp;
  int32_T i;
  int32_T k;
  boolean_T *x_data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  /*  In a 2D environment with standard XY axis, the vehicle is a circular disc
   */
  /*  (20 meters in diamater).  Two thrusts are to the left and right of the */
  /*  center.  Tilting (theta) is defined as positive to left and negative to */
  /*  the right (0 means robot is vertical). */
  /*  */
  /*  x: (1) x position of the center of gravity in m */
  /*     (2) y position of the center of gravity in m */
  /*     (3) theta (tilt with respect to the center of gravity) in radian */
  /*     (4) dxdt */
  /*     (5) dydt */
  /*     (6) dthetadt */
  /*  */
  /*  u: (1) thrust on the left, in Newton */
  /*     (2) thrust on the right, in Newton */
  /*  */
  /*  The continuous-time model is valid only if the vehicle above or at the */
  /*  ground (y>=10). */
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
    memcpy(&A[0], &dv[0], 196U * sizeof(real_T));
    A_not_empty = true;
    memcpy(&B[0], &dv1[0], 56U * sizeof(real_T));
    memcpy(&E[0], &dv2[0], 112U * sizeof(real_T));
    /*  discretization */
    /*  A = expm(A.*dt); */
    /*  B = disc_func(dt,B) - disc_func(0,B); */
    /*  E = disc_func(dt,E) - disc_func(0,E); */
  }
  /*     %% set parameters through horizon */
  memcpy(&b_current_d[0], &p[0], 8U * sizeof(real_T));
  if (!last_param_not_empty) {
    b_param_flag = p[308];
    last_param_not_empty = true;
    memcpy(&b_current_d[0], &p[0], 8U * sizeof(real_T));
    memcpy(&b_current_dis[0], &p[8], 60U * sizeof(real_T));
    memcpy(&b_current_mileage_f[0], &p[68], 60U * sizeof(real_T));
    memcpy(&b_current_mileage_r[0], &p[128], 60U * sizeof(real_T));
    memcpy(&b_current_wheel_traj_f[0], &p[188], 60U * sizeof(real_T));
    memcpy(&b_current_wheel_traj_r[0], &p[248], 60U * sizeof(real_T));
    memcpy(&b_first_states[0], &x[0], 14U * sizeof(real_T));
    memcpy(&b_first_d[0], &b_current_d[0], 8U * sizeof(real_T));
    memcpy(&last_d[0], &b_current_d[0], 8U * sizeof(real_T));
  } else if (b_param_flag != p[308]) {
    b_param_flag = p[308];
    memcpy(&b_current_d[0], &p[0], 8U * sizeof(real_T));
    memcpy(&b_current_dis[0], &p[8], 60U * sizeof(real_T));
    memcpy(&b_current_mileage_f[0], &p[68], 60U * sizeof(real_T));
    memcpy(&b_current_mileage_r[0], &p[128], 60U * sizeof(real_T));
    memcpy(&b_current_wheel_traj_f[0], &p[188], 60U * sizeof(real_T));
    memcpy(&b_current_wheel_traj_r[0], &p[248], 60U * sizeof(real_T));
    memcpy(&b_first_states[0], &x[0], 14U * sizeof(real_T));
    memcpy(&b_first_d[0], &b_current_d[0], 8U * sizeof(real_T));
    memcpy(&last_d[0], &b_current_d[0], 8U * sizeof(real_T));
  } else {
    boolean_T exitg1;
    boolean_T y;
    emxInit_boolean_T(&b_x);
    k = b_x->size[0];
    b_x->size[0] = 14;
    emxEnsureCapacity_boolean_T(b_x, k);
    x_data = b_x->data;
    for (k = 0; k < 14; k++) {
      duration_length_f = muDoubleScalarRound(x[k] * 1.0E+10);
      x_data[k] = (duration_length_f != duration_length_f);
    }
    y = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= 13)) {
      if (x_data[k]) {
        y = true;
        exitg1 = true;
      } else {
        k++;
      }
    }
    emxFree_boolean_T(&b_x);
    if (y) {
      __m128d r;
      for (k = 0; k <= 12; k += 2) {
        r = _mm_loadu_pd(&b_first_states[k]);
        _mm_storeu_pd(&delta_x[k], _mm_sub_pd(_mm_loadu_pd(&x[k]), r));
      }
      __m128d r1;
      real_T temp_duration_f[3];
      real_T d_y[2];
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
      duration_length_f = 0.001 * x[12] * 0.275 / 2.0;
      temp_duration_f_tmp = b_current_d[0] - b_first_d[0];
      temp_duration_f[0] = temp_duration_f_tmp - duration_length_f;
      temp_duration_f[1] = temp_duration_f_tmp;
      temp_duration_f[2] = temp_duration_f_tmp + duration_length_f;
      b_makima(b_current_dis, b_current_wheel_traj_f, temp_duration_f, b_y);
      b_y[0] = muDoubleScalarRound(b_y[0] * 100000.0);
      b_y[1] = muDoubleScalarRound(b_y[1] * 100000.0);
      b_y[2] = muDoubleScalarRound(b_y[2] * 100000.0);
      c_y[0] = temp_duration_f_tmp - temp_duration_f[0];
      c_y[1] = temp_duration_f[2] - temp_duration_f_tmp;
      r = _mm_loadu_pd(&b_y[0]);
      r1 = _mm_set1_pd(1.0E-5);
      _mm_storeu_pd(&b_y[0], _mm_mul_pd(r, r1));
      b_y[2] *= 1.0E-5;
      d_y[0] = b_y[1] - b_y[0];
      d_y[1] = b_y[2] - b_y[1];
      temp_gradient_f[0] = sumColumnB(c_y) / 2.0;
      temp_gradient_f[1] = sumColumnB(d_y) / 2.0;
      duration_length_f = 0.001 * x[13] * 0.275 / 2.0;
      temp_duration_f_tmp = b_current_d[1] - b_first_d[1];
      temp_duration_f[0] = temp_duration_f_tmp - duration_length_f;
      temp_duration_f[1] = temp_duration_f_tmp;
      temp_duration_f[2] = temp_duration_f_tmp + duration_length_f;
      b_makima(b_current_dis, b_current_wheel_traj_r, temp_duration_f, b_y);
      b_y[0] = muDoubleScalarRound(b_y[0] * 100000.0);
      b_y[1] = muDoubleScalarRound(b_y[1] * 100000.0);
      b_y[2] = muDoubleScalarRound(b_y[2] * 100000.0);
      c_y[0] = temp_duration_f_tmp - temp_duration_f[0];
      c_y[1] = temp_duration_f[2] - temp_duration_f_tmp;
      r = _mm_loadu_pd(&b_y[0]);
      _mm_storeu_pd(&b_y[0], _mm_mul_pd(r, r1));
      b_y[2] *= 1.0E-5;
      d_y[0] = b_y[1] - b_y[0];
      d_y[1] = b_y[2] - b_y[1];
      temp_gradient_r[0] = sumColumnB(c_y) / 2.0;
      temp_gradient_r[1] = sumColumnB(d_y) / 2.0;
      r = _mm_loadu_pd(&temp_gradient_f[0]);
      _mm_storeu_pd(&c_y[0], _mm_mul_pd(r, r));
      duration_length_f = sumColumnB(c_y);
      b_current_d[4] =
          x[12] * 0.275 *
          (temp_gradient_f[0] * temp_gradient_f[0] / duration_length_f);
      r = _mm_loadu_pd(&temp_gradient_r[0]);
      _mm_storeu_pd(&c_y[0], _mm_mul_pd(r, r));
      temp_duration_f_tmp = sumColumnB(c_y);
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
  memset(&check_x[0], 0, 8U * sizeof(real_T));
  check_x[8] = -1.0;
  check_x[9] = -1.0;
  check_x[10] = -1.0;
  check_x[11] = 0.0;
  check_x[12] =
      -muDoubleScalarSin(muDoubleScalarAtan(b_current_d[6] / b_current_d[4])) /
      0.275;
  check_x[13] =
      -muDoubleScalarSin(muDoubleScalarAtan(b_current_d[7] / b_current_d[5])) /
      0.275;
  if (!(last_d[0] != b_current_d[0])) {
    check_x[12] = 0.0;
  }
  if (!(last_d[1] != b_current_d[1])) {
    check_x[13] = 0.0;
  }
  memcpy(&last_d[0], &b_current_d[0], 8U * sizeof(real_T));
  /*  G(isnan(G)) = 0; */
  /*  G = disc_func(dt,G) - disc_func(0,G); */
  for (k = 0; k < 14; k++) {
    duration_length_f = 0.0;
    for (i = 0; i < 14; i++) {
      duration_length_f += A[k + 14 * i] * x[i];
    }
    temp_duration_f_tmp = 0.0;
    for (i = 0; i < 8; i++) {
      temp_duration_f_tmp += E[k + 14 * i] * b_current_d[i];
    }
    dxdt[k] = ((duration_length_f +
                (((B[k] * u[0] + B[k + 14] * u[1]) + B[k + 28] * u[2]) +
                 B[k + 42] * u[3])) +
               temp_duration_f_tmp) +
              check_x[k] * 9.80665;
  }
  /*  dxdt = x + dxdt*dt; */
  /*  if sum(isnan(current_d)) ~= 0 */
  /*      disp(current_d) */
  /*  end */
  /*  dxdt(isnan(dxdt)) = 0; */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

void nlmpc_config__stateFcn_init(void)
{
  last_param_not_empty = false;
  A_not_empty = false;
}

/* End of code generation (nlmpc_config__stateFcn.c) */
