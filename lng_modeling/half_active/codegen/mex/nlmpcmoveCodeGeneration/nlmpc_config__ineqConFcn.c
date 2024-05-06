/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpc_config__ineqConFcn.c
 *
 * Code generation for function 'nlmpc_config__ineqConFcn'
 *
 */

/* Include files */
#include "nlmpc_config__ineqConFcn.h"
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
static boolean_T b_A_not_empty;

static real_T ref[14];

static real_T param_flag;

static boolean_T b_last_param_not_empty;

static real_T first_states[14];

static real_T first_d[8];

static real_T current_d[8];

static real_T current_dis[60];

static real_T current_mileage_f[60];

static real_T current_mileage_r[60];

static real_T current_wheel_traj_f[60];

static real_T current_wheel_traj_r[60];

/* Function Definitions */
void b_nlmpc_config__ineqConFcn(const real_T x[14], const real_T p[323],
                                real_T cineq[18])
{
  int32_T i;
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
  if (!b_A_not_empty) {
    b_A_not_empty = true;
    /*  discretization */
    /*  disc_func = @(tau,Mat) (-pinv(A)*expm(A.*(dt-tau)))*Mat; */
    /*  A = expm(A.*dt); */
    /*  B = disc_func(dt,B) - disc_func(0,B); */
    /*  E = disc_func(dt,E) - disc_func(0,E); */
  }
  /*     %% set parameters through horizon */
  /*  if stage == 1 */
  /*      cineq = -1; */
  /*      last_param = p; */
  /*      current_d = p(1:8); */
  /*      current_dis = p(9:8+pHorizon+10); */
  /*      current_mileage_f = p(9+pHorizon+10:8+(pHorizon+10)*2); */
  /*      current_mileage_r = p(9+(pHorizon+10)*2:8+(pHorizon+10)*3); */
  /*      current_wheel_traj_f = [p(9+(pHorizon+10)*3:8+(pHorizon+10)*4)]; */
  /*      current_wheel_traj_r = [p(9+(pHorizon+10)*4:8+(pHorizon+10)*5)]; */
  /*      first_states = x; */
  /*      first_d = current_d; */
  /*      last_d = current_d; */
  /*      last_state = x; */
  /*  else */
  if (!b_last_param_not_empty) {
    for (i = 0; i < 18; i++) {
      cineq[i] = -1.0;
    }
    memcpy(&ref[0], &p[309], 14U * sizeof(real_T));
    param_flag = p[308];
    b_last_param_not_empty = true;
    memcpy(&current_d[0], &p[0], 8U * sizeof(real_T));
    memcpy(&current_dis[0], &p[8], 60U * sizeof(real_T));
    memcpy(&current_mileage_f[0], &p[68], 60U * sizeof(real_T));
    memcpy(&current_mileage_r[0], &p[128], 60U * sizeof(real_T));
    memcpy(&current_wheel_traj_f[0], &p[188], 60U * sizeof(real_T));
    memcpy(&current_wheel_traj_r[0], &p[248], 60U * sizeof(real_T));
    memcpy(&first_states[0], &x[0], 14U * sizeof(real_T));
    memcpy(&first_d[0], &current_d[0], 8U * sizeof(real_T));
    /*  first_d = current_d; */
    /*  last_d = current_d; */
  } else if (param_flag != p[308]) {
    for (i = 0; i < 18; i++) {
      cineq[i] = -1.0;
    }
    memcpy(&ref[0], &p[309], 14U * sizeof(real_T));
    param_flag = p[308];
    memcpy(&current_d[0], &p[0], 8U * sizeof(real_T));
    memcpy(&current_dis[0], &p[8], 60U * sizeof(real_T));
    memcpy(&current_mileage_f[0], &p[68], 60U * sizeof(real_T));
    memcpy(&current_mileage_r[0], &p[128], 60U * sizeof(real_T));
    memcpy(&current_wheel_traj_f[0], &p[188], 60U * sizeof(real_T));
    memcpy(&current_wheel_traj_r[0], &p[248], 60U * sizeof(real_T));
    memcpy(&first_states[0], &x[0], 14U * sizeof(real_T));
    memcpy(&first_d[0], &current_d[0], 8U * sizeof(real_T));
    /*  first_d = current_d; */
    /*  last_d = current_d; */
  } else {
    for (i = 0; i < 18; i++) {
      cineq[i] = -1.0;
    }
  }
  /*  G = [                                     0; */
  /*                                              0; */
  /*                                              -1; */
  /*                                              0; */
  /*      -sin(atan(current_d(7)/current_d(5)))/r; */
  /*      -sin(atan(current_d(8)/current_d(6)))/r]; */
  /*  G = disc_func(dt,G) - disc_func(0,G); */
  /*  if any(u~=0) */
  /*      disp(u) */
  /*  end */
  /*  dxdt = A*x + E*current_d + G*g; */
  /*  dxdt = A*x + B*u + E*current_d + G*g; */
  /*  next_state = x + dxdt*dt; */
  /*  last_d = current_d; */
  /*  if any(cineq == nan) */
  /*      cineq = 0; */
  /*  end */
  /*  if any(cineq > 0) */
  /*      stage */
  /*      next_state(8) */
  /*      x(8) */
  /*  end */
  /*  last_state = x; */
  /*  end */
}

void nlmpc_config__ineqConFcn(const real_T x[14], const real_T u[4],
                              const real_T e[3], const real_T p[323],
                              real_T cineq[18])
{
  emxArray_boolean_T *b_x;
  real_T delta_x[14];
  real_T b_y[3];
  real_T c_y[2];
  real_T temp_gradient_f[2];
  real_T temp_gradient_r[2];
  int32_T k;
  boolean_T *x_data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
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
  if (!b_A_not_empty) {
    b_A_not_empty = true;
    /*  discretization */
    /*  disc_func = @(tau,Mat) (-pinv(A)*expm(A.*(dt-tau)))*Mat; */
    /*  A = expm(A.*dt); */
    /*  B = disc_func(dt,B) - disc_func(0,B); */
    /*  E = disc_func(dt,E) - disc_func(0,E); */
  }
  /*     %% set parameters through horizon */
  /*  if stage == 1 */
  /*      cineq = -1; */
  /*      last_param = p; */
  /*      current_d = p(1:8); */
  /*      current_dis = p(9:8+pHorizon+10); */
  /*      current_mileage_f = p(9+pHorizon+10:8+(pHorizon+10)*2); */
  /*      current_mileage_r = p(9+(pHorizon+10)*2:8+(pHorizon+10)*3); */
  /*      current_wheel_traj_f = [p(9+(pHorizon+10)*3:8+(pHorizon+10)*4)]; */
  /*      current_wheel_traj_r = [p(9+(pHorizon+10)*4:8+(pHorizon+10)*5)]; */
  /*      first_states = x; */
  /*      first_d = current_d; */
  /*      last_d = current_d; */
  /*      last_state = x; */
  /*  else */
  if (!b_last_param_not_empty) {
    for (k = 0; k < 18; k++) {
      cineq[k] = -1.0;
    }
    memcpy(&ref[0], &p[309], 14U * sizeof(real_T));
    param_flag = p[308];
    b_last_param_not_empty = true;
    memcpy(&current_d[0], &p[0], 8U * sizeof(real_T));
    memcpy(&current_dis[0], &p[8], 60U * sizeof(real_T));
    memcpy(&current_mileage_f[0], &p[68], 60U * sizeof(real_T));
    memcpy(&current_mileage_r[0], &p[128], 60U * sizeof(real_T));
    memcpy(&current_wheel_traj_f[0], &p[188], 60U * sizeof(real_T));
    memcpy(&current_wheel_traj_r[0], &p[248], 60U * sizeof(real_T));
    memcpy(&first_states[0], &x[0], 14U * sizeof(real_T));
    memcpy(&first_d[0], &current_d[0], 8U * sizeof(real_T));
    /*  first_d = current_d; */
    /*  last_d = current_d; */
  } else if (param_flag != p[308]) {
    for (k = 0; k < 18; k++) {
      cineq[k] = -1.0;
    }
    memcpy(&ref[0], &p[309], 14U * sizeof(real_T));
    param_flag = p[308];
    memcpy(&current_d[0], &p[0], 8U * sizeof(real_T));
    memcpy(&current_dis[0], &p[8], 60U * sizeof(real_T));
    memcpy(&current_mileage_f[0], &p[68], 60U * sizeof(real_T));
    memcpy(&current_mileage_r[0], &p[128], 60U * sizeof(real_T));
    memcpy(&current_wheel_traj_f[0], &p[188], 60U * sizeof(real_T));
    memcpy(&current_wheel_traj_r[0], &p[248], 60U * sizeof(real_T));
    memcpy(&first_states[0], &x[0], 14U * sizeof(real_T));
    memcpy(&first_d[0], &current_d[0], 8U * sizeof(real_T));
    /*  first_d = current_d; */
    /*  last_d = current_d; */
  } else {
    boolean_T exitg1;
    boolean_T y;
    emxInit_boolean_T(&b_x);
    k = b_x->size[0];
    b_x->size[0] = 14;
    emxEnsureCapacity_boolean_T(b_x, k);
    x_data = b_x->data;
    for (k = 0; k < 14; k++) {
      x_data[k] = (muDoubleScalarRound(x[k] * 1.0E+10) !=
                   muDoubleScalarRound(first_states[k] * 1.0E+10));
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
        r = _mm_loadu_pd(&first_states[k]);
        _mm_storeu_pd(&delta_x[k], _mm_sub_pd(_mm_loadu_pd(&x[k]), r));
      }
      __m128d r1;
      real_T temp_duration_f[3];
      real_T d_y[2];
      real_T duration_length_f;
      real_T temp_duration_f_tmp;
      current_d[0] = first_d[0] +
                     makima(current_mileage_f, current_dis, delta_x[5] * 0.275);
      /*  x_disf */
      current_d[1] = first_d[1] +
                     makima(current_mileage_r, current_dis, delta_x[6] * 0.275);
      /*  x_disr */
      current_d[2] =
          muDoubleScalarRound(makima(current_dis, current_wheel_traj_f,
                                     current_d[0] - first_d[0]) *
                              100000.0) *
          1.0E-5;
      current_d[3] =
          muDoubleScalarRound(makima(current_dis, current_wheel_traj_r,
                                     current_d[1] - first_d[1]) *
                              100000.0) *
          1.0E-5;
      duration_length_f = 0.001 * x[12] * 0.275 / 2.0;
      temp_duration_f_tmp = current_d[0] - first_d[0];
      temp_duration_f[0] = temp_duration_f_tmp - duration_length_f;
      temp_duration_f[1] = temp_duration_f_tmp;
      temp_duration_f[2] = temp_duration_f_tmp + duration_length_f;
      b_makima(current_dis, current_wheel_traj_f, temp_duration_f, b_y);
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
      temp_duration_f_tmp = current_d[1] - first_d[1];
      temp_duration_f[0] = temp_duration_f_tmp - duration_length_f;
      temp_duration_f[1] = temp_duration_f_tmp;
      temp_duration_f[2] = temp_duration_f_tmp + duration_length_f;
      b_makima(current_dis, current_wheel_traj_r, temp_duration_f, b_y);
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
      current_d[4] =
          x[12] * 0.275 *
          (temp_gradient_f[0] * temp_gradient_f[0] / duration_length_f);
      r = _mm_loadu_pd(&temp_gradient_r[0]);
      _mm_storeu_pd(&c_y[0], _mm_mul_pd(r, r));
      temp_duration_f_tmp = sumColumnB(c_y);
      current_d[5] =
          x[13] * 0.275 *
          (temp_gradient_r[0] * temp_gradient_r[0] / temp_duration_f_tmp);
      current_d[6] =
          x[12] * 0.275 *
          (temp_gradient_f[1] * temp_gradient_f[1] / duration_length_f);
      current_d[7] =
          x[13] * 0.275 *
          (temp_gradient_r[1] * temp_gradient_r[1] / temp_duration_f_tmp);
      /*   - e(1) */
      /*   - e(2) */
      /*  cineq2 = ((x(8)-ref(8))^2 - velocity_constraints^2 + e(2)); */
      /*  cineq3 = (current_acc(8) - acc_constraints1- e(1)); % - e(1) */
      /*  cineq4 = (-current_acc(8) - acc_constraints2- e(2)); % - e(2) */
      /*   - e(1) */
      /*   - e(2) */
      /*  cineq6 = ((x(1)-current_d(1)) - front_constraints1 - e(4)); */
      /*  cineq7 = ((x(1)-current_d(1)) - front_constraints2 - e(5)); */
      /*  cineq8 = ((x(1)-current_d(2)) - rear_constraints1 - e(6)); */
      /*  cineq9 = ((x(1)-current_d(2)) - rear_constraints2 - e(7)); */
      /*  cineq5 = ((x(1)-current_d(1))^2 - distance_constraints^2 - e(5)); */
      /*  cineq6 = ((x(1)-current_d(2))^2 - distance_constraints^2 - e(6)); */
      /*  cineq4 = stage*3*((x(1)-current_d(1))^2 - lng_constraints^2); */
      /*  cineq5 = stage*3*((x(1)-current_d(2))^2 - lng_constraints^2); */
      cineq[0] = (current_d[0] - current_d[1]) - 0.05;
      cineq[1] = (current_d[1] - current_d[0]) - 0.05;
      cineq[2] = (10.0 * (current_d[4] - ref[7]) - 0.04) - e[0];
      cineq[3] = (10.0 * (ref[7] - current_d[4]) - 0.04) - e[0];
      cineq[4] = (10.0 * (current_d[5] - ref[7]) - 0.04) - e[1];
      cineq[5] = (10.0 * (ref[7] - current_d[5]) - 0.04) - e[1];
      cineq[6] = 100.0 * ((x[4] - ref[4]) - 0.0034906585039886592);
      cineq[7] = 100.0 * ((ref[4] - x[4]) - 0.0034906585039886592);
      cineq[8] = ((x[7] - ref[7]) - 0.05) - e[2];
      cineq[9] = ((ref[7] - x[7]) - 0.05) - e[2];
      cineq[10] = u[0] - 1000.0;
      cineq[14] = -u[0] - 1000.0;
      cineq[11] = u[1] - 1000.0;
      cineq[15] = -u[1] - 1000.0;
      cineq[12] = u[2] - 3000.0;
      cineq[16] = -u[2] - 3000.0;
      cineq[13] = u[3] - 3000.0;
      cineq[17] = -u[3] - 3000.0;
    } else {
      for (k = 0; k < 18; k++) {
        cineq[k] = -1.0;
      }
    }
  }
  /*  G = [                                     0; */
  /*                                              0; */
  /*                                              -1; */
  /*                                              0; */
  /*      -sin(atan(current_d(7)/current_d(5)))/r; */
  /*      -sin(atan(current_d(8)/current_d(6)))/r]; */
  /*  G = disc_func(dt,G) - disc_func(0,G); */
  /*  if any(u~=0) */
  /*      disp(u) */
  /*  end */
  /*  dxdt = A*x + E*current_d + G*g; */
  /*  dxdt = A*x + B*u + E*current_d + G*g; */
  /*  next_state = x + dxdt*dt; */
  /*  last_d = current_d; */
  /*  if any(cineq == nan) */
  /*      cineq = 0; */
  /*  end */
  /*  if any(cineq > 0) */
  /*      stage */
  /*      next_state(8) */
  /*      x(8) */
  /*  end */
  /*  last_state = x; */
  /*  end */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

void nlmpc_config__ineqConFcn_init(void)
{
  b_last_param_not_empty = false;
  b_A_not_empty = false;
}

/* End of code generation (nlmpc_config__ineqConFcn.c) */
