/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpc_config__costFcnJacobianAD.c
 *
 * Code generation for function 'nlmpc_config__costFcnJacobianAD'
 *
 */

/* Include files */
#include "nlmpc_config__costFcnJacobianAD.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
real_T nlmpc_config__costFcnJacobianAD(
    const real_T inputVariables[25], real_T extraParams_f1,
    const real_T extraParams_f2[196], real_T extraParams_f3,
    const real_T extraParams_f4[16], real_T extraParams_f5,
    const real_T extraParams_f6[9], const real_T params_p[323], real_T grad[25])
{
  __m128d r;
  __m128d r1;
  real_T arg26[14];
  real_T arg7[14];
  real_T arg8[14];
  real_T idxRowsJac[14];
  real_T arg13[4];
  real_T b_rightJac[4];
  real_T arg18[3];
  real_T rightJac[3];
  real_T curRow;
  real_T d;
  real_T d1;
  real_T d2;
  real_T obj;
  int32_T b_i;
  int32_T i;
  int32_T k;
  boolean_T IsFiniteCurrAdjoint;
  boolean_T IsFiniteLeft;
  boolean_T exitg1;
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&idxRowsJac[i],
                  _mm_mul_pd(_mm_sub_pd(_mm_loadu_pd(&inputVariables[i]),
                                        _mm_loadu_pd(&params_p[i + 309])),
                             _mm_set1_pd(extraParams_f1)));
  }
  for (b_i = 0; b_i < 14; b_i++) {
    curRow = 0.0;
    for (i = 0; i < 14; i++) {
      curRow += idxRowsJac[i] * extraParams_f2[i + 14 * b_i];
    }
    arg7[b_i] = curRow;
    arg8[b_i] = inputVariables[b_i] - params_p[b_i + 309];
  }
  curRow = inputVariables[14];
  d = inputVariables[15];
  d1 = inputVariables[16];
  d2 = inputVariables[17];
  for (i = 0; i < 4; i++) {
    k = i << 2;
    arg13[i] = ((curRow * extraParams_f3 * extraParams_f4[k] +
                 d * extraParams_f3 * extraParams_f4[k + 1]) +
                d1 * extraParams_f3 * extraParams_f4[k + 2]) +
               d2 * extraParams_f3 * extraParams_f4[k + 3];
  }
  curRow = inputVariables[22];
  d = inputVariables[23];
  d1 = inputVariables[24];
  for (i = 0; i < 3; i++) {
    arg18[i] = (curRow * extraParams_f5 * extraParams_f6[3 * i] +
                d * extraParams_f5 * extraParams_f6[3 * i + 1]) +
               d1 * extraParams_f5 * extraParams_f6[3 * i + 2];
  }
  curRow = 0.0;
  for (i = 0; i < 14; i++) {
    curRow += arg7[i] * arg8[i];
  }
  obj = (curRow +
         (((arg13[0] * inputVariables[14] + arg13[1] * inputVariables[15]) +
           arg13[2] * inputVariables[16]) +
          arg13[3] * inputVariables[17])) +
        ((arg18[0] * inputVariables[22] + arg18[1] * inputVariables[23]) +
         arg18[2] * inputVariables[24]);
  memset(&grad[0], 0, 25U * sizeof(real_T));
  rightJac[0] = arg18[0];
  rightJac[1] = arg18[1];
  rightJac[2] = arg18[2];
  arg18[0] = inputVariables[22];
  arg18[1] = inputVariables[23];
  arg18[2] = inputVariables[24];
  IsFiniteLeft = true;
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i < 9)) {
    if (muDoubleScalarIsInf(extraParams_f6[b_i]) ||
        muDoubleScalarIsNaN(extraParams_f6[b_i])) {
      IsFiniteLeft = false;
      exitg1 = true;
    } else {
      b_i++;
    }
  }
  IsFiniteCurrAdjoint = true;
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i < 3)) {
    if (muDoubleScalarIsInf(arg18[b_i]) || muDoubleScalarIsNaN(arg18[b_i])) {
      IsFiniteCurrAdjoint = false;
      exitg1 = true;
    } else {
      b_i++;
    }
  }
  if (IsFiniteLeft && IsFiniteCurrAdjoint) {
    curRow = inputVariables[22];
    d = inputVariables[23];
    d1 = inputVariables[24];
    r = _mm_loadu_pd(&extraParams_f6[0]);
    r = _mm_mul_pd(r, _mm_set1_pd(curRow));
    r1 = _mm_loadu_pd(&extraParams_f6[3]);
    r1 = _mm_mul_pd(r1, _mm_set1_pd(d));
    r = _mm_add_pd(r, r1);
    r1 = _mm_loadu_pd(&extraParams_f6[6]);
    r1 = _mm_mul_pd(r1, _mm_set1_pd(d1));
    r = _mm_add_pd(r, r1);
    _mm_storeu_pd(&arg18[0], r);
    arg18[2] = (extraParams_f6[2] * curRow + extraParams_f6[5] * d) +
               extraParams_f6[8] * d1;
  } else {
    curRow = inputVariables[22];
    d = inputVariables[23];
    d1 = inputVariables[24];
    r = _mm_loadu_pd(&extraParams_f6[0]);
    r = _mm_mul_pd(r, _mm_set1_pd(curRow));
    r1 = _mm_loadu_pd(&extraParams_f6[3]);
    r1 = _mm_mul_pd(r1, _mm_set1_pd(d));
    r = _mm_add_pd(r, r1);
    r1 = _mm_loadu_pd(&extraParams_f6[6]);
    r1 = _mm_mul_pd(r1, _mm_set1_pd(d1));
    r = _mm_add_pd(r, r1);
    _mm_storeu_pd(&arg18[0], r);
    arg18[2] = (extraParams_f6[2] * curRow + extraParams_f6[5] * d) +
               extraParams_f6[8] * d1;
  }
  b_rightJac[0] = arg13[0];
  b_rightJac[1] = arg13[1];
  b_rightJac[2] = arg13[2];
  b_rightJac[3] = arg13[3];
  arg13[0] = inputVariables[14];
  arg13[1] = inputVariables[15];
  arg13[2] = inputVariables[16];
  arg13[3] = inputVariables[17];
  IsFiniteLeft = true;
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i < 16)) {
    if (muDoubleScalarIsInf(extraParams_f4[b_i]) ||
        muDoubleScalarIsNaN(extraParams_f4[b_i])) {
      IsFiniteLeft = false;
      exitg1 = true;
    } else {
      b_i++;
    }
  }
  IsFiniteCurrAdjoint = true;
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i < 4)) {
    if (muDoubleScalarIsInf(arg13[b_i]) || muDoubleScalarIsNaN(arg13[b_i])) {
      IsFiniteCurrAdjoint = false;
      exitg1 = true;
    } else {
      b_i++;
    }
  }
  if (IsFiniteLeft && IsFiniteCurrAdjoint) {
    curRow = inputVariables[14];
    d = inputVariables[15];
    d1 = inputVariables[16];
    d2 = inputVariables[17];
    for (i = 0; i <= 2; i += 2) {
      r = _mm_loadu_pd(&extraParams_f4[i]);
      r = _mm_mul_pd(r, _mm_set1_pd(curRow));
      r1 = _mm_loadu_pd(&extraParams_f4[i + 4]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d));
      r = _mm_add_pd(r, r1);
      r1 = _mm_loadu_pd(&extraParams_f4[i + 8]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d1));
      r = _mm_add_pd(r, r1);
      r1 = _mm_loadu_pd(&extraParams_f4[i + 12]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d2));
      r = _mm_add_pd(r, r1);
      _mm_storeu_pd(&arg13[i], r);
    }
  } else {
    curRow = inputVariables[14];
    d = inputVariables[15];
    d1 = inputVariables[16];
    d2 = inputVariables[17];
    for (b_i = 0; b_i <= 2; b_i += 2) {
      r = _mm_loadu_pd(&extraParams_f4[b_i]);
      r = _mm_mul_pd(r, _mm_set1_pd(curRow));
      r1 = _mm_loadu_pd(&extraParams_f4[b_i + 4]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d));
      r = _mm_add_pd(r, r1);
      r1 = _mm_loadu_pd(&extraParams_f4[b_i + 8]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d1));
      r = _mm_add_pd(r, r1);
      r1 = _mm_loadu_pd(&extraParams_f4[b_i + 12]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d2));
      r = _mm_add_pd(r, r1);
      _mm_storeu_pd(&arg13[b_i], r);
    }
  }
  IsFiniteLeft = true;
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i < 14)) {
    if (muDoubleScalarIsInf(arg7[b_i]) || muDoubleScalarIsNaN(arg7[b_i])) {
      IsFiniteLeft = false;
      exitg1 = true;
    } else {
      b_i++;
    }
  }
  if (IsFiniteLeft) {
    memcpy(&arg26[0], &arg7[0], 14U * sizeof(real_T));
  } else {
    for (k = 0; k < 14; k++) {
      arg26[k] = 0.0;
      idxRowsJac[k] = (real_T)k + 1.0;
    }
    for (i = 0; i < 14; i++) {
      arg26[(int32_T)idxRowsJac[i] - 1] = arg7[i];
    }
  }
  memcpy(&idxRowsJac[0], &arg8[0], 14U * sizeof(real_T));
  IsFiniteLeft = true;
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i < 196)) {
    if (muDoubleScalarIsInf(extraParams_f2[b_i]) ||
        muDoubleScalarIsNaN(extraParams_f2[b_i])) {
      IsFiniteLeft = false;
      exitg1 = true;
    } else {
      b_i++;
    }
  }
  IsFiniteCurrAdjoint = true;
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i < 14)) {
    if (muDoubleScalarIsInf(idxRowsJac[b_i]) ||
        muDoubleScalarIsNaN(idxRowsJac[b_i])) {
      IsFiniteCurrAdjoint = false;
      exitg1 = true;
    } else {
      b_i++;
    }
  }
  if (IsFiniteLeft && IsFiniteCurrAdjoint) {
    for (i = 0; i < 14; i++) {
      curRow = 0.0;
      for (k = 0; k < 14; k++) {
        curRow += extraParams_f2[i + 14 * k] * idxRowsJac[k];
      }
      arg7[i] = curRow;
    }
  } else {
    for (b_i = 0; b_i < 14; b_i++) {
      curRow = 0.0;
      for (k = 0; k < 14; k++) {
        curRow += extraParams_f2[b_i + 14 * k] * idxRowsJac[k];
      }
      arg7[b_i] = curRow;
    }
  }
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&arg7[i]);
    r1 = _mm_loadu_pd(&arg26[i]);
    _mm_storeu_pd(&grad[i],
                  _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(extraParams_f1))));
  }
  __m128d r2;
  r = _mm_loadu_pd(&arg13[0]);
  r1 = _mm_loadu_pd(&b_rightJac[0]);
  r2 = _mm_set1_pd(extraParams_f3);
  _mm_storeu_pd(&grad[14], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
  r = _mm_loadu_pd(&arg13[2]);
  r1 = _mm_loadu_pd(&b_rightJac[2]);
  _mm_storeu_pd(&grad[16], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
  r = _mm_loadu_pd(&arg18[0]);
  r1 = _mm_loadu_pd(&rightJac[0]);
  _mm_storeu_pd(&grad[22],
                _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(extraParams_f5))));
  grad[24] = rightJac[2] + arg18[2] * extraParams_f5;
  return obj;
}

/* End of code generation (nlmpc_config__costFcnJacobianAD.c) */
