/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * evaluateStateTrajectory.c
 *
 * Code generation for function 'evaluateStateTrajectory'
 *
 */

/* Include files */
#include "evaluateStateTrajectory.h"
#include "nlmpc_config__stateFcn.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void evaluateStateTrajectory(const real_T Z[2071], const real_T x[14],
                             const real_T signals_StateFcnParameter[309],
                             real_T X[714])
{
  static const int16_T b_iv[52] = {
      0,    41,   82,   123,  164,  205,  246,  287,  328,  369,  410,
      451,  492,  533,  574,  615,  656,  697,  738,  779,  820,  861,
      902,  943,  984,  1025, 1066, 1107, 1148, 1189, 1230, 1271, 1312,
      1353, 1394, 1435, 1476, 1517, 1558, 1599, 1640, 1681, 1722, 1763,
      1804, 1845, 1886, 1927, 1968, 2009, 2050, 2071};
  static const int8_T b_iv1[357] = {
      0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15,
      15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,
      4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15,
      33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,
      7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33,
      41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,
      15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41,
      0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15,
      15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,
      4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15,
      33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,
      7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33,
      41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,
      15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41,
      0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15,
      15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,
      4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15,
      33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,
      7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33, 41, 0,  4,  7,  15, 15, 33,
      41, 0,  4,  7,  15, 15, 33, 41, 0,  0,  3,  3,  3,  21, 21};
  real_T fk[14];
  int32_T ct;
  int32_T i;
  memset(&X[0], 0, 714U * sizeof(real_T));
  memcpy(&X[0], &x[0], 14U * sizeof(real_T));
  for (ct = 0; ct < 50; ct++) {
    real_T mv[4];
    i = b_iv[ct] + b_iv1[7 * ct];
    mv[0] = Z[i];
    mv[1] = Z[i + 1];
    mv[2] = Z[i + 2];
    mv[3] = Z[i + 3];
    nlmpc_config__stateFcn(&X[14 * ct], mv, signals_StateFcnParameter, fk);
    for (i = 0; i <= 12; i += 2) {
      __m128d r;
      __m128d r1;
      r = _mm_loadu_pd(&fk[i]);
      r1 = _mm_loadu_pd(&X[i + 14 * ct]);
      r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
      _mm_storeu_pd(&fk[i], r);
      _mm_storeu_pd(&X[i + 14 * (ct + 1)], r);
    }
  }
}

/* End of code generation (evaluateStateTrajectory.c) */
