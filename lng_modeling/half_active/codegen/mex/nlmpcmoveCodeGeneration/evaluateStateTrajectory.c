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
void evaluateStateTrajectory(const real_T Z[186], const real_T x[14],
                             const real_T signals_StateFcnParameter[109],
                             real_T X[154])
{
  static const int8_T b_iv[77] = {
      0, 4, 6, 10, 10, 14, 18, 0, 4, 6, 10, 10, 14, 18, 0, 4, 6, 10, 10, 14, 18,
      0, 4, 6, 10, 10, 14, 18, 0, 4, 6, 10, 10, 14, 18, 0, 4, 6, 10, 10, 14, 18,
      0, 4, 6, 10, 10, 14, 18, 0, 4, 6, 10, 10, 14, 18, 0, 4, 6, 10, 10, 14, 18,
      0, 4, 6, 10, 10, 14, 18, 0, 0, 2, 2,  2,  6,  6};
  static const uint8_T uv[12] = {0U,   18U,  36U,  54U,  72U,  90U,
                                 108U, 126U, 144U, 162U, 180U, 186U};
  real_T fk[14];
  int32_T ct;
  int32_T i;
  memset(&X[0], 0, 154U * sizeof(real_T));
  memcpy(&X[0], &x[0], 14U * sizeof(real_T));
  for (ct = 0; ct < 10; ct++) {
    real_T mv[4];
    i = uv[ct] + b_iv[7 * ct];
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
