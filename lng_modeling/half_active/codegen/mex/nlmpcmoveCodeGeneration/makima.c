/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * makima.c
 *
 * Code generation for function 'makima'
 *
 */

/* Include files */
#include "makima.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Function Definitions */
void b_makima(const real_T x[60], const real_T y[60], const real_T xx[3],
              real_T output[3])
{
  __m128d r;
  real_T pp_coefs[236];
  real_T slopes[60];
  real_T delta[59];
  real_T h[59];
  real_T d;
  real_T delta_0;
  real_T delta_m1;
  real_T delta_n;
  real_T delta_n1;
  real_T w12;
  real_T w2;
  int32_T k;
  for (k = 0; k <= 56; k += 2) {
    r = _mm_sub_pd(_mm_loadu_pd(&x[k + 1]), _mm_loadu_pd(&x[k]));
    _mm_storeu_pd(&h[k], r);
    _mm_storeu_pd(&delta[k], _mm_div_pd(_mm_sub_pd(_mm_loadu_pd(&y[k + 1]),
                                                   _mm_loadu_pd(&y[k])),
                                        r));
  }
  d = x[59] - x[58];
  h[58] = d;
  delta[58] = (y[59] - y[58]) / d;
  delta_0 = 2.0 * delta[0] - delta[1];
  delta_m1 = 2.0 * delta_0 - delta[0];
  delta_n = 2.0 * delta[58] - delta[57];
  delta_n1 = 2.0 * delta_n - delta[58];
  for (k = 0; k < 60; k++) {
    real_T delta1;
    real_T delta2;
    real_T w1;
    if (k + 1 == 1) {
      delta1 = delta_0;
    } else {
      delta1 = delta[k - 1];
    }
    if (k + 1 == 60) {
      delta2 = delta_n;
    } else {
      delta2 = delta[k];
    }
    if (k + 1 == 1) {
      w1 = muDoubleScalarAbs(delta_0 - delta_m1) +
           muDoubleScalarAbs(delta_0 + delta_m1) / 2.0;
    } else if (k + 1 == 2) {
      w1 = muDoubleScalarAbs(delta[0] - delta_0) +
           muDoubleScalarAbs(delta[0] + delta_0) / 2.0;
    } else {
      w2 = delta[k - 1];
      w12 = delta[k - 2];
      w1 = muDoubleScalarAbs(w2 - w12) + muDoubleScalarAbs(w2 + w12) / 2.0;
    }
    if (k + 1 == 60) {
      w2 = muDoubleScalarAbs(delta_n1 - delta_n) +
           muDoubleScalarAbs(delta_n1 + delta_n) / 2.0;
    } else if (k + 1 == 59) {
      w2 = muDoubleScalarAbs(delta_n - delta[58]) +
           muDoubleScalarAbs(delta_n + delta[58]) / 2.0;
    } else {
      w2 = delta[k + 1];
      w12 = delta[k];
      w2 = muDoubleScalarAbs(w2 - w12) + muDoubleScalarAbs(w2 + w12) / 2.0;
    }
    w12 = w1 + w2;
    if (w12 == 0.0) {
      slopes[k] = 0.0;
    } else {
      slopes[k] = w2 / w12 * delta1 + w1 / w12 * delta2;
    }
  }
  for (k = 0; k <= 56; k += 2) {
    __m128d r1;
    __m128d r2;
    __m128d r3;
    __m128d r4;
    r = _mm_loadu_pd(&delta[k]);
    r1 = _mm_loadu_pd(&slopes[k]);
    r2 = _mm_loadu_pd(&h[k]);
    r3 = _mm_div_pd(_mm_sub_pd(r, r1), r2);
    r4 = _mm_loadu_pd(&slopes[k + 1]);
    r = _mm_div_pd(_mm_sub_pd(r4, r), r2);
    _mm_storeu_pd(&pp_coefs[k], _mm_div_pd(_mm_sub_pd(r, r3), r2));
    _mm_storeu_pd(&pp_coefs[k + 59],
                  _mm_sub_pd(_mm_mul_pd(_mm_set1_pd(2.0), r3), r));
    _mm_storeu_pd(&pp_coefs[k + 118], r1);
    _mm_storeu_pd(&pp_coefs[k + 177], _mm_loadu_pd(&y[k]));
  }
  w2 = (delta[58] - slopes[58]) / d;
  w12 = (slopes[59] - delta[58]) / d;
  pp_coefs[58] = (w12 - w2) / d;
  pp_coefs[117] = 2.0 * w2 - w12;
  pp_coefs[176] = slopes[58];
  pp_coefs[235] = y[58];
  for (k = 0; k < 3; k++) {
    if (muDoubleScalarIsNaN(xx[k])) {
      w2 = rtNaN;
    } else {
      int32_T high_i;
      int32_T low_i;
      int32_T low_ip1;
      low_i = 0;
      low_ip1 = 2;
      high_i = 60;
      while (high_i > low_ip1) {
        int32_T mid_i;
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (xx[k] >= x[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }
      w2 = xx[k] - x[low_i];
      w2 = w2 * (w2 * (w2 * pp_coefs[low_i] + pp_coefs[low_i + 59]) +
                 pp_coefs[low_i + 118]) +
           pp_coefs[low_i + 177];
    }
    output[k] = w2;
  }
}

real_T makima(const real_T x[60], const real_T y[60], real_T xx)
{
  __m128d r;
  real_T pp_coefs[236];
  real_T slopes[60];
  real_T delta[59];
  real_T h[59];
  real_T d;
  real_T delta_0;
  real_T delta_m1;
  real_T delta_n;
  real_T delta_n1;
  real_T output;
  real_T w12;
  real_T w2;
  int32_T low_i;
  for (low_i = 0; low_i <= 56; low_i += 2) {
    r = _mm_sub_pd(_mm_loadu_pd(&x[low_i + 1]), _mm_loadu_pd(&x[low_i]));
    _mm_storeu_pd(&h[low_i], r);
    _mm_storeu_pd(&delta[low_i],
                  _mm_div_pd(_mm_sub_pd(_mm_loadu_pd(&y[low_i + 1]),
                                        _mm_loadu_pd(&y[low_i])),
                             r));
  }
  d = x[59] - x[58];
  h[58] = d;
  delta[58] = (y[59] - y[58]) / d;
  delta_0 = 2.0 * delta[0] - delta[1];
  delta_m1 = 2.0 * delta_0 - delta[0];
  delta_n = 2.0 * delta[58] - delta[57];
  delta_n1 = 2.0 * delta_n - delta[58];
  for (low_i = 0; low_i < 60; low_i++) {
    real_T delta1;
    real_T delta2;
    real_T w1;
    if (low_i + 1 == 1) {
      delta1 = delta_0;
    } else {
      delta1 = delta[low_i - 1];
    }
    if (low_i + 1 == 60) {
      delta2 = delta_n;
    } else {
      delta2 = delta[low_i];
    }
    if (low_i + 1 == 1) {
      w1 = muDoubleScalarAbs(delta_0 - delta_m1) +
           muDoubleScalarAbs(delta_0 + delta_m1) / 2.0;
    } else if (low_i + 1 == 2) {
      w1 = muDoubleScalarAbs(delta[0] - delta_0) +
           muDoubleScalarAbs(delta[0] + delta_0) / 2.0;
    } else {
      w2 = delta[low_i - 1];
      w12 = delta[low_i - 2];
      w1 = muDoubleScalarAbs(w2 - w12) + muDoubleScalarAbs(w2 + w12) / 2.0;
    }
    if (low_i + 1 == 60) {
      w2 = muDoubleScalarAbs(delta_n1 - delta_n) +
           muDoubleScalarAbs(delta_n1 + delta_n) / 2.0;
    } else if (low_i + 1 == 59) {
      w2 = muDoubleScalarAbs(delta_n - delta[58]) +
           muDoubleScalarAbs(delta_n + delta[58]) / 2.0;
    } else {
      w2 = delta[low_i + 1];
      w12 = delta[low_i];
      w2 = muDoubleScalarAbs(w2 - w12) + muDoubleScalarAbs(w2 + w12) / 2.0;
    }
    w12 = w1 + w2;
    if (w12 == 0.0) {
      slopes[low_i] = 0.0;
    } else {
      slopes[low_i] = w2 / w12 * delta1 + w1 / w12 * delta2;
    }
  }
  for (low_i = 0; low_i <= 56; low_i += 2) {
    __m128d r1;
    __m128d r2;
    __m128d r3;
    __m128d r4;
    r = _mm_loadu_pd(&delta[low_i]);
    r1 = _mm_loadu_pd(&slopes[low_i]);
    r2 = _mm_loadu_pd(&h[low_i]);
    r3 = _mm_div_pd(_mm_sub_pd(r, r1), r2);
    r4 = _mm_loadu_pd(&slopes[low_i + 1]);
    r = _mm_div_pd(_mm_sub_pd(r4, r), r2);
    _mm_storeu_pd(&pp_coefs[low_i], _mm_div_pd(_mm_sub_pd(r, r3), r2));
    _mm_storeu_pd(&pp_coefs[low_i + 59],
                  _mm_sub_pd(_mm_mul_pd(_mm_set1_pd(2.0), r3), r));
    _mm_storeu_pd(&pp_coefs[low_i + 118], r1);
    _mm_storeu_pd(&pp_coefs[low_i + 177], _mm_loadu_pd(&y[low_i]));
  }
  w2 = (delta[58] - slopes[58]) / d;
  w12 = (slopes[59] - delta[58]) / d;
  pp_coefs[58] = (w12 - w2) / d;
  pp_coefs[117] = 2.0 * w2 - w12;
  pp_coefs[176] = slopes[58];
  pp_coefs[235] = y[58];
  if (muDoubleScalarIsNaN(xx)) {
    output = rtNaN;
  } else {
    int32_T high_i;
    int32_T low_ip1;
    low_i = 0;
    low_ip1 = 2;
    high_i = 60;
    while (high_i > low_ip1) {
      int32_T mid_i;
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (xx >= x[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }
    w2 = xx - x[low_i];
    output = w2 * (w2 * (w2 * pp_coefs[low_i] + pp_coefs[low_i + 59]) +
                   pp_coefs[low_i + 118]) +
             pp_coefs[low_i + 177];
  }
  return output;
}

/* End of code generation (makima.c) */
