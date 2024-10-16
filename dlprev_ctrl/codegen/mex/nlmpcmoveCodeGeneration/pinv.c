/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * pinv.c
 *
 * Code generation for function 'pinv'
 *
 */

/* Include files */
#include "pinv.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void pinv(const real_T A[196], real_T X[196])
{
  ptrdiff_t info_t;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t n_t;
  real_T U[196];
  real_T V[196];
  real_T Vt[196];
  real_T S[14];
  real_T superb[13];
  real_T absx;
  real_T beta1;
  int32_T b_i;
  int32_T i;
  int32_T j;
  int32_T k;
  int32_T vcol;
  char_T TRANSA1;
  char_T TRANSB1;
  boolean_T p;
  p = true;
  for (k = 0; k < 196; k++) {
    X[k] = 0.0;
    if (p) {
      absx = A[k];
      if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    for (i = 0; i < 196; i++) {
      X[i] = rtNaN;
    }
  } else {
    boolean_T exitg1;
    memcpy(&V[0], &A[0], 196U * sizeof(real_T));
    info_t = LAPACKE_dgesdd(102, 'S', (ptrdiff_t)14, (ptrdiff_t)14, &V[0],
                            (ptrdiff_t)14, &S[0], &U[0], (ptrdiff_t)14, &Vt[0],
                            (ptrdiff_t)14);
    if ((int32_T)info_t < 0) {
      for (i = 0; i < 196; i++) {
        U[i] = rtNaN;
      }
      for (b_i = 0; b_i < 14; b_i++) {
        S[b_i] = rtNaN;
      }
      for (i = 0; i < 196; i++) {
        Vt[i] = rtNaN;
      }
    }
    if ((int32_T)info_t > 0) {
      memcpy(&V[0], &A[0], 196U * sizeof(real_T));
      info_t = LAPACKE_dgesvd(102, 'S', 'S', (ptrdiff_t)14, (ptrdiff_t)14,
                              &V[0], (ptrdiff_t)14, &S[0], &U[0], (ptrdiff_t)14,
                              &Vt[0], (ptrdiff_t)14, &superb[0]);
      for (j = 0; j < 14; j++) {
        for (b_i = 0; b_i < 14; b_i++) {
          V[b_i + 14 * j] = Vt[j + 14 * b_i];
        }
      }
      if ((int32_T)info_t < 0) {
        for (i = 0; i < 196; i++) {
          U[i] = rtNaN;
        }
        for (b_i = 0; b_i < 14; b_i++) {
          S[b_i] = rtNaN;
        }
        for (i = 0; i < 196; i++) {
          V[i] = rtNaN;
        }
      }
    } else {
      for (j = 0; j < 14; j++) {
        for (b_i = 0; b_i < 14; b_i++) {
          V[b_i + 14 * j] = Vt[j + 14 * b_i];
        }
      }
    }
    absx = muDoubleScalarAbs(S[0]);
    if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
      absx = rtNaN;
    } else if (absx < 4.4501477170144028E-308) {
      absx = 4.94065645841247E-324;
    } else {
      frexp(absx, &vcol);
      absx = ldexp(1.0, vcol - 53);
    }
    absx *= 14.0;
    b_i = 0;
    exitg1 = false;
    while ((!exitg1) && (b_i < 14)) {
      if (muDoubleScalarIsInf(S[b_i]) || muDoubleScalarIsNaN(S[b_i])) {
        absx = 1.7976931348623157E+308;
        exitg1 = true;
      } else {
        b_i++;
      }
    }
    b_i = 0;
    k = 0;
    while ((k < 14) && (S[k] > absx)) {
      b_i++;
      k++;
    }
    if (b_i > 0) {
      vcol = 1;
      for (j = 0; j < b_i; j++) {
        int32_T scalarLB_tmp_tmp;
        int32_T vectorUB;
        absx = 1.0 / S[j];
        i = vcol + 13;
        scalarLB_tmp_tmp = vcol + 14;
        vectorUB = vcol + 12;
        for (k = vcol; k <= vectorUB; k += 2) {
          __m128d r;
          r = _mm_loadu_pd(&V[k - 1]);
          _mm_storeu_pd(&V[k - 1], _mm_mul_pd(_mm_set1_pd(absx), r));
        }
        for (k = scalarLB_tmp_tmp; k <= i; k++) {
          V[k - 1] *= absx;
        }
        vcol += 14;
      }
      memset(&X[0], 0, 196U * sizeof(real_T));
      absx = 1.0;
      beta1 = 0.0;
      TRANSB1 = 'C';
      TRANSA1 = 'N';
      info_t = (ptrdiff_t)14;
      n_t = (ptrdiff_t)14;
      k_t = (ptrdiff_t)b_i;
      lda_t = (ptrdiff_t)14;
      ldb_t = (ptrdiff_t)14;
      ldc_t = (ptrdiff_t)14;
      dgemm(&TRANSA1, &TRANSB1, &info_t, &n_t, &k_t, &absx, &V[0], &lda_t,
            &U[0], &ldb_t, &beta1, &X[0], &ldc_t);
    }
  }
}

/* End of code generation (pinv.c) */
