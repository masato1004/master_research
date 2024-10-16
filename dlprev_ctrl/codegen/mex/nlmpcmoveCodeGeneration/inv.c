/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inv.c
 *
 * Code generation for function 'inv'
 *
 */

/* Include files */
#include "inv.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "lapacke.h"
#include <emmintrin.h>
#include <stddef.h>

/* Function Definitions */
void inv(const real_T x[196], real_T y[196])
{
  ptrdiff_t ipiv_t[14];
  ptrdiff_t info_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t n_t;
  real_T b_x[196];
  real_T alpha1;
  int32_T ipiv[14];
  int32_T b_i;
  int32_T i;
  int32_T j;
  int32_T k;
  int32_T pipk;
  char_T DIAGA1;
  char_T SIDE1;
  char_T TRANSA1;
  char_T UPLO1;
  int8_T p[14];
  for (i = 0; i < 196; i++) {
    y[i] = 0.0;
    b_x[i] = x[i];
  }
  info_t = LAPACKE_dgetrf_work(102, (ptrdiff_t)14, (ptrdiff_t)14, &b_x[0],
                               (ptrdiff_t)14, &ipiv_t[0]);
  if ((int32_T)info_t < 0) {
    for (i = 0; i < 196; i++) {
      b_x[i] = rtNaN;
    }
    __m128i r;
    __m128i r1;
    r = _mm_set1_epi32(1);
    r1 = _mm_loadu_si128((const __m128i *)&iv2[0]);
    _mm_storeu_si128((__m128i *)&ipiv[0],
                     _mm_add_epi32(_mm_add_epi32(_mm_set1_epi32(0), r1), r));
    _mm_storeu_si128((__m128i *)&ipiv[4],
                     _mm_add_epi32(_mm_add_epi32(_mm_set1_epi32(4), r1), r));
    _mm_storeu_si128((__m128i *)&ipiv[8],
                     _mm_add_epi32(_mm_add_epi32(_mm_set1_epi32(8), r1), r));
    ipiv[12] = 13;
    ipiv[13] = 14;
  } else {
    for (k = 0; k < 14; k++) {
      ipiv[k] = (int32_T)ipiv_t[k];
    }
  }
  for (i = 0; i < 14; i++) {
    p[i] = (int8_T)(i + 1);
  }
  for (k = 0; k < 13; k++) {
    i = ipiv[k];
    if (i > k + 1) {
      pipk = p[i - 1];
      p[i - 1] = p[k];
      p[k] = (int8_T)pipk;
    }
  }
  for (k = 0; k < 14; k++) {
    pipk = 14 * (p[k] - 1);
    y[k + pipk] = 1.0;
    for (j = k + 1; j < 15; j++) {
      i = (j + pipk) - 1;
      if (y[i] != 0.0) {
        int32_T i1;
        i1 = j + 1;
        for (b_i = i1; b_i < 15; b_i++) {
          int32_T y_tmp;
          y_tmp = (b_i + pipk) - 1;
          y[y_tmp] -= y[i] * b_x[(b_i + 14 * (j - 1)) - 1];
        }
      }
    }
  }
  alpha1 = 1.0;
  DIAGA1 = 'N';
  TRANSA1 = 'N';
  UPLO1 = 'U';
  SIDE1 = 'L';
  info_t = (ptrdiff_t)14;
  n_t = (ptrdiff_t)14;
  lda_t = (ptrdiff_t)14;
  ldb_t = (ptrdiff_t)14;
  dtrsm(&SIDE1, &UPLO1, &TRANSA1, &DIAGA1, &info_t, &n_t, &alpha1, &b_x[0],
        &lda_t, &y[0], &ldb_t);
}

/* End of code generation (inv.c) */
