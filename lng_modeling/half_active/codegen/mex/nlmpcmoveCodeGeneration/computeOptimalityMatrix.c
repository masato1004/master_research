/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeOptimalityMatrix.c
 *
 * Code generation for function 'computeOptimalityMatrix'
 *
 */

/* Include files */
#include "computeOptimalityMatrix.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>

/* Function Definitions */
void computeFischerBurmeister(const real_T a[18], const real_T b[18],
                              real_T phi[18])
{
  int32_T k;
  for (k = 0; k <= 16; k += 2) {
    __m128d r;
    __m128d r1;
    r = _mm_loadu_pd(&a[k]);
    r1 = _mm_loadu_pd(&b[k]);
    _mm_storeu_pd(
        &phi[k],
        _mm_add_pd(_mm_add_pd(r, r1),
                   _mm_sqrt_pd(_mm_add_pd(
                       _mm_add_pd(_mm_mul_pd(r, r), _mm_mul_pd(r1, r1)),
                       _mm_set1_pd(1.0E-12)))));
  }
}

/* End of code generation (computeOptimalityMatrix.c) */
