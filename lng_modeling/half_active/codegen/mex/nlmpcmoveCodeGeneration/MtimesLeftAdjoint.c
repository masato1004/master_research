/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * MtimesLeftAdjoint.c
 *
 * Code generation for function 'MtimesLeftAdjoint'
 *
 */

/* Include files */
#include "MtimesLeftAdjoint.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Function Definitions */
void MtimesLeftAdjoint(const real_T right[4], const real_T currAdjoint[2],
                       real_T leftJac[2])
{
  int32_T i;
  boolean_T IsFiniteCurrAdjoint;
  boolean_T IsFiniteRight;
  boolean_T exitg1;
  IsFiniteRight = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 4)) {
    if (muDoubleScalarIsInf(right[i]) || muDoubleScalarIsNaN(right[i])) {
      IsFiniteRight = false;
      exitg1 = true;
    } else {
      i++;
    }
  }
  IsFiniteCurrAdjoint = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 2)) {
    if (muDoubleScalarIsInf(currAdjoint[i]) ||
        muDoubleScalarIsNaN(currAdjoint[i])) {
      IsFiniteCurrAdjoint = false;
      exitg1 = true;
    } else {
      i++;
    }
  }
  if (IsFiniteRight && IsFiniteCurrAdjoint) {
    leftJac[0] = right[0] * currAdjoint[0] + currAdjoint[1] * right[2];
    leftJac[1] = currAdjoint[0] * right[1] + currAdjoint[1] * right[3];
  } else {
    leftJac[0] = right[0] * currAdjoint[0] + currAdjoint[1] * right[2];
    leftJac[1] = currAdjoint[0] * right[1] + currAdjoint[1] * right[3];
  }
}

void b_MtimesLeftAdjoint(const real_T right[16], const real_T currAdjoint[4],
                         real_T leftJac[4])
{
  int32_T i;
  boolean_T IsFiniteCurrAdjoint;
  boolean_T IsFiniteRight;
  boolean_T exitg1;
  IsFiniteRight = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 16)) {
    if (muDoubleScalarIsInf(right[i]) || muDoubleScalarIsNaN(right[i])) {
      IsFiniteRight = false;
      exitg1 = true;
    } else {
      i++;
    }
  }
  IsFiniteCurrAdjoint = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 4)) {
    if (muDoubleScalarIsInf(currAdjoint[i]) ||
        muDoubleScalarIsNaN(currAdjoint[i])) {
      IsFiniteCurrAdjoint = false;
      exitg1 = true;
    } else {
      i++;
    }
  }
  if (IsFiniteRight && IsFiniteCurrAdjoint) {
    real_T d;
    real_T d1;
    real_T d2;
    real_T d3;
    d = currAdjoint[0];
    d1 = currAdjoint[1];
    d2 = currAdjoint[2];
    d3 = currAdjoint[3];
    for (i = 0; i <= 2; i += 2) {
      __m128d r;
      __m128d r1;
      r = _mm_loadu_pd(&right[i]);
      r = _mm_mul_pd(r, _mm_set1_pd(d));
      r1 = _mm_loadu_pd(&right[i + 4]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d1));
      r = _mm_add_pd(r, r1);
      r1 = _mm_loadu_pd(&right[i + 8]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d2));
      r = _mm_add_pd(r, r1);
      r1 = _mm_loadu_pd(&right[i + 12]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d3));
      r = _mm_add_pd(r, r1);
      _mm_storeu_pd(&leftJac[i], r);
    }
  } else {
    real_T d;
    real_T d1;
    real_T d2;
    real_T d3;
    d = currAdjoint[0];
    d1 = currAdjoint[1];
    d2 = currAdjoint[2];
    d3 = currAdjoint[3];
    for (i = 0; i <= 2; i += 2) {
      __m128d r;
      __m128d r1;
      r = _mm_loadu_pd(&right[i]);
      r = _mm_mul_pd(r, _mm_set1_pd(d));
      r1 = _mm_loadu_pd(&right[i + 4]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d1));
      r = _mm_add_pd(r, r1);
      r1 = _mm_loadu_pd(&right[i + 8]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d2));
      r = _mm_add_pd(r, r1);
      r1 = _mm_loadu_pd(&right[i + 12]);
      r1 = _mm_mul_pd(r1, _mm_set1_pd(d3));
      r = _mm_add_pd(r, r1);
      _mm_storeu_pd(&leftJac[i], r);
    }
  }
}

/* End of code generation (MtimesLeftAdjoint.c) */
