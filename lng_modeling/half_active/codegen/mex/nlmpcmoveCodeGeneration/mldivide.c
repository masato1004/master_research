/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 */

/* Include files */
#include "mldivide.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_emxutil.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>

/* Function Definitions */
void mldivide(const emxArray_real_T *A, const emxArray_real_T *B,
              emxArray_real_T *Y)
{
  static const int32_T offsets[4] = {0, 1, 2, 3};
  ptrdiff_t *jpvt_t_data;
  emxArray_int32_T *jpvt;
  emxArray_ptrdiff_t *jpvt_t;
  emxArray_real_T *b_A;
  emxArray_real_T *b_B;
  emxArray_real_T *tau;
  const real_T *A_data;
  const real_T *B_data;
  real_T *Y_data;
  real_T *b_A_data;
  real_T *b_B_data;
  real_T *tau_data;
  int32_T i;
  int32_T loop_ub;
  int32_T minmn;
  int32_T *jpvt_data;
  B_data = B->data;
  A_data = A->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&b_A, 2);
  emxInit_int32_T(&jpvt);
  emxInit_real_T(&tau, 1);
  emxInit_ptrdiff_t(&jpvt_t);
  emxInit_real_T(&b_B, 1);
  if ((A->size[0] == 0) || (A->size[1] == 0) || (B->size[0] == 0)) {
    loop_ub = A->size[1];
    i = Y->size[0];
    Y->size[0] = A->size[1];
    emxEnsureCapacity_real_T(Y, i);
    Y_data = Y->data;
    for (i = 0; i < loop_ub; i++) {
      Y_data[i] = 0.0;
    }
  } else if (A->size[0] == A->size[1]) {
    ptrdiff_t INFO;
    ptrdiff_t LDA;
    ptrdiff_t info_t;
    int32_T maxmn;
    loop_ub = B->size[0];
    i = Y->size[0];
    Y->size[0] = B->size[0];
    emxEnsureCapacity_real_T(Y, i);
    Y_data = Y->data;
    for (i = 0; i < loop_ub; i++) {
      Y_data[i] = B_data[i];
    }
    maxmn = A->size[0];
    minmn = A->size[1];
    i = muIntScalarMin_sint32(maxmn, minmn);
    maxmn = muIntScalarMin_sint32(loop_ub, i);
    i = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(b_A, i);
    b_A_data = b_A->data;
    minmn = A->size[0] * A->size[1];
    for (i = 0; i < minmn; i++) {
      b_A_data[i] = A_data[i];
    }
    i = jpvt_t->size[0];
    jpvt_t->size[0] = maxmn;
    emxEnsureCapacity_ptrdiff_t(jpvt_t, i);
    jpvt_t_data = jpvt_t->data;
    info_t = (ptrdiff_t)maxmn;
    LDA = (ptrdiff_t)b_A->size[0];
    INFO = LAPACKE_dgetrf_work(102, info_t, info_t, &b_A_data[0], LDA,
                               &jpvt_t_data[0]);
    if ((int32_T)INFO < 0) {
      i = Y->size[0];
      Y->size[0] = B->size[0];
      emxEnsureCapacity_real_T(Y, i);
      Y_data = Y->data;
      for (i = 0; i < loop_ub; i++) {
        Y_data[i] = rtNaN;
      }
    } else {
      LAPACKE_dgetrs_work(102, 'N', info_t, (ptrdiff_t)1, &b_A_data[0], LDA,
                          &jpvt_t_data[0], &Y_data[0], (ptrdiff_t)B->size[0]);
    }
  } else {
    ptrdiff_t info_t;
    real_T tol;
    int32_T maxmn;
    int32_T minmana;
    int32_T na;
    maxmn = A->size[0];
    i = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    loop_ub = A->size[1];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(b_A, i);
    b_A_data = b_A->data;
    minmn = A->size[0] * A->size[1];
    for (i = 0; i < minmn; i++) {
      b_A_data[i] = A_data[i];
    }
    na = b_A->size[1] - 1;
    i = jpvt->size[0] * jpvt->size[1];
    jpvt->size[0] = 1;
    jpvt->size[1] = A->size[1];
    emxEnsureCapacity_int32_T(jpvt, i);
    jpvt_data = jpvt->data;
    minmana = muIntScalarMin_sint32(maxmn, loop_ub);
    i = tau->size[0];
    tau->size[0] = minmana;
    emxEnsureCapacity_real_T(tau, i);
    tau_data = tau->data;
    i = jpvt_t->size[0];
    jpvt_t->size[0] = A->size[1];
    emxEnsureCapacity_ptrdiff_t(jpvt_t, i);
    jpvt_t_data = jpvt_t->data;
    for (i = 0; i < loop_ub; i++) {
      jpvt_data[i] = 0;
      jpvt_t_data[i] = (ptrdiff_t)0;
    }
    info_t = LAPACKE_dgeqp3(
        102, (ptrdiff_t)b_A->size[0], (ptrdiff_t)b_A->size[1], &b_A_data[0],
        (ptrdiff_t)b_A->size[0], &jpvt_t_data[0], &tau_data[0]);
    if ((int32_T)info_t != 0) {
      for (minmn = 0; minmn <= na; minmn++) {
        for (loop_ub = 0; loop_ub < maxmn; loop_ub++) {
          b_A_data[minmn * maxmn + loop_ub] = rtNaN;
        }
      }
      minmn = muIntScalarMin_sint32(maxmn, b_A->size[1]) - 1;
      for (loop_ub = 0; loop_ub <= minmn; loop_ub++) {
        tau_data[loop_ub] = rtNaN;
      }
      i = minmn + 2;
      for (loop_ub = i; loop_ub <= minmana; loop_ub++) {
        tau_data[loop_ub - 1] = 0.0;
      }
      maxmn = (b_A->size[1] / 4) << 2;
      minmn = maxmn - 4;
      for (loop_ub = 0; loop_ub <= minmn; loop_ub += 4) {
        _mm_storeu_si128(
            (__m128i *)&jpvt_data[loop_ub],
            _mm_add_epi32(
                _mm_add_epi32(_mm_set1_epi32(loop_ub),
                              _mm_loadu_si128((const __m128i *)&offsets[0])),
                _mm_set1_epi32(1)));
      }
      for (loop_ub = maxmn; loop_ub <= na; loop_ub++) {
        jpvt_data[loop_ub] = loop_ub + 1;
      }
    } else {
      for (loop_ub = 0; loop_ub <= na; loop_ub++) {
        jpvt_data[loop_ub] = (int32_T)jpvt_t_data[loop_ub];
      }
    }
    minmana = 0;
    if (b_A->size[0] < b_A->size[1]) {
      minmn = b_A->size[0];
      maxmn = b_A->size[1];
    } else {
      minmn = b_A->size[1];
      maxmn = b_A->size[0];
    }
    tol = muDoubleScalarMin(1.4901161193847656E-8,
                            2.2204460492503131E-15 * (real_T)maxmn) *
          muDoubleScalarAbs(b_A_data[0]);
    while ((minmana < minmn) &&
           (!(muDoubleScalarAbs(b_A_data[minmana + b_A->size[0] * minmana]) <=
              tol))) {
      minmana++;
    }
    loop_ub = B->size[0];
    i = b_B->size[0];
    b_B->size[0] = B->size[0];
    emxEnsureCapacity_real_T(b_B, i);
    b_B_data = b_B->data;
    for (i = 0; i < loop_ub; i++) {
      b_B_data[i] = B_data[i];
    }
    loop_ub = b_A->size[1];
    i = Y->size[0];
    Y->size[0] = b_A->size[1];
    emxEnsureCapacity_real_T(Y, i);
    Y_data = Y->data;
    for (i = 0; i < loop_ub; i++) {
      Y_data[i] = 0.0;
    }
    info_t = (ptrdiff_t)b_B->size[0];
    info_t = LAPACKE_dormqr(
        102, 'L', 'T', info_t, (ptrdiff_t)1,
        (ptrdiff_t)muIntScalarMin_sint32(b_A->size[0], b_A->size[1]),
        &b_A_data[0], (ptrdiff_t)b_A->size[0], &tau_data[0], &b_B_data[0],
        info_t);
    if ((int32_T)info_t != 0) {
      maxmn = b_B->size[0];
      i = b_B->size[0];
      b_B->size[0] = maxmn;
      emxEnsureCapacity_real_T(b_B, i);
      b_B_data = b_B->data;
      for (i = 0; i < maxmn; i++) {
        b_B_data[i] = rtNaN;
      }
    }
    for (loop_ub = 0; loop_ub < minmana; loop_ub++) {
      Y_data[jpvt_data[loop_ub] - 1] = b_B_data[loop_ub];
    }
    for (minmn = minmana; minmn >= 1; minmn--) {
      i = jpvt_data[minmn - 1];
      Y_data[i - 1] /= b_A_data[(minmn + b_A->size[0] * (minmn - 1)) - 1];
      for (loop_ub = 0; loop_ub <= minmn - 2; loop_ub++) {
        Y_data[jpvt_data[loop_ub] - 1] -=
            Y_data[i - 1] * b_A_data[loop_ub + b_A->size[0] * (minmn - 1)];
      }
    }
  }
  emxFree_real_T(&b_B);
  emxFree_ptrdiff_t(&jpvt_t);
  emxFree_real_T(&tau);
  emxFree_int32_T(&jpvt);
  emxFree_real_T(&b_A);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (mldivide.c) */
