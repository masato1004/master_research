/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fdgmres.c
 *
 * Code generation for function 'fdgmres'
 *
 */

/* Include files */
#include "fdgmres.h"
#include "mldivide.h"
#include "nlmpcmoveCodeGeneration.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_emxutil.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
real_T fdgmres(c_nlmpcmoveCodeGenerationStackD *SD,
               const real_T F_workspace_lastMV[4],
               const real_T c_F_workspace_onlinedata_StateF[309],
               const real_T c_F_workspace_onlinedata_StageP[16473],
               const real_T U[2071], real_T dUdt[2071], const real_T x[14],
               const real_T dxdt[14], emxArray_real_T *resvec, real_T *relres,
               real_T *iter)
{
  static const int8_T b[21] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  __m128d r;
  __m128d r1;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t n_t;
  emxArray_boolean_T *c_x;
  emxArray_real_T *A;
  emxArray_real_T *b_g;
  emxArray_real_T *b_h;
  emxArray_real_T *y;
  real_T b_U[2071];
  real_T b_f1[2071];
  real_T b_f2[2071];
  real_T f1[2071];
  real_T f2[2071];
  real_T h[462];
  real_T h_[462];
  real_T c[21];
  real_T err[21];
  real_T g[21];
  real_T s[21];
  real_T b_x[14];
  real_T absxk;
  real_T flag;
  real_T htemp;
  real_T rho;
  real_T tolb;
  real_T tolb_tmp;
  real_T *A_data;
  real_T *y_data;
  int32_T b_flag;
  int32_T b_i;
  int32_T b_k;
  int32_T i;
  int32_T j;
  int32_T k;
  int32_T v_tmp;
  char_T TRANSA1;
  char_T TRANSB1;
  boolean_T exitg1;
  boolean_T tf;
  boolean_T *x_data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  memset(&h[0], 0, 462U * sizeof(real_T));
  memset(&SD->f0.v[0], 0, 43491U * sizeof(real_T));
  memset(&c[0], 0, 21U * sizeof(real_T));
  memset(&s[0], 0, 21U * sizeof(real_T));
  memset(&err[0], 0, 21U * sizeof(real_T));
  b_flag = 0;
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&b_x[i], _mm_add_pd(_mm_loadu_pd(&x[i]),
                                      _mm_mul_pd(_mm_set1_pd(1.0E-9),
                                                 _mm_loadu_pd(&dxdt[i]))));
  }
  c_nlmpcmoveCodeGeneration_anonF(F_workspace_lastMV,
                                  c_F_workspace_onlinedata_StateF,
                                  c_F_workspace_onlinedata_StageP, U, b_x, f1);
  for (b_i = 0; b_i <= 2068; b_i += 2) {
    r = _mm_loadu_pd(&dUdt[b_i]);
    _mm_storeu_pd(&b_U[b_i], _mm_add_pd(_mm_loadu_pd(&U[b_i]),
                                        _mm_mul_pd(_mm_set1_pd(1.0E-9), r)));
  }
  b_U[2070] = U[2070] + 1.0E-9 * dUdt[2070];
  c_nlmpcmoveCodeGeneration_anonF(
      F_workspace_lastMV, c_F_workspace_onlinedata_StateF,
      c_F_workspace_onlinedata_StageP, b_U, b_x, f2);
  c_nlmpcmoveCodeGeneration_anonF(F_workspace_lastMV,
                                  c_F_workspace_onlinedata_StateF,
                                  c_F_workspace_onlinedata_StageP, U, x, b_U);
  c_nlmpcmoveCodeGeneration_anonF(F_workspace_lastMV,
                                  c_F_workspace_onlinedata_StateF,
                                  c_F_workspace_onlinedata_StageP, U, x, b_f1);
  for (b_i = 0; b_i <= 12; b_i += 2) {
    _mm_storeu_pd(&b_x[b_i], _mm_add_pd(_mm_loadu_pd(&x[b_i]),
                                        _mm_mul_pd(_mm_set1_pd(1.0E-9),
                                                   _mm_loadu_pd(&dxdt[b_i]))));
  }
  c_nlmpcmoveCodeGeneration_anonF(
      F_workspace_lastMV, c_F_workspace_onlinedata_StateF,
      c_F_workspace_onlinedata_StageP, U, b_x, b_f2);
  r = _mm_set1_pd(1.0E-9);
  for (b_i = 0; b_i <= 2068; b_i += 2) {
    __m128d r2;
    __m128d r3;
    r1 = _mm_loadu_pd(&b_U[b_i]);
    r2 = _mm_loadu_pd(&b_f2[b_i]);
    r3 = _mm_loadu_pd(&b_f1[b_i]);
    r1 = _mm_sub_pd(_mm_mul_pd(_mm_set1_pd(-1000.0), r1),
                    _mm_div_pd(_mm_sub_pd(r2, r3), r));
    _mm_storeu_pd(&b_U[b_i], r1);
    r2 = _mm_loadu_pd(&f2[b_i]);
    r3 = _mm_loadu_pd(&f1[b_i]);
    _mm_storeu_pd(&f2[b_i], _mm_sub_pd(r1, _mm_div_pd(_mm_sub_pd(r2, r3), r)));
  }
  htemp = -1000.0 * b_U[2070] - (b_f2[2070] - b_f1[2070]) / 1.0E-9;
  b_U[2070] = htemp;
  f2[2070] = htemp - (f2[2070] - f1[2070]) / 1.0E-9;
  n_t = (ptrdiff_t)2071;
  incx_t = (ptrdiff_t)1;
  rho = dnrm2(&n_t, &f2[0], &incx_t);
  for (i = 0; i < 21; i++) {
    g[i] = rho * (real_T)b[i];
  }
  err[0] = rho;
  memset(&SD->f0.v_[0], 0, 43491U * sizeof(real_T));
  memset(&h_[0], 0, 462U * sizeof(real_T));
  n_t = (ptrdiff_t)2071;
  incx_t = (ptrdiff_t)1;
  tolb_tmp = dnrm2(&n_t, &b_U[0], &incx_t);
  tolb = 1.0E-7 * tolb_tmp;
  for (b_i = 0; b_i <= 2068; b_i += 2) {
    r = _mm_loadu_pd(&f2[b_i]);
    _mm_storeu_pd(&SD->f0.v[b_i], _mm_div_pd(r, _mm_set1_pd(rho)));
  }
  SD->f0.v[2070] = f2[2070] / rho;
  k = 1;
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&b_x[i], _mm_add_pd(_mm_loadu_pd(&x[i]),
                                      _mm_mul_pd(_mm_set1_pd(1.0E-9),
                                                 _mm_loadu_pd(&dxdt[i]))));
  }
  b_k = 0;
  emxInit_real_T(&y, 1);
  exitg1 = false;
  while ((!exitg1) && (b_k < 20)) {
    int32_T h_tmp_tmp_tmp;
    int32_T k_tmp_tmp;
    k_tmp_tmp = b_k + 1;
    k = b_k + 1;
    c_nlmpcmoveCodeGeneration_anonF(
        F_workspace_lastMV, c_F_workspace_onlinedata_StateF,
        c_F_workspace_onlinedata_StageP, U, b_x, f1);
    for (b_i = 0; b_i <= 2068; b_i += 2) {
      r = _mm_loadu_pd(&SD->f0.v[b_i + 2071 * b_k]);
      _mm_storeu_pd(&b_U[b_i], _mm_add_pd(_mm_loadu_pd(&U[b_i]),
                                          _mm_mul_pd(_mm_set1_pd(1.0E-9), r)));
    }
    b_U[2070] = U[2070] + 1.0E-9 * SD->f0.v[2071 * b_k + 2070];
    c_nlmpcmoveCodeGeneration_anonF(
        F_workspace_lastMV, c_F_workspace_onlinedata_StateF,
        c_F_workspace_onlinedata_StageP, b_U, b_x, f2);
    for (b_i = 0; b_i <= 2068; b_i += 2) {
      r = _mm_loadu_pd(&f2[b_i]);
      r1 = _mm_loadu_pd(&f1[b_i]);
      _mm_storeu_pd(&SD->f0.v[b_i + 2071 * (b_k + 1)],
                    _mm_div_pd(_mm_sub_pd(r, r1), _mm_set1_pd(1.0E-9)));
    }
    v_tmp = 2071 * (b_k + 1);
    SD->f0.v[v_tmp + 2070] = (f2[2070] - f1[2070]) / 1.0E-9;
    n_t = (ptrdiff_t)2071;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    for (j = 0; j <= b_k; j++) {
      i = j + 21 * b_k;
      h[i] =
          ddot(&n_t, &SD->f0.v[2071 * j], &incx_t, &SD->f0.v[v_tmp], &incy_t);
      htemp = h[i];
      for (b_i = 0; b_i <= 2068; b_i += 2) {
        r = _mm_loadu_pd(&SD->f0.v[b_i + 2071 * j]);
        r1 = _mm_loadu_pd(&SD->f0.v[b_i + v_tmp]);
        _mm_storeu_pd(&b_U[b_i],
                      _mm_sub_pd(r1, _mm_mul_pd(_mm_set1_pd(htemp), r)));
      }
      b_U[2070] = SD->f0.v[v_tmp + 2070] - htemp * SD->f0.v[2071 * j + 2070];
      memcpy(&SD->f0.v[v_tmp], &b_U[0], 2071U * sizeof(real_T));
    }
    n_t = (ptrdiff_t)2071;
    incx_t = (ptrdiff_t)1;
    h_tmp_tmp_tmp = b_k + 21 * b_k;
    h[h_tmp_tmp_tmp + 1] = dnrm2(&n_t, &SD->f0.v[v_tmp], &incx_t);
    n_t = (ptrdiff_t)2071;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    for (j = 0; j <= b_k; j++) {
      htemp =
          ddot(&n_t, &SD->f0.v[2071 * j], &incx_t, &SD->f0.v[v_tmp], &incy_t);
      i = j + 21 * b_k;
      h[i] += htemp;
      for (b_i = 0; b_i <= 2068; b_i += 2) {
        r = _mm_loadu_pd(&SD->f0.v[b_i + 2071 * j]);
        r1 = _mm_loadu_pd(&SD->f0.v[b_i + v_tmp]);
        _mm_storeu_pd(&b_U[b_i],
                      _mm_sub_pd(r1, _mm_mul_pd(_mm_set1_pd(htemp), r)));
      }
      b_U[2070] = SD->f0.v[v_tmp + 2070] - htemp * SD->f0.v[2071 * j + 2070];
      memcpy(&SD->f0.v[v_tmp], &b_U[0], 2071U * sizeof(real_T));
    }
    n_t = (ptrdiff_t)2071;
    incx_t = (ptrdiff_t)1;
    htemp = dnrm2(&n_t, &SD->f0.v[v_tmp], &incx_t);
    h[h_tmp_tmp_tmp + 1] = htemp;
    absxk = h[h_tmp_tmp_tmp + 1];
    if (absxk <= 1.4901161193847656E-8) {
      b_flag = 2;
      exitg1 = true;
    } else {
      real_T eta;
      real_T t;
      for (b_i = 0; b_i <= 2068; b_i += 2) {
        r = _mm_loadu_pd(&SD->f0.v[b_i + v_tmp]);
        _mm_storeu_pd(&f1[b_i], _mm_mul_pd(_mm_set1_pd(absxk), r));
      }
      t = SD->f0.v[v_tmp + 2070];
      f1[2070] = absxk * t;
      n_t = (ptrdiff_t)2071;
      incx_t = (ptrdiff_t)1;
      eta = dnrm2(&n_t, &f1[0], &incx_t);
      if (eta > tolb * htemp) {
        for (b_i = 0; b_i <= 2068; b_i += 2) {
          r = _mm_loadu_pd(&SD->f0.v[b_i + v_tmp]);
          _mm_storeu_pd(&b_U[b_i], _mm_div_pd(r, _mm_set1_pd(absxk)));
        }
        b_U[2070] = t / absxk;
        memcpy(&SD->f0.v[v_tmp], &b_U[0], 2071U * sizeof(real_T));
      } else {
        b_flag = 3;
      }
      if (b_k + 1 > 1) {
        b_i = y->size[0];
        y->size[0] = b_k + 1;
        emxEnsureCapacity_real_T(y, b_i);
        y_data = y->data;
        for (b_i = 0; b_i <= b_k; b_i++) {
          y_data[b_i] = h[b_i + 21 * b_k];
        }
        for (i = 0; i < b_k; i++) {
          absxk = y_data[i + 1];
          eta = y_data[i];
          htemp = c[i];
          t = s[i];
          y_data[i] = htemp * y_data[i] - t * absxk;
          y_data[i + 1] = t * eta + htemp * absxk;
        }
        for (b_i = 0; b_i < k_tmp_tmp; b_i++) {
          h[b_i + 21 * b_k] = y_data[b_i];
        }
      }
      htemp = 3.3121686421112381E-170;
      absxk = muDoubleScalarAbs(h[h_tmp_tmp_tmp]);
      if (absxk > 3.3121686421112381E-170) {
        eta = 1.0;
        htemp = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        eta = t * t;
      }
      absxk = muDoubleScalarAbs(h[(b_k + 21 * b_k) + 1]);
      if (absxk > htemp) {
        t = htemp / absxk;
        eta = eta * t * t + 1.0;
        htemp = absxk;
      } else {
        t = absxk / htemp;
        eta += t * t;
      }
      eta = htemp * muDoubleScalarSqrt(eta);
      if (eta > 1.4901161193847656E-8) {
        htemp = h[h_tmp_tmp_tmp];
        c[b_k] = htemp / eta;
        absxk = h[h_tmp_tmp_tmp + 1];
        s[b_k] = -absxk / eta;
        h[h_tmp_tmp_tmp] = c[b_k] * htemp - s[b_k] * absxk;
        h[h_tmp_tmp_tmp + 1] = 0.0;
        htemp = g[b_k];
        absxk = g[b_k + 1];
        t = htemp;
        htemp = c[b_k] * htemp - s[b_k] * absxk;
        g[b_k] = htemp;
        g[b_k + 1] = s[b_k] * t + c[b_k] * absxk;
        memcpy(&h_[0], &h[0], 462U * sizeof(real_T));
        memcpy(&SD->f0.v_[0], &SD->f0.v[0], 43491U * sizeof(real_T));
        rho = muDoubleScalarAbs(g[b_k + 1]);
        err[b_k + 1] = rho;
        if (rho <= tolb) {
          b_flag = 1;
          exitg1 = true;
        } else if (b_flag == 3) {
          exitg1 = true;
        } else {
          b_k++;
        }
      } else {
        b_flag = 3;
        exitg1 = true;
      }
    }
  }
  if (b_flag == 2) {
    k--;
    memcpy(&SD->f0.v[0], &SD->f0.v_[0], 43491U * sizeof(real_T));
    memcpy(&h[0], &h_[0], 462U * sizeof(real_T));
  }
  b_i = resvec->size[0];
  resvec->size[0] = k;
  emxEnsureCapacity_real_T(resvec, b_i);
  y_data = resvec->data;
  for (b_i = 0; b_i < k; b_i++) {
    y_data[b_i] = err[b_i];
  }
  emxInit_real_T(&b_h, 2);
  b_i = b_h->size[0] * b_h->size[1];
  b_h->size[0] = k;
  b_h->size[1] = k;
  emxEnsureCapacity_real_T(b_h, b_i);
  y_data = b_h->data;
  for (b_i = 0; b_i < k; b_i++) {
    for (v_tmp = 0; v_tmp < k; v_tmp++) {
      y_data[v_tmp + b_h->size[0] * b_i] = h[v_tmp + 21 * b_i];
    }
  }
  emxInit_real_T(&b_g, 1);
  b_i = b_g->size[0];
  b_g->size[0] = k;
  emxEnsureCapacity_real_T(b_g, b_i);
  y_data = b_g->data;
  for (b_i = 0; b_i < k; b_i++) {
    y_data[b_i] = g[b_i];
  }
  mldivide(b_h, b_g, y);
  y_data = y->data;
  emxFree_real_T(&b_g);
  emxFree_real_T(&b_h);
  i = y->size[0];
  tf = true;
  for (b_k = 0; b_k < i; b_k++) {
    if ((!tf) || (muDoubleScalarIsInf(y_data[b_k]) ||
                  muDoubleScalarIsNaN(y_data[b_k]))) {
      tf = false;
    }
  }
  emxInit_boolean_T(&c_x);
  emxInit_real_T(&A, 2);
  if (!tf) {
    b_flag = -1;
  } else {
    b_i = c_x->size[0];
    c_x->size[0] = 2071;
    emxEnsureCapacity_boolean_T(c_x, b_i);
    x_data = c_x->data;
    for (b_i = 0; b_i < 2071; b_i++) {
      x_data[b_i] = muDoubleScalarIsNaN(dUdt[b_i]);
    }
    tf = false;
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k <= 2070)) {
      if (x_data[b_k]) {
        tf = true;
        exitg1 = true;
      } else {
        b_k++;
      }
    }
    if (tf) {
      b_flag = -1;
    } else {
      b_i = A->size[0] * A->size[1];
      A->size[0] = 2071;
      A->size[1] = k;
      emxEnsureCapacity_real_T(A, b_i);
      A_data = A->data;
      for (b_i = 0; b_i < k; b_i++) {
        for (v_tmp = 0; v_tmp < 2071; v_tmp++) {
          i = v_tmp + 2071 * b_i;
          A_data[i] = SD->f0.v[i];
        }
      }
      if ((k == 0) || (y->size[0] == 0)) {
        memset(&f1[0], 0, 2071U * sizeof(real_T));
      } else {
        TRANSB1 = 'N';
        TRANSA1 = 'N';
        htemp = 1.0;
        absxk = 0.0;
        incx_t = (ptrdiff_t)2071;
        n_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)k;
        lda_t = (ptrdiff_t)2071;
        ldb_t = (ptrdiff_t)y->size[0];
        ldc_t = (ptrdiff_t)2071;
        dgemm(&TRANSA1, &TRANSB1, &incx_t, &n_t, &incy_t, &htemp, &A_data[0],
              &lda_t, &y_data[0], &ldb_t, &absxk, &f1[0], &ldc_t);
      }
      for (b_i = 0; b_i <= 2068; b_i += 2) {
        r = _mm_loadu_pd(&dUdt[b_i]);
        r1 = _mm_loadu_pd(&f1[b_i]);
        _mm_storeu_pd(&dUdt[b_i], _mm_add_pd(r, r1));
      }
      dUdt[2070] += f1[2070];
    }
  }
  emxFree_real_T(&A);
  emxFree_boolean_T(&c_x);
  emxFree_real_T(&y);
  flag = b_flag;
  *relres = rho / tolb_tmp;
  *iter = k;
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return flag;
}

/* End of code generation (fdgmres.c) */
