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
real_T fdgmres(const real_T F_workspace_lastMV[4],
               const real_T c_F_workspace_onlinedata_StateF[109],
               const real_T c_F_workspace_onlinedata_StageP[1353],
               const real_T U[186], real_T dUdt[186], const real_T x[14],
               const real_T dxdt[14], emxArray_real_T *resvec, real_T *relres,
               real_T *iter)
{
  static const int8_T b[11] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  __m128d r;
  __m128d r1;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emxArray_boolean_T *c_x;
  emxArray_real_T *A;
  emxArray_real_T *b_g;
  emxArray_real_T *b_h;
  emxArray_real_T *y;
  real_T v[2046];
  real_T v_[2046];
  real_T b_U[186];
  real_T b_f1[186];
  real_T b_f2[186];
  real_T f1[186];
  real_T f2[186];
  real_T h[132];
  real_T h_[132];
  real_T b_x[14];
  real_T y_tmp[14];
  real_T c[11];
  real_T err[11];
  real_T g[11];
  real_T s[11];
  real_T absxk;
  real_T alpha1;
  real_T beta1;
  real_T eta;
  real_T flag;
  real_T rho;
  real_T scale;
  real_T t;
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
  int32_T k_tmp_tmp;
  char_T TRANSA1;
  char_T TRANSB1;
  boolean_T exitg1;
  boolean_T tf;
  boolean_T *x_data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  memset(&h[0], 0, 132U * sizeof(real_T));
  memset(&v[0], 0, 2046U * sizeof(real_T));
  memset(&c[0], 0, 11U * sizeof(real_T));
  memset(&s[0], 0, 11U * sizeof(real_T));
  memset(&err[0], 0, 11U * sizeof(real_T));
  b_flag = 0;
  for (i = 0; i <= 12; i += 2) {
    r = _mm_mul_pd(_mm_set1_pd(1.0E-9), _mm_loadu_pd(&dxdt[i]));
    _mm_storeu_pd(&y_tmp[i], r);
    _mm_storeu_pd(&b_x[i], _mm_add_pd(_mm_loadu_pd(&x[i]), r));
  }
  c_nlmpcmoveCodeGeneration_anonF(F_workspace_lastMV,
                                  c_F_workspace_onlinedata_StateF,
                                  c_F_workspace_onlinedata_StageP, U, b_x, f1);
  for (b_i = 0; b_i <= 184; b_i += 2) {
    r = _mm_loadu_pd(&dUdt[b_i]);
    _mm_storeu_pd(&b_U[b_i], _mm_add_pd(_mm_loadu_pd(&U[b_i]),
                                        _mm_mul_pd(_mm_set1_pd(1.0E-9), r)));
  }
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
    r = _mm_loadu_pd(&y_tmp[b_i]);
    _mm_storeu_pd(&y_tmp[b_i], _mm_add_pd(_mm_loadu_pd(&x[b_i]), r));
  }
  c_nlmpcmoveCodeGeneration_anonF(
      F_workspace_lastMV, c_F_workspace_onlinedata_StateF,
      c_F_workspace_onlinedata_StageP, U, y_tmp, b_f2);
  rho = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 186; k++) {
    eta = -1000.0 * b_U[k] - (b_f2[k] - b_f1[k]) / 1.0E-9;
    b_U[k] = eta;
    eta -= (f2[k] - f1[k]) / 1.0E-9;
    f2[k] = eta;
    absxk = muDoubleScalarAbs(eta);
    if (absxk > scale) {
      t = scale / absxk;
      rho = rho * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      rho += t * t;
    }
  }
  rho = scale * muDoubleScalarSqrt(rho);
  for (i = 0; i < 11; i++) {
    g[i] = rho * (real_T)b[i];
  }
  err[0] = rho;
  memset(&v_[0], 0, 2046U * sizeof(real_T));
  memset(&h_[0], 0, 132U * sizeof(real_T));
  tolb_tmp = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 186; k++) {
    absxk = muDoubleScalarAbs(b_U[k]);
    if (absxk > scale) {
      t = scale / absxk;
      tolb_tmp = tolb_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      tolb_tmp += t * t;
    }
    v[k] = f2[k] / rho;
  }
  tolb_tmp = scale * muDoubleScalarSqrt(tolb_tmp);
  tolb = 1.0E-7 * tolb_tmp;
  k = 1;
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&b_x[i], _mm_add_pd(_mm_loadu_pd(&x[i]),
                                      _mm_mul_pd(_mm_set1_pd(1.0E-9),
                                                 _mm_loadu_pd(&dxdt[i]))));
  }
  b_k = 0;
  emxInit_real_T(&y, 1);
  exitg1 = false;
  while ((!exitg1) && (b_k < 10)) {
    real_T b_y;
    int32_T h_tmp_tmp_tmp;
    k_tmp_tmp = b_k + 1;
    k = b_k + 1;
    c_nlmpcmoveCodeGeneration_anonF(
        F_workspace_lastMV, c_F_workspace_onlinedata_StateF,
        c_F_workspace_onlinedata_StageP, U, b_x, f1);
    for (b_i = 0; b_i <= 184; b_i += 2) {
      r = _mm_loadu_pd(&v[b_i + 186 * b_k]);
      _mm_storeu_pd(&b_U[b_i], _mm_add_pd(_mm_loadu_pd(&U[b_i]),
                                          _mm_mul_pd(_mm_set1_pd(1.0E-9), r)));
    }
    c_nlmpcmoveCodeGeneration_anonF(
        F_workspace_lastMV, c_F_workspace_onlinedata_StateF,
        c_F_workspace_onlinedata_StageP, b_U, b_x, f2);
    for (b_i = 0; b_i <= 184; b_i += 2) {
      r = _mm_loadu_pd(&f2[b_i]);
      r1 = _mm_loadu_pd(&f1[b_i]);
      _mm_storeu_pd(&v[b_i + 186 * (b_k + 1)],
                    _mm_div_pd(_mm_sub_pd(r, r1), _mm_set1_pd(1.0E-9)));
    }
    for (j = 0; j <= b_k; j++) {
      alpha1 = 0.0;
      for (b_i = 0; b_i < 186; b_i++) {
        alpha1 += v[b_i + 186 * j] * v[b_i + 186 * (b_k + 1)];
      }
      i = j + 11 * b_k;
      h[i] = alpha1;
      alpha1 = h[i];
      for (b_i = 0; b_i <= 184; b_i += 2) {
        r = _mm_loadu_pd(&v[b_i + 186 * j]);
        r1 = _mm_loadu_pd(&v[b_i + 186 * (b_k + 1)]);
        _mm_storeu_pd(&b_U[b_i],
                      _mm_sub_pd(r1, _mm_mul_pd(_mm_set1_pd(alpha1), r)));
      }
      memcpy(&v[b_k * 186 + 186], &b_U[0], 186U * sizeof(real_T));
    }
    b_y = 0.0;
    scale = 3.3121686421112381E-170;
    for (i = 0; i < 186; i++) {
      absxk = muDoubleScalarAbs(v[i + 186 * (b_k + 1)]);
      if (absxk > scale) {
        t = scale / absxk;
        b_y = b_y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        b_y += t * t;
      }
    }
    h_tmp_tmp_tmp = b_k + 11 * b_k;
    h[h_tmp_tmp_tmp + 1] = scale * muDoubleScalarSqrt(b_y);
    for (j = 0; j <= b_k; j++) {
      alpha1 = 0.0;
      for (b_i = 0; b_i < 186; b_i++) {
        alpha1 += v[b_i + 186 * j] * v[b_i + 186 * (b_k + 1)];
      }
      i = j + 11 * b_k;
      h[i] += alpha1;
      for (b_i = 0; b_i <= 184; b_i += 2) {
        r = _mm_loadu_pd(&v[b_i + 186 * j]);
        r1 = _mm_loadu_pd(&v[b_i + 186 * (b_k + 1)]);
        _mm_storeu_pd(&b_U[b_i],
                      _mm_sub_pd(r1, _mm_mul_pd(_mm_set1_pd(alpha1), r)));
      }
      memcpy(&v[b_k * 186 + 186], &b_U[0], 186U * sizeof(real_T));
    }
    alpha1 = 0.0;
    scale = 3.3121686421112381E-170;
    for (i = 0; i < 186; i++) {
      absxk = muDoubleScalarAbs(v[i + 186 * (b_k + 1)]);
      if (absxk > scale) {
        t = scale / absxk;
        alpha1 = alpha1 * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        alpha1 += t * t;
      }
    }
    alpha1 = scale * muDoubleScalarSqrt(alpha1);
    h[h_tmp_tmp_tmp + 1] = alpha1;
    beta1 = h[h_tmp_tmp_tmp + 1];
    if (beta1 <= 1.4901161193847656E-8) {
      b_flag = 2;
      exitg1 = true;
    } else {
      b_y = 0.0;
      scale = 3.3121686421112381E-170;
      for (i = 0; i < 186; i++) {
        absxk = muDoubleScalarAbs(beta1 * v[i + 186 * (b_k + 1)]);
        if (absxk > scale) {
          t = scale / absxk;
          b_y = b_y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          b_y += t * t;
        }
      }
      b_y = scale * muDoubleScalarSqrt(b_y);
      if (b_y > tolb * alpha1) {
        for (b_i = 0; b_i <= 184; b_i += 2) {
          r = _mm_loadu_pd(&v[b_i + 186 * (b_k + 1)]);
          _mm_storeu_pd(&b_U[b_i], _mm_div_pd(r, _mm_set1_pd(beta1)));
        }
        memcpy(&v[b_k * 186 + 186], &b_U[0], 186U * sizeof(real_T));
      } else {
        b_flag = 3;
      }
      if (b_k + 1 > 1) {
        b_i = y->size[0];
        y->size[0] = b_k + 1;
        emxEnsureCapacity_real_T(y, b_i);
        y_data = y->data;
        for (b_i = 0; b_i <= b_k; b_i++) {
          y_data[b_i] = h[b_i + 11 * b_k];
        }
        for (i = 0; i < b_k; i++) {
          alpha1 = y_data[i + 1];
          b_y = y_data[i];
          eta = c[i];
          beta1 = s[i];
          y_data[i] = eta * y_data[i] - beta1 * alpha1;
          y_data[i + 1] = beta1 * b_y + eta * alpha1;
        }
        for (b_i = 0; b_i < k_tmp_tmp; b_i++) {
          h[b_i + 11 * b_k] = y_data[b_i];
        }
      }
      scale = 3.3121686421112381E-170;
      absxk = muDoubleScalarAbs(h[h_tmp_tmp_tmp]);
      if (absxk > 3.3121686421112381E-170) {
        eta = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        eta = t * t;
      }
      absxk = muDoubleScalarAbs(h[(b_k + 11 * b_k) + 1]);
      if (absxk > scale) {
        t = scale / absxk;
        eta = eta * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        eta += t * t;
      }
      eta = scale * muDoubleScalarSqrt(eta);
      if (eta > 1.4901161193847656E-8) {
        alpha1 = h[h_tmp_tmp_tmp];
        c[b_k] = alpha1 / eta;
        beta1 = h[h_tmp_tmp_tmp + 1];
        s[b_k] = -beta1 / eta;
        h[h_tmp_tmp_tmp] = c[b_k] * alpha1 - s[b_k] * beta1;
        h[h_tmp_tmp_tmp + 1] = 0.0;
        alpha1 = g[b_k];
        beta1 = g[b_k + 1];
        eta = alpha1;
        alpha1 = c[b_k] * alpha1 - s[b_k] * beta1;
        g[b_k] = alpha1;
        g[b_k + 1] = s[b_k] * eta + c[b_k] * beta1;
        memcpy(&h_[0], &h[0], 132U * sizeof(real_T));
        memcpy(&v_[0], &v[0], 2046U * sizeof(real_T));
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
    memcpy(&v[0], &v_[0], 2046U * sizeof(real_T));
    memcpy(&h[0], &h_[0], 132U * sizeof(real_T));
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
    for (k_tmp_tmp = 0; k_tmp_tmp < k; k_tmp_tmp++) {
      y_data[k_tmp_tmp + b_h->size[0] * b_i] = h[k_tmp_tmp + 11 * b_i];
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
    c_x->size[0] = 186;
    emxEnsureCapacity_boolean_T(c_x, b_i);
    x_data = c_x->data;
    for (b_i = 0; b_i < 186; b_i++) {
      x_data[b_i] = muDoubleScalarIsNaN(dUdt[b_i]);
    }
    tf = false;
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k <= 185)) {
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
      A->size[0] = 186;
      A->size[1] = k;
      emxEnsureCapacity_real_T(A, b_i);
      A_data = A->data;
      for (b_i = 0; b_i < k; b_i++) {
        for (k_tmp_tmp = 0; k_tmp_tmp < 186; k_tmp_tmp++) {
          i = k_tmp_tmp + 186 * b_i;
          A_data[i] = v[i];
        }
      }
      if ((k == 0) || (y->size[0] == 0)) {
        memset(&f1[0], 0, 186U * sizeof(real_T));
      } else {
        TRANSB1 = 'N';
        TRANSA1 = 'N';
        alpha1 = 1.0;
        beta1 = 0.0;
        m_t = (ptrdiff_t)186;
        n_t = (ptrdiff_t)1;
        k_t = (ptrdiff_t)k;
        lda_t = (ptrdiff_t)186;
        ldb_t = (ptrdiff_t)y->size[0];
        ldc_t = (ptrdiff_t)186;
        dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &A_data[0], &lda_t,
              &y_data[0], &ldb_t, &beta1, &f1[0], &ldc_t);
      }
      for (b_i = 0; b_i <= 184; b_i += 2) {
        r = _mm_loadu_pd(&dUdt[b_i]);
        r1 = _mm_loadu_pd(&f1[b_i]);
        _mm_storeu_pd(&dUdt[b_i], _mm_add_pd(r, r1));
      }
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
