/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * expm.c
 *
 * Code generation for function 'expm'
 *
 */

/* Include files */
#include "expm.h"
#include "log2.h"
#include "mpower.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/* Function Declarations */
static int32_T getExpmParams(const real_T A[196], real_T A2[196],
                             real_T A4[196], real_T A6[196], real_T *s);

static void recomputeBlockDiag(const real_T A[196], real_T F[196],
                               const int32_T blockFormat[13]);

/* Function Definitions */
static int32_T getExpmParams(const real_T A[196], real_T A2[196],
                             real_T A4[196], real_T A6[196], real_T *s)
{
  __m128d r;
  real_T T[196];
  real_T b_y[196];
  real_T b_dv[2];
  real_T dv1[2];
  real_T dv2[2];
  real_T dv3[2];
  real_T dv4[2];
  real_T dv5[2];
  real_T b_s;
  real_T c_y;
  real_T d6;
  real_T d8;
  real_T eta1;
  real_T y;
  int32_T eint;
  int32_T i;
  int32_T j;
  int32_T m;
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T guard3;
  boolean_T guard4;
  *s = 0.0;
  mtimes(A, A, A2);
  mtimes(A2, A2, A4);
  mtimes(A4, A2, A6);
  y = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 14)) {
    b_s = 0.0;
    for (i = 0; i < 14; i++) {
      b_s += muDoubleScalarAbs(A6[i + 14 * j]);
    }
    if (muDoubleScalarIsNaN(b_s)) {
      y = rtNaN;
      exitg1 = true;
    } else {
      if (b_s > y) {
        y = b_s;
      }
      j++;
    }
  }
  d6 = muDoubleScalarPower(y, 0.16666666666666666);
  y = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 14)) {
    b_s = 0.0;
    for (i = 0; i < 14; i++) {
      b_s += muDoubleScalarAbs(A4[i + 14 * j]);
    }
    if (muDoubleScalarIsNaN(b_s)) {
      y = rtNaN;
      exitg1 = true;
    } else {
      if (b_s > y) {
        y = b_s;
      }
      j++;
    }
  }
  eta1 = muDoubleScalarMax(muDoubleScalarPower(y, 0.25), d6);
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (eta1 <= 0.01495585217958292) {
    for (j = 0; j <= 194; j += 2) {
      b_dv[0] = muDoubleScalarAbs(A[j]);
      b_dv[1] = muDoubleScalarAbs(A[j + 1]);
      r = _mm_loadu_pd(&b_dv[0]);
      _mm_storeu_pd(&b_y[j], _mm_mul_pd(_mm_set1_pd(0.19285012468241128), r));
    }
    mpower(b_y, 7.0);
    y = 0.0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 14)) {
      b_s = 0.0;
      for (i = 0; i < 14; i++) {
        b_s += muDoubleScalarAbs(b_y[i + 14 * j]);
      }
      if (muDoubleScalarIsNaN(b_s)) {
        y = rtNaN;
        exitg1 = true;
      } else {
        if (b_s > y) {
          y = b_s;
        }
        j++;
      }
    }
    c_y = 0.0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 14)) {
      b_s = 0.0;
      for (i = 0; i < 14; i++) {
        b_s += muDoubleScalarAbs(A[i + 14 * j]);
      }
      if (muDoubleScalarIsNaN(b_s)) {
        c_y = rtNaN;
        exitg1 = true;
      } else {
        if (b_s > c_y) {
          c_y = b_s;
        }
        j++;
      }
    }
    if (muDoubleScalarMax(
            muDoubleScalarCeil(
                b_log2(2.0 * (y / c_y) / 2.2204460492503131E-16) / 6.0),
            0.0) == 0.0) {
      m = 3;
    } else {
      guard4 = true;
    }
  } else {
    guard4 = true;
  }
  if (guard4) {
    if (eta1 <= 0.253939833006323) {
      for (j = 0; j <= 194; j += 2) {
        dv1[0] = muDoubleScalarAbs(A[j]);
        dv1[1] = muDoubleScalarAbs(A[j + 1]);
        r = _mm_loadu_pd(&dv1[0]);
        _mm_storeu_pd(&b_y[j], _mm_mul_pd(_mm_set1_pd(0.12321872304378752), r));
      }
      mpower(b_y, 11.0);
      y = 0.0;
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j < 14)) {
        b_s = 0.0;
        for (i = 0; i < 14; i++) {
          b_s += muDoubleScalarAbs(b_y[i + 14 * j]);
        }
        if (muDoubleScalarIsNaN(b_s)) {
          y = rtNaN;
          exitg1 = true;
        } else {
          if (b_s > y) {
            y = b_s;
          }
          j++;
        }
      }
      c_y = 0.0;
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j < 14)) {
        b_s = 0.0;
        for (i = 0; i < 14; i++) {
          b_s += muDoubleScalarAbs(A[i + 14 * j]);
        }
        if (muDoubleScalarIsNaN(b_s)) {
          c_y = rtNaN;
          exitg1 = true;
        } else {
          if (b_s > c_y) {
            c_y = b_s;
          }
          j++;
        }
      }
      if (muDoubleScalarMax(
              muDoubleScalarCeil(
                  b_log2(2.0 * (y / c_y) / 2.2204460492503131E-16) / 10.0),
              0.0) == 0.0) {
        m = 5;
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  }
  if (guard3) {
    memcpy(&b_y[0], &A4[0], 196U * sizeof(real_T));
    mpower(b_y, 2.0);
    eta1 = 0.0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 14)) {
      b_s = 0.0;
      for (i = 0; i < 14; i++) {
        b_s += muDoubleScalarAbs(b_y[i + 14 * j]);
      }
      if (muDoubleScalarIsNaN(b_s)) {
        eta1 = rtNaN;
        exitg1 = true;
      } else {
        if (b_s > eta1) {
          eta1 = b_s;
        }
        j++;
      }
    }
    d8 = muDoubleScalarPower(eta1, 0.125);
    eta1 = muDoubleScalarMax(d6, d8);
    if (eta1 <= 0.95041789961629319) {
      for (j = 0; j <= 194; j += 2) {
        dv3[0] = muDoubleScalarAbs(A[j]);
        dv3[1] = muDoubleScalarAbs(A[j + 1]);
        r = _mm_loadu_pd(&dv3[0]);
        _mm_storeu_pd(&b_y[j],
                      _mm_mul_pd(_mm_set1_pd(0.090475336558796943), r));
      }
      mpower(b_y, 15.0);
      y = 0.0;
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j < 14)) {
        b_s = 0.0;
        for (i = 0; i < 14; i++) {
          b_s += muDoubleScalarAbs(b_y[i + 14 * j]);
        }
        if (muDoubleScalarIsNaN(b_s)) {
          y = rtNaN;
          exitg1 = true;
        } else {
          if (b_s > y) {
            y = b_s;
          }
          j++;
        }
      }
      c_y = 0.0;
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j < 14)) {
        b_s = 0.0;
        for (i = 0; i < 14; i++) {
          b_s += muDoubleScalarAbs(A[i + 14 * j]);
        }
        if (muDoubleScalarIsNaN(b_s)) {
          c_y = rtNaN;
          exitg1 = true;
        } else {
          if (b_s > c_y) {
            c_y = b_s;
          }
          j++;
        }
      }
      if (muDoubleScalarMax(
              muDoubleScalarCeil(
                  b_log2(2.0 * (y / c_y) / 2.2204460492503131E-16) / 14.0),
              0.0) == 0.0) {
        m = 7;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }
  if (guard2) {
    if (eta1 <= 2.097847961257068) {
      for (j = 0; j <= 194; j += 2) {
        dv2[0] = muDoubleScalarAbs(A[j]);
        dv2[1] = muDoubleScalarAbs(A[j + 1]);
        r = _mm_loadu_pd(&dv2[0]);
        _mm_storeu_pd(&b_y[j],
                      _mm_mul_pd(_mm_set1_pd(0.071467735648795785), r));
      }
      mpower(b_y, 19.0);
      y = 0.0;
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j < 14)) {
        b_s = 0.0;
        for (i = 0; i < 14; i++) {
          b_s += muDoubleScalarAbs(b_y[i + 14 * j]);
        }
        if (muDoubleScalarIsNaN(b_s)) {
          y = rtNaN;
          exitg1 = true;
        } else {
          if (b_s > y) {
            y = b_s;
          }
          j++;
        }
      }
      c_y = 0.0;
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j < 14)) {
        b_s = 0.0;
        for (i = 0; i < 14; i++) {
          b_s += muDoubleScalarAbs(A[i + 14 * j]);
        }
        if (muDoubleScalarIsNaN(b_s)) {
          c_y = rtNaN;
          exitg1 = true;
        } else {
          if (b_s > c_y) {
            c_y = b_s;
          }
          j++;
        }
      }
      if (muDoubleScalarMax(
              muDoubleScalarCeil(
                  b_log2(2.0 * (y / c_y) / 2.2204460492503131E-16) / 18.0),
              0.0) == 0.0) {
        m = 9;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }
  if (guard1) {
    mtimes(A4, A6, b_y);
    y = 0.0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 14)) {
      b_s = 0.0;
      for (i = 0; i < 14; i++) {
        b_s += muDoubleScalarAbs(b_y[i + 14 * j]);
      }
      if (muDoubleScalarIsNaN(b_s)) {
        y = rtNaN;
        exitg1 = true;
      } else {
        if (b_s > y) {
          y = b_s;
        }
        j++;
      }
    }
    *s = muDoubleScalarMax(
        muDoubleScalarCeil(b_log2(
            muDoubleScalarMin(
                eta1, muDoubleScalarMax(d8, muDoubleScalarPower(y, 0.1))) /
            5.3719203511481517)),
        0.0);
    eta1 = muDoubleScalarPower(2.0, *s);
    for (j = 0; j <= 194; j += 2) {
      r = _mm_div_pd(_mm_loadu_pd(&A[j]), _mm_set1_pd(eta1));
      _mm_storeu_pd(&T[j], r);
      _mm_storeu_pd(&dv4[0], r);
      dv5[0] = muDoubleScalarAbs(dv4[0]);
      dv5[1] = muDoubleScalarAbs(dv4[1]);
      r = _mm_loadu_pd(&dv5[0]);
      _mm_storeu_pd(&b_y[j], _mm_mul_pd(_mm_set1_pd(0.05031554467093536), r));
    }
    mpower(b_y, 27.0);
    y = 0.0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 14)) {
      b_s = 0.0;
      for (i = 0; i < 14; i++) {
        b_s += muDoubleScalarAbs(b_y[i + 14 * j]);
      }
      if (muDoubleScalarIsNaN(b_s)) {
        y = rtNaN;
        exitg1 = true;
      } else {
        if (b_s > y) {
          y = b_s;
        }
        j++;
      }
    }
    c_y = 0.0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 14)) {
      b_s = 0.0;
      for (i = 0; i < 14; i++) {
        b_s += muDoubleScalarAbs(T[i + 14 * j]);
      }
      if (muDoubleScalarIsNaN(b_s)) {
        c_y = rtNaN;
        exitg1 = true;
      } else {
        if (b_s > c_y) {
          c_y = b_s;
        }
        j++;
      }
    }
    *s += muDoubleScalarMax(
        muDoubleScalarCeil(b_log2(2.0 * (y / c_y) / 2.2204460492503131E-16) /
                           26.0),
        0.0);
    if (muDoubleScalarIsInf(*s)) {
      y = 0.0;
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j < 14)) {
        b_s = 0.0;
        for (i = 0; i < 14; i++) {
          b_s += muDoubleScalarAbs(A[i + 14 * j]);
        }
        if (muDoubleScalarIsNaN(b_s)) {
          y = rtNaN;
          exitg1 = true;
        } else {
          if (b_s > y) {
            y = b_s;
          }
          j++;
        }
      }
      eta1 = y / 5.3719203511481517;
      if ((!muDoubleScalarIsInf(eta1)) && (!muDoubleScalarIsNaN(eta1))) {
        eta1 = frexp(eta1, &eint);
      } else {
        eint = 0;
      }
      *s = eint;
      if (eta1 == 0.5) {
        *s = (real_T)eint - 1.0;
      }
    }
    m = 13;
  }
  return m;
}

static void recomputeBlockDiag(const real_T A[196], real_T F[196],
                               const int32_T blockFormat[13])
{
  int32_T j;
  for (j = 0; j < 13; j++) {
    int32_T delta_tmp;
    delta_tmp = blockFormat[j];
    if (delta_tmp != 0) {
      if (delta_tmp == 1) {
        real_T avg;
        real_T b_delta_tmp;
        real_T delta;
        real_T expa;
        real_T sinchdelta;
        int32_T delta_tmp_tmp;
        delta_tmp = j + 14 * j;
        delta = A[delta_tmp];
        expa = muDoubleScalarExp(delta);
        delta_tmp_tmp = j + 14 * (j + 1);
        b_delta_tmp = A[delta_tmp_tmp + 1];
        sinchdelta = muDoubleScalarExp(b_delta_tmp);
        avg = (delta + b_delta_tmp) / 2.0;
        if (muDoubleScalarMax(avg, muDoubleScalarAbs(delta - b_delta_tmp) /
                                       2.0) < 709.782712893384) {
          delta = (b_delta_tmp - delta) / 2.0;
          if (delta == 0.0) {
            delta = 1.0;
          } else {
            delta = muDoubleScalarSinh(delta) / delta;
          }
          delta *= A[delta_tmp_tmp] * muDoubleScalarExp(avg);
        } else {
          delta =
              A[delta_tmp_tmp] * (sinchdelta - expa) / (b_delta_tmp - delta);
        }
        F[delta_tmp] = expa;
        F[delta_tmp_tmp] = delta;
        F[delta_tmp_tmp + 1] = sinchdelta;
      } else if (delta_tmp == 2) {
        real_T avg;
        real_T b_delta_tmp;
        real_T delta;
        real_T expa;
        real_T sinchdelta;
        int32_T delta_tmp_tmp;
        delta_tmp = j + 14 * j;
        b_delta_tmp = A[delta_tmp + 1];
        delta_tmp_tmp = j + 14 * (j + 1);
        avg = A[delta_tmp_tmp];
        delta = muDoubleScalarSqrt(muDoubleScalarAbs(avg * b_delta_tmp));
        expa = muDoubleScalarExp(A[delta_tmp]);
        if (delta == 0.0) {
          sinchdelta = 1.0;
        } else {
          sinchdelta = muDoubleScalarSin(delta) / delta;
        }
        delta = expa * muDoubleScalarCos(delta);
        F[delta_tmp] = delta;
        F[delta_tmp + 1] = expa * b_delta_tmp * sinchdelta;
        F[delta_tmp_tmp] = expa * avg * sinchdelta;
        F[delta_tmp_tmp + 1] = delta;
      }
    }
  }
  if (blockFormat[12] == 0) {
    F[195] = muDoubleScalarExp(A[195]);
  }
}

void expm(real_T A[196], real_T F[196])
{
  ptrdiff_t IPIV[14];
  ptrdiff_t INFO;
  ptrdiff_t LDA;
  ptrdiff_t N;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  real_T A2[196];
  real_T A4[196];
  real_T A6[196];
  real_T V[196];
  real_T b_A6[196];
  real_T y[196];
  real_T w[14];
  real_T beta1;
  real_T d;
  real_T s;
  int32_T i;
  int32_T j;
  int32_T k;
  int32_T m;
  char_T TRANSA1;
  char_T TRANSB1;
  boolean_T recomputeDiags;
  recomputeDiags = true;
  for (k = 0; k < 196; k++) {
    if (recomputeDiags) {
      d = A[k];
      if (muDoubleScalarIsInf(d) || muDoubleScalarIsNaN(d)) {
        recomputeDiags = false;
      }
    } else {
      recomputeDiags = false;
    }
  }
  if (!recomputeDiags) {
    for (i = 0; i < 196; i++) {
      F[i] = rtNaN;
    }
  } else {
    int32_T exitg1;
    boolean_T exitg2;
    recomputeDiags = true;
    j = 0;
    exitg2 = false;
    while ((!exitg2) && (j < 14)) {
      k = 0;
      do {
        exitg1 = 0;
        if (k < 14) {
          if ((k != j) && (!(A[k + 14 * j] == 0.0))) {
            recomputeDiags = false;
            exitg1 = 1;
          } else {
            k++;
          }
        } else {
          j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (recomputeDiags) {
      memset(&F[0], 0, 196U * sizeof(real_T));
      for (k = 0; k < 14; k++) {
        j = k + 14 * k;
        F[j] = muDoubleScalarExp(A[j]);
      }
    } else {
      recomputeDiags = true;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j < 14)) {
        k = 0;
        do {
          exitg1 = 0;
          if (k <= j) {
            if (!(A[k + 14 * j] == A[j + 14 * k])) {
              recomputeDiags = false;
              exitg1 = 1;
            } else {
              k++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (recomputeDiags) {
        memcpy(&y[0], &A[0], 196U * sizeof(real_T));
        N = (ptrdiff_t)14;
        N = LAPACKE_dsyev(102, 'V', 'L', N, &y[0], N, &w[0]);
        if ((int32_T)N < 0) {
          for (k = 0; k < 14; k++) {
            w[k] = rtNaN;
          }
          for (i = 0; i < 196; i++) {
            y[i] = rtNaN;
          }
        }
        for (j = 0; j < 14; j++) {
          d = muDoubleScalarExp(w[j]);
          for (k = 0; k <= 12; k += 2) {
            __m128d r;
            i = k + 14 * j;
            r = _mm_loadu_pd(&y[i]);
            _mm_storeu_pd(&F[i], _mm_mul_pd(r, _mm_set1_pd(d)));
          }
        }
        memcpy(&V[0], &F[0], 196U * sizeof(real_T));
        TRANSB1 = 'T';
        TRANSA1 = 'N';
        d = 1.0;
        beta1 = 0.0;
        LDA = (ptrdiff_t)14;
        N = (ptrdiff_t)14;
        INFO = (ptrdiff_t)14;
        lda_t = (ptrdiff_t)14;
        ldb_t = (ptrdiff_t)14;
        ldc_t = (ptrdiff_t)14;
        dgemm(&TRANSA1, &TRANSB1, &LDA, &N, &INFO, &d, &V[0], &lda_t, &y[0],
              &ldb_t, &beta1, &F[0], &ldc_t);
        for (i = 0; i < 14; i++) {
          for (m = 0; m < 14; m++) {
            j = m + 14 * i;
            b_A6[j] = (F[j] + F[i + 14 * m]) / 2.0;
          }
        }
        memcpy(&F[0], &b_A6[0], 196U * sizeof(real_T));
      } else {
        __m128d r;
        __m128d r1;
        int32_T blockFormat[13];
        recomputeDiags = true;
        j = 3;
        while (recomputeDiags && (j <= 14)) {
          k = j;
          while (recomputeDiags && (k <= 14)) {
            recomputeDiags = (A[(k + 14 * (j - 3)) - 1] == 0.0);
            k++;
          }
          j++;
        }
        if (recomputeDiags) {
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k < 13)) {
            i = k + 14 * k;
            d = A[i + 1];
            if (d != 0.0) {
              if ((k + 1 != 13) && (A[(k + 14 * (k + 1)) + 2] != 0.0)) {
                recomputeDiags = false;
                exitg2 = true;
              } else {
                m = k + 14 * (k + 1);
                if ((A[i] != A[m + 1]) ||
                    (muDoubleScalarSign(d) * muDoubleScalarSign(A[m]) !=
                     -1.0)) {
                  recomputeDiags = false;
                  exitg2 = true;
                } else {
                  k++;
                }
              }
            } else {
              k++;
            }
          }
        }
        m = getExpmParams(A, A2, A4, A6, &s);
        if (s != 0.0) {
          real_T b_y;
          real_T c_y;
          d = muDoubleScalarPower(2.0, s);
          beta1 = muDoubleScalarPower(2.0, 2.0 * s);
          b_y = muDoubleScalarPower(2.0, 4.0 * s);
          c_y = muDoubleScalarPower(2.0, 6.0 * s);
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&A[i]);
            _mm_storeu_pd(&A[i], _mm_div_pd(r, _mm_set1_pd(d)));
            r = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(&A2[i], _mm_div_pd(r, _mm_set1_pd(beta1)));
            r = _mm_loadu_pd(&A4[i]);
            _mm_storeu_pd(&A4[i], _mm_div_pd(r, _mm_set1_pd(b_y)));
            r = _mm_loadu_pd(&A6[i]);
            _mm_storeu_pd(&A6[i], _mm_div_pd(r, _mm_set1_pd(c_y)));
          }
        }
        if (recomputeDiags) {
          for (k = 0; k < 13; k++) {
            blockFormat[k] = 0;
          }
          j = 0;
          while (j + 1 < 13) {
            if (A[(j + 14 * j) + 1] != 0.0) {
              blockFormat[j] = 2;
              blockFormat[j + 1] = 0;
              j += 2;
            } else if (A[(j + 14 * (j + 1)) + 2] == 0.0) {
              blockFormat[j] = 1;
              j++;
            } else {
              blockFormat[j] = 0;
              j++;
            }
          }
          if (A[181] != 0.0) {
            blockFormat[12] = 2;
          } else if ((blockFormat[11] == 0) || (blockFormat[11] == 1)) {
            blockFormat[12] = 1;
          }
        }
        if (m == 3) {
          memcpy(&F[0], &A2[0], 196U * sizeof(real_T));
          for (k = 0; k < 14; k++) {
            j = k + 14 * k;
            F[j] += 60.0;
          }
          memcpy(&b_A6[0], &F[0], 196U * sizeof(real_T));
          mtimes(A, b_A6, F);
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(&V[i], _mm_mul_pd(_mm_set1_pd(12.0), r));
          }
          d = 120.0;
        } else if (m == 5) {
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&A2[i]);
            r1 = _mm_loadu_pd(&A4[i]);
            _mm_storeu_pd(&F[i],
                          _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(420.0), r)));
          }
          for (k = 0; k < 14; k++) {
            j = k + 14 * k;
            F[j] += 15120.0;
          }
          memcpy(&b_A6[0], &F[0], 196U * sizeof(real_T));
          mtimes(A, b_A6, F);
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&A4[i]);
            r1 = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(&V[i],
                          _mm_add_pd(_mm_mul_pd(_mm_set1_pd(30.0), r),
                                     _mm_mul_pd(_mm_set1_pd(3360.0), r1)));
          }
          d = 30240.0;
        } else if (m == 7) {
          __m128d r2;
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&A4[i]);
            r1 = _mm_loadu_pd(&A6[i]);
            r2 = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(
                &F[i],
                _mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(1512.0), r)),
                           _mm_mul_pd(_mm_set1_pd(277200.0), r2)));
          }
          for (k = 0; k < 14; k++) {
            j = k + 14 * k;
            F[j] += 8.64864E+6;
          }
          memcpy(&b_A6[0], &F[0], 196U * sizeof(real_T));
          mtimes(A, b_A6, F);
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&A6[i]);
            r1 = _mm_loadu_pd(&A4[i]);
            r2 = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(
                &V[i],
                _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(56.0), r),
                                      _mm_mul_pd(_mm_set1_pd(25200.0), r1)),
                           _mm_mul_pd(_mm_set1_pd(1.99584E+6), r2)));
          }
          d = 1.729728E+7;
        } else if (m == 9) {
          __m128d r2;
          __m128d r3;
          mtimes(A6, A2, V);
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&A6[i]);
            r1 = _mm_loadu_pd(&V[i]);
            r2 = _mm_loadu_pd(&A4[i]);
            r3 = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(
                &F[i],
                _mm_add_pd(
                    _mm_add_pd(
                        _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(3960.0), r)),
                        _mm_mul_pd(_mm_set1_pd(2.16216E+6), r2)),
                    _mm_mul_pd(_mm_set1_pd(3.027024E+8), r3)));
          }
          for (k = 0; k < 14; k++) {
            j = k + 14 * k;
            F[j] += 8.8216128E+9;
          }
          memcpy(&b_A6[0], &F[0], 196U * sizeof(real_T));
          mtimes(A, b_A6, F);
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&V[i]);
            r1 = _mm_loadu_pd(&A6[i]);
            r2 = _mm_loadu_pd(&A4[i]);
            r3 = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(
                &V[i],
                _mm_add_pd(
                    _mm_add_pd(
                        _mm_add_pd(_mm_mul_pd(_mm_set1_pd(90.0), r),
                                   _mm_mul_pd(_mm_set1_pd(110880.0), r1)),
                        _mm_mul_pd(_mm_set1_pd(3.027024E+7), r2)),
                    _mm_mul_pd(_mm_set1_pd(2.0756736E+9), r3)));
          }
          d = 1.76432256E+10;
        } else {
          __m128d r2;
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&A6[i]);
            r1 = _mm_loadu_pd(&A4[i]);
            r2 = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(
                &F[i],
                _mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(_mm_set1_pd(3.352212864E+10), r),
                               _mm_mul_pd(_mm_set1_pd(1.05594705216E+13), r1)),
                    _mm_mul_pd(_mm_set1_pd(1.1873537964288E+15), r2)));
          }
          for (k = 0; k < 14; k++) {
            j = k + 14 * k;
            F[j] += 3.238237626624E+16;
          }
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&A4[i]);
            r1 = _mm_loadu_pd(&A6[i]);
            r2 = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(
                &b_A6[i],
                _mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(16380.0), r)),
                           _mm_mul_pd(_mm_set1_pd(4.08408E+7), r2)));
          }
          mtimes(A6, b_A6, y);
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&y[i]);
            r1 = _mm_loadu_pd(&F[i]);
            _mm_storeu_pd(&y[i], _mm_add_pd(r, r1));
          }
          mtimes(A, y, F);
          for (i = 0; i <= 194; i += 2) {
            r = _mm_loadu_pd(&A6[i]);
            r1 = _mm_loadu_pd(&A4[i]);
            r2 = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(
                &b_A6[i],
                _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(182.0), r),
                                      _mm_mul_pd(_mm_set1_pd(960960.0), r1)),
                           _mm_mul_pd(_mm_set1_pd(1.32324192E+9), r2)));
          }
          mtimes(A6, b_A6, V);
          for (i = 0; i <= 194; i += 2) {
            __m128d r3;
            r = _mm_loadu_pd(&A6[i]);
            r1 = _mm_loadu_pd(&V[i]);
            r2 = _mm_loadu_pd(&A4[i]);
            r3 = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(
                &V[i],
                _mm_add_pd(
                    _mm_add_pd(
                        _mm_add_pd(r1,
                                   _mm_mul_pd(_mm_set1_pd(6.704425728E+11), r)),
                        _mm_mul_pd(_mm_set1_pd(1.29060195264E+14), r2)),
                    _mm_mul_pd(_mm_set1_pd(7.7717703038976E+15), r3)));
          }
          d = 6.476475253248E+16;
        }
        for (k = 0; k < 14; k++) {
          j = k + 14 * k;
          V[j] += d;
        }
        for (k = 0; k <= 194; k += 2) {
          r = _mm_loadu_pd(&V[k]);
          r1 = _mm_loadu_pd(&F[k]);
          _mm_storeu_pd(&V[k], _mm_sub_pd(r, r1));
          _mm_storeu_pd(&F[k], _mm_mul_pd(_mm_set1_pd(2.0), r1));
        }
        N = (ptrdiff_t)14;
        LDA = (ptrdiff_t)14;
        INFO = LAPACKE_dgetrf_work(102, N, N, &V[0], LDA, &IPIV[0]);
        if ((int32_T)INFO < 0) {
          for (i = 0; i < 196; i++) {
            F[i] = rtNaN;
          }
        } else {
          LAPACKE_dgetrs_work(102, 'N', N, (ptrdiff_t)14, &V[0], LDA, &IPIV[0],
                              &F[0], (ptrdiff_t)14);
        }
        for (k = 0; k < 14; k++) {
          j = k + 14 * k;
          F[j]++;
        }
        if (recomputeDiags) {
          recomputeBlockDiag(A, F, blockFormat);
        }
        i = (int32_T)s;
        for (k = 0; k < i; k++) {
          memcpy(&b_A6[0], &F[0], 196U * sizeof(real_T));
          mtimes(b_A6, b_A6, F);
          if (recomputeDiags) {
            for (m = 0; m <= 194; m += 2) {
              r = _mm_loadu_pd(&A[m]);
              _mm_storeu_pd(&A[m], _mm_mul_pd(_mm_set1_pd(2.0), r));
            }
            recomputeBlockDiag(A, F, blockFormat);
          }
        }
      }
    }
  }
}

/* End of code generation (expm.c) */
