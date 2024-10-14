/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * getHamiltonianDerivatives.c
 *
 * Code generation for function 'getHamiltonianDerivatives'
 *
 */

/* Include files */
#include "getHamiltonianDerivatives.h"
#include "nlmpc_config__costFcnJacobian.h"
#include "nlmpc_config__ineqConFcn.h"
#include "nlmpc_config__stateFcnJacobian.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void b_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[2],
                                 const real_T d[4], const real_T muineq[4],
                                 const real_T mudummy[4], const real_T dmv[4],
                                 const real_T pv[123], real_T Hx[14],
                                 real_T Hu[10], real_T Cbds[4])
{
  real_T A[196];
  real_T B[56];
  real_T Cx[56];
  real_T Cbdsmv[16];
  real_T Cmv[16];
  real_T a__3[16];
  real_T Jx[14];
  real_T Ce[8];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[2];
  real_T b_Je[2];
  real_T b_d;
  real_T d1;
  real_T d2;
  real_T d3;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv[3], iv1[3], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv[13], iv1[13], &Cbdsmv[5]);
  Cbds[2] = boundsToConstraints(u[2], iv[23], iv1[23], &Cbdsmv[10]);
  Cbds[3] = boundsToConstraints(u[3], iv[33], iv1[33], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    i1 = i << 2;
    d1 = ((Cx[i1] * muineq[0] + Cx[i1 + 1] * muineq[1]) +
          Cx[i1 + 2] * muineq[2]) +
         Cx[i1 + 3] * muineq[3];
    Jx[i] = d1;
    Hx[i] = (b_d + d1) +
            (((0.0 * mudummy[0] + 0.0 * mudummy[1]) + 0.0 * mudummy[2]) +
             0.0 * mudummy[3]);
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    i1 = i << 2;
    Jmv[i] =
        ((Jmv[i] + b_d) + (((Cmv[i1] * muineq[0] + Cmv[i1 + 1] * muineq[1]) +
                            Cmv[i1 + 2] * muineq[2]) +
                           Cmv[i1 + 3] * muineq[3])) +
        (((Cbdsmv[i1] * mudummy[0] + Cbdsmv[i1 + 1] * mudummy[1]) +
          Cbdsmv[i1 + 2] * mudummy[2]) +
         Cbdsmv[i1 + 3] * mudummy[3]);
  }
  b_d = muineq[0];
  d1 = muineq[1];
  d2 = muineq[2];
  d3 = muineq[3];
  for (i = 0; i < 2; i++) {
    i1 = i << 2;
    b_Je[i] = Je[i] + (((Ce[i1] * b_d + Ce[i1 + 1] * d1) + Ce[i1 + 2] * d2) +
                       Ce[i1 + 3] * d3);
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(500.0);
  _mm_storeu_pd(&Hu[6],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[8],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
}

real_T boundsToConstraints(real_T z, real_T zlb, real_T zub, real_T *cz)
{
  real_T c;
  c = ((zlb * zub - z * zub) - z * zlb) + z * z;
  *cz = (2.0 * z - zub) - zlb;
  return c;
}

void c_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[2],
                                 const real_T d[4], const real_T muineq[4],
                                 const real_T mudummy[4], const real_T dmv[4],
                                 const real_T pv[123], real_T Hx[14],
                                 real_T Hu[10], real_T Cbds[4])
{
  real_T A[196];
  real_T B[56];
  real_T Cx[56];
  real_T Cbdsmv[16];
  real_T Cmv[16];
  real_T a__3[16];
  real_T Jx[14];
  real_T Ce[8];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[2];
  real_T b_Je[2];
  real_T b_d;
  real_T d1;
  real_T d2;
  real_T d3;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv[2], iv1[2], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv[12], iv1[12], &Cbdsmv[5]);
  Cbds[2] = boundsToConstraints(u[2], iv[22], iv1[22], &Cbdsmv[10]);
  Cbds[3] = boundsToConstraints(u[3], iv[32], iv1[32], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    i1 = i << 2;
    d1 = ((Cx[i1] * muineq[0] + Cx[i1 + 1] * muineq[1]) +
          Cx[i1 + 2] * muineq[2]) +
         Cx[i1 + 3] * muineq[3];
    Jx[i] = d1;
    Hx[i] = (b_d + d1) +
            (((0.0 * mudummy[0] + 0.0 * mudummy[1]) + 0.0 * mudummy[2]) +
             0.0 * mudummy[3]);
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    i1 = i << 2;
    Jmv[i] =
        ((Jmv[i] + b_d) + (((Cmv[i1] * muineq[0] + Cmv[i1 + 1] * muineq[1]) +
                            Cmv[i1 + 2] * muineq[2]) +
                           Cmv[i1 + 3] * muineq[3])) +
        (((Cbdsmv[i1] * mudummy[0] + Cbdsmv[i1 + 1] * mudummy[1]) +
          Cbdsmv[i1 + 2] * mudummy[2]) +
         Cbdsmv[i1 + 3] * mudummy[3]);
  }
  b_d = muineq[0];
  d1 = muineq[1];
  d2 = muineq[2];
  d3 = muineq[3];
  for (i = 0; i < 2; i++) {
    i1 = i << 2;
    b_Je[i] = Je[i] + (((Ce[i1] * b_d + Ce[i1 + 1] * d1) + Ce[i1 + 2] * d2) +
                       Ce[i1 + 3] * d3);
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(500.0);
  _mm_storeu_pd(&Hu[6],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[8],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
}

void d_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[2],
                                 const real_T d[4], const real_T muineq[4],
                                 const real_T mudummy[4], const real_T dmv[4],
                                 const real_T pv[123], real_T Hx[14],
                                 real_T Hu[10], real_T Cbds[4])
{
  real_T A[196];
  real_T B[56];
  real_T Cx[56];
  real_T Cbdsmv[16];
  real_T Cmv[16];
  real_T a__3[16];
  real_T Jx[14];
  real_T Ce[8];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[2];
  real_T b_Je[2];
  real_T b_d;
  real_T d1;
  real_T d2;
  real_T d3;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv[1], iv1[1], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv[11], iv1[11], &Cbdsmv[5]);
  Cbds[2] = boundsToConstraints(u[2], iv[21], iv1[21], &Cbdsmv[10]);
  Cbds[3] = boundsToConstraints(u[3], iv[31], iv1[31], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    i1 = i << 2;
    d1 = ((Cx[i1] * muineq[0] + Cx[i1 + 1] * muineq[1]) +
          Cx[i1 + 2] * muineq[2]) +
         Cx[i1 + 3] * muineq[3];
    Jx[i] = d1;
    Hx[i] = (b_d + d1) +
            (((0.0 * mudummy[0] + 0.0 * mudummy[1]) + 0.0 * mudummy[2]) +
             0.0 * mudummy[3]);
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    i1 = i << 2;
    Jmv[i] =
        ((Jmv[i] + b_d) + (((Cmv[i1] * muineq[0] + Cmv[i1 + 1] * muineq[1]) +
                            Cmv[i1 + 2] * muineq[2]) +
                           Cmv[i1 + 3] * muineq[3])) +
        (((Cbdsmv[i1] * mudummy[0] + Cbdsmv[i1 + 1] * mudummy[1]) +
          Cbdsmv[i1 + 2] * mudummy[2]) +
         Cbdsmv[i1 + 3] * mudummy[3]);
  }
  b_d = muineq[0];
  d1 = muineq[1];
  d2 = muineq[2];
  d3 = muineq[3];
  for (i = 0; i < 2; i++) {
    i1 = i << 2;
    b_Je[i] = Je[i] + (((Ce[i1] * b_d + Ce[i1 + 1] * d1) + Ce[i1 + 2] * d2) +
                       Ce[i1 + 3] * d3);
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(500.0);
  _mm_storeu_pd(&Hu[6],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[8],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
}

void e_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[2],
                                 const real_T d[4], const real_T muineq[4],
                                 const real_T mudummy[4], const real_T dmv[4],
                                 const real_T pv[123], real_T Hx[14],
                                 real_T Hu[10], real_T Cbds[4])
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  real_T A[196];
  real_T B[56];
  real_T Gx[56];
  real_T Cbdsmv[16];
  real_T Gmv[16];
  real_T Jx[14];
  real_T b_x[14];
  real_T delta_x[14];
  real_T Ge[8];
  real_T Jmv[4];
  real_T delta_mv[4];
  real_T f0[4];
  real_T fpert[4];
  real_T Je[2];
  real_T b_dv[2];
  real_T delta_e[2];
  real_T dv1[2];
  real_T b_d;
  real_T d1;
  real_T d2;
  real_T d3;
  int32_T i;
  int32_T k;
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, fpert, Je);
  b_nlmpc_config__ineqConFcn(x, pv, f0);
  for (k = 0; k <= 12; k += 2) {
    _mm_storeu_pd(&b_x[k], _mm_loadu_pd(&x[k]));
    delta_e[0] = muDoubleScalarMax(muDoubleScalarAbs(x[k]), 1.0);
    delta_e[1] = muDoubleScalarMax(muDoubleScalarAbs(x[k + 1]), 1.0);
    r = _mm_loadu_pd(&delta_e[0]);
    _mm_storeu_pd(&delta_x[k], _mm_mul_pd(_mm_set1_pd(1.0E-7), r));
  }
  for (k = 0; k < 14; k++) {
    b_d = delta_x[k];
    b_x[k] += b_d;
    b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
    b_x[k] -= b_d;
    r = _mm_loadu_pd(&fpert[0]);
    r1 = _mm_loadu_pd(&f0[0]);
    i = k << 2;
    r2 = _mm_set1_pd(b_d);
    _mm_storeu_pd(&Gx[i], _mm_div_pd(_mm_sub_pd(r, r1), r2));
    r = _mm_loadu_pd(&fpert[2]);
    r1 = _mm_loadu_pd(&f0[2]);
    _mm_storeu_pd(&Gx[i + 2], _mm_div_pd(_mm_sub_pd(r, r1), r2));
  }
  b_dv[0] = muDoubleScalarMax(muDoubleScalarAbs(u[0]), 1.0);
  b_dv[1] = muDoubleScalarMax(muDoubleScalarAbs(u[1]), 1.0);
  r = _mm_loadu_pd(&b_dv[0]);
  r1 = _mm_set1_pd(1.0E-7);
  _mm_storeu_pd(&delta_mv[0], _mm_mul_pd(r1, r));
  b_dv[0] = muDoubleScalarMax(muDoubleScalarAbs(u[2]), 1.0);
  b_dv[1] = muDoubleScalarMax(muDoubleScalarAbs(u[3]), 1.0);
  r = _mm_loadu_pd(&b_dv[0]);
  _mm_storeu_pd(&delta_mv[2], _mm_mul_pd(r1, r));
  for (k = 0; k < 4; k++) {
    __m128d r3;
    b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
    r = _mm_loadu_pd(&fpert[0]);
    r2 = _mm_loadu_pd(&f0[0]);
    i = k << 2;
    r3 = _mm_set1_pd(delta_mv[k]);
    _mm_storeu_pd(&Gmv[i], _mm_div_pd(_mm_sub_pd(r, r2), r3));
    r = _mm_loadu_pd(&fpert[2]);
    r2 = _mm_loadu_pd(&f0[2]);
    _mm_storeu_pd(&Gmv[i + 2], _mm_div_pd(_mm_sub_pd(r, r2), r3));
  }
  b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
  b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
  b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
  b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
  dv1[0] = muDoubleScalarMax(muDoubleScalarAbs(e[0]), 1.0);
  dv1[1] = muDoubleScalarMax(muDoubleScalarAbs(e[1]), 1.0);
  r = _mm_loadu_pd(&dv1[0]);
  _mm_storeu_pd(&delta_e[0], _mm_mul_pd(r1, r));
  b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
  r = _mm_loadu_pd(&fpert[0]);
  r1 = _mm_loadu_pd(&f0[0]);
  r2 = _mm_set1_pd(delta_e[0]);
  _mm_storeu_pd(&Ge[0], _mm_div_pd(_mm_sub_pd(r, r1), r2));
  r = _mm_loadu_pd(&fpert[2]);
  r1 = _mm_loadu_pd(&f0[2]);
  _mm_storeu_pd(&Ge[2], _mm_div_pd(_mm_sub_pd(r, r1), r2));
  b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
  r = _mm_loadu_pd(&fpert[0]);
  r1 = _mm_loadu_pd(&f0[0]);
  r2 = _mm_set1_pd(delta_e[1]);
  _mm_storeu_pd(&Ge[4], _mm_div_pd(_mm_sub_pd(r, r1), r2));
  r = _mm_loadu_pd(&fpert[2]);
  r1 = _mm_loadu_pd(&f0[2]);
  _mm_storeu_pd(&Ge[6], _mm_div_pd(_mm_sub_pd(r, r1), r2));
  Cbds[0] = boundsToConstraints(u[0], iv[0], iv1[0], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv[10], iv1[10], &Cbdsmv[5]);
  Cbds[2] = boundsToConstraints(u[2], iv[20], iv1[20], &Cbdsmv[10]);
  Cbds[3] = boundsToConstraints(u[3], iv[30], iv1[30], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (k = 0; k < 14; k++) {
      b_d += A[k + 14 * i] * lbd[k];
    }
    k = i << 2;
    Hx[i] = ((Jx[i] + b_d) + (((Gx[k] * muineq[0] + Gx[k + 1] * muineq[1]) +
                               Gx[k + 2] * muineq[2]) +
                              Gx[k + 3] * muineq[3])) +
            (((0.0 * mudummy[0] + 0.0 * mudummy[1]) + 0.0 * mudummy[2]) +
             0.0 * mudummy[3]);
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (k = 0; k < 14; k++) {
      b_d += B[k + 14 * i] * lbd[k];
    }
    k = i << 2;
    Jmv[i] = ((Jmv[i] + b_d) + (((Gmv[k] * muineq[0] + Gmv[k + 1] * muineq[1]) +
                                 Gmv[k + 2] * muineq[2]) +
                                Gmv[k + 3] * muineq[3])) +
             (((Cbdsmv[k] * mudummy[0] + Cbdsmv[k + 1] * mudummy[1]) +
               Cbdsmv[k + 2] * mudummy[2]) +
              Cbdsmv[k + 3] * mudummy[3]);
  }
  b_d = muineq[0];
  d1 = muineq[1];
  d2 = muineq[2];
  d3 = muineq[3];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    delta_e[i] = Je[i] + (((Ge[k] * b_d + Ge[k + 1] * d1) + Ge[k + 2] * d2) +
                          Ge[k + 3] * d3);
  }
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = delta_e[0];
  Hu[5] = delta_e[1];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(500.0);
  _mm_storeu_pd(&Hu[6],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[8],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
}

void evaluateJacobian(const real_T x[14], const real_T u[4],
                      const real_T dmv[4], const real_T e[2],
                      const real_T pvstage[123], real_T Gx[56], real_T Gmv[16],
                      real_T Gdmv[16], real_T Ge[8])
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  __m128d r3;
  real_T b_x[14];
  real_T delta_x[14];
  real_T delta_mv[4];
  real_T f0[4];
  real_T fpert[4];
  real_T b_dv[2];
  real_T b_e[2];
  real_T delta_e[2];
  real_T dv1[2];
  real_T dv2[2];
  int32_T i;
  int32_T k;
  nlmpc_config__ineqConFcn(x, e, pvstage, f0);
  memcpy(&b_x[0], &x[0], 14U * sizeof(real_T));
  b_e[1] = e[1];
  for (k = 0; k <= 12; k += 2) {
    delta_e[0] = muDoubleScalarMax(muDoubleScalarAbs(x[k]), 1.0);
    delta_e[1] = muDoubleScalarMax(muDoubleScalarAbs(x[k + 1]), 1.0);
    r = _mm_loadu_pd(&delta_e[0]);
    _mm_storeu_pd(&delta_x[k], _mm_mul_pd(_mm_set1_pd(1.0E-7), r));
  }
  for (k = 0; k < 14; k++) {
    real_T d;
    d = delta_x[k];
    b_x[k] += d;
    nlmpc_config__ineqConFcn(b_x, e, pvstage, fpert);
    b_x[k] -= d;
    r = _mm_loadu_pd(&fpert[0]);
    r1 = _mm_loadu_pd(&f0[0]);
    i = k << 2;
    r2 = _mm_set1_pd(d);
    _mm_storeu_pd(&Gx[i], _mm_div_pd(_mm_sub_pd(r, r1), r2));
    r = _mm_loadu_pd(&fpert[2]);
    r1 = _mm_loadu_pd(&f0[2]);
    _mm_storeu_pd(&Gx[i + 2], _mm_div_pd(_mm_sub_pd(r, r1), r2));
  }
  b_dv[0] = muDoubleScalarMax(muDoubleScalarAbs(u[0]), 1.0);
  b_dv[1] = muDoubleScalarMax(muDoubleScalarAbs(u[1]), 1.0);
  r = _mm_loadu_pd(&b_dv[0]);
  r1 = _mm_set1_pd(1.0E-7);
  _mm_storeu_pd(&delta_mv[0], _mm_mul_pd(r1, r));
  b_dv[0] = muDoubleScalarMax(muDoubleScalarAbs(u[2]), 1.0);
  b_dv[1] = muDoubleScalarMax(muDoubleScalarAbs(u[3]), 1.0);
  r = _mm_loadu_pd(&b_dv[0]);
  _mm_storeu_pd(&delta_mv[2], _mm_mul_pd(r1, r));
  for (k = 0; k < 4; k++) {
    nlmpc_config__ineqConFcn(b_x, e, pvstage, fpert);
    r = _mm_loadu_pd(&fpert[0]);
    r2 = _mm_loadu_pd(&f0[0]);
    i = k << 2;
    r3 = _mm_set1_pd(delta_mv[k]);
    _mm_storeu_pd(&Gmv[i], _mm_div_pd(_mm_sub_pd(r, r2), r3));
    r = _mm_loadu_pd(&fpert[2]);
    r2 = _mm_loadu_pd(&f0[2]);
    _mm_storeu_pd(&Gmv[i + 2], _mm_div_pd(_mm_sub_pd(r, r2), r3));
  }
  dv1[0] = muDoubleScalarMax(muDoubleScalarAbs(dmv[0]), 1.0);
  dv1[1] = muDoubleScalarMax(muDoubleScalarAbs(dmv[1]), 1.0);
  r = _mm_loadu_pd(&dv1[0]);
  _mm_storeu_pd(&delta_mv[0], _mm_mul_pd(r1, r));
  dv1[0] = muDoubleScalarMax(muDoubleScalarAbs(dmv[2]), 1.0);
  dv1[1] = muDoubleScalarMax(muDoubleScalarAbs(dmv[3]), 1.0);
  r = _mm_loadu_pd(&dv1[0]);
  _mm_storeu_pd(&delta_mv[2], _mm_mul_pd(r1, r));
  for (k = 0; k < 4; k++) {
    nlmpc_config__ineqConFcn(b_x, e, pvstage, fpert);
    r = _mm_loadu_pd(&fpert[0]);
    r2 = _mm_loadu_pd(&f0[0]);
    i = k << 2;
    r3 = _mm_set1_pd(delta_mv[k]);
    _mm_storeu_pd(&Gdmv[i], _mm_div_pd(_mm_sub_pd(r, r2), r3));
    r = _mm_loadu_pd(&fpert[2]);
    r2 = _mm_loadu_pd(&f0[2]);
    _mm_storeu_pd(&Gdmv[i + 2], _mm_div_pd(_mm_sub_pd(r, r2), r3));
  }
  dv2[0] = muDoubleScalarMax(muDoubleScalarAbs(e[0]), 1.0);
  dv2[1] = muDoubleScalarMax(muDoubleScalarAbs(e[1]), 1.0);
  r = _mm_loadu_pd(&dv2[0]);
  _mm_storeu_pd(&delta_e[0], _mm_mul_pd(r1, r));
  b_e[0] = e[0] + delta_e[0];
  nlmpc_config__ineqConFcn(b_x, b_e, pvstage, fpert);
  b_e[0] -= delta_e[0];
  r = _mm_loadu_pd(&fpert[0]);
  r1 = _mm_loadu_pd(&f0[0]);
  r2 = _mm_set1_pd(delta_e[0]);
  _mm_storeu_pd(&Ge[0], _mm_div_pd(_mm_sub_pd(r, r1), r2));
  r = _mm_loadu_pd(&fpert[2]);
  r1 = _mm_loadu_pd(&f0[2]);
  _mm_storeu_pd(&Ge[2], _mm_div_pd(_mm_sub_pd(r, r1), r2));
  b_e[1] = e[1] + delta_e[1];
  nlmpc_config__ineqConFcn(b_x, b_e, pvstage, fpert);
  r = _mm_loadu_pd(&fpert[0]);
  r1 = _mm_loadu_pd(&f0[0]);
  r2 = _mm_set1_pd(delta_e[1]);
  _mm_storeu_pd(&Ge[4], _mm_div_pd(_mm_sub_pd(r, r1), r2));
  r = _mm_loadu_pd(&fpert[2]);
  r1 = _mm_loadu_pd(&f0[2]);
  _mm_storeu_pd(&Ge[6], _mm_div_pd(_mm_sub_pd(r, r1), r2));
}

void getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                               const real_T u[4], const real_T e[2],
                               const real_T d[4], const real_T muineq[4],
                               const real_T mudummy[4], const real_T dmv[4],
                               const real_T pv[123], real_T Hx[14],
                               real_T Hu[10], real_T Cbds[4])
{
  real_T A[196];
  real_T B[56];
  real_T Cx[56];
  real_T Cbdsmv[16];
  real_T Cmv[16];
  real_T a__3[16];
  real_T Jx[14];
  real_T Ce[8];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[2];
  real_T b_Je[2];
  real_T b_d;
  real_T d1;
  real_T d2;
  real_T d3;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv[4], iv1[4], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv[14], iv1[14], &Cbdsmv[5]);
  Cbds[2] = boundsToConstraints(u[2], iv[24], iv1[24], &Cbdsmv[10]);
  Cbds[3] = boundsToConstraints(u[3], iv[34], iv1[34], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    i1 = i << 2;
    d1 = ((Cx[i1] * muineq[0] + Cx[i1 + 1] * muineq[1]) +
          Cx[i1 + 2] * muineq[2]) +
         Cx[i1 + 3] * muineq[3];
    Jx[i] = d1;
    Hx[i] = (b_d + d1) +
            (((0.0 * mudummy[0] + 0.0 * mudummy[1]) + 0.0 * mudummy[2]) +
             0.0 * mudummy[3]);
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    i1 = i << 2;
    Jmv[i] =
        ((Jmv[i] + b_d) + (((Cmv[i1] * muineq[0] + Cmv[i1 + 1] * muineq[1]) +
                            Cmv[i1 + 2] * muineq[2]) +
                           Cmv[i1 + 3] * muineq[3])) +
        (((Cbdsmv[i1] * mudummy[0] + Cbdsmv[i1 + 1] * mudummy[1]) +
          Cbdsmv[i1 + 2] * mudummy[2]) +
         Cbdsmv[i1 + 3] * mudummy[3]);
  }
  b_d = muineq[0];
  d1 = muineq[1];
  d2 = muineq[2];
  d3 = muineq[3];
  for (i = 0; i < 2; i++) {
    i1 = i << 2;
    b_Je[i] = Je[i] + (((Ce[i1] * b_d + Ce[i1 + 1] * d1) + Ce[i1 + 2] * d2) +
                       Ce[i1 + 3] * d3);
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(500.0);
  _mm_storeu_pd(&Hu[6],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[8],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
}

/* End of code generation (getHamiltonianDerivatives.c) */
