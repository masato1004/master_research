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
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static const int16_T iv1[200] = {
    -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,
    -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,
    -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,
    -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,
    -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,
    -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,
    -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,
    -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,
    -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,  -800,
    -800,  -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000,
    -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000,
    -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000,
    -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000,
    -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000,
    -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000,
    -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000,
    -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000,
    -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000, -3000,
    -3000, -3000};

static const int16_T iv2[200] = {
    800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,
    800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,
    800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,
    800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,
    800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,
    800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,
    800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,
    800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,  800,
    800,  800,  800,  800,  3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};

static const int16_T iv3[200] = {
    -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600,
    -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600,
    -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600,
    -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600,
    -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600,
    -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600,
    -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600,
    -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600, -600,
    -600, -600, -600, -600, -100, -100, -100, -100, -100, -100, -100, -100,
    -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100,
    -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100,
    -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100,
    -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100,
    -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100,
    -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100,
    -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100,
    -100, -100, -100, -100, -100, -100, -100, -100};

static const int16_T iv4[200] = {
    600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600,
    600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600,
    600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600,
    600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600,
    600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600,
    600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600,
    600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100};

/* Function Declarations */
static real_T boundsToConstraints(real_T z, real_T zlb, real_T zub, real_T *cz);

static void evaluateJacobian(const real_T x[14], const real_T u[4],
                             const real_T dmv[4], const real_T e[3],
                             const real_T pvstage[323], real_T Gx[252],
                             real_T Gmv[72], real_T Gdmv[72], real_T Ge[54]);

/* Function Definitions */
static real_T boundsToConstraints(real_T z, real_T zlb, real_T zub, real_T *cz)
{
  real_T c;
  c = ((zlb * zub - z * zub) - z * zlb) + z * z;
  *cz = (2.0 * z - zub) - zlb;
  return c;
}

static void evaluateJacobian(const real_T x[14], const real_T u[4],
                             const real_T dmv[4], const real_T e[3],
                             const real_T pvstage[323], real_T Gx[252],
                             real_T Gmv[72], real_T Gdmv[72], real_T Ge[54])
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  real_T f0[18];
  real_T fpert[18];
  real_T b_x[14];
  real_T delta_x[14];
  real_T b_u[4];
  real_T delta_mv[4];
  real_T b_e[3];
  real_T delta_e[3];
  real_T b_dv[2];
  real_T b_dv1[2];
  real_T b_dv2[2];
  real_T b_dv3[2];
  real_T d;
  int32_T i;
  int32_T k;
  nlmpc_config__ineqConFcn(x, u, e, pvstage, f0);
  memcpy(&b_x[0], &x[0], 14U * sizeof(real_T));
  b_u[0] = u[0];
  b_u[1] = u[1];
  b_u[2] = u[2];
  b_u[3] = u[3];
  b_e[0] = e[0];
  b_e[1] = e[1];
  b_e[2] = e[2];
  for (k = 0; k <= 12; k += 2) {
    b_dv[0] = muDoubleScalarMax(muDoubleScalarAbs(x[k]), 1.0);
    b_dv[1] = muDoubleScalarMax(muDoubleScalarAbs(x[k + 1]), 1.0);
    r = _mm_loadu_pd(&b_dv[0]);
    _mm_storeu_pd(&delta_x[k], _mm_mul_pd(_mm_set1_pd(1.0E-7), r));
  }
  for (k = 0; k < 14; k++) {
    d = delta_x[k];
    b_x[k] += d;
    nlmpc_config__ineqConFcn(b_x, u, e, pvstage, fpert);
    b_x[k] -= d;
    for (i = 0; i <= 16; i += 2) {
      r = _mm_loadu_pd(&fpert[i]);
      r1 = _mm_loadu_pd(&f0[i]);
      _mm_storeu_pd(&Gx[i + 18 * k],
                    _mm_div_pd(_mm_sub_pd(r, r1), _mm_set1_pd(delta_x[k])));
    }
  }
  b_dv1[0] = muDoubleScalarMax(muDoubleScalarAbs(u[0]), 1.0);
  b_dv1[1] = muDoubleScalarMax(muDoubleScalarAbs(u[1]), 1.0);
  r = _mm_loadu_pd(&b_dv1[0]);
  r1 = _mm_set1_pd(1.0E-7);
  _mm_storeu_pd(&delta_mv[0], _mm_mul_pd(r1, r));
  b_dv1[0] = muDoubleScalarMax(muDoubleScalarAbs(u[2]), 1.0);
  b_dv1[1] = muDoubleScalarMax(muDoubleScalarAbs(u[3]), 1.0);
  r = _mm_loadu_pd(&b_dv1[0]);
  _mm_storeu_pd(&delta_mv[2], _mm_mul_pd(r1, r));
  for (k = 0; k < 4; k++) {
    d = delta_mv[k];
    b_u[k] += d;
    nlmpc_config__ineqConFcn(b_x, b_u, e, pvstage, fpert);
    b_u[k] -= d;
    for (i = 0; i <= 16; i += 2) {
      r = _mm_loadu_pd(&fpert[i]);
      r2 = _mm_loadu_pd(&f0[i]);
      _mm_storeu_pd(&Gmv[i + 18 * k],
                    _mm_div_pd(_mm_sub_pd(r, r2), _mm_set1_pd(delta_mv[k])));
    }
  }
  b_dv2[0] = muDoubleScalarMax(muDoubleScalarAbs(dmv[0]), 1.0);
  b_dv2[1] = muDoubleScalarMax(muDoubleScalarAbs(dmv[1]), 1.0);
  r = _mm_loadu_pd(&b_dv2[0]);
  _mm_storeu_pd(&delta_mv[0], _mm_mul_pd(r1, r));
  b_dv2[0] = muDoubleScalarMax(muDoubleScalarAbs(dmv[2]), 1.0);
  b_dv2[1] = muDoubleScalarMax(muDoubleScalarAbs(dmv[3]), 1.0);
  r = _mm_loadu_pd(&b_dv2[0]);
  _mm_storeu_pd(&delta_mv[2], _mm_mul_pd(r1, r));
  for (k = 0; k < 4; k++) {
    nlmpc_config__ineqConFcn(b_x, b_u, e, pvstage, fpert);
    for (i = 0; i <= 16; i += 2) {
      r = _mm_loadu_pd(&fpert[i]);
      r2 = _mm_loadu_pd(&f0[i]);
      _mm_storeu_pd(&Gdmv[i + 18 * k],
                    _mm_div_pd(_mm_sub_pd(r, r2), _mm_set1_pd(delta_mv[k])));
    }
  }
  b_dv3[0] = muDoubleScalarMax(muDoubleScalarAbs(e[0]), 1.0);
  b_dv3[1] = muDoubleScalarMax(muDoubleScalarAbs(e[1]), 1.0);
  r = _mm_loadu_pd(&b_dv3[0]);
  _mm_storeu_pd(&delta_e[0], _mm_mul_pd(r1, r));
  delta_e[2] = 1.0E-7 * muDoubleScalarMax(muDoubleScalarAbs(e[2]), 1.0);
  for (k = 0; k < 3; k++) {
    d = delta_e[k];
    b_e[k] += d;
    nlmpc_config__ineqConFcn(b_x, b_u, b_e, pvstage, fpert);
    b_e[k] -= d;
    for (i = 0; i <= 16; i += 2) {
      r = _mm_loadu_pd(&fpert[i]);
      r1 = _mm_loadu_pd(&f0[i]);
      _mm_storeu_pd(&Ge[i + 18 * k],
                    _mm_div_pd(_mm_sub_pd(r, r1), _mm_set1_pd(delta_e[k])));
    }
  }
}

void ab_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[24], iv2[24], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[74], iv2[74], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[124], iv2[124], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[174], iv2[174], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[24], iv4[24], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[74], iv4[74], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[124], iv4[124], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[174], iv4[174], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void b_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[48], iv2[48], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[98], iv2[98], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[148], iv2[148], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[198], iv2[198], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[48], iv4[48], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[98], iv4[98], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[148], iv4[148], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[198], iv4[198], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void bb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[23], iv2[23], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[73], iv2[73], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[123], iv2[123], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[173], iv2[173], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[23], iv4[23], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[73], iv4[73], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[123], iv4[123], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[173], iv4[173], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void c_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[47], iv2[47], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[97], iv2[97], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[147], iv2[147], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[197], iv2[197], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[47], iv4[47], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[97], iv4[97], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[147], iv4[147], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[197], iv4[197], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void cb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[22], iv2[22], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[72], iv2[72], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[122], iv2[122], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[172], iv2[172], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[22], iv4[22], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[72], iv4[72], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[122], iv4[122], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[172], iv4[172], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void d_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[46], iv2[46], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[96], iv2[96], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[146], iv2[146], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[196], iv2[196], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[46], iv4[46], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[96], iv4[96], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[146], iv4[146], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[196], iv4[196], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void db_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[21], iv2[21], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[71], iv2[71], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[121], iv2[121], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[171], iv2[171], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[21], iv4[21], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[71], iv4[71], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[121], iv4[121], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[171], iv4[171], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void e_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[45], iv2[45], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[95], iv2[95], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[145], iv2[145], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[195], iv2[195], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[45], iv4[45], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[95], iv4[95], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[145], iv4[145], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[195], iv4[195], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void eb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[20], iv2[20], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[70], iv2[70], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[120], iv2[120], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[170], iv2[170], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[20], iv4[20], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[70], iv4[70], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[120], iv4[120], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[170], iv4[170], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void f_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[44], iv2[44], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[94], iv2[94], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[144], iv2[144], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[194], iv2[194], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[44], iv4[44], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[94], iv4[94], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[144], iv4[144], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[194], iv4[194], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void fb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[19], iv2[19], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[69], iv2[69], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[119], iv2[119], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[169], iv2[169], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[19], iv4[19], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[69], iv4[69], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[119], iv4[119], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[169], iv4[169], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void g_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[43], iv2[43], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[93], iv2[93], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[143], iv2[143], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[193], iv2[193], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[43], iv4[43], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[93], iv4[93], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[143], iv4[143], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[193], iv4[193], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void gb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[18], iv2[18], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[68], iv2[68], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[118], iv2[118], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[168], iv2[168], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[18], iv4[18], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[68], iv4[68], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[118], iv4[118], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[168], iv4[168], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                               const real_T u[4], const real_T e[3],
                               const real_T d[8], const real_T muineq[18],
                               const real_T mudummy[8], const real_T dmv[4],
                               const real_T pv[323], real_T Hx[14],
                               real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[49], iv2[49], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[99], iv2[99], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[149], iv2[149], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[199], iv2[199], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[49], iv4[49], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[99], iv4[99], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[149], iv4[149], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[199], iv4[199], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void h_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[42], iv2[42], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[92], iv2[92], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[142], iv2[142], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[192], iv2[192], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[42], iv4[42], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[92], iv4[92], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[142], iv4[142], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[192], iv4[192], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void hb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[17], iv2[17], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[67], iv2[67], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[117], iv2[117], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[167], iv2[167], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[17], iv4[17], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[67], iv4[67], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[117], iv4[117], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[167], iv4[167], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void i_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[41], iv2[41], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[91], iv2[91], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[141], iv2[141], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[191], iv2[191], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[41], iv4[41], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[91], iv4[91], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[141], iv4[141], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[191], iv4[191], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void ib_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[16], iv2[16], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[66], iv2[66], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[116], iv2[116], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[166], iv2[166], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[16], iv4[16], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[66], iv4[66], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[116], iv4[116], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[166], iv4[166], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void j_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[40], iv2[40], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[90], iv2[90], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[140], iv2[140], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[190], iv2[190], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[40], iv4[40], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[90], iv4[90], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[140], iv4[140], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[190], iv4[190], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void jb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[15], iv2[15], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[65], iv2[65], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[115], iv2[115], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[165], iv2[165], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[15], iv4[15], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[65], iv4[65], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[115], iv4[115], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[165], iv4[165], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void k_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[39], iv2[39], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[89], iv2[89], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[139], iv2[139], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[189], iv2[189], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[39], iv4[39], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[89], iv4[89], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[139], iv4[139], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[189], iv4[189], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void kb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[14], iv2[14], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[64], iv2[64], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[114], iv2[114], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[164], iv2[164], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[14], iv4[14], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[64], iv4[64], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[114], iv4[114], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[164], iv4[164], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void l_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[38], iv2[38], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[88], iv2[88], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[138], iv2[138], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[188], iv2[188], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[38], iv4[38], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[88], iv4[88], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[138], iv4[138], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[188], iv4[188], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void lb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[13], iv2[13], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[63], iv2[63], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[113], iv2[113], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[163], iv2[163], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[13], iv4[13], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[63], iv4[63], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[113], iv4[113], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[163], iv4[163], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void m_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[37], iv2[37], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[87], iv2[87], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[137], iv2[137], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[187], iv2[187], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[37], iv4[37], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[87], iv4[87], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[137], iv4[137], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[187], iv4[187], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void mb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[12], iv2[12], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[62], iv2[62], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[112], iv2[112], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[162], iv2[162], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[12], iv4[12], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[62], iv4[62], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[112], iv4[112], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[162], iv4[162], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void n_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[36], iv2[36], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[86], iv2[86], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[136], iv2[136], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[186], iv2[186], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[36], iv4[36], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[86], iv4[86], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[136], iv4[136], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[186], iv4[186], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void nb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[11], iv2[11], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[61], iv2[61], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[111], iv2[111], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[161], iv2[161], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[11], iv4[11], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[61], iv4[61], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[111], iv4[111], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[161], iv4[161], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void o_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[35], iv2[35], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[85], iv2[85], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[135], iv2[135], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[185], iv2[185], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[35], iv4[35], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[85], iv4[85], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[135], iv4[135], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[185], iv4[185], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void ob_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[10], iv2[10], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[60], iv2[60], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[110], iv2[110], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[160], iv2[160], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[10], iv4[10], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[60], iv4[60], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[110], iv4[110], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[160], iv4[160], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void p_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[34], iv2[34], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[84], iv2[84], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[134], iv2[134], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[184], iv2[184], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[34], iv4[34], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[84], iv4[84], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[134], iv4[134], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[184], iv4[184], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void pb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[9], iv2[9], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[59], iv2[59], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[109], iv2[109], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[159], iv2[159], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[9], iv4[9], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[59], iv4[59], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[109], iv4[109], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[159], iv4[159], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void q_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[33], iv2[33], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[83], iv2[83], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[133], iv2[133], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[183], iv2[183], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[33], iv4[33], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[83], iv4[83], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[133], iv4[133], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[183], iv4[183], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void qb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[8], iv2[8], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[58], iv2[58], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[108], iv2[108], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[158], iv2[158], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[8], iv4[8], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[58], iv4[58], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[108], iv4[108], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[158], iv4[158], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void r_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[32], iv2[32], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[82], iv2[82], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[132], iv2[132], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[182], iv2[182], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[32], iv4[32], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[82], iv4[82], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[132], iv4[132], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[182], iv4[182], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void rb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[7], iv2[7], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[57], iv2[57], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[107], iv2[107], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[157], iv2[157], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[7], iv4[7], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[57], iv4[57], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[107], iv4[107], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[157], iv4[157], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void s_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[31], iv2[31], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[81], iv2[81], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[131], iv2[131], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[181], iv2[181], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[31], iv4[31], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[81], iv4[81], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[131], iv4[131], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[181], iv4[181], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void sb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[6], iv2[6], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[56], iv2[56], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[106], iv2[106], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[156], iv2[156], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[6], iv4[6], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[56], iv4[56], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[106], iv4[106], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[156], iv4[156], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void t_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[30], iv2[30], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[80], iv2[80], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[130], iv2[130], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[180], iv2[180], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[30], iv4[30], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[80], iv4[80], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[130], iv4[130], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[180], iv4[180], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void tb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[5], iv2[5], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[55], iv2[55], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[105], iv2[105], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[155], iv2[155], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[5], iv4[5], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[55], iv4[55], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[105], iv4[105], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[155], iv4[155], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void u_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[29], iv2[29], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[79], iv2[79], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[129], iv2[129], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[179], iv2[179], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[29], iv4[29], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[79], iv4[79], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[129], iv4[129], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[179], iv4[179], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void ub_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[4], iv2[4], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[54], iv2[54], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[104], iv2[104], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[154], iv2[154], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[4], iv4[4], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[54], iv4[54], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[104], iv4[104], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[154], iv4[154], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void v_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[28], iv2[28], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[78], iv2[78], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[128], iv2[128], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[178], iv2[178], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[28], iv4[28], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[78], iv4[78], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[128], iv4[128], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[178], iv4[178], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void vb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[3], iv2[3], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[53], iv2[53], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[103], iv2[103], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[153], iv2[153], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[3], iv4[3], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[53], iv4[53], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[103], iv4[103], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[153], iv4[153], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void w_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[27], iv2[27], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[77], iv2[77], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[127], iv2[127], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[177], iv2[177], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[27], iv4[27], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[77], iv4[77], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[127], iv4[127], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[177], iv4[177], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void wb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[2], iv2[2], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[52], iv2[52], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[102], iv2[102], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[152], iv2[152], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[2], iv4[2], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[52], iv4[52], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[102], iv4[102], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[152], iv4[152], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void x_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[26], iv2[26], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[76], iv2[76], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[126], iv2[126], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[176], iv2[176], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[26], iv4[26], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[76], iv4[76], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[126], iv4[126], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[176], iv4[176], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void xb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[1], iv2[1], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[51], iv2[51], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[101], iv2[101], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[151], iv2[151], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[1], iv4[1], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[51], iv4[51], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[101], iv4[101], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[151], iv4[151], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void y_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8])
{
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T a__3[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T Jx[14];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T Je[3];
  real_T b_Je[3];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T i1;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, Jdmv, Je);
  evaluateJacobian(x, u, dmv, e, pv, Cx, Cmv, a__3, Ce);
  Cbds[0] = boundsToConstraints(u[0], iv1[25], iv2[25], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[75], iv2[75], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[125], iv2[125], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[175], iv2[175], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[25], iv4[25], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[75], iv4[75], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[125], iv4[125], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[175], iv4[175], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += A[i1 + 14 * i] * lbd[i1];
    }
    b_d += Jx[i];
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cx[i1 + 18 * i] * muineq[i1];
    }
    Jx[i] = d1;
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += 0.0 * mudummy[i1];
    }
    Hx[i] = (b_d + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_d += B[i1 + 14 * i] * lbd[i1];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      d1 += Cmv[i1 + 18 * i] * muineq[i1];
    }
    d2 = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d2 += Cbdsmv[i1 + (i << 3)] * mudummy[i1];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 18; i1++) {
      b_d += Ce[i1 + 18 * i] * muineq[i1];
    }
    b_Je[i] = Je[i] + b_d;
  }
  __m128d r;
  __m128d r1;
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = b_Je[0];
  Hu[5] = b_Je[1];
  Hu[6] = b_Je[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

void yb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8])
{
  __m128d r;
  __m128d r1;
  real_T Cx[252];
  real_T A[196];
  real_T Cmv[72];
  real_T B[56];
  real_T Ce[54];
  real_T Cbdsmv[32];
  real_T f0[18];
  real_T fpert[18];
  real_T Jx[14];
  real_T b_x[14];
  real_T delta_x[14];
  real_T Jmv[4];
  real_T delta_mv[4];
  real_T Je[3];
  real_T delta_e[3];
  real_T b_dv[2];
  real_T b_dv1[2];
  real_T b_dv2[2];
  real_T b_d;
  real_T d1;
  real_T d2;
  int32_T i;
  int32_T k;
  memset(&Cbdsmv[0], 0, 32U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(x, u, dmv, e, pv, Jx, Jmv, delta_mv, Je);
  b_nlmpc_config__ineqConFcn(x, pv, f0);
  for (k = 0; k <= 12; k += 2) {
    _mm_storeu_pd(&b_x[k], _mm_loadu_pd(&x[k]));
    b_dv[0] = muDoubleScalarMax(muDoubleScalarAbs(x[k]), 1.0);
    b_dv[1] = muDoubleScalarMax(muDoubleScalarAbs(x[k + 1]), 1.0);
    r = _mm_loadu_pd(&b_dv[0]);
    _mm_storeu_pd(&delta_x[k], _mm_mul_pd(_mm_set1_pd(1.0E-7), r));
  }
  for (k = 0; k < 14; k++) {
    b_d = delta_x[k];
    b_x[k] += b_d;
    b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
    b_x[k] -= b_d;
    for (i = 0; i <= 16; i += 2) {
      r = _mm_loadu_pd(&fpert[i]);
      r1 = _mm_loadu_pd(&f0[i]);
      _mm_storeu_pd(&Cx[i + 18 * k],
                    _mm_div_pd(_mm_sub_pd(r, r1), _mm_set1_pd(delta_x[k])));
    }
  }
  b_dv1[0] = muDoubleScalarMax(muDoubleScalarAbs(u[0]), 1.0);
  b_dv1[1] = muDoubleScalarMax(muDoubleScalarAbs(u[1]), 1.0);
  r = _mm_loadu_pd(&b_dv1[0]);
  r1 = _mm_set1_pd(1.0E-7);
  _mm_storeu_pd(&delta_mv[0], _mm_mul_pd(r1, r));
  b_dv1[0] = muDoubleScalarMax(muDoubleScalarAbs(u[2]), 1.0);
  b_dv1[1] = muDoubleScalarMax(muDoubleScalarAbs(u[3]), 1.0);
  r = _mm_loadu_pd(&b_dv1[0]);
  _mm_storeu_pd(&delta_mv[2], _mm_mul_pd(r1, r));
  for (k = 0; k < 4; k++) {
    b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
    for (i = 0; i <= 16; i += 2) {
      __m128d r2;
      r = _mm_loadu_pd(&fpert[i]);
      r2 = _mm_loadu_pd(&f0[i]);
      _mm_storeu_pd(&Cmv[i + 18 * k],
                    _mm_div_pd(_mm_sub_pd(r, r2), _mm_set1_pd(delta_mv[k])));
    }
  }
  b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
  b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
  b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
  b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
  b_dv2[0] = muDoubleScalarMax(muDoubleScalarAbs(e[0]), 1.0);
  b_dv2[1] = muDoubleScalarMax(muDoubleScalarAbs(e[1]), 1.0);
  r = _mm_loadu_pd(&b_dv2[0]);
  _mm_storeu_pd(&delta_e[0], _mm_mul_pd(r1, r));
  delta_e[2] = 1.0E-7 * muDoubleScalarMax(muDoubleScalarAbs(e[2]), 1.0);
  for (k = 0; k < 3; k++) {
    b_nlmpc_config__ineqConFcn(b_x, pv, fpert);
    for (i = 0; i <= 16; i += 2) {
      r = _mm_loadu_pd(&fpert[i]);
      r1 = _mm_loadu_pd(&f0[i]);
      _mm_storeu_pd(&Ce[i + 18 * k],
                    _mm_div_pd(_mm_sub_pd(r, r1), _mm_set1_pd(delta_e[k])));
    }
  }
  Cbds[0] = boundsToConstraints(u[0], iv1[0], iv2[0], &Cbdsmv[0]);
  Cbds[1] = boundsToConstraints(u[1], iv1[50], iv2[50], &Cbdsmv[9]);
  Cbds[2] = boundsToConstraints(u[2], iv1[100], iv2[100], &Cbdsmv[18]);
  Cbds[3] = boundsToConstraints(u[3], iv1[150], iv2[150], &Cbdsmv[27]);
  Cbds[4] = boundsToConstraints(dmv[0], iv3[0], iv4[0], &b_d);
  Cbds[5] = boundsToConstraints(dmv[1], iv3[50], iv4[50], &b_d);
  Cbds[6] = boundsToConstraints(dmv[2], iv3[100], iv4[100], &b_d);
  Cbds[7] = boundsToConstraints(dmv[3], iv3[150], iv4[150], &b_d);
  for (i = 0; i < 14; i++) {
    b_d = 0.0;
    for (k = 0; k < 14; k++) {
      b_d += A[k + 14 * i] * lbd[k];
    }
    d1 = 0.0;
    for (k = 0; k < 18; k++) {
      d1 += Cx[k + 18 * i] * muineq[k];
    }
    d2 = 0.0;
    for (k = 0; k < 8; k++) {
      d2 += 0.0 * mudummy[k];
    }
    Hx[i] = ((Jx[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 4; i++) {
    b_d = 0.0;
    for (k = 0; k < 14; k++) {
      b_d += B[k + 14 * i] * lbd[k];
    }
    d1 = 0.0;
    for (k = 0; k < 18; k++) {
      d1 += Cmv[k + 18 * i] * muineq[k];
    }
    d2 = 0.0;
    for (k = 0; k < 8; k++) {
      d2 += Cbdsmv[k + (i << 3)] * mudummy[k];
    }
    Jmv[i] = ((Jmv[i] + b_d) + d1) + d2;
  }
  for (i = 0; i < 3; i++) {
    b_d = 0.0;
    for (k = 0; k < 18; k++) {
      b_d += Ce[k + 18 * i] * muineq[k];
    }
    delta_e[i] = Je[i] + b_d;
  }
  Hu[0] = Jmv[0];
  Hu[1] = Jmv[1];
  Hu[2] = Jmv[2];
  Hu[3] = Jmv[3];
  Hu[4] = delta_e[0];
  Hu[5] = delta_e[1];
  Hu[6] = delta_e[2];
  r = _mm_set1_pd(2.0);
  r1 = _mm_set1_pd(1.0E+10);
  _mm_storeu_pd(&Hu[7],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[0])),
                                      _mm_loadu_pd(&d[0])),
                           r1));
  _mm_storeu_pd(&Hu[9],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[2])),
                                      _mm_loadu_pd(&d[2])),
                           r1));
  _mm_storeu_pd(&Hu[11],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[4])),
                                      _mm_loadu_pd(&d[4])),
                           r1));
  _mm_storeu_pd(&Hu[13],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, _mm_loadu_pd(&mudummy[6])),
                                      _mm_loadu_pd(&d[6])),
                           r1));
}

/* End of code generation (getHamiltonianDerivatives.c) */
