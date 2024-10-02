/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpcmoveCodeGeneration.c
 *
 * Code generation for function 'nlmpcmoveCodeGeneration'
 *
 */

/* Include files */
#include "nlmpcmoveCodeGeneration.h"
#include "computeInfo.h"
#include "evaluateStateTrajectory.h"
#include "fdgmres.h"
#include "getHamiltonianDerivatives.h"
#include "nlmpc_config__costFcnJacobian.h"
#include "nlmpc_config__ineqConFcn.h"
#include "nlmpc_config__stateFcn.h"
#include "nlmpc_config__stateFcnJacobian.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_emxutil.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Function Declarations */
static real_T d_nlmpcmoveCodeGeneration_anonF(
    const real_T lastMV[4], const real_T onlinedata_StateFcnParameter[109],
    const real_T onlinedata_StageParameter[1353], const real_T U[186],
    const real_T x[14], real_T varargout_1[186], real_T varargout_2_MVopt[44],
    real_T varargout_2_Xopt[154], real_T varargout_2_Topt[11],
    real_T varargout_2_Slack[22], real_T varargout_3_x[186],
    real_T varargout_3_fval[186], real_T *varargout_3_firstorderopt,
    real_T varargout_3_lambda[154]);

static real_T e_nlmpcmoveCodeGeneration_anonF(
    const real_T lastMV[4], const real_T onlinedata_StateFcnParameter[109],
    const real_T onlinedata_StageParameter[1353], const real_T U[186],
    const real_T x[14], real_T varargout_1[186], real_T varargout_2_MVopt[44],
    real_T varargout_2_Xopt[154], real_T varargout_2_Topt[11],
    real_T varargout_2_Slack[22]);

/* Function Definitions */
static real_T d_nlmpcmoveCodeGeneration_anonF(
    const real_T lastMV[4], const real_T onlinedata_StateFcnParameter[109],
    const real_T onlinedata_StageParameter[1353], const real_T U[186],
    const real_T x[14], real_T varargout_1[186], real_T varargout_2_MVopt[44],
    real_T varargout_2_Xopt[154], real_T varargout_2_Topt[11],
    real_T varargout_2_Slack[22], real_T varargout_3_x[186],
    real_T varargout_3_fval[186], real_T *varargout_3_firstorderopt,
    real_T varargout_3_lambda[154])
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  __m128d r3;
  __m128d r4;
  __m128d r5;
  real_T A[196];
  real_T X[154];
  real_T B[56];
  real_T Cx[56];
  real_T Cbdsmv[16];
  real_T Cmv[16];
  real_T a__3[16];
  real_T Jx[14];
  real_T Lx[14];
  real_T Hmv[10];
  real_T Ce[8];
  real_T Cbnds[4];
  real_T Cineq[4];
  real_T Jmv[4];
  real_T b_U[4];
  real_T dmv[4];
  real_T y[4];
  real_T Je[2];
  real_T b_Je[2];
  real_T absx;
  real_T d;
  real_T d1;
  real_T d2;
  real_T varargout_2_Cost;
  int32_T i;
  int32_T k;
  memset(&varargout_1[0], 0, 186U * sizeof(real_T));
  memset(&varargout_3_lambda[0], 0, 154U * sizeof(real_T));
  evaluateStateTrajectory(U, x, onlinedata_StateFcnParameter, X);
  r = _mm_set1_pd(0.0);
  _mm_storeu_pd(&y[0], r);
  r1 = _mm_set1_pd(-1.0);
  _mm_storeu_pd(&b_U[0], _mm_mul_pd(_mm_loadu_pd(&U[162]), r1));
  _mm_storeu_pd(&y[2], r);
  _mm_storeu_pd(&b_U[2], _mm_mul_pd(_mm_loadu_pd(&U[164]), r1));
  nlmpc_config__costFcnJacobian(&X[140], y, b_U, &U[180],
                                &onlinedata_StageParameter[1230], Lx, Jmv,
                                Cineq, Je);
  memcpy(&varargout_3_lambda[140], &Lx[0], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[162]), _mm_loadu_pd(&U[144])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[164]), _mm_loadu_pd(&U[146])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[126], &U[162], dmv, &U[166],
                                &onlinedata_StageParameter[1107], Jx, Jmv,
                                Cineq, Je);
  evaluateJacobian(&X[126], &U[162], dmv, &U[166],
                   &onlinedata_StageParameter[1107], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[162], iv[9], iv1[9], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[163], iv[19], iv1[19], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[164], iv[29], iv1[29], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[165], iv[39], iv1[39], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * varargout_3_lambda[k + 140];
    }
    k = i << 2;
    d = varargout_3_lambda[i + 140] +
        (((Jx[i] + d) +
          (((Cx[k] * U[172] + Cx[k + 1] * U[173]) + Cx[k + 2] * U[174]) +
           Cx[k + 3] * U[175])) +
         (((0.0 * U[176] + 0.0 * U[177]) + 0.0 * U[178]) + 0.0 * U[179])) *
            0.001;
    Jx[i] = d;
    varargout_3_lambda[i + 126] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[126], b_Je, &onlinedata_StageParameter[1107],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    absx = U[k + 172];
    dmv[k] = muDoubleScalarSqrt((absx * absx + d * d) + 1.0E-12);
    absx = U[k + 168];
    y[k] = absx * absx;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[172] + Cmv[i + 1] * U[173]) + Cmv[i + 2] * U[174]) +
               Cmv[i + 3] * U[175])) +
             (((Cbdsmv[i] * U[176] + Cbdsmv[i + 1] * U[177]) +
               Cbdsmv[i + 2] * U[178]) +
              Cbdsmv[i + 3] * U[179]);
  }
  d = U[172];
  absx = U[173];
  d1 = U[174];
  d2 = U[175];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * absx) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[162] = Jmv[0];
  varargout_1[163] = Jmv[1];
  varargout_1[164] = Jmv[2];
  varargout_1[165] = Jmv[3];
  varargout_1[166] = b_Je[0];
  varargout_1[167] = b_Je[1];
  r2 = _mm_set1_pd(2.0);
  r3 = _mm_set1_pd(500.0);
  _mm_storeu_pd(&varargout_1[168],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[176])),
                                      _mm_loadu_pd(&U[168])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[172],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[172]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[176], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[170],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[178])),
                                      _mm_loadu_pd(&U[170])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[174],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[174]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[178], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &varargout_3_lambda[126], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[144]), _mm_loadu_pd(&U[126])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[146]), _mm_loadu_pd(&U[128])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[112], &U[144], dmv, &U[148],
                                &onlinedata_StageParameter[984], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[112], &U[144], dmv, &U[148],
                   &onlinedata_StageParameter[984], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[144], iv[8], iv1[8], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[145], iv[18], iv1[18], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[146], iv[28], iv1[28], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[147], iv[38], iv1[38], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * varargout_3_lambda[k + 126];
    }
    k = i << 2;
    d = varargout_3_lambda[i + 126] +
        (((Jx[i] + d) +
          (((Cx[k] * U[154] + Cx[k + 1] * U[155]) + Cx[k + 2] * U[156]) +
           Cx[k + 3] * U[157])) +
         (((0.0 * U[158] + 0.0 * U[159]) + 0.0 * U[160]) + 0.0 * U[161])) *
            0.001;
    Jx[i] = d;
    varargout_3_lambda[i + 112] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[112], b_Je, &onlinedata_StageParameter[984],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    absx = U[k + 154];
    dmv[k] = muDoubleScalarSqrt((absx * absx + d * d) + 1.0E-12);
    absx = U[k + 150];
    y[k] = absx * absx;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[154] + Cmv[i + 1] * U[155]) + Cmv[i + 2] * U[156]) +
               Cmv[i + 3] * U[157])) +
             (((Cbdsmv[i] * U[158] + Cbdsmv[i + 1] * U[159]) +
               Cbdsmv[i + 2] * U[160]) +
              Cbdsmv[i + 3] * U[161]);
  }
  d = U[154];
  absx = U[155];
  d1 = U[156];
  d2 = U[157];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * absx) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[144] = Jmv[0];
  varargout_1[145] = Jmv[1];
  varargout_1[146] = Jmv[2];
  varargout_1[147] = Jmv[3];
  varargout_1[148] = b_Je[0];
  varargout_1[149] = b_Je[1];
  _mm_storeu_pd(&varargout_1[150],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[158])),
                                      _mm_loadu_pd(&U[150])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[154],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[154]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[158], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[152],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[160])),
                                      _mm_loadu_pd(&U[152])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[156],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[156]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[160], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &varargout_3_lambda[112], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[126]), _mm_loadu_pd(&U[108])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[128]), _mm_loadu_pd(&U[110])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[98], &U[126], dmv, &U[130],
                                &onlinedata_StageParameter[861], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[98], &U[126], dmv, &U[130],
                   &onlinedata_StageParameter[861], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[126], iv[7], iv1[7], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[127], iv[17], iv1[17], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[128], iv[27], iv1[27], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[129], iv[37], iv1[37], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * varargout_3_lambda[k + 112];
    }
    k = i << 2;
    d = varargout_3_lambda[i + 112] +
        (((Jx[i] + d) +
          (((Cx[k] * U[136] + Cx[k + 1] * U[137]) + Cx[k + 2] * U[138]) +
           Cx[k + 3] * U[139])) +
         (((0.0 * U[140] + 0.0 * U[141]) + 0.0 * U[142]) + 0.0 * U[143])) *
            0.001;
    Jx[i] = d;
    varargout_3_lambda[i + 98] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[98], b_Je, &onlinedata_StageParameter[861],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    absx = U[k + 136];
    dmv[k] = muDoubleScalarSqrt((absx * absx + d * d) + 1.0E-12);
    absx = U[k + 132];
    y[k] = absx * absx;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[136] + Cmv[i + 1] * U[137]) + Cmv[i + 2] * U[138]) +
               Cmv[i + 3] * U[139])) +
             (((Cbdsmv[i] * U[140] + Cbdsmv[i + 1] * U[141]) +
               Cbdsmv[i + 2] * U[142]) +
              Cbdsmv[i + 3] * U[143]);
  }
  d = U[136];
  absx = U[137];
  d1 = U[138];
  d2 = U[139];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * absx) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[126] = Jmv[0];
  varargout_1[127] = Jmv[1];
  varargout_1[128] = Jmv[2];
  varargout_1[129] = Jmv[3];
  varargout_1[130] = b_Je[0];
  varargout_1[131] = b_Je[1];
  _mm_storeu_pd(&varargout_1[132],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[140])),
                                      _mm_loadu_pd(&U[132])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[136],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[136]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[140], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[134],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[142])),
                                      _mm_loadu_pd(&U[134])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[138],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[138]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[142], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &varargout_3_lambda[98], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[108]), _mm_loadu_pd(&U[90])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[110]), _mm_loadu_pd(&U[92])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[84], &U[108], dmv, &U[112],
                                &onlinedata_StageParameter[738], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[84], &U[108], dmv, &U[112],
                   &onlinedata_StageParameter[738], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[108], iv[6], iv1[6], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[109], iv[16], iv1[16], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[110], iv[26], iv1[26], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[111], iv[36], iv1[36], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * varargout_3_lambda[k + 98];
    }
    k = i << 2;
    d = varargout_3_lambda[i + 98] +
        (((Jx[i] + d) +
          (((Cx[k] * U[118] + Cx[k + 1] * U[119]) + Cx[k + 2] * U[120]) +
           Cx[k + 3] * U[121])) +
         (((0.0 * U[122] + 0.0 * U[123]) + 0.0 * U[124]) + 0.0 * U[125])) *
            0.001;
    Jx[i] = d;
    varargout_3_lambda[i + 84] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[84], b_Je, &onlinedata_StageParameter[738],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    absx = U[k + 118];
    dmv[k] = muDoubleScalarSqrt((absx * absx + d * d) + 1.0E-12);
    absx = U[k + 114];
    y[k] = absx * absx;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[118] + Cmv[i + 1] * U[119]) + Cmv[i + 2] * U[120]) +
               Cmv[i + 3] * U[121])) +
             (((Cbdsmv[i] * U[122] + Cbdsmv[i + 1] * U[123]) +
               Cbdsmv[i + 2] * U[124]) +
              Cbdsmv[i + 3] * U[125]);
  }
  d = U[118];
  absx = U[119];
  d1 = U[120];
  d2 = U[121];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * absx) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[108] = Jmv[0];
  varargout_1[109] = Jmv[1];
  varargout_1[110] = Jmv[2];
  varargout_1[111] = Jmv[3];
  varargout_1[112] = b_Je[0];
  varargout_1[113] = b_Je[1];
  _mm_storeu_pd(&varargout_1[114],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[122])),
                                      _mm_loadu_pd(&U[114])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[118],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[118]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[122], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[116],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[124])),
                                      _mm_loadu_pd(&U[116])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[120],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[120]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[124], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &varargout_3_lambda[84], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[90]), _mm_loadu_pd(&U[72])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[92]), _mm_loadu_pd(&U[74])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[70], &U[90], dmv, &U[94],
                                &onlinedata_StageParameter[615], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[70], &U[90], dmv, &U[94], &onlinedata_StageParameter[615],
                   Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[90], iv[5], iv1[5], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[91], iv[15], iv1[15], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[92], iv[25], iv1[25], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[93], iv[35], iv1[35], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * varargout_3_lambda[k + 84];
    }
    k = i << 2;
    d = varargout_3_lambda[i + 84] +
        (((Jx[i] + d) +
          (((Cx[k] * U[100] + Cx[k + 1] * U[101]) + Cx[k + 2] * U[102]) +
           Cx[k + 3] * U[103])) +
         (((0.0 * U[104] + 0.0 * U[105]) + 0.0 * U[106]) + 0.0 * U[107])) *
            0.001;
    Jx[i] = d;
    varargout_3_lambda[i + 70] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[70], b_Je, &onlinedata_StageParameter[615],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    absx = U[k + 100];
    dmv[k] = muDoubleScalarSqrt((absx * absx + d * d) + 1.0E-12);
    absx = U[k + 96];
    y[k] = absx * absx;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[100] + Cmv[i + 1] * U[101]) + Cmv[i + 2] * U[102]) +
               Cmv[i + 3] * U[103])) +
             (((Cbdsmv[i] * U[104] + Cbdsmv[i + 1] * U[105]) +
               Cbdsmv[i + 2] * U[106]) +
              Cbdsmv[i + 3] * U[107]);
  }
  d = U[100];
  absx = U[101];
  d1 = U[102];
  d2 = U[103];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * absx) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[90] = Jmv[0];
  varargout_1[91] = Jmv[1];
  varargout_1[92] = Jmv[2];
  varargout_1[93] = Jmv[3];
  varargout_1[94] = b_Je[0];
  varargout_1[95] = b_Je[1];
  _mm_storeu_pd(&varargout_1[96],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[104])),
                                      _mm_loadu_pd(&U[96])),
                           r3));
  r = _mm_loadu_pd(&Cineq[0]);
  r4 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[100],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[100]), r), r4));
  r = _mm_loadu_pd(&Cbnds[0]);
  r4 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[104], _mm_add_pd(r, r4));
  _mm_storeu_pd(&b_U[0],
                _mm_sub_pd(_mm_loadu_pd(&U[72]), _mm_loadu_pd(&U[54])));
  _mm_storeu_pd(&varargout_1[98],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[106])),
                                      _mm_loadu_pd(&U[98])),
                           r3));
  r = _mm_loadu_pd(&Cineq[2]);
  r4 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[102],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[102]), r), r4));
  r = _mm_loadu_pd(&Cbnds[2]);
  r4 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[106], _mm_add_pd(r, r4));
  _mm_storeu_pd(&b_U[2],
                _mm_sub_pd(_mm_loadu_pd(&U[74]), _mm_loadu_pd(&U[56])));
  getHamiltonianDerivatives(&X[56], &varargout_3_lambda[70], &U[72], &U[76],
                            &U[78], &U[82], &U[86], b_U,
                            &onlinedata_StageParameter[492], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r2 = _mm_loadu_pd(&varargout_3_lambda[i + 70]);
    r = _mm_add_pd(r2, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 56], r);
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[56], b_Je, &onlinedata_StageParameter[492],
                           Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[82]);
  r3 = _mm_set1_pd(1.0E-12);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[78]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[84]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[80]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[72], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r2 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[82],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[82]), r), r2));
  r = _mm_loadu_pd(&Cbnds[0]);
  r2 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[86], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[0],
                _mm_sub_pd(_mm_loadu_pd(&U[54]), _mm_loadu_pd(&U[36])));
  r = _mm_loadu_pd(&Cineq[2]);
  r2 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[84],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[84]), r), r2));
  r = _mm_loadu_pd(&Cbnds[2]);
  r2 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[88], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[2],
                _mm_sub_pd(_mm_loadu_pd(&U[56]), _mm_loadu_pd(&U[38])));
  b_getHamiltonianDerivatives(&X[42], &varargout_3_lambda[56], &U[54], &U[58],
                              &U[60], &U[64], &U[68], b_U,
                              &onlinedata_StageParameter[369], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r2 = _mm_loadu_pd(&varargout_3_lambda[i + 56]);
    r = _mm_add_pd(r2, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 42], r);
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[42], b_Je, &onlinedata_StageParameter[369],
                           Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[64]);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[60]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[66]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[62]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[54], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r2 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[64],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[64]), r), r2));
  r = _mm_loadu_pd(&Cbnds[0]);
  r2 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[68], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[0],
                _mm_sub_pd(_mm_loadu_pd(&U[36]), _mm_loadu_pd(&U[18])));
  r = _mm_loadu_pd(&Cineq[2]);
  r2 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[66],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[66]), r), r2));
  r = _mm_loadu_pd(&Cbnds[2]);
  r2 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[70], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[2],
                _mm_sub_pd(_mm_loadu_pd(&U[38]), _mm_loadu_pd(&U[20])));
  c_getHamiltonianDerivatives(&X[28], &varargout_3_lambda[42], &U[36], &U[40],
                              &U[42], &U[46], &U[50], b_U,
                              &onlinedata_StageParameter[246], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r2 = _mm_loadu_pd(&varargout_3_lambda[i + 42]);
    r = _mm_add_pd(r2, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 28], r);
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[28], b_Je, &onlinedata_StageParameter[246],
                           Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[46]);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[42]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[48]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[44]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[36], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r2 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[46],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[46]), r), r2));
  r = _mm_loadu_pd(&Cbnds[0]);
  r2 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[50], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[0], _mm_sub_pd(_mm_loadu_pd(&U[18]), _mm_loadu_pd(&U[0])));
  r = _mm_loadu_pd(&Cineq[2]);
  r2 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[48],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[48]), r), r2));
  r = _mm_loadu_pd(&Cbnds[2]);
  r2 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[52], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[2], _mm_sub_pd(_mm_loadu_pd(&U[20]), _mm_loadu_pd(&U[2])));
  d_getHamiltonianDerivatives(&X[14], &varargout_3_lambda[28], &U[18], &U[22],
                              &U[24], &U[28], &U[32], b_U,
                              &onlinedata_StageParameter[123], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r2 = _mm_loadu_pd(&varargout_3_lambda[i + 28]);
    r = _mm_add_pd(r2, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 14], r);
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[14], b_Je, &onlinedata_StageParameter[123],
                           Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[28]);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[24]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[30]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[26]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[18], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r2 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[28],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[28]), r), r2));
  r = _mm_loadu_pd(&Cbnds[0]);
  r2 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[32], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[0],
                _mm_sub_pd(_mm_loadu_pd(&U[0]), _mm_loadu_pd(&lastMV[0])));
  r = _mm_loadu_pd(&Cineq[2]);
  r2 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[30],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[30]), r), r2));
  r = _mm_loadu_pd(&Cbnds[2]);
  r2 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[34], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[2],
                _mm_sub_pd(_mm_loadu_pd(&U[2]), _mm_loadu_pd(&lastMV[2])));
  e_getHamiltonianDerivatives(&X[0], &varargout_3_lambda[14], &U[0], &U[4],
                              &U[6], &U[10], &U[14], b_U,
                              &onlinedata_StageParameter[0], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r2 = _mm_loadu_pd(&varargout_3_lambda[i + 14]);
    r = _mm_add_pd(r2, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i], r);
  }
  b_nlmpc_config__ineqConFcn(&X[0], &onlinedata_StageParameter[0], Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[10]);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[6]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[12]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[8]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[0], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r1 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[10],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[10]), r), r1));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[14], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cineq[2]);
  r1 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[12],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[12]), r), r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[16], _mm_add_pd(r, r1));
  computeInfo(X, U, onlinedata_StageParameter, lastMV, varargout_2_MVopt,
              varargout_2_Xopt, varargout_2_Topt, varargout_2_Slack,
              &varargout_2_Cost);
  *varargout_3_firstorderopt = 0.0;
  for (k = 0; k < 186; k++) {
    d = varargout_1[k];
    absx = muDoubleScalarAbs(d);
    if (muDoubleScalarIsNaN(absx) || (absx > *varargout_3_firstorderopt)) {
      *varargout_3_firstorderopt = absx;
    }
    varargout_3_x[k] = U[k];
    varargout_3_fval[k] = d;
  }
  return varargout_2_Cost;
}

static real_T e_nlmpcmoveCodeGeneration_anonF(
    const real_T lastMV[4], const real_T onlinedata_StateFcnParameter[109],
    const real_T onlinedata_StageParameter[1353], const real_T U[186],
    const real_T x[14], real_T varargout_1[186], real_T varargout_2_MVopt[44],
    real_T varargout_2_Xopt[154], real_T varargout_2_Topt[11],
    real_T varargout_2_Slack[22])
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  __m128d r3;
  __m128d r4;
  __m128d r5;
  real_T A[196];
  real_T X[154];
  real_T lambda[154];
  real_T B[56];
  real_T Cx[56];
  real_T Cbdsmv[16];
  real_T Cmv[16];
  real_T a__3[16];
  real_T Jx[14];
  real_T Lx[14];
  real_T Hmv[10];
  real_T Ce[8];
  real_T Cbnds[4];
  real_T Cineq[4];
  real_T Jmv[4];
  real_T b_U[4];
  real_T dmv[4];
  real_T y[4];
  real_T Je[2];
  real_T b_Je[2];
  real_T d;
  real_T d1;
  real_T d2;
  real_T dmv_tmp;
  real_T varargout_2_Cost;
  int32_T i;
  int32_T k;
  memset(&varargout_1[0], 0, 186U * sizeof(real_T));
  memset(&lambda[0], 0, 154U * sizeof(real_T));
  evaluateStateTrajectory(U, x, onlinedata_StateFcnParameter, X);
  r = _mm_set1_pd(0.0);
  _mm_storeu_pd(&y[0], r);
  r1 = _mm_set1_pd(-1.0);
  _mm_storeu_pd(&b_U[0], _mm_mul_pd(_mm_loadu_pd(&U[162]), r1));
  _mm_storeu_pd(&y[2], r);
  _mm_storeu_pd(&b_U[2], _mm_mul_pd(_mm_loadu_pd(&U[164]), r1));
  nlmpc_config__costFcnJacobian(&X[140], y, b_U, &U[180],
                                &onlinedata_StageParameter[1230], Lx, Jmv,
                                Cineq, Je);
  memcpy(&lambda[140], &Lx[0], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[162]), _mm_loadu_pd(&U[144])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[164]), _mm_loadu_pd(&U[146])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[126], &U[162], dmv, &U[166],
                                &onlinedata_StageParameter[1107], Jx, Jmv,
                                Cineq, Je);
  evaluateJacobian(&X[126], &U[162], dmv, &U[166],
                   &onlinedata_StageParameter[1107], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[162], iv[9], iv1[9], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[163], iv[19], iv1[19], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[164], iv[29], iv1[29], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[165], iv[39], iv1[39], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 140];
    }
    k = i << 2;
    d = lambda[i + 140] +
        (((Jx[i] + d) +
          (((Cx[k] * U[172] + Cx[k + 1] * U[173]) + Cx[k + 2] * U[174]) +
           Cx[k + 3] * U[175])) +
         (((0.0 * U[176] + 0.0 * U[177]) + 0.0 * U[178]) + 0.0 * U[179])) *
            0.001;
    Jx[i] = d;
    lambda[i + 126] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[126], b_Je, &onlinedata_StageParameter[1107],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 172];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 168];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[172] + Cmv[i + 1] * U[173]) + Cmv[i + 2] * U[174]) +
               Cmv[i + 3] * U[175])) +
             (((Cbdsmv[i] * U[176] + Cbdsmv[i + 1] * U[177]) +
               Cbdsmv[i + 2] * U[178]) +
              Cbdsmv[i + 3] * U[179]);
  }
  d = U[172];
  dmv_tmp = U[173];
  d1 = U[174];
  d2 = U[175];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[162] = Jmv[0];
  varargout_1[163] = Jmv[1];
  varargout_1[164] = Jmv[2];
  varargout_1[165] = Jmv[3];
  varargout_1[166] = b_Je[0];
  varargout_1[167] = b_Je[1];
  r2 = _mm_set1_pd(2.0);
  r3 = _mm_set1_pd(500.0);
  _mm_storeu_pd(&varargout_1[168],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[176])),
                                      _mm_loadu_pd(&U[168])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[172],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[172]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[176], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[170],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[178])),
                                      _mm_loadu_pd(&U[170])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[174],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[174]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[178], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &lambda[126], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[144]), _mm_loadu_pd(&U[126])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[146]), _mm_loadu_pd(&U[128])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[112], &U[144], dmv, &U[148],
                                &onlinedata_StageParameter[984], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[112], &U[144], dmv, &U[148],
                   &onlinedata_StageParameter[984], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[144], iv[8], iv1[8], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[145], iv[18], iv1[18], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[146], iv[28], iv1[28], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[147], iv[38], iv1[38], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 126];
    }
    k = i << 2;
    d = lambda[i + 126] +
        (((Jx[i] + d) +
          (((Cx[k] * U[154] + Cx[k + 1] * U[155]) + Cx[k + 2] * U[156]) +
           Cx[k + 3] * U[157])) +
         (((0.0 * U[158] + 0.0 * U[159]) + 0.0 * U[160]) + 0.0 * U[161])) *
            0.001;
    Jx[i] = d;
    lambda[i + 112] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[112], b_Je, &onlinedata_StageParameter[984],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 154];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 150];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[154] + Cmv[i + 1] * U[155]) + Cmv[i + 2] * U[156]) +
               Cmv[i + 3] * U[157])) +
             (((Cbdsmv[i] * U[158] + Cbdsmv[i + 1] * U[159]) +
               Cbdsmv[i + 2] * U[160]) +
              Cbdsmv[i + 3] * U[161]);
  }
  d = U[154];
  dmv_tmp = U[155];
  d1 = U[156];
  d2 = U[157];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[144] = Jmv[0];
  varargout_1[145] = Jmv[1];
  varargout_1[146] = Jmv[2];
  varargout_1[147] = Jmv[3];
  varargout_1[148] = b_Je[0];
  varargout_1[149] = b_Je[1];
  _mm_storeu_pd(&varargout_1[150],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[158])),
                                      _mm_loadu_pd(&U[150])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[154],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[154]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[158], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[152],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[160])),
                                      _mm_loadu_pd(&U[152])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[156],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[156]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[160], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &lambda[112], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[126]), _mm_loadu_pd(&U[108])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[128]), _mm_loadu_pd(&U[110])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[98], &U[126], dmv, &U[130],
                                &onlinedata_StageParameter[861], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[98], &U[126], dmv, &U[130],
                   &onlinedata_StageParameter[861], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[126], iv[7], iv1[7], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[127], iv[17], iv1[17], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[128], iv[27], iv1[27], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[129], iv[37], iv1[37], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 112];
    }
    k = i << 2;
    d = lambda[i + 112] +
        (((Jx[i] + d) +
          (((Cx[k] * U[136] + Cx[k + 1] * U[137]) + Cx[k + 2] * U[138]) +
           Cx[k + 3] * U[139])) +
         (((0.0 * U[140] + 0.0 * U[141]) + 0.0 * U[142]) + 0.0 * U[143])) *
            0.001;
    Jx[i] = d;
    lambda[i + 98] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[98], b_Je, &onlinedata_StageParameter[861],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 136];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 132];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[136] + Cmv[i + 1] * U[137]) + Cmv[i + 2] * U[138]) +
               Cmv[i + 3] * U[139])) +
             (((Cbdsmv[i] * U[140] + Cbdsmv[i + 1] * U[141]) +
               Cbdsmv[i + 2] * U[142]) +
              Cbdsmv[i + 3] * U[143]);
  }
  d = U[136];
  dmv_tmp = U[137];
  d1 = U[138];
  d2 = U[139];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[126] = Jmv[0];
  varargout_1[127] = Jmv[1];
  varargout_1[128] = Jmv[2];
  varargout_1[129] = Jmv[3];
  varargout_1[130] = b_Je[0];
  varargout_1[131] = b_Je[1];
  _mm_storeu_pd(&varargout_1[132],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[140])),
                                      _mm_loadu_pd(&U[132])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[136],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[136]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[140], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[134],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[142])),
                                      _mm_loadu_pd(&U[134])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[138],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[138]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[142], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &lambda[98], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[108]), _mm_loadu_pd(&U[90])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[110]), _mm_loadu_pd(&U[92])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[84], &U[108], dmv, &U[112],
                                &onlinedata_StageParameter[738], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[84], &U[108], dmv, &U[112],
                   &onlinedata_StageParameter[738], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[108], iv[6], iv1[6], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[109], iv[16], iv1[16], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[110], iv[26], iv1[26], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[111], iv[36], iv1[36], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 98];
    }
    k = i << 2;
    d = lambda[i + 98] +
        (((Jx[i] + d) +
          (((Cx[k] * U[118] + Cx[k + 1] * U[119]) + Cx[k + 2] * U[120]) +
           Cx[k + 3] * U[121])) +
         (((0.0 * U[122] + 0.0 * U[123]) + 0.0 * U[124]) + 0.0 * U[125])) *
            0.001;
    Jx[i] = d;
    lambda[i + 84] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[84], b_Je, &onlinedata_StageParameter[738],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 118];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 114];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[118] + Cmv[i + 1] * U[119]) + Cmv[i + 2] * U[120]) +
               Cmv[i + 3] * U[121])) +
             (((Cbdsmv[i] * U[122] + Cbdsmv[i + 1] * U[123]) +
               Cbdsmv[i + 2] * U[124]) +
              Cbdsmv[i + 3] * U[125]);
  }
  d = U[118];
  dmv_tmp = U[119];
  d1 = U[120];
  d2 = U[121];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[108] = Jmv[0];
  varargout_1[109] = Jmv[1];
  varargout_1[110] = Jmv[2];
  varargout_1[111] = Jmv[3];
  varargout_1[112] = b_Je[0];
  varargout_1[113] = b_Je[1];
  _mm_storeu_pd(&varargout_1[114],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[122])),
                                      _mm_loadu_pd(&U[114])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[118],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[118]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[122], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[116],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[124])),
                                      _mm_loadu_pd(&U[116])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[120],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[120]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[124], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &lambda[84], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[90]), _mm_loadu_pd(&U[72])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[92]), _mm_loadu_pd(&U[74])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[70], &U[90], dmv, &U[94],
                                &onlinedata_StageParameter[615], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[70], &U[90], dmv, &U[94], &onlinedata_StageParameter[615],
                   Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[90], iv[5], iv1[5], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[91], iv[15], iv1[15], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[92], iv[25], iv1[25], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[93], iv[35], iv1[35], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 84];
    }
    k = i << 2;
    d = lambda[i + 84] +
        (((Jx[i] + d) +
          (((Cx[k] * U[100] + Cx[k + 1] * U[101]) + Cx[k + 2] * U[102]) +
           Cx[k + 3] * U[103])) +
         (((0.0 * U[104] + 0.0 * U[105]) + 0.0 * U[106]) + 0.0 * U[107])) *
            0.001;
    Jx[i] = d;
    lambda[i + 70] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[70], b_Je, &onlinedata_StageParameter[615],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 100];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 96];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[100] + Cmv[i + 1] * U[101]) + Cmv[i + 2] * U[102]) +
               Cmv[i + 3] * U[103])) +
             (((Cbdsmv[i] * U[104] + Cbdsmv[i + 1] * U[105]) +
               Cbdsmv[i + 2] * U[106]) +
              Cbdsmv[i + 3] * U[107]);
  }
  d = U[100];
  dmv_tmp = U[101];
  d1 = U[102];
  d2 = U[103];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[90] = Jmv[0];
  varargout_1[91] = Jmv[1];
  varargout_1[92] = Jmv[2];
  varargout_1[93] = Jmv[3];
  varargout_1[94] = b_Je[0];
  varargout_1[95] = b_Je[1];
  _mm_storeu_pd(&varargout_1[96],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[104])),
                                      _mm_loadu_pd(&U[96])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[100],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[100]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[104], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[98],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[106])),
                                      _mm_loadu_pd(&U[98])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[102],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[102]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[106], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &lambda[70], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[72]), _mm_loadu_pd(&U[54])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[74]), _mm_loadu_pd(&U[56])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[56], &U[72], dmv, &U[76],
                                &onlinedata_StageParameter[492], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[56], &U[72], dmv, &U[76], &onlinedata_StageParameter[492],
                   Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[72], iv[4], iv1[4], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[73], iv[14], iv1[14], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[74], iv[24], iv1[24], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[75], iv[34], iv1[34], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 70];
    }
    k = i << 2;
    d = lambda[i + 70] +
        (((Jx[i] + d) +
          (((Cx[k] * U[82] + Cx[k + 1] * U[83]) + Cx[k + 2] * U[84]) +
           Cx[k + 3] * U[85])) +
         (((0.0 * U[86] + 0.0 * U[87]) + 0.0 * U[88]) + 0.0 * U[89])) *
            0.001;
    Jx[i] = d;
    lambda[i + 56] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[56], b_Je, &onlinedata_StageParameter[492],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 82];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 78];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] =
        ((Jmv[k] + d) +
         (((Cmv[i] * U[82] + Cmv[i + 1] * U[83]) + Cmv[i + 2] * U[84]) +
          Cmv[i + 3] * U[85])) +
        (((Cbdsmv[i] * U[86] + Cbdsmv[i + 1] * U[87]) + Cbdsmv[i + 2] * U[88]) +
         Cbdsmv[i + 3] * U[89]);
  }
  d = U[82];
  dmv_tmp = U[83];
  d1 = U[84];
  d2 = U[85];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[72] = Jmv[0];
  varargout_1[73] = Jmv[1];
  varargout_1[74] = Jmv[2];
  varargout_1[75] = Jmv[3];
  varargout_1[76] = b_Je[0];
  varargout_1[77] = b_Je[1];
  _mm_storeu_pd(&varargout_1[78],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[86])),
                                      _mm_loadu_pd(&U[78])),
                           r3));
  r = _mm_loadu_pd(&Cineq[0]);
  r4 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[82],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[82]), r), r4));
  r = _mm_loadu_pd(&Cbnds[0]);
  r4 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[86], _mm_add_pd(r, r4));
  _mm_storeu_pd(&b_U[0],
                _mm_sub_pd(_mm_loadu_pd(&U[54]), _mm_loadu_pd(&U[36])));
  _mm_storeu_pd(&varargout_1[80],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[88])),
                                      _mm_loadu_pd(&U[80])),
                           r3));
  r = _mm_loadu_pd(&Cineq[2]);
  r4 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[84],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[84]), r), r4));
  r = _mm_loadu_pd(&Cbnds[2]);
  r4 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[88], _mm_add_pd(r, r4));
  _mm_storeu_pd(&b_U[2],
                _mm_sub_pd(_mm_loadu_pd(&U[56]), _mm_loadu_pd(&U[38])));
  b_getHamiltonianDerivatives(&X[42], &lambda[56], &U[54], &U[58], &U[60],
                              &U[64], &U[68], b_U,
                              &onlinedata_StageParameter[369], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r2 = _mm_loadu_pd(&lambda[i + 56]);
    r = _mm_add_pd(r2, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 42], r);
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[42], b_Je, &onlinedata_StageParameter[369],
                           Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[64]);
  r3 = _mm_set1_pd(1.0E-12);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[60]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[66]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[62]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[54], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r2 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[64],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[64]), r), r2));
  r = _mm_loadu_pd(&Cbnds[0]);
  r2 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[68], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[0],
                _mm_sub_pd(_mm_loadu_pd(&U[36]), _mm_loadu_pd(&U[18])));
  r = _mm_loadu_pd(&Cineq[2]);
  r2 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[66],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[66]), r), r2));
  r = _mm_loadu_pd(&Cbnds[2]);
  r2 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[70], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[2],
                _mm_sub_pd(_mm_loadu_pd(&U[38]), _mm_loadu_pd(&U[20])));
  c_getHamiltonianDerivatives(&X[28], &lambda[42], &U[36], &U[40], &U[42],
                              &U[46], &U[50], b_U,
                              &onlinedata_StageParameter[246], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r2 = _mm_loadu_pd(&lambda[i + 42]);
    r = _mm_add_pd(r2, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 28], r);
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[28], b_Je, &onlinedata_StageParameter[246],
                           Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[46]);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[42]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[48]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[44]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[36], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r2 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[46],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[46]), r), r2));
  r = _mm_loadu_pd(&Cbnds[0]);
  r2 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[50], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[0], _mm_sub_pd(_mm_loadu_pd(&U[18]), _mm_loadu_pd(&U[0])));
  r = _mm_loadu_pd(&Cineq[2]);
  r2 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[48],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[48]), r), r2));
  r = _mm_loadu_pd(&Cbnds[2]);
  r2 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[52], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[2], _mm_sub_pd(_mm_loadu_pd(&U[20]), _mm_loadu_pd(&U[2])));
  d_getHamiltonianDerivatives(&X[14], &lambda[28], &U[18], &U[22], &U[24],
                              &U[28], &U[32], b_U,
                              &onlinedata_StageParameter[123], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r2 = _mm_loadu_pd(&lambda[i + 28]);
    r = _mm_add_pd(r2, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 14], r);
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[14], b_Je, &onlinedata_StageParameter[123],
                           Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[28]);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[24]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[30]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[26]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[18], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r2 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[28],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[28]), r), r2));
  r = _mm_loadu_pd(&Cbnds[0]);
  r2 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[32], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[0],
                _mm_sub_pd(_mm_loadu_pd(&U[0]), _mm_loadu_pd(&lastMV[0])));
  r = _mm_loadu_pd(&Cineq[2]);
  r2 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[30],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[30]), r), r2));
  r = _mm_loadu_pd(&Cbnds[2]);
  r2 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[34], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[2],
                _mm_sub_pd(_mm_loadu_pd(&U[2]), _mm_loadu_pd(&lastMV[2])));
  e_getHamiltonianDerivatives(&X[0], &lambda[14], &U[0], &U[4], &U[6], &U[10],
                              &U[14], b_U, &onlinedata_StageParameter[0], Lx,
                              Hmv, Cbnds);
  b_nlmpc_config__ineqConFcn(&X[0], &onlinedata_StageParameter[0], Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[10]);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[6]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[12]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[8]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[0], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r1 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[10],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[10]), r), r1));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[14], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cineq[2]);
  r1 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[12],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[12]), r), r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[16], _mm_add_pd(r, r1));
  computeInfo(X, U, onlinedata_StageParameter, lastMV, varargout_2_MVopt,
              varargout_2_Xopt, varargout_2_Topt, varargout_2_Slack,
              &varargout_2_Cost);
  return varargout_2_Cost;
}

void c_nlmpcmoveCodeGeneration_anonF(
    const real_T lastMV[4], const real_T onlinedata_StateFcnParameter[109],
    const real_T onlinedata_StageParameter[1353], const real_T U[186],
    const real_T x[14], real_T varargout_1[186])
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  __m128d r3;
  __m128d r4;
  __m128d r5;
  real_T A[196];
  real_T X[154];
  real_T lambda[154];
  real_T B[56];
  real_T Cx[56];
  real_T Cbdsmv[16];
  real_T Cmv[16];
  real_T a__3[16];
  real_T Jx[14];
  real_T Lx[14];
  real_T Hmv[10];
  real_T Ce[8];
  real_T Cbnds[4];
  real_T Cineq[4];
  real_T Jmv[4];
  real_T b_U[4];
  real_T dmv[4];
  real_T y[4];
  real_T Je[2];
  real_T b_Je[2];
  real_T d;
  real_T d1;
  real_T d2;
  real_T dmv_tmp;
  int32_T i;
  int32_T k;
  memset(&varargout_1[0], 0, 186U * sizeof(real_T));
  memset(&lambda[0], 0, 154U * sizeof(real_T));
  evaluateStateTrajectory(U, x, onlinedata_StateFcnParameter, X);
  r = _mm_set1_pd(0.0);
  _mm_storeu_pd(&y[0], r);
  r1 = _mm_set1_pd(-1.0);
  _mm_storeu_pd(&b_U[0], _mm_mul_pd(_mm_loadu_pd(&U[162]), r1));
  _mm_storeu_pd(&y[2], r);
  _mm_storeu_pd(&b_U[2], _mm_mul_pd(_mm_loadu_pd(&U[164]), r1));
  nlmpc_config__costFcnJacobian(&X[140], y, b_U, &U[180],
                                &onlinedata_StageParameter[1230], Lx, Jmv,
                                Cineq, Je);
  memcpy(&lambda[140], &Lx[0], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[162]), _mm_loadu_pd(&U[144])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[164]), _mm_loadu_pd(&U[146])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[126], &U[162], dmv, &U[166],
                                &onlinedata_StageParameter[1107], Jx, Jmv,
                                Cineq, Je);
  evaluateJacobian(&X[126], &U[162], dmv, &U[166],
                   &onlinedata_StageParameter[1107], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[162], iv[9], iv1[9], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[163], iv[19], iv1[19], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[164], iv[29], iv1[29], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[165], iv[39], iv1[39], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 140];
    }
    k = i << 2;
    d = lambda[i + 140] +
        (((Jx[i] + d) +
          (((Cx[k] * U[172] + Cx[k + 1] * U[173]) + Cx[k + 2] * U[174]) +
           Cx[k + 3] * U[175])) +
         (((0.0 * U[176] + 0.0 * U[177]) + 0.0 * U[178]) + 0.0 * U[179])) *
            0.001;
    Jx[i] = d;
    lambda[i + 126] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[126], b_Je, &onlinedata_StageParameter[1107],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 172];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 168];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[172] + Cmv[i + 1] * U[173]) + Cmv[i + 2] * U[174]) +
               Cmv[i + 3] * U[175])) +
             (((Cbdsmv[i] * U[176] + Cbdsmv[i + 1] * U[177]) +
               Cbdsmv[i + 2] * U[178]) +
              Cbdsmv[i + 3] * U[179]);
  }
  d = U[172];
  dmv_tmp = U[173];
  d1 = U[174];
  d2 = U[175];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[162] = Jmv[0];
  varargout_1[163] = Jmv[1];
  varargout_1[164] = Jmv[2];
  varargout_1[165] = Jmv[3];
  varargout_1[166] = b_Je[0];
  varargout_1[167] = b_Je[1];
  r2 = _mm_set1_pd(2.0);
  r3 = _mm_set1_pd(500.0);
  _mm_storeu_pd(&varargout_1[168],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[176])),
                                      _mm_loadu_pd(&U[168])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[172],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[172]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[176], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[170],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[178])),
                                      _mm_loadu_pd(&U[170])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[174],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[174]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[178], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &lambda[126], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[144]), _mm_loadu_pd(&U[126])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[146]), _mm_loadu_pd(&U[128])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[112], &U[144], dmv, &U[148],
                                &onlinedata_StageParameter[984], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[112], &U[144], dmv, &U[148],
                   &onlinedata_StageParameter[984], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[144], iv[8], iv1[8], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[145], iv[18], iv1[18], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[146], iv[28], iv1[28], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[147], iv[38], iv1[38], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 126];
    }
    k = i << 2;
    d = lambda[i + 126] +
        (((Jx[i] + d) +
          (((Cx[k] * U[154] + Cx[k + 1] * U[155]) + Cx[k + 2] * U[156]) +
           Cx[k + 3] * U[157])) +
         (((0.0 * U[158] + 0.0 * U[159]) + 0.0 * U[160]) + 0.0 * U[161])) *
            0.001;
    Jx[i] = d;
    lambda[i + 112] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[112], b_Je, &onlinedata_StageParameter[984],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 154];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 150];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[154] + Cmv[i + 1] * U[155]) + Cmv[i + 2] * U[156]) +
               Cmv[i + 3] * U[157])) +
             (((Cbdsmv[i] * U[158] + Cbdsmv[i + 1] * U[159]) +
               Cbdsmv[i + 2] * U[160]) +
              Cbdsmv[i + 3] * U[161]);
  }
  d = U[154];
  dmv_tmp = U[155];
  d1 = U[156];
  d2 = U[157];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[144] = Jmv[0];
  varargout_1[145] = Jmv[1];
  varargout_1[146] = Jmv[2];
  varargout_1[147] = Jmv[3];
  varargout_1[148] = b_Je[0];
  varargout_1[149] = b_Je[1];
  _mm_storeu_pd(&varargout_1[150],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[158])),
                                      _mm_loadu_pd(&U[150])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[154],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[154]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[158], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[152],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[160])),
                                      _mm_loadu_pd(&U[152])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[156],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[156]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[160], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &lambda[112], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[126]), _mm_loadu_pd(&U[108])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[128]), _mm_loadu_pd(&U[110])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[98], &U[126], dmv, &U[130],
                                &onlinedata_StageParameter[861], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[98], &U[126], dmv, &U[130],
                   &onlinedata_StageParameter[861], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[126], iv[7], iv1[7], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[127], iv[17], iv1[17], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[128], iv[27], iv1[27], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[129], iv[37], iv1[37], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 112];
    }
    k = i << 2;
    d = lambda[i + 112] +
        (((Jx[i] + d) +
          (((Cx[k] * U[136] + Cx[k + 1] * U[137]) + Cx[k + 2] * U[138]) +
           Cx[k + 3] * U[139])) +
         (((0.0 * U[140] + 0.0 * U[141]) + 0.0 * U[142]) + 0.0 * U[143])) *
            0.001;
    Jx[i] = d;
    lambda[i + 98] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[98], b_Je, &onlinedata_StageParameter[861],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 136];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 132];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[136] + Cmv[i + 1] * U[137]) + Cmv[i + 2] * U[138]) +
               Cmv[i + 3] * U[139])) +
             (((Cbdsmv[i] * U[140] + Cbdsmv[i + 1] * U[141]) +
               Cbdsmv[i + 2] * U[142]) +
              Cbdsmv[i + 3] * U[143]);
  }
  d = U[136];
  dmv_tmp = U[137];
  d1 = U[138];
  d2 = U[139];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[126] = Jmv[0];
  varargout_1[127] = Jmv[1];
  varargout_1[128] = Jmv[2];
  varargout_1[129] = Jmv[3];
  varargout_1[130] = b_Je[0];
  varargout_1[131] = b_Je[1];
  _mm_storeu_pd(&varargout_1[132],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[140])),
                                      _mm_loadu_pd(&U[132])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[136],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[136]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[140], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[134],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[142])),
                                      _mm_loadu_pd(&U[134])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[138],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[138]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[142], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &lambda[98], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[108]), _mm_loadu_pd(&U[90])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[110]), _mm_loadu_pd(&U[92])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[84], &U[108], dmv, &U[112],
                                &onlinedata_StageParameter[738], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[84], &U[108], dmv, &U[112],
                   &onlinedata_StageParameter[738], Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[108], iv[6], iv1[6], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[109], iv[16], iv1[16], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[110], iv[26], iv1[26], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[111], iv[36], iv1[36], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 98];
    }
    k = i << 2;
    d = lambda[i + 98] +
        (((Jx[i] + d) +
          (((Cx[k] * U[118] + Cx[k + 1] * U[119]) + Cx[k + 2] * U[120]) +
           Cx[k + 3] * U[121])) +
         (((0.0 * U[122] + 0.0 * U[123]) + 0.0 * U[124]) + 0.0 * U[125])) *
            0.001;
    Jx[i] = d;
    lambda[i + 84] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[84], b_Je, &onlinedata_StageParameter[738],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 118];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 114];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[118] + Cmv[i + 1] * U[119]) + Cmv[i + 2] * U[120]) +
               Cmv[i + 3] * U[121])) +
             (((Cbdsmv[i] * U[122] + Cbdsmv[i + 1] * U[123]) +
               Cbdsmv[i + 2] * U[124]) +
              Cbdsmv[i + 3] * U[125]);
  }
  d = U[118];
  dmv_tmp = U[119];
  d1 = U[120];
  d2 = U[121];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[108] = Jmv[0];
  varargout_1[109] = Jmv[1];
  varargout_1[110] = Jmv[2];
  varargout_1[111] = Jmv[3];
  varargout_1[112] = b_Je[0];
  varargout_1[113] = b_Je[1];
  _mm_storeu_pd(&varargout_1[114],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[122])),
                                      _mm_loadu_pd(&U[114])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[118],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[118]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[122], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[116],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[124])),
                                      _mm_loadu_pd(&U[116])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[120],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[120]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[124], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &lambda[84], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[90]), _mm_loadu_pd(&U[72])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[92]), _mm_loadu_pd(&U[74])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[70], &U[90], dmv, &U[94],
                                &onlinedata_StageParameter[615], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[70], &U[90], dmv, &U[94], &onlinedata_StageParameter[615],
                   Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[90], iv[5], iv1[5], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[91], iv[15], iv1[15], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[92], iv[25], iv1[25], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[93], iv[35], iv1[35], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 84];
    }
    k = i << 2;
    d = lambda[i + 84] +
        (((Jx[i] + d) +
          (((Cx[k] * U[100] + Cx[k + 1] * U[101]) + Cx[k + 2] * U[102]) +
           Cx[k + 3] * U[103])) +
         (((0.0 * U[104] + 0.0 * U[105]) + 0.0 * U[106]) + 0.0 * U[107])) *
            0.001;
    Jx[i] = d;
    lambda[i + 70] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[70], b_Je, &onlinedata_StageParameter[615],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 100];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 96];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] = ((Jmv[k] + d) +
              (((Cmv[i] * U[100] + Cmv[i + 1] * U[101]) + Cmv[i + 2] * U[102]) +
               Cmv[i + 3] * U[103])) +
             (((Cbdsmv[i] * U[104] + Cbdsmv[i + 1] * U[105]) +
               Cbdsmv[i + 2] * U[106]) +
              Cbdsmv[i + 3] * U[107]);
  }
  d = U[100];
  dmv_tmp = U[101];
  d1 = U[102];
  d2 = U[103];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[90] = Jmv[0];
  varargout_1[91] = Jmv[1];
  varargout_1[92] = Jmv[2];
  varargout_1[93] = Jmv[3];
  varargout_1[94] = b_Je[0];
  varargout_1[95] = b_Je[1];
  _mm_storeu_pd(&varargout_1[96],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[104])),
                                      _mm_loadu_pd(&U[96])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[0]);
  r5 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[100],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[100]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[0]);
  r5 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[104], _mm_add_pd(r4, r5));
  _mm_storeu_pd(&varargout_1[98],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[106])),
                                      _mm_loadu_pd(&U[98])),
                           r3));
  r4 = _mm_loadu_pd(&Cineq[2]);
  r5 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[102],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[102]), r4), r5));
  r4 = _mm_loadu_pd(&Cbnds[2]);
  r5 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[106], _mm_add_pd(r4, r5));
  memcpy(&Lx[0], &lambda[70], 14U * sizeof(real_T));
  _mm_storeu_pd(&dmv[0],
                _mm_sub_pd(_mm_loadu_pd(&U[72]), _mm_loadu_pd(&U[54])));
  _mm_storeu_pd(&Cbnds[0], r);
  _mm_storeu_pd(&dmv[2],
                _mm_sub_pd(_mm_loadu_pd(&U[74]), _mm_loadu_pd(&U[56])));
  _mm_storeu_pd(&Cbnds[2], r);
  memset(&Cbdsmv[0], 0, 16U * sizeof(real_T));
  nlmpc_config__stateFcnJacobian(A, B);
  nlmpc_config__costFcnJacobian(&X[56], &U[72], dmv, &U[76],
                                &onlinedata_StageParameter[492], Jx, Jmv, Cineq,
                                Je);
  evaluateJacobian(&X[56], &U[72], dmv, &U[76], &onlinedata_StageParameter[492],
                   Cx, Cmv, a__3, Ce);
  Cbnds[0] = boundsToConstraints(U[72], iv[4], iv1[4], &Cbdsmv[0]);
  Cbnds[1] = boundsToConstraints(U[73], iv[14], iv1[14], &Cbdsmv[5]);
  Cbnds[2] = boundsToConstraints(U[74], iv[24], iv1[24], &Cbdsmv[10]);
  Cbnds[3] = boundsToConstraints(U[75], iv[34], iv1[34], &Cbdsmv[15]);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += A[k + 14 * i] * lambda[k + 70];
    }
    k = i << 2;
    d = lambda[i + 70] +
        (((Jx[i] + d) +
          (((Cx[k] * U[82] + Cx[k + 1] * U[83]) + Cx[k + 2] * U[84]) +
           Cx[k + 3] * U[85])) +
         (((0.0 * U[86] + 0.0 * U[87]) + 0.0 * U[88]) + 0.0 * U[89])) *
            0.001;
    Jx[i] = d;
    lambda[i + 56] = d;
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[56], b_Je, &onlinedata_StageParameter[492],
                           Cineq);
  for (k = 0; k < 4; k++) {
    d = -Cineq[k];
    Cineq[k] = d;
    dmv_tmp = U[k + 82];
    dmv[k] = muDoubleScalarSqrt((dmv_tmp * dmv_tmp + d * d) + 1.0E-12);
    dmv_tmp = U[k + 78];
    y[k] = dmv_tmp * dmv_tmp;
    d = 0.0;
    for (i = 0; i < 14; i++) {
      d += B[i + 14 * k] * Lx[i];
    }
    i = k << 2;
    Jmv[k] =
        ((Jmv[k] + d) +
         (((Cmv[i] * U[82] + Cmv[i + 1] * U[83]) + Cmv[i + 2] * U[84]) +
          Cmv[i + 3] * U[85])) +
        (((Cbdsmv[i] * U[86] + Cbdsmv[i + 1] * U[87]) + Cbdsmv[i + 2] * U[88]) +
         Cbdsmv[i + 3] * U[89]);
  }
  d = U[82];
  dmv_tmp = U[83];
  d1 = U[84];
  d2 = U[85];
  for (i = 0; i < 2; i++) {
    k = i << 2;
    b_Je[i] = Je[i] + (((Ce[k] * d + Ce[k + 1] * dmv_tmp) + Ce[k + 2] * d1) +
                       Ce[k + 3] * d2);
  }
  varargout_1[72] = Jmv[0];
  varargout_1[73] = Jmv[1];
  varargout_1[74] = Jmv[2];
  varargout_1[75] = Jmv[3];
  varargout_1[76] = b_Je[0];
  varargout_1[77] = b_Je[1];
  _mm_storeu_pd(&varargout_1[78],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[86])),
                                      _mm_loadu_pd(&U[78])),
                           r3));
  r = _mm_loadu_pd(&Cineq[0]);
  r4 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[82],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[82]), r), r4));
  r = _mm_loadu_pd(&Cbnds[0]);
  r4 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[86], _mm_add_pd(r, r4));
  _mm_storeu_pd(&b_U[0],
                _mm_sub_pd(_mm_loadu_pd(&U[54]), _mm_loadu_pd(&U[36])));
  _mm_storeu_pd(&varargout_1[80],
                _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r2, _mm_loadu_pd(&U[88])),
                                      _mm_loadu_pd(&U[80])),
                           r3));
  r = _mm_loadu_pd(&Cineq[2]);
  r4 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[84],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[84]), r), r4));
  r = _mm_loadu_pd(&Cbnds[2]);
  r4 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[88], _mm_add_pd(r, r4));
  _mm_storeu_pd(&b_U[2],
                _mm_sub_pd(_mm_loadu_pd(&U[56]), _mm_loadu_pd(&U[38])));
  b_getHamiltonianDerivatives(&X[42], &lambda[56], &U[54], &U[58], &U[60],
                              &U[64], &U[68], b_U,
                              &onlinedata_StageParameter[369], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r2 = _mm_loadu_pd(&lambda[i + 56]);
    r = _mm_add_pd(r2, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 42], r);
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[42], b_Je, &onlinedata_StageParameter[369],
                           Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[64]);
  r3 = _mm_set1_pd(1.0E-12);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[60]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[66]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[62]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[54], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r2 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[64],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[64]), r), r2));
  r = _mm_loadu_pd(&Cbnds[0]);
  r2 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[68], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[0],
                _mm_sub_pd(_mm_loadu_pd(&U[36]), _mm_loadu_pd(&U[18])));
  r = _mm_loadu_pd(&Cineq[2]);
  r2 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[66],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[66]), r), r2));
  r = _mm_loadu_pd(&Cbnds[2]);
  r2 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[70], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[2],
                _mm_sub_pd(_mm_loadu_pd(&U[38]), _mm_loadu_pd(&U[20])));
  c_getHamiltonianDerivatives(&X[28], &lambda[42], &U[36], &U[40], &U[42],
                              &U[46], &U[50], b_U,
                              &onlinedata_StageParameter[246], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r2 = _mm_loadu_pd(&lambda[i + 42]);
    r = _mm_add_pd(r2, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 28], r);
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[28], b_Je, &onlinedata_StageParameter[246],
                           Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[46]);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[42]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[48]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[44]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[36], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r2 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[46],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[46]), r), r2));
  r = _mm_loadu_pd(&Cbnds[0]);
  r2 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[50], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[0], _mm_sub_pd(_mm_loadu_pd(&U[18]), _mm_loadu_pd(&U[0])));
  r = _mm_loadu_pd(&Cineq[2]);
  r2 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[48],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[48]), r), r2));
  r = _mm_loadu_pd(&Cbnds[2]);
  r2 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[52], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[2], _mm_sub_pd(_mm_loadu_pd(&U[20]), _mm_loadu_pd(&U[2])));
  d_getHamiltonianDerivatives(&X[14], &lambda[28], &U[18], &U[22], &U[24],
                              &U[28], &U[32], b_U,
                              &onlinedata_StageParameter[123], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r2 = _mm_loadu_pd(&lambda[i + 28]);
    r = _mm_add_pd(r2, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 14], r);
  }
  b_Je[0] = 0.0;
  b_Je[1] = 0.0;
  nlmpc_config__ineqConFcn(&X[14], b_Je, &onlinedata_StageParameter[123],
                           Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[28]);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[24]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[30]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[26]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[18], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r2 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[28],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[28]), r), r2));
  r = _mm_loadu_pd(&Cbnds[0]);
  r2 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[32], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[0],
                _mm_sub_pd(_mm_loadu_pd(&U[0]), _mm_loadu_pd(&lastMV[0])));
  r = _mm_loadu_pd(&Cineq[2]);
  r2 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[30],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[30]), r), r2));
  r = _mm_loadu_pd(&Cbnds[2]);
  r2 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[34], _mm_add_pd(r, r2));
  _mm_storeu_pd(&b_U[2],
                _mm_sub_pd(_mm_loadu_pd(&U[2]), _mm_loadu_pd(&lastMV[2])));
  e_getHamiltonianDerivatives(&X[0], &lambda[14], &U[0], &U[4], &U[6], &U[10],
                              &U[14], b_U, &onlinedata_StageParameter[0], Lx,
                              Hmv, Cbnds);
  b_nlmpc_config__ineqConFcn(&X[0], &onlinedata_StageParameter[0], Cineq);
  r = _mm_loadu_pd(&Cineq[0]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[0], r);
  r2 = _mm_loadu_pd(&U[10]);
  _mm_storeu_pd(&dmv[0],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[6]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&Cineq[2]);
  r = _mm_mul_pd(r, r1);
  _mm_storeu_pd(&Cineq[2], r);
  r2 = _mm_loadu_pd(&U[12]);
  _mm_storeu_pd(&dmv[2],
                _mm_sqrt_pd(_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(r2, r2), _mm_mul_pd(r, r)), r3)));
  r = _mm_loadu_pd(&U[8]);
  _mm_storeu_pd(&y[2], _mm_mul_pd(r, r));
  memcpy(&varargout_1[0], &Hmv[0], 10U * sizeof(real_T));
  r = _mm_loadu_pd(&Cineq[0]);
  r1 = _mm_loadu_pd(&dmv[0]);
  _mm_storeu_pd(&varargout_1[10],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[10]), r), r1));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&varargout_1[14], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cineq[2]);
  r1 = _mm_loadu_pd(&dmv[2]);
  _mm_storeu_pd(&varargout_1[12],
                _mm_add_pd(_mm_add_pd(_mm_loadu_pd(&U[12]), r), r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&y[2]);
  _mm_storeu_pd(&varargout_1[16], _mm_add_pd(r, r1));
}

void nlmpcmoveCodeGeneration(const real_T x[14], const real_T lastMV[4],
                             struct3_T *onlinedata, real_T mv[4],
                             struct4_T *info)
{
  emxArray_real_T *resvec;
  real_T U_[186];
  real_T dZdt[186];
  real_T varargout_1[186];
  real_T z0[186];
  real_T dxdt[14];
  real_T ctInter;
  real_T exitflag;
  real_T fknorm;
  real_T scale;
  real_T varargout_3_firstorderopt;
  int32_T i;
  int32_T k;
  boolean_T exitg1;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  ctInter = 0.0;
  for (i = 0; i < 186; i++) {
    fknorm = onlinedata->InitialGuess[i];
    z0[i] = fknorm;
    dZdt[i] = onlinedata->InitialGuess[i + 186];
    U_[i] = fknorm;
  }
  exitflag = 0.0;
  i = 0;
  emxInit_real_T(&resvec, 1);
  exitg1 = false;
  while ((!exitg1) && (i < 20)) {
    exitflag = fdgmres(lastMV, onlinedata->StateFcnParameter,
                       onlinedata->StageParameter, z0, dZdt, x,
                       &onlinedata->InitialGuess[372], resvec, &fknorm, &scale);
    ctInter += scale;
    for (k = 0; k <= 184; k += 2) {
      __m128d r;
      __m128d r1;
      r = _mm_loadu_pd(&dZdt[k]);
      r1 = _mm_loadu_pd(&z0[k]);
      _mm_storeu_pd(&z0[k], _mm_add_pd(r1, _mm_div_pd(r, _mm_set1_pd(1000.0))));
    }
    real_T b_expl_temp[186];
    real_T expl_temp[186];
    real_T c_expl_temp[154];
    d_nlmpcmoveCodeGeneration_anonF(
        lastMV, onlinedata->StateFcnParameter, onlinedata->StageParameter, z0,
        x, varargout_1, info->MVopt, info->Xopt, info->Topt, info->Slack,
        expl_temp, b_expl_temp, &varargout_3_firstorderopt, c_expl_temp);
    fknorm = 0.0;
    scale = 3.3121686421112381E-170;
    for (k = 0; k < 186; k++) {
      real_T absxk;
      absxk = muDoubleScalarAbs(varargout_1[k]);
      if (absxk > scale) {
        real_T t;
        t = scale / absxk;
        fknorm = fknorm * t * t + 1.0;
        scale = absxk;
      } else {
        real_T t;
        t = absxk / scale;
        fknorm += t * t;
      }
    }
    fknorm = scale * muDoubleScalarSqrt(fknorm);
    if ((varargout_3_firstorderopt < 1.0E-6) && (exitflag == 1.0)) {
      exitg1 = true;
    } else if (fknorm < 1.0E-7) {
      exitflag = 1.0;
      exitg1 = true;
    } else if (muDoubleScalarIsNaN(fknorm) || muDoubleScalarIsInf(fknorm)) {
      exitflag = -1.0;
      exitg1 = true;
    } else {
      memcpy(&U_[0], &z0[0], 186U * sizeof(real_T));
      i++;
    }
  }
  emxFree_real_T(&resvec);
  if (exitflag == -1.0) {
    memcpy(&z0[0], &U_[0], 186U * sizeof(real_T));
  }
  fknorm = e_nlmpcmoveCodeGeneration_anonF(
      lastMV, onlinedata->StateFcnParameter, onlinedata->StageParameter, z0, x,
      varargout_1, info->MVopt, info->Xopt, info->Topt, info->Slack);
  info->Cost = fknorm;
  info->ExitFlag = exitflag;
  info->Iterations = ctInter;
  mv[0] = z0[0];
  mv[1] = z0[1];
  mv[2] = z0[2];
  mv[3] = z0[3];
  nlmpc_config__stateFcn(x, &z0[0], onlinedata->StateFcnParameter, dxdt);
  for (i = 0; i < 186; i++) {
    onlinedata->InitialGuess[i] = z0[i];
    onlinedata->InitialGuess[i + 186] = dZdt[i];
  }
  memcpy(&onlinedata->InitialGuess[372], &dxdt[0], 14U * sizeof(real_T));
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (nlmpcmoveCodeGeneration.c) */
