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
#include "computeOptimalityMatrix.h"
#include "evaluateStateTrajectory.h"
#include "fdgmres.h"
#include "getHamiltonianDerivatives.h"
#include "nlmpc_config__costFcnJacobian.h"
#include "nlmpc_config__ineqConFcn.h"
#include "nlmpc_config__stateFcn.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_emxutil.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "unpackDecisionVariables.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Declarations */
static real_T d_nlmpcmoveCodeGeneration_anonF(const real_T lastMV[4], const
  real_T onlinedata_StateFcnParameter[309], const real_T
  onlinedata_StageParameter[16473], const real_T U[2071], const real_T x[14],
  real_T varargout_1[2071], real_T varargout_2_MVopt[204], real_T
  varargout_2_Xopt[714], real_T varargout_2_Topt[51], real_T varargout_2_Slack
  [153], real_T varargout_3_x[2071], real_T varargout_3_fval[2071], real_T
  *varargout_3_firstorderopt, real_T varargout_3_lambda[714]);
static real_T e_nlmpcmoveCodeGeneration_anonF(const real_T lastMV[4], const
  real_T onlinedata_StateFcnParameter[309], const real_T
  onlinedata_StageParameter[16473], const real_T U[2071], const real_T x[14],
  real_T varargout_1[2071], real_T varargout_2_MVopt[204], real_T
  varargout_2_Xopt[714], real_T varargout_2_Topt[51], real_T varargout_2_Slack
  [153]);

/* Function Definitions */
static real_T d_nlmpcmoveCodeGeneration_anonF(const real_T lastMV[4], const
  real_T onlinedata_StateFcnParameter[309], const real_T
  onlinedata_StageParameter[16473], const real_T U[2071], const real_T x[14],
  real_T varargout_1[2071], real_T varargout_2_MVopt[204], real_T
  varargout_2_Xopt[714], real_T varargout_2_Topt[51], real_T varargout_2_Slack
  [153], real_T varargout_3_x[2071], real_T varargout_3_fval[2071], real_T
  *varargout_3_firstorderopt, real_T varargout_3_lambda[714])
{
  __m128d r;
  __m128d r1;
  real_T X[714];
  real_T Cineq[18];
  real_T a__5[18];
  real_T b_Cineq[18];
  real_T Hmv[15];
  real_T Lx[14];
  real_T Cbnds[8];
  real_T d[8];
  real_T mudummy[8];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T b_dv[4];
  real_T dmv[4];
  real_T Je[3];
  real_T e[3];
  real_T varargout_2_Cost;
  int32_T i;
  memset(&varargout_1[0], 0, 2071U * sizeof(real_T));
  memset(&varargout_3_lambda[0], 0, 714U * sizeof(real_T));
  evaluateStateTrajectory(U, x, onlinedata_StateFcnParameter, X);
  ae_unpackDecisionVariables(U, dmv, e, a__5);
  b_dv[0] = 0.0;
  b_dv[1] = 0.0;
  b_dv[2] = 0.0;
  b_dv[3] = 0.0;
  nlmpc_config__costFcnJacobian(&X[700], b_dv, dmv, e,
    &onlinedata_StageParameter[16150], Lx, Jmv, Jdmv, Je);
  memcpy(&varargout_3_lambda[700], &Lx[0], 14U * sizeof(real_T));
  unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  getHamiltonianDerivatives(&X[686], &varargout_3_lambda[700], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[15827], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 700]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 686], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[686], Jmv, e, &onlinedata_StageParameter[15827],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[2009], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[2024], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[2042], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[2044], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[2046], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[2048], _mm_add_pd(r, r1));
  b_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  b_getHamiltonianDerivatives(&X[672], &varargout_3_lambda[686], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[15504], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 686]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 672], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[672], Jmv, e, &onlinedata_StageParameter[15504],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1968], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1983], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[2001], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[2003], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[2005], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[2007], _mm_add_pd(r, r1));
  c_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  c_getHamiltonianDerivatives(&X[658], &varargout_3_lambda[672], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[15181], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 672]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 658], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[658], Jmv, e, &onlinedata_StageParameter[15181],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1927], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1942], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1960], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1962], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1964], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1966], _mm_add_pd(r, r1));
  d_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  d_getHamiltonianDerivatives(&X[644], &varargout_3_lambda[658], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[14858], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 658]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 644], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[644], Jmv, e, &onlinedata_StageParameter[14858],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1886], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1901], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1919], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1921], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1923], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1925], _mm_add_pd(r, r1));
  e_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  e_getHamiltonianDerivatives(&X[630], &varargout_3_lambda[644], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[14535], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 644]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 630], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[630], Jmv, e, &onlinedata_StageParameter[14535],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1845], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1860], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1878], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1880], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1882], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1884], _mm_add_pd(r, r1));
  f_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  f_getHamiltonianDerivatives(&X[616], &varargout_3_lambda[630], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[14212], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 630]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 616], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[616], Jmv, e, &onlinedata_StageParameter[14212],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1804], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1819], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1837], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1839], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1841], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1843], _mm_add_pd(r, r1));
  g_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  g_getHamiltonianDerivatives(&X[602], &varargout_3_lambda[616], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[13889], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 616]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 602], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[602], Jmv, e, &onlinedata_StageParameter[13889],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1763], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1778], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1796], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1798], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1800], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1802], _mm_add_pd(r, r1));
  h_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  h_getHamiltonianDerivatives(&X[588], &varargout_3_lambda[602], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[13566], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 602]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 588], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[588], Jmv, e, &onlinedata_StageParameter[13566],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1722], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1737], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1755], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1757], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1759], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1761], _mm_add_pd(r, r1));
  i_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  i_getHamiltonianDerivatives(&X[574], &varargout_3_lambda[588], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[13243], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 588]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 574], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[574], Jmv, e, &onlinedata_StageParameter[13243],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1681], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1696], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1714], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1716], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1718], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1720], _mm_add_pd(r, r1));
  j_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  j_getHamiltonianDerivatives(&X[560], &varargout_3_lambda[574], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[12920], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 574]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 560], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[560], Jmv, e, &onlinedata_StageParameter[12920],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1640], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1655], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1673], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1675], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1677], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1679], _mm_add_pd(r, r1));
  k_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  k_getHamiltonianDerivatives(&X[546], &varargout_3_lambda[560], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[12597], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 560]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 546], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[546], Jmv, e, &onlinedata_StageParameter[12597],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1599], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1614], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1632], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1634], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1636], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1638], _mm_add_pd(r, r1));
  l_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  l_getHamiltonianDerivatives(&X[532], &varargout_3_lambda[546], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[12274], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 546]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 532], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[532], Jmv, e, &onlinedata_StageParameter[12274],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1558], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1573], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1591], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1593], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1595], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1597], _mm_add_pd(r, r1));
  m_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  m_getHamiltonianDerivatives(&X[518], &varargout_3_lambda[532], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[11951], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 532]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 518], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[518], Jmv, e, &onlinedata_StageParameter[11951],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1517], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1532], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1550], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1552], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1554], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1556], _mm_add_pd(r, r1));
  n_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  n_getHamiltonianDerivatives(&X[504], &varargout_3_lambda[518], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[11628], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 518]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 504], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[504], Jmv, e, &onlinedata_StageParameter[11628],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1476], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1491], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1509], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1511], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1513], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1515], _mm_add_pd(r, r1));
  o_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  o_getHamiltonianDerivatives(&X[490], &varargout_3_lambda[504], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[11305], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 504]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 490], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[490], Jmv, e, &onlinedata_StageParameter[11305],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1435], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1450], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1468], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1470], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1472], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1474], _mm_add_pd(r, r1));
  p_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  p_getHamiltonianDerivatives(&X[476], &varargout_3_lambda[490], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[10982], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 490]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 476], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[476], Jmv, e, &onlinedata_StageParameter[10982],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1394], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1409], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1427], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1429], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1431], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1433], _mm_add_pd(r, r1));
  q_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  q_getHamiltonianDerivatives(&X[462], &varargout_3_lambda[476], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[10659], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 476]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 462], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[462], Jmv, e, &onlinedata_StageParameter[10659],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1353], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1368], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1386], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1388], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1390], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1392], _mm_add_pd(r, r1));
  r_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  r_getHamiltonianDerivatives(&X[448], &varargout_3_lambda[462], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[10336], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 462]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 448], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[448], Jmv, e, &onlinedata_StageParameter[10336],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1312], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1327], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1345], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1347], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1349], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1351], _mm_add_pd(r, r1));
  s_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  s_getHamiltonianDerivatives(&X[434], &varargout_3_lambda[448], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[10013], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 448]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 434], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[434], Jmv, e, &onlinedata_StageParameter[10013],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1271], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1286], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1304], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1306], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1308], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1310], _mm_add_pd(r, r1));
  t_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  t_getHamiltonianDerivatives(&X[420], &varargout_3_lambda[434], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[9690], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 434]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 420], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[420], Jmv, e, &onlinedata_StageParameter[9690],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1230], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1245], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1263], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1265], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1267], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1269], _mm_add_pd(r, r1));
  u_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  u_getHamiltonianDerivatives(&X[406], &varargout_3_lambda[420], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[9367], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 420]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 406], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[406], Jmv, e, &onlinedata_StageParameter[9367],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1189], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1204], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1222], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1224], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1226], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1228], _mm_add_pd(r, r1));
  v_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  v_getHamiltonianDerivatives(&X[392], &varargout_3_lambda[406], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[9044], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 406]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 392], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[392], Jmv, e, &onlinedata_StageParameter[9044],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1148], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1163], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1181], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1183], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1185], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1187], _mm_add_pd(r, r1));
  w_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  w_getHamiltonianDerivatives(&X[378], &varargout_3_lambda[392], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[8721], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 392]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 378], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[378], Jmv, e, &onlinedata_StageParameter[8721],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1107], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1122], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1140], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1142], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1144], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1146], _mm_add_pd(r, r1));
  x_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  x_getHamiltonianDerivatives(&X[364], &varargout_3_lambda[378], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[8398], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 378]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 364], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[364], Jmv, e, &onlinedata_StageParameter[8398],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1066], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1081], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1099], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1101], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1103], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1105], _mm_add_pd(r, r1));
  y_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  y_getHamiltonianDerivatives(&X[350], &varargout_3_lambda[364], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[8075], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 364]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 350], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[350], Jmv, e, &onlinedata_StageParameter[8075],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1025], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1040], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1058], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1060], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1062], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1064], _mm_add_pd(r, r1));
  ab_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ab_getHamiltonianDerivatives(&X[336], &varargout_3_lambda[350], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[7752], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 350]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 336], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[336], Jmv, e, &onlinedata_StageParameter[7752],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[984], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[999], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1017], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1019], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1021], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1023], _mm_add_pd(r, r1));
  bb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  bb_getHamiltonianDerivatives(&X[322], &varargout_3_lambda[336], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[7429], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 336]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 322], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[322], Jmv, e, &onlinedata_StageParameter[7429],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[943], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[958], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[976], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[978], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[980], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[982], _mm_add_pd(r, r1));
  cb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  cb_getHamiltonianDerivatives(&X[308], &varargout_3_lambda[322], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[7106], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 322]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 308], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[308], Jmv, e, &onlinedata_StageParameter[7106],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[902], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[917], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[935], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[937], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[939], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[941], _mm_add_pd(r, r1));
  db_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  db_getHamiltonianDerivatives(&X[294], &varargout_3_lambda[308], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[6783], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 308]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 294], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[294], Jmv, e, &onlinedata_StageParameter[6783],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[861], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[876], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[894], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[896], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[898], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[900], _mm_add_pd(r, r1));
  eb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  eb_getHamiltonianDerivatives(&X[280], &varargout_3_lambda[294], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[6460], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 294]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 280], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[280], Jmv, e, &onlinedata_StageParameter[6460],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[820], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[835], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[853], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[855], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[857], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[859], _mm_add_pd(r, r1));
  fb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  fb_getHamiltonianDerivatives(&X[266], &varargout_3_lambda[280], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[6137], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 280]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 266], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[266], Jmv, e, &onlinedata_StageParameter[6137],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[779], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[794], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[812], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[814], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[816], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[818], _mm_add_pd(r, r1));
  gb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  gb_getHamiltonianDerivatives(&X[252], &varargout_3_lambda[266], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[5814], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 266]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 252], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[252], Jmv, e, &onlinedata_StageParameter[5814],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[738], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[753], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[771], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[773], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[775], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[777], _mm_add_pd(r, r1));
  hb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  hb_getHamiltonianDerivatives(&X[238], &varargout_3_lambda[252], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[5491], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 252]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 238], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[238], Jmv, e, &onlinedata_StageParameter[5491],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[697], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[712], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[730], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[732], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[734], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[736], _mm_add_pd(r, r1));
  ib_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ib_getHamiltonianDerivatives(&X[224], &varargout_3_lambda[238], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[5168], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 238]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 224], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[224], Jmv, e, &onlinedata_StageParameter[5168],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[656], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[671], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[689], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[691], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[693], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[695], _mm_add_pd(r, r1));
  jb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  jb_getHamiltonianDerivatives(&X[210], &varargout_3_lambda[224], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[4845], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 224]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 210], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[210], Jmv, e, &onlinedata_StageParameter[4845],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[615], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[630], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[648], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[650], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[652], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[654], _mm_add_pd(r, r1));
  kb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  kb_getHamiltonianDerivatives(&X[196], &varargout_3_lambda[210], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[4522], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 210]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 196], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[196], Jmv, e, &onlinedata_StageParameter[4522],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[574], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[589], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[607], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[609], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[611], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[613], _mm_add_pd(r, r1));
  lb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  lb_getHamiltonianDerivatives(&X[182], &varargout_3_lambda[196], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[4199], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 196]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 182], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[182], Jmv, e, &onlinedata_StageParameter[4199],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[533], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[548], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[566], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[568], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[570], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[572], _mm_add_pd(r, r1));
  mb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  mb_getHamiltonianDerivatives(&X[168], &varargout_3_lambda[182], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[3876], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 182]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 168], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[168], Jmv, e, &onlinedata_StageParameter[3876],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[492], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[507], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[525], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[527], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[529], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[531], _mm_add_pd(r, r1));
  nb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  nb_getHamiltonianDerivatives(&X[154], &varargout_3_lambda[168], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[3553], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 168]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 154], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[154], Jmv, e, &onlinedata_StageParameter[3553],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[451], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[466], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[484], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[486], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[488], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[490], _mm_add_pd(r, r1));
  ob_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ob_getHamiltonianDerivatives(&X[140], &varargout_3_lambda[154], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[3230], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 154]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 140], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[140], Jmv, e, &onlinedata_StageParameter[3230],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[410], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[425], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[443], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[445], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[447], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[449], _mm_add_pd(r, r1));
  pb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  pb_getHamiltonianDerivatives(&X[126], &varargout_3_lambda[140], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[2907], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 140]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 126], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[126], Jmv, e, &onlinedata_StageParameter[2907],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[369], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[384], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[402], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[404], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[406], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[408], _mm_add_pd(r, r1));
  qb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  qb_getHamiltonianDerivatives(&X[112], &varargout_3_lambda[126], Jmv, e, d,
    a__5, mudummy, dmv, &onlinedata_StageParameter[2584], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 126]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 112], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[112], Jmv, e, &onlinedata_StageParameter[2584],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[328], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[343], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[361], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[363], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[365], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[367], _mm_add_pd(r, r1));
  rb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  rb_getHamiltonianDerivatives(&X[98], &varargout_3_lambda[112], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[2261], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 112]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 98], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[98], Jmv, e, &onlinedata_StageParameter[2261],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[287], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[302], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[320], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[322], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[324], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[326], _mm_add_pd(r, r1));
  sb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  sb_getHamiltonianDerivatives(&X[84], &varargout_3_lambda[98], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[1938], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 98]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 84], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[84], Jmv, e, &onlinedata_StageParameter[1938],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[246], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[261], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[279], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[281], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[283], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[285], _mm_add_pd(r, r1));
  tb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  tb_getHamiltonianDerivatives(&X[70], &varargout_3_lambda[84], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[1615], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 84]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 70], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[70], Jmv, e, &onlinedata_StageParameter[1615],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[205], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[220], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[238], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[240], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[242], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[244], _mm_add_pd(r, r1));
  ub_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ub_getHamiltonianDerivatives(&X[56], &varargout_3_lambda[70], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[1292], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 70]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 56], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[56], Jmv, e, &onlinedata_StageParameter[1292],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[164], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[179], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[197], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[199], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[201], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[203], _mm_add_pd(r, r1));
  vb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  vb_getHamiltonianDerivatives(&X[42], &varargout_3_lambda[56], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[969], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 56]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 42], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[42], Jmv, e, &onlinedata_StageParameter[969],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[123], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[138], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[156], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[158], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[160], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[162], _mm_add_pd(r, r1));
  wb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  wb_getHamiltonianDerivatives(&X[28], &varargout_3_lambda[42], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[646], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 42]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 28], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[28], Jmv, e, &onlinedata_StageParameter[646],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[82], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[97], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[115], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[117], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[119], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[121], _mm_add_pd(r, r1));
  xb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  xb_getHamiltonianDerivatives(&X[14], &varargout_3_lambda[28], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[323], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 28]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i + 14], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[14], Jmv, e, &onlinedata_StageParameter[323],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[41], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[56], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[74], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[76], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[78], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[80], _mm_add_pd(r, r1));
  yb_unpackDecisionVariables(U, lastMV, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  yb_getHamiltonianDerivatives(&X[0], &varargout_3_lambda[14], Jmv, e, d, a__5,
    mudummy, dmv, &onlinedata_StageParameter[0], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&varargout_3_lambda[i + 14]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&varargout_3_lambda[i], r);
  }

  b_nlmpc_config__ineqConFcn(&X[0], &onlinedata_StageParameter[0], Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[0], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[15], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[33], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[35], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[37], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[39], _mm_add_pd(r, r1));
  computeInfo(X, U, onlinedata_StageParameter, lastMV, varargout_2_MVopt,
              varargout_2_Xopt, varargout_2_Topt, varargout_2_Slack,
              &varargout_2_Cost);
  *varargout_3_firstorderopt = b_norm(varargout_1);
  memcpy(&varargout_3_x[0], &U[0], 2071U * sizeof(real_T));
  memcpy(&varargout_3_fval[0], &varargout_1[0], 2071U * sizeof(real_T));
  return varargout_2_Cost;
}

static real_T e_nlmpcmoveCodeGeneration_anonF(const real_T lastMV[4], const
  real_T onlinedata_StateFcnParameter[309], const real_T
  onlinedata_StageParameter[16473], const real_T U[2071], const real_T x[14],
  real_T varargout_1[2071], real_T varargout_2_MVopt[204], real_T
  varargout_2_Xopt[714], real_T varargout_2_Topt[51], real_T varargout_2_Slack
  [153])
{
  __m128d r;
  __m128d r1;
  real_T X[714];
  real_T lambda[714];
  real_T Cineq[18];
  real_T a__5[18];
  real_T b_Cineq[18];
  real_T Hmv[15];
  real_T Lx[14];
  real_T Cbnds[8];
  real_T d[8];
  real_T mudummy[8];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T b_dv[4];
  real_T dmv[4];
  real_T Je[3];
  real_T e[3];
  real_T varargout_2_Cost;
  int32_T i;
  memset(&varargout_1[0], 0, 2071U * sizeof(real_T));
  memset(&lambda[0], 0, 714U * sizeof(real_T));
  evaluateStateTrajectory(U, x, onlinedata_StateFcnParameter, X);
  ae_unpackDecisionVariables(U, dmv, e, a__5);
  b_dv[0] = 0.0;
  b_dv[1] = 0.0;
  b_dv[2] = 0.0;
  b_dv[3] = 0.0;
  nlmpc_config__costFcnJacobian(&X[700], b_dv, dmv, e,
    &onlinedata_StageParameter[16150], Lx, Jmv, Jdmv, Je);
  memcpy(&lambda[700], &Lx[0], 14U * sizeof(real_T));
  unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  getHamiltonianDerivatives(&X[686], &lambda[700], Jmv, e, d, a__5, mudummy, dmv,
    &onlinedata_StageParameter[15827], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 700]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 686], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[686], Jmv, e, &onlinedata_StageParameter[15827],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[2009], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[2024], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[2042], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[2044], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[2046], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[2048], _mm_add_pd(r, r1));
  b_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  b_getHamiltonianDerivatives(&X[672], &lambda[686], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[15504], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 686]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 672], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[672], Jmv, e, &onlinedata_StageParameter[15504],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1968], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1983], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[2001], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[2003], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[2005], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[2007], _mm_add_pd(r, r1));
  c_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  c_getHamiltonianDerivatives(&X[658], &lambda[672], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[15181], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 672]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 658], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[658], Jmv, e, &onlinedata_StageParameter[15181],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1927], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1942], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1960], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1962], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1964], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1966], _mm_add_pd(r, r1));
  d_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  d_getHamiltonianDerivatives(&X[644], &lambda[658], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[14858], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 658]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 644], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[644], Jmv, e, &onlinedata_StageParameter[14858],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1886], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1901], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1919], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1921], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1923], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1925], _mm_add_pd(r, r1));
  e_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  e_getHamiltonianDerivatives(&X[630], &lambda[644], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[14535], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 644]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 630], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[630], Jmv, e, &onlinedata_StageParameter[14535],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1845], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1860], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1878], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1880], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1882], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1884], _mm_add_pd(r, r1));
  f_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  f_getHamiltonianDerivatives(&X[616], &lambda[630], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[14212], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 630]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 616], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[616], Jmv, e, &onlinedata_StageParameter[14212],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1804], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1819], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1837], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1839], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1841], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1843], _mm_add_pd(r, r1));
  g_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  g_getHamiltonianDerivatives(&X[602], &lambda[616], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[13889], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 616]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 602], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[602], Jmv, e, &onlinedata_StageParameter[13889],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1763], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1778], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1796], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1798], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1800], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1802], _mm_add_pd(r, r1));
  h_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  h_getHamiltonianDerivatives(&X[588], &lambda[602], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[13566], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 602]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 588], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[588], Jmv, e, &onlinedata_StageParameter[13566],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1722], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1737], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1755], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1757], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1759], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1761], _mm_add_pd(r, r1));
  i_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  i_getHamiltonianDerivatives(&X[574], &lambda[588], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[13243], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 588]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 574], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[574], Jmv, e, &onlinedata_StageParameter[13243],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1681], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1696], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1714], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1716], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1718], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1720], _mm_add_pd(r, r1));
  j_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  j_getHamiltonianDerivatives(&X[560], &lambda[574], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[12920], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 574]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 560], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[560], Jmv, e, &onlinedata_StageParameter[12920],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1640], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1655], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1673], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1675], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1677], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1679], _mm_add_pd(r, r1));
  k_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  k_getHamiltonianDerivatives(&X[546], &lambda[560], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[12597], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 560]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 546], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[546], Jmv, e, &onlinedata_StageParameter[12597],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1599], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1614], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1632], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1634], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1636], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1638], _mm_add_pd(r, r1));
  l_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  l_getHamiltonianDerivatives(&X[532], &lambda[546], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[12274], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 546]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 532], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[532], Jmv, e, &onlinedata_StageParameter[12274],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1558], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1573], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1591], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1593], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1595], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1597], _mm_add_pd(r, r1));
  m_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  m_getHamiltonianDerivatives(&X[518], &lambda[532], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[11951], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 532]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 518], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[518], Jmv, e, &onlinedata_StageParameter[11951],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1517], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1532], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1550], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1552], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1554], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1556], _mm_add_pd(r, r1));
  n_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  n_getHamiltonianDerivatives(&X[504], &lambda[518], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[11628], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 518]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 504], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[504], Jmv, e, &onlinedata_StageParameter[11628],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1476], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1491], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1509], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1511], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1513], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1515], _mm_add_pd(r, r1));
  o_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  o_getHamiltonianDerivatives(&X[490], &lambda[504], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[11305], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 504]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 490], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[490], Jmv, e, &onlinedata_StageParameter[11305],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1435], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1450], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1468], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1470], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1472], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1474], _mm_add_pd(r, r1));
  p_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  p_getHamiltonianDerivatives(&X[476], &lambda[490], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[10982], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 490]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 476], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[476], Jmv, e, &onlinedata_StageParameter[10982],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1394], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1409], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1427], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1429], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1431], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1433], _mm_add_pd(r, r1));
  q_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  q_getHamiltonianDerivatives(&X[462], &lambda[476], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[10659], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 476]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 462], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[462], Jmv, e, &onlinedata_StageParameter[10659],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1353], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1368], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1386], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1388], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1390], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1392], _mm_add_pd(r, r1));
  r_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  r_getHamiltonianDerivatives(&X[448], &lambda[462], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[10336], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 462]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 448], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[448], Jmv, e, &onlinedata_StageParameter[10336],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1312], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1327], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1345], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1347], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1349], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1351], _mm_add_pd(r, r1));
  s_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  s_getHamiltonianDerivatives(&X[434], &lambda[448], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[10013], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 448]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 434], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[434], Jmv, e, &onlinedata_StageParameter[10013],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1271], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1286], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1304], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1306], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1308], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1310], _mm_add_pd(r, r1));
  t_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  t_getHamiltonianDerivatives(&X[420], &lambda[434], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[9690], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 434]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 420], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[420], Jmv, e, &onlinedata_StageParameter[9690],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1230], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1245], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1263], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1265], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1267], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1269], _mm_add_pd(r, r1));
  u_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  u_getHamiltonianDerivatives(&X[406], &lambda[420], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[9367], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 420]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 406], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[406], Jmv, e, &onlinedata_StageParameter[9367],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1189], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1204], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1222], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1224], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1226], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1228], _mm_add_pd(r, r1));
  v_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  v_getHamiltonianDerivatives(&X[392], &lambda[406], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[9044], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 406]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 392], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[392], Jmv, e, &onlinedata_StageParameter[9044],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1148], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1163], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1181], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1183], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1185], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1187], _mm_add_pd(r, r1));
  w_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  w_getHamiltonianDerivatives(&X[378], &lambda[392], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[8721], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 392]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 378], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[378], Jmv, e, &onlinedata_StageParameter[8721],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1107], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1122], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1140], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1142], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1144], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1146], _mm_add_pd(r, r1));
  x_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  x_getHamiltonianDerivatives(&X[364], &lambda[378], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[8398], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 378]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 364], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[364], Jmv, e, &onlinedata_StageParameter[8398],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1066], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1081], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1099], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1101], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1103], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1105], _mm_add_pd(r, r1));
  y_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  y_getHamiltonianDerivatives(&X[350], &lambda[364], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[8075], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 364]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 350], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[350], Jmv, e, &onlinedata_StageParameter[8075],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1025], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1040], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1058], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1060], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1062], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1064], _mm_add_pd(r, r1));
  ab_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ab_getHamiltonianDerivatives(&X[336], &lambda[350], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[7752], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 350]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 336], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[336], Jmv, e, &onlinedata_StageParameter[7752],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[984], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[999], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1017], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1019], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1021], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1023], _mm_add_pd(r, r1));
  bb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  bb_getHamiltonianDerivatives(&X[322], &lambda[336], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[7429], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 336]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 322], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[322], Jmv, e, &onlinedata_StageParameter[7429],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[943], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[958], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[976], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[978], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[980], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[982], _mm_add_pd(r, r1));
  cb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  cb_getHamiltonianDerivatives(&X[308], &lambda[322], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[7106], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 322]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 308], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[308], Jmv, e, &onlinedata_StageParameter[7106],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[902], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[917], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[935], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[937], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[939], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[941], _mm_add_pd(r, r1));
  db_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  db_getHamiltonianDerivatives(&X[294], &lambda[308], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[6783], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 308]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 294], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[294], Jmv, e, &onlinedata_StageParameter[6783],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[861], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[876], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[894], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[896], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[898], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[900], _mm_add_pd(r, r1));
  eb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  eb_getHamiltonianDerivatives(&X[280], &lambda[294], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[6460], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 294]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 280], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[280], Jmv, e, &onlinedata_StageParameter[6460],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[820], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[835], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[853], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[855], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[857], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[859], _mm_add_pd(r, r1));
  fb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  fb_getHamiltonianDerivatives(&X[266], &lambda[280], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[6137], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 280]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 266], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[266], Jmv, e, &onlinedata_StageParameter[6137],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[779], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[794], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[812], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[814], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[816], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[818], _mm_add_pd(r, r1));
  gb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  gb_getHamiltonianDerivatives(&X[252], &lambda[266], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[5814], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 266]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 252], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[252], Jmv, e, &onlinedata_StageParameter[5814],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[738], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[753], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[771], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[773], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[775], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[777], _mm_add_pd(r, r1));
  hb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  hb_getHamiltonianDerivatives(&X[238], &lambda[252], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[5491], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 252]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 238], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[238], Jmv, e, &onlinedata_StageParameter[5491],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[697], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[712], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[730], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[732], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[734], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[736], _mm_add_pd(r, r1));
  ib_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ib_getHamiltonianDerivatives(&X[224], &lambda[238], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[5168], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 238]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 224], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[224], Jmv, e, &onlinedata_StageParameter[5168],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[656], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[671], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[689], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[691], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[693], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[695], _mm_add_pd(r, r1));
  jb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  jb_getHamiltonianDerivatives(&X[210], &lambda[224], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[4845], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 224]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 210], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[210], Jmv, e, &onlinedata_StageParameter[4845],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[615], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[630], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[648], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[650], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[652], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[654], _mm_add_pd(r, r1));
  kb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  kb_getHamiltonianDerivatives(&X[196], &lambda[210], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[4522], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 210]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 196], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[196], Jmv, e, &onlinedata_StageParameter[4522],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[574], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[589], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[607], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[609], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[611], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[613], _mm_add_pd(r, r1));
  lb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  lb_getHamiltonianDerivatives(&X[182], &lambda[196], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[4199], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 196]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 182], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[182], Jmv, e, &onlinedata_StageParameter[4199],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[533], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[548], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[566], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[568], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[570], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[572], _mm_add_pd(r, r1));
  mb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  mb_getHamiltonianDerivatives(&X[168], &lambda[182], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[3876], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 182]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 168], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[168], Jmv, e, &onlinedata_StageParameter[3876],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[492], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[507], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[525], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[527], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[529], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[531], _mm_add_pd(r, r1));
  nb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  nb_getHamiltonianDerivatives(&X[154], &lambda[168], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[3553], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 168]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 154], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[154], Jmv, e, &onlinedata_StageParameter[3553],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[451], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[466], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[484], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[486], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[488], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[490], _mm_add_pd(r, r1));
  ob_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ob_getHamiltonianDerivatives(&X[140], &lambda[154], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[3230], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 154]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 140], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[140], Jmv, e, &onlinedata_StageParameter[3230],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[410], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[425], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[443], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[445], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[447], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[449], _mm_add_pd(r, r1));
  pb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  pb_getHamiltonianDerivatives(&X[126], &lambda[140], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[2907], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 140]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 126], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[126], Jmv, e, &onlinedata_StageParameter[2907],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[369], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[384], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[402], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[404], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[406], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[408], _mm_add_pd(r, r1));
  qb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  qb_getHamiltonianDerivatives(&X[112], &lambda[126], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[2584], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 126]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 112], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[112], Jmv, e, &onlinedata_StageParameter[2584],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[328], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[343], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[361], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[363], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[365], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[367], _mm_add_pd(r, r1));
  rb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  rb_getHamiltonianDerivatives(&X[98], &lambda[112], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[2261], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 112]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 98], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[98], Jmv, e, &onlinedata_StageParameter[2261],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[287], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[302], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[320], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[322], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[324], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[326], _mm_add_pd(r, r1));
  sb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  sb_getHamiltonianDerivatives(&X[84], &lambda[98], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[1938], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 98]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 84], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[84], Jmv, e, &onlinedata_StageParameter[1938],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[246], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[261], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[279], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[281], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[283], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[285], _mm_add_pd(r, r1));
  tb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  tb_getHamiltonianDerivatives(&X[70], &lambda[84], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[1615], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 84]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 70], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[70], Jmv, e, &onlinedata_StageParameter[1615],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[205], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[220], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[238], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[240], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[242], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[244], _mm_add_pd(r, r1));
  ub_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ub_getHamiltonianDerivatives(&X[56], &lambda[70], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[1292], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 70]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 56], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[56], Jmv, e, &onlinedata_StageParameter[1292],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[164], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[179], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[197], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[199], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[201], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[203], _mm_add_pd(r, r1));
  vb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  vb_getHamiltonianDerivatives(&X[42], &lambda[56], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[969], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 56]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 42], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[42], Jmv, e, &onlinedata_StageParameter[969],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[123], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[138], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[156], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[158], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[160], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[162], _mm_add_pd(r, r1));
  wb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  wb_getHamiltonianDerivatives(&X[28], &lambda[42], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[646], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 42]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 28], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[28], Jmv, e, &onlinedata_StageParameter[646],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[82], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[97], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[115], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[117], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[119], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[121], _mm_add_pd(r, r1));
  xb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  xb_getHamiltonianDerivatives(&X[14], &lambda[28], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[323], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 28]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 14], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[14], Jmv, e, &onlinedata_StageParameter[323],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[41], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[56], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[74], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[76], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[78], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[80], _mm_add_pd(r, r1));
  yb_unpackDecisionVariables(U, lastMV, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  yb_getHamiltonianDerivatives(&X[0], &lambda[14], Jmv, e, d, a__5, mudummy, dmv,
    &onlinedata_StageParameter[0], Lx, Hmv, Cbnds);
  b_nlmpc_config__ineqConFcn(&X[0], &onlinedata_StageParameter[0], Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[0], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[15], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[33], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[35], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[37], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[39], _mm_add_pd(r, r1));
  computeInfo(X, U, onlinedata_StageParameter, lastMV, varargout_2_MVopt,
              varargout_2_Xopt, varargout_2_Topt, varargout_2_Slack,
              &varargout_2_Cost);
  return varargout_2_Cost;
}

void c_nlmpcmoveCodeGeneration_anonF(const real_T lastMV[4], const real_T
  onlinedata_StateFcnParameter[309], const real_T onlinedata_StageParameter
  [16473], const real_T U[2071], const real_T x[14], real_T varargout_1[2071])
{
  __m128d r;
  __m128d r1;
  real_T X[714];
  real_T lambda[714];
  real_T Cineq[18];
  real_T a__5[18];
  real_T b_Cineq[18];
  real_T Hmv[15];
  real_T Lx[14];
  real_T Cbnds[8];
  real_T d[8];
  real_T mudummy[8];
  real_T Jdmv[4];
  real_T Jmv[4];
  real_T b_dv[4];
  real_T dmv[4];
  real_T Je[3];
  real_T e[3];
  int32_T i;
  memset(&varargout_1[0], 0, 2071U * sizeof(real_T));
  memset(&lambda[0], 0, 714U * sizeof(real_T));
  evaluateStateTrajectory(U, x, onlinedata_StateFcnParameter, X);
  ae_unpackDecisionVariables(U, dmv, e, a__5);
  b_dv[0] = 0.0;
  b_dv[1] = 0.0;
  b_dv[2] = 0.0;
  b_dv[3] = 0.0;
  nlmpc_config__costFcnJacobian(&X[700], b_dv, dmv, e,
    &onlinedata_StageParameter[16150], Lx, Jmv, Jdmv, Je);
  memcpy(&lambda[700], &Lx[0], 14U * sizeof(real_T));
  unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  getHamiltonianDerivatives(&X[686], &lambda[700], Jmv, e, d, a__5, mudummy, dmv,
    &onlinedata_StageParameter[15827], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 700]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 686], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[686], Jmv, e, &onlinedata_StageParameter[15827],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[2009], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[2024], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[2042], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[2044], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[2046], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[2048], _mm_add_pd(r, r1));
  b_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  b_getHamiltonianDerivatives(&X[672], &lambda[686], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[15504], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 686]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 672], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[672], Jmv, e, &onlinedata_StageParameter[15504],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1968], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1983], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[2001], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[2003], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[2005], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[2007], _mm_add_pd(r, r1));
  c_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  c_getHamiltonianDerivatives(&X[658], &lambda[672], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[15181], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 672]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 658], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[658], Jmv, e, &onlinedata_StageParameter[15181],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1927], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1942], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1960], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1962], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1964], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1966], _mm_add_pd(r, r1));
  d_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  d_getHamiltonianDerivatives(&X[644], &lambda[658], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[14858], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 658]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 644], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[644], Jmv, e, &onlinedata_StageParameter[14858],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1886], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1901], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1919], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1921], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1923], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1925], _mm_add_pd(r, r1));
  e_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  e_getHamiltonianDerivatives(&X[630], &lambda[644], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[14535], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 644]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 630], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[630], Jmv, e, &onlinedata_StageParameter[14535],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1845], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1860], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1878], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1880], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1882], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1884], _mm_add_pd(r, r1));
  f_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  f_getHamiltonianDerivatives(&X[616], &lambda[630], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[14212], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 630]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 616], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[616], Jmv, e, &onlinedata_StageParameter[14212],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1804], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1819], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1837], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1839], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1841], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1843], _mm_add_pd(r, r1));
  g_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  g_getHamiltonianDerivatives(&X[602], &lambda[616], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[13889], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 616]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 602], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[602], Jmv, e, &onlinedata_StageParameter[13889],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1763], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1778], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1796], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1798], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1800], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1802], _mm_add_pd(r, r1));
  h_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  h_getHamiltonianDerivatives(&X[588], &lambda[602], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[13566], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 602]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 588], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[588], Jmv, e, &onlinedata_StageParameter[13566],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1722], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1737], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1755], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1757], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1759], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1761], _mm_add_pd(r, r1));
  i_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  i_getHamiltonianDerivatives(&X[574], &lambda[588], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[13243], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 588]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 574], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[574], Jmv, e, &onlinedata_StageParameter[13243],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1681], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1696], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1714], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1716], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1718], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1720], _mm_add_pd(r, r1));
  j_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  j_getHamiltonianDerivatives(&X[560], &lambda[574], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[12920], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 574]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 560], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[560], Jmv, e, &onlinedata_StageParameter[12920],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1640], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1655], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1673], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1675], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1677], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1679], _mm_add_pd(r, r1));
  k_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  k_getHamiltonianDerivatives(&X[546], &lambda[560], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[12597], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 560]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 546], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[546], Jmv, e, &onlinedata_StageParameter[12597],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1599], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1614], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1632], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1634], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1636], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1638], _mm_add_pd(r, r1));
  l_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  l_getHamiltonianDerivatives(&X[532], &lambda[546], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[12274], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 546]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 532], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[532], Jmv, e, &onlinedata_StageParameter[12274],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1558], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1573], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1591], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1593], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1595], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1597], _mm_add_pd(r, r1));
  m_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  m_getHamiltonianDerivatives(&X[518], &lambda[532], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[11951], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 532]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 518], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[518], Jmv, e, &onlinedata_StageParameter[11951],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1517], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1532], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1550], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1552], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1554], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1556], _mm_add_pd(r, r1));
  n_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  n_getHamiltonianDerivatives(&X[504], &lambda[518], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[11628], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 518]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 504], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[504], Jmv, e, &onlinedata_StageParameter[11628],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1476], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1491], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1509], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1511], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1513], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1515], _mm_add_pd(r, r1));
  o_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  o_getHamiltonianDerivatives(&X[490], &lambda[504], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[11305], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 504]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 490], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[490], Jmv, e, &onlinedata_StageParameter[11305],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1435], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1450], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1468], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1470], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1472], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1474], _mm_add_pd(r, r1));
  p_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  p_getHamiltonianDerivatives(&X[476], &lambda[490], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[10982], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 490]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 476], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[476], Jmv, e, &onlinedata_StageParameter[10982],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1394], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1409], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1427], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1429], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1431], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1433], _mm_add_pd(r, r1));
  q_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  q_getHamiltonianDerivatives(&X[462], &lambda[476], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[10659], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 476]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 462], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[462], Jmv, e, &onlinedata_StageParameter[10659],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1353], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1368], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1386], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1388], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1390], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1392], _mm_add_pd(r, r1));
  r_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  r_getHamiltonianDerivatives(&X[448], &lambda[462], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[10336], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 462]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 448], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[448], Jmv, e, &onlinedata_StageParameter[10336],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1312], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1327], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1345], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1347], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1349], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1351], _mm_add_pd(r, r1));
  s_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  s_getHamiltonianDerivatives(&X[434], &lambda[448], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[10013], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 448]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 434], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[434], Jmv, e, &onlinedata_StageParameter[10013],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1271], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1286], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1304], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1306], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1308], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1310], _mm_add_pd(r, r1));
  t_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  t_getHamiltonianDerivatives(&X[420], &lambda[434], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[9690], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 434]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 420], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[420], Jmv, e, &onlinedata_StageParameter[9690],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1230], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1245], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1263], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1265], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1267], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1269], _mm_add_pd(r, r1));
  u_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  u_getHamiltonianDerivatives(&X[406], &lambda[420], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[9367], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 420]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 406], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[406], Jmv, e, &onlinedata_StageParameter[9367],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1189], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1204], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1222], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1224], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1226], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1228], _mm_add_pd(r, r1));
  v_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  v_getHamiltonianDerivatives(&X[392], &lambda[406], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[9044], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 406]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 392], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[392], Jmv, e, &onlinedata_StageParameter[9044],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1148], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1163], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1181], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1183], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1185], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1187], _mm_add_pd(r, r1));
  w_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  w_getHamiltonianDerivatives(&X[378], &lambda[392], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[8721], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 392]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 378], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[378], Jmv, e, &onlinedata_StageParameter[8721],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1107], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1122], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1140], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1142], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1144], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1146], _mm_add_pd(r, r1));
  x_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  x_getHamiltonianDerivatives(&X[364], &lambda[378], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[8398], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 378]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 364], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[364], Jmv, e, &onlinedata_StageParameter[8398],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1066], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1081], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1099], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1101], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1103], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1105], _mm_add_pd(r, r1));
  y_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  y_getHamiltonianDerivatives(&X[350], &lambda[364], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[8075], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 364]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 350], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[350], Jmv, e, &onlinedata_StageParameter[8075],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[1025], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[1040], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1058], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1060], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1062], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1064], _mm_add_pd(r, r1));
  ab_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ab_getHamiltonianDerivatives(&X[336], &lambda[350], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[7752], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 350]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 336], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[336], Jmv, e, &onlinedata_StageParameter[7752],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[984], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[999], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[1017], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[1019], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[1021], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[1023], _mm_add_pd(r, r1));
  bb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  bb_getHamiltonianDerivatives(&X[322], &lambda[336], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[7429], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 336]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 322], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[322], Jmv, e, &onlinedata_StageParameter[7429],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[943], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[958], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[976], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[978], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[980], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[982], _mm_add_pd(r, r1));
  cb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  cb_getHamiltonianDerivatives(&X[308], &lambda[322], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[7106], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 322]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 308], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[308], Jmv, e, &onlinedata_StageParameter[7106],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[902], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[917], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[935], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[937], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[939], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[941], _mm_add_pd(r, r1));
  db_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  db_getHamiltonianDerivatives(&X[294], &lambda[308], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[6783], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 308]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 294], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[294], Jmv, e, &onlinedata_StageParameter[6783],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[861], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[876], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[894], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[896], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[898], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[900], _mm_add_pd(r, r1));
  eb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  eb_getHamiltonianDerivatives(&X[280], &lambda[294], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[6460], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 294]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 280], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[280], Jmv, e, &onlinedata_StageParameter[6460],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[820], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[835], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[853], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[855], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[857], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[859], _mm_add_pd(r, r1));
  fb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  fb_getHamiltonianDerivatives(&X[266], &lambda[280], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[6137], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 280]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 266], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[266], Jmv, e, &onlinedata_StageParameter[6137],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[779], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[794], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[812], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[814], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[816], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[818], _mm_add_pd(r, r1));
  gb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  gb_getHamiltonianDerivatives(&X[252], &lambda[266], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[5814], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 266]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 252], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[252], Jmv, e, &onlinedata_StageParameter[5814],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[738], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[753], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[771], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[773], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[775], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[777], _mm_add_pd(r, r1));
  hb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  hb_getHamiltonianDerivatives(&X[238], &lambda[252], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[5491], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 252]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 238], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[238], Jmv, e, &onlinedata_StageParameter[5491],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[697], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[712], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[730], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[732], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[734], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[736], _mm_add_pd(r, r1));
  ib_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ib_getHamiltonianDerivatives(&X[224], &lambda[238], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[5168], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 238]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 224], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[224], Jmv, e, &onlinedata_StageParameter[5168],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[656], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[671], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[689], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[691], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[693], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[695], _mm_add_pd(r, r1));
  jb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  jb_getHamiltonianDerivatives(&X[210], &lambda[224], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[4845], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 224]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 210], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[210], Jmv, e, &onlinedata_StageParameter[4845],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[615], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[630], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[648], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[650], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[652], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[654], _mm_add_pd(r, r1));
  kb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  kb_getHamiltonianDerivatives(&X[196], &lambda[210], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[4522], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 210]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 196], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[196], Jmv, e, &onlinedata_StageParameter[4522],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[574], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[589], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[607], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[609], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[611], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[613], _mm_add_pd(r, r1));
  lb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  lb_getHamiltonianDerivatives(&X[182], &lambda[196], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[4199], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 196]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 182], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[182], Jmv, e, &onlinedata_StageParameter[4199],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[533], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[548], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[566], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[568], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[570], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[572], _mm_add_pd(r, r1));
  mb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  mb_getHamiltonianDerivatives(&X[168], &lambda[182], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[3876], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 182]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 168], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[168], Jmv, e, &onlinedata_StageParameter[3876],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[492], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[507], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[525], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[527], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[529], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[531], _mm_add_pd(r, r1));
  nb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  nb_getHamiltonianDerivatives(&X[154], &lambda[168], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[3553], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 168]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 154], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[154], Jmv, e, &onlinedata_StageParameter[3553],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[451], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[466], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[484], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[486], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[488], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[490], _mm_add_pd(r, r1));
  ob_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ob_getHamiltonianDerivatives(&X[140], &lambda[154], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[3230], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 154]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 140], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[140], Jmv, e, &onlinedata_StageParameter[3230],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[410], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[425], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[443], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[445], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[447], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[449], _mm_add_pd(r, r1));
  pb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  pb_getHamiltonianDerivatives(&X[126], &lambda[140], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[2907], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 140]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 126], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[126], Jmv, e, &onlinedata_StageParameter[2907],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[369], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[384], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[402], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[404], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[406], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[408], _mm_add_pd(r, r1));
  qb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  qb_getHamiltonianDerivatives(&X[112], &lambda[126], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[2584], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 126]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 112], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[112], Jmv, e, &onlinedata_StageParameter[2584],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[328], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[343], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[361], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[363], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[365], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[367], _mm_add_pd(r, r1));
  rb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  rb_getHamiltonianDerivatives(&X[98], &lambda[112], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[2261], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 112]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 98], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[98], Jmv, e, &onlinedata_StageParameter[2261],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[287], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[302], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[320], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[322], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[324], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[326], _mm_add_pd(r, r1));
  sb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  sb_getHamiltonianDerivatives(&X[84], &lambda[98], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[1938], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 98]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 84], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[84], Jmv, e, &onlinedata_StageParameter[1938],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[246], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[261], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[279], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[281], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[283], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[285], _mm_add_pd(r, r1));
  tb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  tb_getHamiltonianDerivatives(&X[70], &lambda[84], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[1615], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 84]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 70], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[70], Jmv, e, &onlinedata_StageParameter[1615],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[205], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[220], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[238], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[240], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[242], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[244], _mm_add_pd(r, r1));
  ub_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  ub_getHamiltonianDerivatives(&X[56], &lambda[70], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[1292], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 70]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 56], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[56], Jmv, e, &onlinedata_StageParameter[1292],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[164], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[179], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[197], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[199], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[201], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[203], _mm_add_pd(r, r1));
  vb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  vb_getHamiltonianDerivatives(&X[42], &lambda[56], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[969], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 56]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 42], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[42], Jmv, e, &onlinedata_StageParameter[969],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[123], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[138], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[156], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[158], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[160], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[162], _mm_add_pd(r, r1));
  wb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  wb_getHamiltonianDerivatives(&X[28], &lambda[42], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[646], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 42]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 28], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[28], Jmv, e, &onlinedata_StageParameter[646],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[82], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[97], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[115], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[117], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[119], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[121], _mm_add_pd(r, r1));
  xb_unpackDecisionVariables(U, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  xb_getHamiltonianDerivatives(&X[14], &lambda[28], Jmv, e, d, a__5, mudummy,
    dmv, &onlinedata_StageParameter[323], Lx, Hmv, Cbnds);
  for (i = 0; i <= 12; i += 2) {
    r = _mm_loadu_pd(&Lx[i]);
    r1 = _mm_loadu_pd(&lambda[i + 28]);
    r = _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(0.001)));
    _mm_storeu_pd(&Lx[i], r);
    _mm_storeu_pd(&lambda[i + 14], r);
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  nlmpc_config__ineqConFcn(&X[14], Jmv, e, &onlinedata_StageParameter[323],
    Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[41], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[56], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[74], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[76], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[78], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[80], _mm_add_pd(r, r1));
  yb_unpackDecisionVariables(U, lastMV, Jmv, Jdmv, dmv, e, d, a__5, mudummy);
  yb_getHamiltonianDerivatives(&X[0], &lambda[14], Jmv, e, d, a__5, mudummy, dmv,
    &onlinedata_StageParameter[0], Lx, Hmv, Cbnds);
  b_nlmpc_config__ineqConFcn(&X[0], &onlinedata_StageParameter[0], Cineq);
  for (i = 0; i <= 16; i += 2) {
    r = _mm_loadu_pd(&Cineq[i]);
    _mm_storeu_pd(&b_Cineq[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }

  computeFischerBurmeister(a__5, b_Cineq, Cineq);
  r = _mm_loadu_pd(&d[0]);
  _mm_storeu_pd(&mudummy[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[2]);
  _mm_storeu_pd(&mudummy[2], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[4]);
  _mm_storeu_pd(&mudummy[4], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&d[6]);
  _mm_storeu_pd(&mudummy[6], _mm_mul_pd(r, r));
  memcpy(&varargout_1[0], &Hmv[0], 15U * sizeof(real_T));
  memcpy(&varargout_1[15], &Cineq[0], 18U * sizeof(real_T));
  r = _mm_loadu_pd(&Cbnds[0]);
  r1 = _mm_loadu_pd(&mudummy[0]);
  _mm_storeu_pd(&varargout_1[33], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[2]);
  r1 = _mm_loadu_pd(&mudummy[2]);
  _mm_storeu_pd(&varargout_1[35], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[4]);
  r1 = _mm_loadu_pd(&mudummy[4]);
  _mm_storeu_pd(&varargout_1[37], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&Cbnds[6]);
  r1 = _mm_loadu_pd(&mudummy[6]);
  _mm_storeu_pd(&varargout_1[39], _mm_add_pd(r, r1));
}

void nlmpcmoveCodeGeneration(c_nlmpcmoveCodeGenerationStackD *SD, const real_T
  x[14], const real_T lastMV[4], struct3_T *onlinedata, real_T mv[4], struct4_T *
  info)
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  emxArray_real_T *resvec;
  real_T varargout_1[2071];
  real_T dxdt[14];
  real_T ctInter;
  real_T exitflag;
  real_T fknorm;
  real_T iters;
  int32_T b_i;
  int32_T i;
  boolean_T exitg1;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  ctInter = 0.0;
  for (i = 0; i < 2071; i++) {
    fknorm = onlinedata->InitialGuess[i];
    SD->f1.z[i] = fknorm;
    SD->f1.zdt[i] = onlinedata->InitialGuess[i + 2071];
    SD->f1.U_[i] = fknorm;
  }

  exitflag = 0.0;
  i = 0;
  emxInit_real_T(&resvec, 1);
  exitg1 = false;
  while ((!exitg1) && (i < 5)) {
    exitflag = fdgmres(SD, lastMV, onlinedata->StateFcnParameter,
                       onlinedata->StageParameter, SD->f1.z, SD->f1.zdt, x,
                       &onlinedata->InitialGuess[4142], resvec, &fknorm, &iters);
    ctInter += iters;
    for (b_i = 0; b_i <= 2068; b_i += 2) {
      __m128d r;
      __m128d r1;
      r = _mm_loadu_pd(&SD->f1.zdt[b_i]);
      r1 = _mm_loadu_pd(&SD->f1.z[b_i]);
      _mm_storeu_pd(&SD->f1.z[b_i], _mm_add_pd(r1, _mm_div_pd(r, _mm_set1_pd
        (1000.0))));
    }

    real_T b_expl_temp[2071];
    real_T expl_temp[2071];
    real_T c_expl_temp[714];
    SD->f1.z[2070] += SD->f1.zdt[2070] / 1000.0;
    d_nlmpcmoveCodeGeneration_anonF(lastMV, onlinedata->StateFcnParameter,
      onlinedata->StageParameter, SD->f1.z, x, varargout_1, info->MVopt,
      info->Xopt, info->Topt, info->Slack, expl_temp, b_expl_temp, &iters,
      c_expl_temp);
    n_t = (ptrdiff_t)2071;
    incx_t = (ptrdiff_t)1;
    fknorm = dnrm2(&n_t, &varargout_1[0], &incx_t);
    if ((iters < 1.0E-6) && (exitflag == 1.0)) {
      exitg1 = true;
    } else if (fknorm < 1.0E-7) {
      exitflag = 1.0;
      exitg1 = true;
    } else if (muDoubleScalarIsNaN(fknorm) || muDoubleScalarIsInf(fknorm)) {
      exitflag = -1.0;
      exitg1 = true;
    } else {
      memcpy(&SD->f1.U_[0], &SD->f1.z[0], 2071U * sizeof(real_T));
      i++;
    }
  }

  emxFree_real_T(&resvec);
  if (exitflag == -1.0) {
    memcpy(&SD->f1.z[0], &SD->f1.U_[0], 2071U * sizeof(real_T));
  }

  fknorm = e_nlmpcmoveCodeGeneration_anonF(lastMV, onlinedata->StateFcnParameter,
    onlinedata->StageParameter, SD->f1.z, x, varargout_1, info->MVopt,
    info->Xopt, info->Topt, info->Slack);
  info->Cost = fknorm;
  info->ExitFlag = exitflag;
  info->Iterations = ctInter;
  mv[0] = SD->f1.z[0];
  mv[1] = SD->f1.z[1];
  mv[2] = SD->f1.z[2];
  mv[3] = SD->f1.z[3];
  nlmpc_config__stateFcn(x, &SD->f1.z[0], onlinedata->StateFcnParameter, dxdt);
  for (i = 0; i < 2071; i++) {
    onlinedata->InitialGuess[i] = SD->f1.z[i];
    onlinedata->InitialGuess[i + 2071] = SD->f1.zdt[i];
  }

  memcpy(&onlinedata->InitialGuess[4142], &dxdt[0], 14U * sizeof(real_T));
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (nlmpcmoveCodeGeneration.c) */
