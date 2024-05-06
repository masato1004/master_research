/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeInfo.c
 *
 * Code generation for function 'computeInfo'
 *
 */

/* Include files */
#include "computeInfo.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "unpackDecisionVariables.h"
#include <emmintrin.h>

/* Function Definitions */
real_T computeInfo(const real_T X[714], const real_T Z[2071],
                   const real_T signals_StageParameter[16473],
                   const real_T lastMV[4], real_T info_MVopt[204],
                   real_T info_Xopt[714], real_T info_Topt[51],
                   real_T info_Slack[153], real_T *info_Cost)
{
  real_T er[14];
  real_T dmv[4];
  real_T mv[4];
  real_T u[4];
  real_T e[3];
  real_T L;
  real_T b_e;
  real_T b_er;
  real_T b_u;
  real_T c_e;
  real_T c_er;
  real_T c_u;
  real_T d_e;
  real_T d_er;
  real_T d_u;
  real_T e_e;
  real_T e_er;
  real_T e_u;
  real_T f_e;
  real_T f_er;
  real_T f_u;
  real_T g_e;
  real_T g_er;
  real_T g_u;
  real_T h_e;
  real_T h_er;
  real_T h_u;
  real_T i_e;
  real_T i_er;
  real_T i_u;
  real_T info_Iterations;
  real_T j_e;
  real_T j_er;
  real_T j_u;
  real_T k_e;
  real_T k_er;
  real_T k_u;
  real_T l_e;
  real_T l_er;
  real_T l_u;
  real_T m_e;
  real_T m_er;
  real_T m_u;
  real_T n_e;
  real_T n_er;
  real_T n_u;
  real_T o_e;
  real_T o_er;
  real_T o_u;
  real_T p_e;
  real_T p_er;
  real_T p_u;
  real_T q_e;
  real_T q_er;
  int32_T i;
  int32_T i1;
  for (i = 0; i < 14; i++) {
    for (i1 = 0; i1 < 51; i1++) {
      info_Xopt[i1 + 51 * i] = X[i + 14 * i1];
    }
  }
  for (i = 0; i < 51; i++) {
    info_Topt[i] = 0.001 * (real_T)i;
  }
  info_Iterations = 0.0;
  ac_unpackDecisionVariables(Z, lastMV, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i]),
                             _mm_loadu_pd(&signals_StageParameter[i + 309])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  info_MVopt[0] = mv[0];
  info_MVopt[51] = mv[1];
  info_MVopt[102] = mv[2];
  info_MVopt[153] = mv[3];
  info_Slack[0] = e[0];
  info_Slack[1] = e[1];
  info_Slack[2] = e[2];
  bc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 14]),
                             _mm_loadu_pd(&signals_StageParameter[i + 632])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  c_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    c_er += b_e * er[i];
  }
  c_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    c_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  d_e = 0.0;
  for (i = 0; i < 3; i++) {
    d_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[1] = mv[0];
  info_MVopt[52] = mv[1];
  info_MVopt[103] = mv[2];
  info_MVopt[154] = mv[3];
  info_Slack[3] = e[0];
  info_Slack[4] = e[1];
  info_Slack[5] = e[2];
  cc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 28]),
                             _mm_loadu_pd(&signals_StageParameter[i + 955])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  d_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    d_er += b_e * er[i];
  }
  d_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    d_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  e_e = 0.0;
  for (i = 0; i < 3; i++) {
    e_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[2] = mv[0];
  info_MVopt[53] = mv[1];
  info_MVopt[104] = mv[2];
  info_MVopt[155] = mv[3];
  info_Slack[6] = e[0];
  info_Slack[7] = e[1];
  info_Slack[8] = e[2];
  dc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 42]),
                             _mm_loadu_pd(&signals_StageParameter[i + 1278])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  e_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    e_er += b_e * er[i];
  }
  e_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    e_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  f_e = 0.0;
  for (i = 0; i < 3; i++) {
    f_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[3] = mv[0];
  info_MVopt[54] = mv[1];
  info_MVopt[105] = mv[2];
  info_MVopt[156] = mv[3];
  info_Slack[9] = e[0];
  info_Slack[10] = e[1];
  info_Slack[11] = e[2];
  ec_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 56]),
                             _mm_loadu_pd(&signals_StageParameter[i + 1601])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  f_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    f_er += b_e * er[i];
  }
  f_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    f_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  g_e = 0.0;
  for (i = 0; i < 3; i++) {
    g_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[4] = mv[0];
  info_MVopt[55] = mv[1];
  info_MVopt[106] = mv[2];
  info_MVopt[157] = mv[3];
  info_Slack[12] = e[0];
  info_Slack[13] = e[1];
  info_Slack[14] = e[2];
  fc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 70]),
                             _mm_loadu_pd(&signals_StageParameter[i + 1924])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  g_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    g_er += b_e * er[i];
  }
  g_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    g_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  h_e = 0.0;
  for (i = 0; i < 3; i++) {
    h_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[5] = mv[0];
  info_MVopt[56] = mv[1];
  info_MVopt[107] = mv[2];
  info_MVopt[158] = mv[3];
  info_Slack[15] = e[0];
  info_Slack[16] = e[1];
  info_Slack[17] = e[2];
  gc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 84]),
                             _mm_loadu_pd(&signals_StageParameter[i + 2247])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  h_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    h_er += b_e * er[i];
  }
  h_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    h_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  i_e = 0.0;
  for (i = 0; i < 3; i++) {
    i_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[6] = mv[0];
  info_MVopt[57] = mv[1];
  info_MVopt[108] = mv[2];
  info_MVopt[159] = mv[3];
  info_Slack[18] = e[0];
  info_Slack[19] = e[1];
  info_Slack[20] = e[2];
  hc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 98]),
                             _mm_loadu_pd(&signals_StageParameter[i + 2570])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  i_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    i_er += b_e * er[i];
  }
  i_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    i_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  j_e = 0.0;
  for (i = 0; i < 3; i++) {
    j_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[7] = mv[0];
  info_MVopt[58] = mv[1];
  info_MVopt[109] = mv[2];
  info_MVopt[160] = mv[3];
  info_Slack[21] = e[0];
  info_Slack[22] = e[1];
  info_Slack[23] = e[2];
  ic_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 112]),
                             _mm_loadu_pd(&signals_StageParameter[i + 2893])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  j_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    j_er += b_e * er[i];
  }
  j_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    j_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  k_e = 0.0;
  for (i = 0; i < 3; i++) {
    k_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[8] = mv[0];
  info_MVopt[59] = mv[1];
  info_MVopt[110] = mv[2];
  info_MVopt[161] = mv[3];
  info_Slack[24] = e[0];
  info_Slack[25] = e[1];
  info_Slack[26] = e[2];
  jc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 126]),
                             _mm_loadu_pd(&signals_StageParameter[i + 3216])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  k_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    k_er += b_e * er[i];
  }
  k_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    k_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  l_e = 0.0;
  for (i = 0; i < 3; i++) {
    l_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[9] = mv[0];
  info_MVopt[60] = mv[1];
  info_MVopt[111] = mv[2];
  info_MVopt[162] = mv[3];
  info_Slack[27] = e[0];
  info_Slack[28] = e[1];
  info_Slack[29] = e[2];
  kc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 140]),
                             _mm_loadu_pd(&signals_StageParameter[i + 3539])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  l_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    l_er += b_e * er[i];
  }
  l_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    l_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  m_e = 0.0;
  for (i = 0; i < 3; i++) {
    m_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[10] = mv[0];
  info_MVopt[61] = mv[1];
  info_MVopt[112] = mv[2];
  info_MVopt[163] = mv[3];
  info_Slack[30] = e[0];
  info_Slack[31] = e[1];
  info_Slack[32] = e[2];
  lc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 154]),
                             _mm_loadu_pd(&signals_StageParameter[i + 3862])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  m_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    m_er += b_e * er[i];
  }
  m_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    m_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  n_e = 0.0;
  for (i = 0; i < 3; i++) {
    n_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[11] = mv[0];
  info_MVopt[62] = mv[1];
  info_MVopt[113] = mv[2];
  info_MVopt[164] = mv[3];
  info_Slack[33] = e[0];
  info_Slack[34] = e[1];
  info_Slack[35] = e[2];
  mc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 168]),
                             _mm_loadu_pd(&signals_StageParameter[i + 4185])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  n_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    n_er += b_e * er[i];
  }
  n_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    n_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  o_e = 0.0;
  for (i = 0; i < 3; i++) {
    o_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[12] = mv[0];
  info_MVopt[63] = mv[1];
  info_MVopt[114] = mv[2];
  info_MVopt[165] = mv[3];
  info_Slack[36] = e[0];
  info_Slack[37] = e[1];
  info_Slack[38] = e[2];
  nc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 182]),
                             _mm_loadu_pd(&signals_StageParameter[i + 4508])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  o_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    o_er += b_e * er[i];
  }
  o_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    o_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  p_e = 0.0;
  for (i = 0; i < 3; i++) {
    p_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[13] = mv[0];
  info_MVopt[64] = mv[1];
  info_MVopt[115] = mv[2];
  info_MVopt[166] = mv[3];
  info_Slack[39] = e[0];
  info_Slack[40] = e[1];
  info_Slack[41] = e[2];
  oc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 196]),
                             _mm_loadu_pd(&signals_StageParameter[i + 4831])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  p_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    p_er += b_e * er[i];
  }
  p_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    p_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  q_e = 0.0;
  for (i = 0; i < 3; i++) {
    q_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  info_MVopt[14] = mv[0];
  info_MVopt[65] = mv[1];
  info_MVopt[116] = mv[2];
  info_MVopt[167] = mv[3];
  info_Slack[42] = e[0];
  info_Slack[43] = e[1];
  info_Slack[44] = e[2];
  pc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 210]),
                             _mm_loadu_pd(&signals_StageParameter[i + 5154])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  q_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    q_er += b_e * er[i];
  }
  L = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    L += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
           u[2] * 0.5 * dv4[i1 + 2]) +
          u[3] * 0.5 * dv4[i1 + 3]) *
         u[i];
  }
  b_e = 0.0;
  for (i = 0; i < 3; i++) {
    b_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L = ((((((((((((((((b_er + b_u) + c_e) + ((c_er + c_u) + d_e)) +
                   ((d_er + d_u) + e_e)) +
                  ((e_er + e_u) + f_e)) +
                 ((f_er + f_u) + g_e)) +
                ((g_er + g_u) + h_e)) +
               ((h_er + h_u) + i_e)) +
              ((i_er + i_u) + j_e)) +
             ((j_er + j_u) + k_e)) +
            ((k_er + k_u) + l_e)) +
           ((l_er + l_u) + m_e)) +
          ((m_er + m_u) + n_e)) +
         ((n_er + n_u) + o_e)) +
        ((o_er + o_u) + p_e)) +
       ((p_er + p_u) + q_e)) +
      ((q_er + L) + b_e);
  info_MVopt[15] = mv[0];
  info_MVopt[66] = mv[1];
  info_MVopt[117] = mv[2];
  info_MVopt[168] = mv[3];
  info_Slack[45] = e[0];
  info_Slack[46] = e[1];
  info_Slack[47] = e[2];
  qc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 224]),
                             _mm_loadu_pd(&signals_StageParameter[i + 5477])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[16] = mv[0];
  info_MVopt[67] = mv[1];
  info_MVopt[118] = mv[2];
  info_MVopt[169] = mv[3];
  info_Slack[48] = e[0];
  info_Slack[49] = e[1];
  info_Slack[50] = e[2];
  rc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 238]),
                             _mm_loadu_pd(&signals_StageParameter[i + 5800])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[17] = mv[0];
  info_MVopt[68] = mv[1];
  info_MVopt[119] = mv[2];
  info_MVopt[170] = mv[3];
  info_Slack[51] = e[0];
  info_Slack[52] = e[1];
  info_Slack[53] = e[2];
  sc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 252]),
                             _mm_loadu_pd(&signals_StageParameter[i + 6123])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[18] = mv[0];
  info_MVopt[69] = mv[1];
  info_MVopt[120] = mv[2];
  info_MVopt[171] = mv[3];
  info_Slack[54] = e[0];
  info_Slack[55] = e[1];
  info_Slack[56] = e[2];
  tc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 266]),
                             _mm_loadu_pd(&signals_StageParameter[i + 6446])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[19] = mv[0];
  info_MVopt[70] = mv[1];
  info_MVopt[121] = mv[2];
  info_MVopt[172] = mv[3];
  info_Slack[57] = e[0];
  info_Slack[58] = e[1];
  info_Slack[59] = e[2];
  uc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 280]),
                             _mm_loadu_pd(&signals_StageParameter[i + 6769])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[20] = mv[0];
  info_MVopt[71] = mv[1];
  info_MVopt[122] = mv[2];
  info_MVopt[173] = mv[3];
  info_Slack[60] = e[0];
  info_Slack[61] = e[1];
  info_Slack[62] = e[2];
  vc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 294]),
                             _mm_loadu_pd(&signals_StageParameter[i + 7092])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[21] = mv[0];
  info_MVopt[72] = mv[1];
  info_MVopt[123] = mv[2];
  info_MVopt[174] = mv[3];
  info_Slack[63] = e[0];
  info_Slack[64] = e[1];
  info_Slack[65] = e[2];
  wc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 308]),
                             _mm_loadu_pd(&signals_StageParameter[i + 7415])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[22] = mv[0];
  info_MVopt[73] = mv[1];
  info_MVopt[124] = mv[2];
  info_MVopt[175] = mv[3];
  info_Slack[66] = e[0];
  info_Slack[67] = e[1];
  info_Slack[68] = e[2];
  xc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 322]),
                             _mm_loadu_pd(&signals_StageParameter[i + 7738])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[23] = mv[0];
  info_MVopt[74] = mv[1];
  info_MVopt[125] = mv[2];
  info_MVopt[176] = mv[3];
  info_Slack[69] = e[0];
  info_Slack[70] = e[1];
  info_Slack[71] = e[2];
  yc_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 336]),
                             _mm_loadu_pd(&signals_StageParameter[i + 8061])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[24] = mv[0];
  info_MVopt[75] = mv[1];
  info_MVopt[126] = mv[2];
  info_MVopt[177] = mv[3];
  info_Slack[72] = e[0];
  info_Slack[73] = e[1];
  info_Slack[74] = e[2];
  ad_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 350]),
                             _mm_loadu_pd(&signals_StageParameter[i + 8384])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[25] = mv[0];
  info_MVopt[76] = mv[1];
  info_MVopt[127] = mv[2];
  info_MVopt[178] = mv[3];
  info_Slack[75] = e[0];
  info_Slack[76] = e[1];
  info_Slack[77] = e[2];
  bd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 364]),
                             _mm_loadu_pd(&signals_StageParameter[i + 8707])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[26] = mv[0];
  info_MVopt[77] = mv[1];
  info_MVopt[128] = mv[2];
  info_MVopt[179] = mv[3];
  info_Slack[78] = e[0];
  info_Slack[79] = e[1];
  info_Slack[80] = e[2];
  cd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 378]),
                             _mm_loadu_pd(&signals_StageParameter[i + 9030])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[27] = mv[0];
  info_MVopt[78] = mv[1];
  info_MVopt[129] = mv[2];
  info_MVopt[180] = mv[3];
  info_Slack[81] = e[0];
  info_Slack[82] = e[1];
  info_Slack[83] = e[2];
  dd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 392]),
                             _mm_loadu_pd(&signals_StageParameter[i + 9353])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[28] = mv[0];
  info_MVopt[79] = mv[1];
  info_MVopt[130] = mv[2];
  info_MVopt[181] = mv[3];
  info_Slack[84] = e[0];
  info_Slack[85] = e[1];
  info_Slack[86] = e[2];
  ed_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 406]),
                             _mm_loadu_pd(&signals_StageParameter[i + 9676])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[29] = mv[0];
  info_MVopt[80] = mv[1];
  info_MVopt[131] = mv[2];
  info_MVopt[182] = mv[3];
  info_Slack[87] = e[0];
  info_Slack[88] = e[1];
  info_Slack[89] = e[2];
  fd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 420]),
                             _mm_loadu_pd(&signals_StageParameter[i + 9999])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[30] = mv[0];
  info_MVopt[81] = mv[1];
  info_MVopt[132] = mv[2];
  info_MVopt[183] = mv[3];
  info_Slack[90] = e[0];
  info_Slack[91] = e[1];
  info_Slack[92] = e[2];
  gd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 434]),
                             _mm_loadu_pd(&signals_StageParameter[i + 10322])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[31] = mv[0];
  info_MVopt[82] = mv[1];
  info_MVopt[133] = mv[2];
  info_MVopt[184] = mv[3];
  info_Slack[93] = e[0];
  info_Slack[94] = e[1];
  info_Slack[95] = e[2];
  hd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 448]),
                             _mm_loadu_pd(&signals_StageParameter[i + 10645])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[32] = mv[0];
  info_MVopt[83] = mv[1];
  info_MVopt[134] = mv[2];
  info_MVopt[185] = mv[3];
  info_Slack[96] = e[0];
  info_Slack[97] = e[1];
  info_Slack[98] = e[2];
  id_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 462]),
                             _mm_loadu_pd(&signals_StageParameter[i + 10968])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[33] = mv[0];
  info_MVopt[84] = mv[1];
  info_MVopt[135] = mv[2];
  info_MVopt[186] = mv[3];
  info_Slack[99] = e[0];
  info_Slack[100] = e[1];
  info_Slack[101] = e[2];
  jd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 476]),
                             _mm_loadu_pd(&signals_StageParameter[i + 11291])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[34] = mv[0];
  info_MVopt[85] = mv[1];
  info_MVopt[136] = mv[2];
  info_MVopt[187] = mv[3];
  info_Slack[102] = e[0];
  info_Slack[103] = e[1];
  info_Slack[104] = e[2];
  kd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 490]),
                             _mm_loadu_pd(&signals_StageParameter[i + 11614])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[35] = mv[0];
  info_MVopt[86] = mv[1];
  info_MVopt[137] = mv[2];
  info_MVopt[188] = mv[3];
  info_Slack[105] = e[0];
  info_Slack[106] = e[1];
  info_Slack[107] = e[2];
  ld_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 504]),
                             _mm_loadu_pd(&signals_StageParameter[i + 11937])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[36] = mv[0];
  info_MVopt[87] = mv[1];
  info_MVopt[138] = mv[2];
  info_MVopt[189] = mv[3];
  info_Slack[108] = e[0];
  info_Slack[109] = e[1];
  info_Slack[110] = e[2];
  md_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 518]),
                             _mm_loadu_pd(&signals_StageParameter[i + 12260])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[37] = mv[0];
  info_MVopt[88] = mv[1];
  info_MVopt[139] = mv[2];
  info_MVopt[190] = mv[3];
  info_Slack[111] = e[0];
  info_Slack[112] = e[1];
  info_Slack[113] = e[2];
  nd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 532]),
                             _mm_loadu_pd(&signals_StageParameter[i + 12583])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[38] = mv[0];
  info_MVopt[89] = mv[1];
  info_MVopt[140] = mv[2];
  info_MVopt[191] = mv[3];
  info_Slack[114] = e[0];
  info_Slack[115] = e[1];
  info_Slack[116] = e[2];
  od_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 546]),
                             _mm_loadu_pd(&signals_StageParameter[i + 12906])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[39] = mv[0];
  info_MVopt[90] = mv[1];
  info_MVopt[141] = mv[2];
  info_MVopt[192] = mv[3];
  info_Slack[117] = e[0];
  info_Slack[118] = e[1];
  info_Slack[119] = e[2];
  pd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 560]),
                             _mm_loadu_pd(&signals_StageParameter[i + 13229])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[40] = mv[0];
  info_MVopt[91] = mv[1];
  info_MVopt[142] = mv[2];
  info_MVopt[193] = mv[3];
  info_Slack[120] = e[0];
  info_Slack[121] = e[1];
  info_Slack[122] = e[2];
  qd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 574]),
                             _mm_loadu_pd(&signals_StageParameter[i + 13552])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[41] = mv[0];
  info_MVopt[92] = mv[1];
  info_MVopt[143] = mv[2];
  info_MVopt[194] = mv[3];
  info_Slack[123] = e[0];
  info_Slack[124] = e[1];
  info_Slack[125] = e[2];
  rd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 588]),
                             _mm_loadu_pd(&signals_StageParameter[i + 13875])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[42] = mv[0];
  info_MVopt[93] = mv[1];
  info_MVopt[144] = mv[2];
  info_MVopt[195] = mv[3];
  info_Slack[126] = e[0];
  info_Slack[127] = e[1];
  info_Slack[128] = e[2];
  sd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 602]),
                             _mm_loadu_pd(&signals_StageParameter[i + 14198])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[43] = mv[0];
  info_MVopt[94] = mv[1];
  info_MVopt[145] = mv[2];
  info_MVopt[196] = mv[3];
  info_Slack[129] = e[0];
  info_Slack[130] = e[1];
  info_Slack[131] = e[2];
  td_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 616]),
                             _mm_loadu_pd(&signals_StageParameter[i + 14521])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[44] = mv[0];
  info_MVopt[95] = mv[1];
  info_MVopt[146] = mv[2];
  info_MVopt[197] = mv[3];
  info_Slack[132] = e[0];
  info_Slack[133] = e[1];
  info_Slack[134] = e[2];
  ud_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 630]),
                             _mm_loadu_pd(&signals_StageParameter[i + 14844])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[45] = mv[0];
  info_MVopt[96] = mv[1];
  info_MVopt[147] = mv[2];
  info_MVopt[198] = mv[3];
  info_Slack[135] = e[0];
  info_Slack[136] = e[1];
  info_Slack[137] = e[2];
  vd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 644]),
                             _mm_loadu_pd(&signals_StageParameter[i + 15167])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[46] = mv[0];
  info_MVopt[97] = mv[1];
  info_MVopt[148] = mv[2];
  info_MVopt[199] = mv[3];
  info_Slack[138] = e[0];
  info_Slack[139] = e[1];
  info_Slack[140] = e[2];
  wd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 658]),
                             _mm_loadu_pd(&signals_StageParameter[i + 15490])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[47] = mv[0];
  info_MVopt[98] = mv[1];
  info_MVopt[149] = mv[2];
  info_MVopt[200] = mv[3];
  info_Slack[141] = e[0];
  info_Slack[142] = e[1];
  info_Slack[143] = e[2];
  xd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 672]),
                             _mm_loadu_pd(&signals_StageParameter[i + 15813])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[48] = mv[0];
  info_MVopt[99] = mv[1];
  info_MVopt[150] = mv[2];
  info_MVopt[201] = mv[3];
  info_Slack[144] = e[0];
  info_Slack[145] = e[1];
  info_Slack[146] = e[2];
  yd_unpackDecisionVariables(Z, u, mv, dmv, e);
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 686]),
                             _mm_loadu_pd(&signals_StageParameter[i + 16136])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  b_u = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_u += (((u[0] * 0.5 * dv4[i1] + u[1] * 0.5 * dv4[i1 + 1]) +
             u[2] * 0.5 * dv4[i1 + 2]) +
            u[3] * 0.5 * dv4[i1 + 3]) *
           u[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e +=
        ((e[0] * 0.5 * (real_T)iv[3 * i] + e[1] * 0.5 * (real_T)iv[3 * i + 1]) +
         e[2] * 0.5 * (real_T)iv[3 * i + 2]) *
        e[i];
  }
  L += (b_er + b_u) + c_e;
  info_MVopt[49] = mv[0];
  info_MVopt[100] = mv[1];
  info_MVopt[151] = mv[2];
  info_MVopt[202] = mv[3];
  info_Slack[147] = e[0];
  info_Slack[148] = e[1];
  info_Slack[149] = e[2];
  for (i = 0; i <= 12; i += 2) {
    _mm_storeu_pd(&er[i],
                  _mm_sub_pd(_mm_loadu_pd(&X[i + 700]),
                             _mm_loadu_pd(&signals_StageParameter[i + 16459])));
  }
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  if stage == 1 */
  /*      J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv; */
  /*  elseif stage == pHorizon+1 */
  /*      J = (x(C)-ref)'*0.5*Q*(x(C)-ref); */
  /*  else */
  /*      J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv +
   * (pHorizon+1 - stage)*u'*R*u); */
  /*  end */
  /*  if any(u > 5000)  */
  /*      J = J*J; */
  /*  end */
  /*  ((pHorizon+1 -stage)*Rs./pHorizon) */
  b_er = 0.0;
  for (i = 0; i < 14; i++) {
    b_e = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      b_e += er[i1] * 0.5 * dv3[i1 + 14 * i];
    }
    b_er += b_e * er[i];
  }
  c_e = 0.0;
  for (i = 0; i < 3; i++) {
    c_e += ((Z[2050] * 0.5 * (real_T)iv[3 * i] +
             Z[2051] * 0.5 * (real_T)iv[3 * i + 1]) +
            Z[2052] * 0.5 * (real_T)iv[3 * i + 2]) *
           Z[i + 2050];
  }
  *info_Cost = L + (b_er + c_e);
  info_Slack[150] = Z[2050];
  info_Slack[151] = Z[2051];
  info_Slack[152] = Z[2052];
  info_MVopt[50] = mv[0];
  info_MVopt[101] = mv[1];
  info_MVopt[152] = mv[2];
  info_MVopt[203] = mv[3];
  return info_Iterations;
}

/* End of code generation (computeInfo.c) */
