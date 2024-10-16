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
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
real_T computeInfo(const real_T X[154], const real_T Z[186],
                   const real_T signals_StageParameter[1353],
                   const real_T lastMV[4], real_T info_MVopt[44],
                   real_T info_Xopt[154], real_T info_Topt[11],
                   real_T info_Slack[22], real_T *info_Cost)
{
  __m128d r;
  real_T dmv[4];
  real_T mv[4];
  real_T b_er[2];
  real_T er[2];
  real_T b_dmv;
  real_T c_dmv;
  real_T c_er;
  real_T d_dmv;
  real_T d_er;
  real_T e_dmv;
  real_T e_er;
  real_T er_idx_0;
  real_T er_idx_1;
  real_T f_dmv;
  real_T f_er;
  real_T g_dmv;
  real_T g_er;
  real_T h_dmv;
  real_T h_er;
  real_T i_dmv;
  real_T i_er;
  real_T info_Iterations;
  real_T j_dmv;
  real_T j_er;
  real_T k_dmv;
  real_T l_dmv;
  real_T m_dmv;
  real_T n_dmv;
  real_T o_dmv;
  real_T p_dmv;
  real_T q_dmv;
  real_T r_dmv;
  real_T s_dmv;
  real_T t_dmv;
  real_T u_dmv;
  real_T v_dmv;
  int32_T i;
  int32_T i1;
  memset(&info_MVopt[0], 0, 44U * sizeof(real_T));
  for (i = 0; i < 14; i++) {
    for (i1 = 0; i1 < 11; i1++) {
      info_Xopt[i1 + 11 * i] = X[i + 14 * i1];
    }
  }
  for (i = 0; i < 11; i++) {
    info_Topt[i] = 0.001 * (real_T)i;
  }
  info_Iterations = 0.0;
  r = _mm_loadu_pd(&Z[0]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&lastMV[0])));
  r = _mm_loadu_pd(&Z[2]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&lastMV[2])));
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  dR = diag([1e-02 1e-02 1e-02 1e-02]); */
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
  _mm_storeu_pd(&er[0], _mm_sub_pd(_mm_loadu_pd(&X[12]),
                                   _mm_loadu_pd(&signals_StageParameter[121])));
  /*  ercost = er'*0.5*Q(C,C)*er */
  /*  inputcost = u'*0.5*R*u */
  b_dmv = 0.0;
  /*  % ((pHorizon+1 -stage)*Rs./pHorizon)  */
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    b_dmv += (((dmv[0] * 0.5 * dv[i1] + dmv[1] * 0.5 * dv[i1 + 1]) +
               dmv[2] * 0.5 * dv[i1 + 2]) +
              dmv[3] * 0.5 * dv[i1 + 3]) *
             dmv[i];
    info_MVopt[11 * i] = mv[i];
  }
  info_Slack[0] = Z[4];
  info_Slack[1] = Z[5];
  r = _mm_loadu_pd(&Z[18]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[0])));
  r = _mm_loadu_pd(&Z[20]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[2])));
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  dR = diag([1e-02 1e-02 1e-02 1e-02]); */
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
  _mm_storeu_pd(&b_er[0],
                _mm_sub_pd(_mm_loadu_pd(&X[26]),
                           _mm_loadu_pd(&signals_StageParameter[244])));
  /*  ercost = er'*0.5*Q(C,C)*er */
  /*  inputcost = u'*0.5*R*u */
  c_dmv = 0.0;
  /*  % ((pHorizon+1 -stage)*Rs./pHorizon)  */
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    c_dmv += (((dmv[0] * 0.5 * dv[i1] + dmv[1] * 0.5 * dv[i1 + 1]) +
               dmv[2] * 0.5 * dv[i1 + 2]) +
              dmv[3] * 0.5 * dv[i1 + 3]) *
             dmv[i];
    info_MVopt[11 * i + 1] = mv[i];
  }
  info_Slack[2] = Z[22];
  info_Slack[3] = Z[23];
  r = _mm_loadu_pd(&Z[36]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[18])));
  r = _mm_loadu_pd(&Z[38]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[20])));
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  dR = diag([1e-02 1e-02 1e-02 1e-02]); */
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
  /*  ercost = er'*0.5*Q(C,C)*er */
  /*  inputcost = u'*0.5*R*u */
  /*  % ((pHorizon+1 -stage)*Rs./pHorizon)  */
  er_idx_0 = X[40] - signals_StageParameter[367];
  er_idx_1 = X[41] - signals_StageParameter[368];
  c_er = (er[0] * 0.5 * 100000.0 + er[1] * 0.5 * 0.0) * er[0] +
         (er[0] * 0.5 * 0.0 + er[1] * 0.5 * 100000.0) * er[1];
  d_dmv = 0.0;
  e_dmv = 0.0;
  f_dmv = 0.0;
  g_dmv = 0.0;
  for (i = 0; i < 4; i++) {
    int32_T i2;
    int32_T i3;
    int32_T i4;
    int32_T i5;
    i1 = i << 2;
    i2 = iv3[i1];
    i3 = iv3[i1 + 1];
    i4 = iv3[i1 + 2];
    i5 = iv3[i1 + 3];
    d_dmv += (((Z[0] * 0.5 * (real_T)i2 + Z[1] * 0.5 * (real_T)i3) +
               Z[2] * 0.5 * (real_T)i4) +
              Z[3] * 0.5 * (real_T)i5) *
             Z[i];
    e_dmv += (((Z[18] * 0.5 * (real_T)i2 + Z[19] * 0.5 * (real_T)i3) +
               Z[20] * 0.5 * (real_T)i4) +
              Z[21] * 0.5 * (real_T)i5) *
             Z[i + 18];
    f_dmv += (((Z[36] * 0.5 * (real_T)i2 + Z[37] * 0.5 * (real_T)i3) +
               Z[38] * 0.5 * (real_T)i4) +
              Z[39] * 0.5 * (real_T)i5) *
             Z[i + 36];
    g_dmv += (((dmv[0] * 0.5 * dv[i1] + dmv[1] * 0.5 * dv[i1 + 1]) +
               dmv[2] * 0.5 * dv[i1 + 2]) +
              dmv[3] * 0.5 * dv[i1 + 3]) *
             dmv[i];
  }
  info_MVopt[2] = mv[0];
  info_MVopt[13] = mv[1];
  info_MVopt[24] = mv[2];
  info_MVopt[35] = mv[3];
  info_Slack[4] = Z[40];
  info_Slack[5] = Z[41];
  r = _mm_loadu_pd(&Z[54]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[36])));
  r = _mm_loadu_pd(&Z[56]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[38])));
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  dR = diag([1e-02 1e-02 1e-02 1e-02]); */
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
  _mm_storeu_pd(&er[0], _mm_sub_pd(_mm_loadu_pd(&X[54]),
                                   _mm_loadu_pd(&signals_StageParameter[490])));
  /*  ercost = er'*0.5*Q(C,C)*er */
  /*  inputcost = u'*0.5*R*u */
  /*  % ((pHorizon+1 -stage)*Rs./pHorizon)  */
  d_er = (er[0] * 0.5 * 100000.0 + er[1] * 0.5 * 0.0) * er[0] +
         (er[0] * 0.5 * 0.0 + er[1] * 0.5 * 100000.0) * er[1];
  h_dmv = 0.0;
  i_dmv = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    h_dmv +=
        (((Z[54] * 0.5 * (real_T)iv3[i1] + Z[55] * 0.5 * (real_T)iv3[i1 + 1]) +
          Z[56] * 0.5 * (real_T)iv3[i1 + 2]) +
         Z[57] * 0.5 * (real_T)iv3[i1 + 3]) *
        Z[i + 54];
    i_dmv += (((dmv[0] * 0.5 * dv[i1] + dmv[1] * 0.5 * dv[i1 + 1]) +
               dmv[2] * 0.5 * dv[i1 + 2]) +
              dmv[3] * 0.5 * dv[i1 + 3]) *
             dmv[i];
  }
  info_MVopt[3] = mv[0];
  info_MVopt[14] = mv[1];
  info_MVopt[25] = mv[2];
  info_MVopt[36] = mv[3];
  info_Slack[6] = Z[58];
  info_Slack[7] = Z[59];
  r = _mm_loadu_pd(&Z[72]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[54])));
  r = _mm_loadu_pd(&Z[74]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[56])));
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  dR = diag([1e-02 1e-02 1e-02 1e-02]); */
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
  _mm_storeu_pd(&er[0], _mm_sub_pd(_mm_loadu_pd(&X[68]),
                                   _mm_loadu_pd(&signals_StageParameter[613])));
  /*  ercost = er'*0.5*Q(C,C)*er */
  /*  inputcost = u'*0.5*R*u */
  /*  % ((pHorizon+1 -stage)*Rs./pHorizon)  */
  e_er = (er[0] * 0.5 * 100000.0 + er[1] * 0.5 * 0.0) * er[0] +
         (er[0] * 0.5 * 0.0 + er[1] * 0.5 * 100000.0) * er[1];
  j_dmv = 0.0;
  k_dmv = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    j_dmv +=
        (((Z[72] * 0.5 * (real_T)iv3[i1] + Z[73] * 0.5 * (real_T)iv3[i1 + 1]) +
          Z[74] * 0.5 * (real_T)iv3[i1 + 2]) +
         Z[75] * 0.5 * (real_T)iv3[i1 + 3]) *
        Z[i + 72];
    k_dmv += (((dmv[0] * 0.5 * dv[i1] + dmv[1] * 0.5 * dv[i1 + 1]) +
               dmv[2] * 0.5 * dv[i1 + 2]) +
              dmv[3] * 0.5 * dv[i1 + 3]) *
             dmv[i];
  }
  info_MVopt[4] = mv[0];
  info_MVopt[15] = mv[1];
  info_MVopt[26] = mv[2];
  info_MVopt[37] = mv[3];
  info_Slack[8] = Z[76];
  info_Slack[9] = Z[77];
  r = _mm_loadu_pd(&Z[90]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[72])));
  r = _mm_loadu_pd(&Z[92]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[74])));
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  dR = diag([1e-02 1e-02 1e-02 1e-02]); */
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
  _mm_storeu_pd(&er[0], _mm_sub_pd(_mm_loadu_pd(&X[82]),
                                   _mm_loadu_pd(&signals_StageParameter[736])));
  /*  ercost = er'*0.5*Q(C,C)*er */
  /*  inputcost = u'*0.5*R*u */
  /*  % ((pHorizon+1 -stage)*Rs./pHorizon)  */
  f_er = (er[0] * 0.5 * 100000.0 + er[1] * 0.5 * 0.0) * er[0] +
         (er[0] * 0.5 * 0.0 + er[1] * 0.5 * 100000.0) * er[1];
  l_dmv = 0.0;
  m_dmv = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    l_dmv +=
        (((Z[90] * 0.5 * (real_T)iv3[i1] + Z[91] * 0.5 * (real_T)iv3[i1 + 1]) +
          Z[92] * 0.5 * (real_T)iv3[i1 + 2]) +
         Z[93] * 0.5 * (real_T)iv3[i1 + 3]) *
        Z[i + 90];
    m_dmv += (((dmv[0] * 0.5 * dv[i1] + dmv[1] * 0.5 * dv[i1 + 1]) +
               dmv[2] * 0.5 * dv[i1 + 2]) +
              dmv[3] * 0.5 * dv[i1 + 3]) *
             dmv[i];
  }
  info_MVopt[5] = mv[0];
  info_MVopt[16] = mv[1];
  info_MVopt[27] = mv[2];
  info_MVopt[38] = mv[3];
  info_Slack[10] = Z[94];
  info_Slack[11] = Z[95];
  r = _mm_loadu_pd(&Z[108]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[90])));
  r = _mm_loadu_pd(&Z[110]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[92])));
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  dR = diag([1e-02 1e-02 1e-02 1e-02]); */
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
  _mm_storeu_pd(&er[0], _mm_sub_pd(_mm_loadu_pd(&X[96]),
                                   _mm_loadu_pd(&signals_StageParameter[859])));
  /*  ercost = er'*0.5*Q(C,C)*er */
  /*  inputcost = u'*0.5*R*u */
  /*  % ((pHorizon+1 -stage)*Rs./pHorizon)  */
  g_er = (er[0] * 0.5 * 100000.0 + er[1] * 0.5 * 0.0) * er[0] +
         (er[0] * 0.5 * 0.0 + er[1] * 0.5 * 100000.0) * er[1];
  n_dmv = 0.0;
  o_dmv = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    n_dmv += (((Z[108] * 0.5 * (real_T)iv3[i1] +
                Z[109] * 0.5 * (real_T)iv3[i1 + 1]) +
               Z[110] * 0.5 * (real_T)iv3[i1 + 2]) +
              Z[111] * 0.5 * (real_T)iv3[i1 + 3]) *
             Z[i + 108];
    o_dmv += (((dmv[0] * 0.5 * dv[i1] + dmv[1] * 0.5 * dv[i1 + 1]) +
               dmv[2] * 0.5 * dv[i1 + 2]) +
              dmv[3] * 0.5 * dv[i1 + 3]) *
             dmv[i];
  }
  info_MVopt[6] = mv[0];
  info_MVopt[17] = mv[1];
  info_MVopt[28] = mv[2];
  info_MVopt[39] = mv[3];
  info_Slack[12] = Z[112];
  info_Slack[13] = Z[113];
  r = _mm_loadu_pd(&Z[126]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[108])));
  r = _mm_loadu_pd(&Z[128]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[110])));
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  dR = diag([1e-02 1e-02 1e-02 1e-02]); */
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
  _mm_storeu_pd(&er[0], _mm_sub_pd(_mm_loadu_pd(&X[110]),
                                   _mm_loadu_pd(&signals_StageParameter[982])));
  /*  ercost = er'*0.5*Q(C,C)*er */
  /*  inputcost = u'*0.5*R*u */
  /*  % ((pHorizon+1 -stage)*Rs./pHorizon)  */
  h_er = (er[0] * 0.5 * 100000.0 + er[1] * 0.5 * 0.0) * er[0] +
         (er[0] * 0.5 * 0.0 + er[1] * 0.5 * 100000.0) * er[1];
  p_dmv = 0.0;
  q_dmv = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    p_dmv += (((Z[126] * 0.5 * (real_T)iv3[i1] +
                Z[127] * 0.5 * (real_T)iv3[i1 + 1]) +
               Z[128] * 0.5 * (real_T)iv3[i1 + 2]) +
              Z[129] * 0.5 * (real_T)iv3[i1 + 3]) *
             Z[i + 126];
    q_dmv += (((dmv[0] * 0.5 * dv[i1] + dmv[1] * 0.5 * dv[i1 + 1]) +
               dmv[2] * 0.5 * dv[i1 + 2]) +
              dmv[3] * 0.5 * dv[i1 + 3]) *
             dmv[i];
  }
  info_MVopt[7] = mv[0];
  info_MVopt[18] = mv[1];
  info_MVopt[29] = mv[2];
  info_MVopt[40] = mv[3];
  info_Slack[14] = Z[130];
  info_Slack[15] = Z[131];
  r = _mm_loadu_pd(&Z[144]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[126])));
  r = _mm_loadu_pd(&Z[146]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[128])));
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  dR = diag([1e-02 1e-02 1e-02 1e-02]); */
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
  _mm_storeu_pd(&er[0],
                _mm_sub_pd(_mm_loadu_pd(&X[124]),
                           _mm_loadu_pd(&signals_StageParameter[1105])));
  /*  ercost = er'*0.5*Q(C,C)*er */
  /*  inputcost = u'*0.5*R*u */
  /*  % ((pHorizon+1 -stage)*Rs./pHorizon)  */
  i_er = (er[0] * 0.5 * 100000.0 + er[1] * 0.5 * 0.0) * er[0] +
         (er[0] * 0.5 * 0.0 + er[1] * 0.5 * 100000.0) * er[1];
  r_dmv = 0.0;
  s_dmv = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    r_dmv += (((Z[144] * 0.5 * (real_T)iv3[i1] +
                Z[145] * 0.5 * (real_T)iv3[i1 + 1]) +
               Z[146] * 0.5 * (real_T)iv3[i1 + 2]) +
              Z[147] * 0.5 * (real_T)iv3[i1 + 3]) *
             Z[i + 144];
    s_dmv += (((dmv[0] * 0.5 * dv[i1] + dmv[1] * 0.5 * dv[i1 + 1]) +
               dmv[2] * 0.5 * dv[i1 + 2]) +
              dmv[3] * 0.5 * dv[i1 + 3]) *
             dmv[i];
  }
  info_MVopt[8] = mv[0];
  info_MVopt[19] = mv[1];
  info_MVopt[30] = mv[2];
  info_MVopt[41] = mv[3];
  info_Slack[16] = Z[148];
  info_Slack[17] = Z[149];
  r = _mm_loadu_pd(&Z[162]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[144])));
  r = _mm_loadu_pd(&Z[164]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[146])));
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  dR = diag([1e-02 1e-02 1e-02 1e-02]); */
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
  _mm_storeu_pd(&er[0],
                _mm_sub_pd(_mm_loadu_pd(&X[138]),
                           _mm_loadu_pd(&signals_StageParameter[1228])));
  /*  ercost = er'*0.5*Q(C,C)*er */
  /*  inputcost = u'*0.5*R*u */
  /*  % ((pHorizon+1 -stage)*Rs./pHorizon)  */
  j_er = (er[0] * 0.5 * 100000.0 + er[1] * 0.5 * 0.0) * er[0] +
         (er[0] * 0.5 * 0.0 + er[1] * 0.5 * 100000.0) * er[1];
  t_dmv = 0.0;
  u_dmv = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    t_dmv += (((Z[162] * 0.5 * (real_T)iv3[i1] +
                Z[163] * 0.5 * (real_T)iv3[i1 + 1]) +
               Z[164] * 0.5 * (real_T)iv3[i1 + 2]) +
              Z[165] * 0.5 * (real_T)iv3[i1 + 3]) *
             Z[i + 162];
    u_dmv += (((dmv[0] * 0.5 * dv[i1] + dmv[1] * 0.5 * dv[i1 + 1]) +
               dmv[2] * 0.5 * dv[i1 + 2]) +
              dmv[3] * 0.5 * dv[i1 + 3]) *
             dmv[i];
  }
  info_MVopt[9] = mv[0];
  info_MVopt[20] = mv[1];
  info_MVopt[31] = mv[2];
  info_MVopt[42] = mv[3];
  info_Slack[18] = Z[166];
  info_Slack[19] = Z[167];
  r = _mm_set1_pd(-1.0);
  _mm_storeu_pd(&dmv[0], _mm_mul_pd(_mm_loadu_pd(&Z[162]), r));
  _mm_storeu_pd(&dmv[2], _mm_mul_pd(_mm_loadu_pd(&Z[164]), r));
  /*  */
  /*  x: (1) Longitudinal Position */
  /*     (2) Body Vertical Displacement */
  /*     (3) Front Wheel Vertical Displacement */
  /*     (4) Rear Wheel Vertical Displacement */
  /*     (5) Body Pitch Angle */
  /*     (6) Front Wheel Angle */
  /*     (7) Rear Wheel Angle */
  /*     (8) Velocity */
  /*     (9) Body Vertical Velocity */
  /*     (10) Front Wheel Vertical Velocity */
  /*     (11) Rear Wheel Vertical Velocity */
  /*     (12) Body Pitch Angular Velocity */
  /*     (13) Front Wheel Angular Velocity */
  /*     (14) Rear Wheel Angular Velocity */
  /*  */
  /*  u: (1) front torque */
  /*     (2) rear torque */
  /*     (3) front sus */
  /*     (4) rear sus */
  /*  */
  /*  d: (1) longitudinal position of front wheel center */
  /*     (2) longitudinal position of rear wheel center */
  /*     (3) vertical position of front wheel center */
  /*     (4) vertical position of rear wheel center */
  /*     (5) gradient of longitudinal position of front wheel center */
  /*     (6) gradient of longitudinal position of rear wheel center */
  /*     (7) gradient of vertical position of front wheel center */
  /*     (8) gradient of vertical position of rear wheel center */
  /*  */
  /*  New Half-Car-Model cost function. */
  /*  R = diag([1e-20 1e-20 1e-20 1e-20]); */
  /*  Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]); */
  /*  dR = diag([1e-02 1e-02 1e-02 1e-02]); */
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
  _mm_storeu_pd(&er[0],
                _mm_sub_pd(_mm_loadu_pd(&X[152]),
                           _mm_loadu_pd(&signals_StageParameter[1351])));
  /*  ercost = er'*0.5*Q(C,C)*er */
  /*  inputcost = u'*0.5*R*u */
  /*  % ((pHorizon+1 -stage)*Rs./pHorizon)  */
  v_dmv = 0.0;
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    v_dmv += (((dmv[0] * 0.5 * dv[i1] + dmv[1] * 0.5 * dv[i1 + 1]) +
               dmv[2] * 0.5 * dv[i1 + 2]) +
              dmv[3] * 0.5 * dv[i1 + 3]) *
             dmv[i];
  }
  *info_Cost =
      ((((((((((((c_er + d_dmv) + b_dmv) +
                ((Z[4] * 0.5 + Z[5] * 0.5 * 0.0) * Z[4] +
                 (Z[4] * 0.5 * 0.0 + Z[5] * 0.5) * Z[5])) +
               (((((b_er[0] * 0.5 * 100000.0 + b_er[1] * 0.5 * 0.0) * b_er[0] +
                   (b_er[0] * 0.5 * 0.0 + b_er[1] * 0.5 * 100000.0) * b_er[1]) +
                  e_dmv) +
                 c_dmv) +
                ((Z[22] * 0.5 + Z[23] * 0.5 * 0.0) * Z[22] +
                 (Z[22] * 0.5 * 0.0 + Z[23] * 0.5) * Z[23]))) +
              (((((er_idx_0 * 0.5 * 100000.0 + er_idx_1 * 0.5 * 0.0) *
                      er_idx_0 +
                  (er_idx_0 * 0.5 * 0.0 + er_idx_1 * 0.5 * 100000.0) *
                      er_idx_1) +
                 f_dmv) +
                g_dmv) +
               ((Z[40] * 0.5 + Z[41] * 0.5 * 0.0) * Z[40] +
                (Z[40] * 0.5 * 0.0 + Z[41] * 0.5) * Z[41]))) +
             (((d_er + h_dmv) + i_dmv) +
              ((Z[58] * 0.5 + Z[59] * 0.5 * 0.0) * Z[58] +
               (Z[58] * 0.5 * 0.0 + Z[59] * 0.5) * Z[59]))) +
            (((e_er + j_dmv) + k_dmv) +
             ((Z[76] * 0.5 + Z[77] * 0.5 * 0.0) * Z[76] +
              (Z[76] * 0.5 * 0.0 + Z[77] * 0.5) * Z[77]))) +
           (((f_er + l_dmv) + m_dmv) +
            ((Z[94] * 0.5 + Z[95] * 0.5 * 0.0) * Z[94] +
             (Z[94] * 0.5 * 0.0 + Z[95] * 0.5) * Z[95]))) +
          (((g_er + n_dmv) + o_dmv) +
           ((Z[112] * 0.5 + Z[113] * 0.5 * 0.0) * Z[112] +
            (Z[112] * 0.5 * 0.0 + Z[113] * 0.5) * Z[113]))) +
         (((h_er + p_dmv) + q_dmv) +
          ((Z[130] * 0.5 + Z[131] * 0.5 * 0.0) * Z[130] +
           (Z[130] * 0.5 * 0.0 + Z[131] * 0.5) * Z[131]))) +
        (((i_er + r_dmv) + s_dmv) +
         ((Z[148] * 0.5 + Z[149] * 0.5 * 0.0) * Z[148] +
          (Z[148] * 0.5 * 0.0 + Z[149] * 0.5) * Z[149]))) +
       (((j_er + t_dmv) + u_dmv) +
        ((Z[166] * 0.5 + Z[167] * 0.5 * 0.0) * Z[166] +
         (Z[166] * 0.5 * 0.0 + Z[167] * 0.5) * Z[167]))) +
      ((((er[0] * 0.5 * 100000.0 + er[1] * 0.5 * 0.0) * er[0] +
         (er[0] * 0.5 * 0.0 + er[1] * 0.5 * 100000.0) * er[1]) +
        v_dmv) +
       ((Z[180] * 0.5 + Z[181] * 0.5 * 0.0) * Z[180] +
        (Z[180] * 0.5 * 0.0 + Z[181] * 0.5) * Z[181]));
  info_Slack[20] = Z[180];
  info_Slack[21] = Z[181];
  info_MVopt[10] = mv[0];
  info_MVopt[21] = mv[1];
  info_MVopt[32] = mv[2];
  info_MVopt[43] = mv[3];
  return info_Iterations;
}

/* End of code generation (computeInfo.c) */
