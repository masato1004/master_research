#include "rtwtypes.h"
#include "automldiffas_Cc54lCzj.h"

void automldiffas_Cc54lCzj(const real_T u[5], real_T b_bw1, real_T b_bd, real_T
  b_bw2, real_T b_Ndiff, real_T Nl, real_T Nr, real_T shaftSwitch, real_T b_Jd,
  real_T b_Jw1, real_T b_Jw2, real_T b_Jgc, const real_T x[2], real_T y[5],
  real_T xdot[2])
{
  real_T diffDir_e[10];
  real_T diffDir_p[10];
  real_T invJ[4];
  real_T NbdTerm;
  real_T invJ_e;
  real_T invJ_p;
  real_T invJ_tmp;
  real_T term1;
  real_T term2;
  int32_T diffDir;
  int32_T i;
  int32_T invJ_tmp_p;
  term1 = b_Ndiff * b_Ndiff;
  NbdTerm = term1 * b_bd / 4.0;
  if (shaftSwitch != 0.0) {
    diffDir = -1;
  } else {
    diffDir = 1;
  }

  b_Jd += b_Jgc;
  term1 *= b_Jd;
  term2 = (4.0 * b_Jw1 * b_Jw2 + term1 * b_Jw1) + b_Jw2 * term1;
  invJ[0] = (b_Jw2 * 4.0 + term1) / term2;
  invJ_tmp = -term1 / term2;
  invJ[2] = invJ_tmp;
  invJ[1] = invJ_tmp;
  invJ[3] = (b_Jw1 * 4.0 + term1) / term2;
  invJ_tmp = (real_T)diffDir / 2.0 * b_Ndiff;
  diffDir_p[0] = invJ_tmp;
  diffDir_p[2] = -1.0;
  diffDir_p[4] = 0.0;
  diffDir_p[6] = Nl / 2.0;
  diffDir_p[8] = -Nr / 2.0;
  diffDir_p[1] = invJ_tmp;
  diffDir_p[3] = 0.0;
  diffDir_p[5] = -1.0;
  diffDir_p[7] = Nl / 2.0 - 1.0;
  diffDir_p[9] = Nr / 2.0 + 1.0;
  term1 = -(b_bw1 + NbdTerm);
  term2 = -(NbdTerm + b_bw2);
  diffDir_e[0] = invJ_tmp;
  diffDir_e[5] = invJ_tmp;
  for (diffDir = 0; diffDir < 2; diffDir++) {
    invJ_tmp = invJ[diffDir + 2];
    invJ_e = invJ[diffDir];
    invJ_p = 0.0;
    for (i = 0; i < 5; i++) {
      invJ_tmp_p = i << 1;
      invJ_p += (diffDir_p[invJ_tmp_p + 1] * invJ_tmp + diffDir_p[invJ_tmp_p] *
                 invJ_e) * u[i];
    }

    xdot[diffDir] = ((invJ_tmp * term2 + invJ_e * -NbdTerm) * x[1] + (invJ_tmp *
      -NbdTerm + invJ_e * term1) * x[0]) + invJ_p;
    diffDir_e[5 * diffDir + 1] = (real_T)diffDir - 1.0;
    diffDir_e[5 * diffDir + 2] = 0.0 - (real_T)diffDir;
  }

  diffDir_e[3] = -Nl / 2.0;
  diffDir_e[8] = 1.0 - Nl / 2.0;
  diffDir_e[4] = Nr / 2.0;
  diffDir_e[9] = Nr / 2.0 - 1.0;
  for (diffDir = 0; diffDir < 5; diffDir++) {
    NbdTerm = 0.0;
    for (i = 0; i < 5; i++) {
      NbdTerm += 0.0 * u[i];
    }

    y[diffDir] = (diffDir_e[diffDir + 5] * x[1] + diffDir_e[diffDir] * x[0]) +
      NbdTerm;
  }
}
