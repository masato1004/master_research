/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * MtimesRightAdjoint.c
 *
 * Code generation for function 'MtimesRightAdjoint'
 *
 */

/* Include files */
#include "MtimesRightAdjoint.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void MtimesRightAdjoint(const real_T left[2], real_T rightJac[2])
{
  rightJac[0] = left[0];
  rightJac[1] = left[1];
}

void b_MtimesRightAdjoint(const real_T left[4], real_T rightJac[4])
{
  rightJac[0] = left[0];
  rightJac[1] = left[1];
  rightJac[2] = left[2];
  rightJac[3] = left[3];
}

/* End of code generation (MtimesRightAdjoint.c) */
