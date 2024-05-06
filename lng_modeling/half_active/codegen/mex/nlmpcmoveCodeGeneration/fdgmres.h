/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fdgmres.h
 *
 * Code generation for function 'fdgmres'
 *
 */

#pragma once

/* Include files */
#include "nlmpcmoveCodeGeneration_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
real_T fdgmres(c_nlmpcmoveCodeGenerationStackD *SD,
               const real_T F_workspace_lastMV[4],
               const real_T c_F_workspace_onlinedata_StateF[309],
               const real_T c_F_workspace_onlinedata_StageP[16473],
               const real_T U[2071], real_T dUdt[2071], const real_T x[14],
               const real_T dxdt[14], emxArray_real_T *resvec, real_T *relres,
               real_T *iter);

/* End of code generation (fdgmres.h) */
