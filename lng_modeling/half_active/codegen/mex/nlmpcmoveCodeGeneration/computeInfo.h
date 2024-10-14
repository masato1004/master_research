/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeInfo.h
 *
 * Code generation for function 'computeInfo'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
real_T computeInfo(const real_T X[154], const real_T Z[186],
                   const real_T signals_StageParameter[1353],
                   const real_T lastMV[4], real_T info_MVopt[44],
                   real_T info_Xopt[154], real_T info_Topt[11],
                   real_T info_Slack[22], real_T *info_Cost);

/* End of code generation (computeInfo.h) */
