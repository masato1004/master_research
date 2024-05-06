/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpcmoveCodeGeneration.h
 *
 * Code generation for function 'nlmpcmoveCodeGeneration'
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
void c_nlmpcmoveCodeGeneration_anonF(
    const real_T lastMV[4], const real_T onlinedata_StateFcnParameter[309],
    const real_T onlinedata_StageParameter[16473], const real_T U[2071],
    const real_T x[14], real_T varargout_1[2071]);

void nlmpcmoveCodeGeneration(c_nlmpcmoveCodeGenerationStackD *SD,
                             const real_T x[14], const real_T lastMV[4],
                             struct3_T *onlinedata, real_T mv[4],
                             struct4_T *info);

/* End of code generation (nlmpcmoveCodeGeneration.h) */
