/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * evaluateStateTrajectory.h
 *
 * Code generation for function 'evaluateStateTrajectory'
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
void evaluateStateTrajectory(const real_T Z[186], const real_T x[14],
                             const real_T signals_StateFcnParameter[109],
                             real_T X[154]);

/* End of code generation (evaluateStateTrajectory.h) */
