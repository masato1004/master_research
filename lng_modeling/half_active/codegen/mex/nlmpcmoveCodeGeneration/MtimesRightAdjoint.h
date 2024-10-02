/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * MtimesRightAdjoint.h
 *
 * Code generation for function 'MtimesRightAdjoint'
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
void MtimesRightAdjoint(const real_T left[2], real_T rightJac[2]);

void b_MtimesRightAdjoint(const real_T left[4], real_T rightJac[4]);

/* End of code generation (MtimesRightAdjoint.h) */
