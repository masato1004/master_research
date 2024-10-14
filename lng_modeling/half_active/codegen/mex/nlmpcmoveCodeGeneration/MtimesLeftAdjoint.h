/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * MtimesLeftAdjoint.h
 *
 * Code generation for function 'MtimesLeftAdjoint'
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
void MtimesLeftAdjoint(const real_T right[4], const real_T currAdjoint[2],
                       real_T leftJac[2]);

void b_MtimesLeftAdjoint(const real_T right[16], const real_T currAdjoint[4],
                         real_T leftJac[4]);

/* End of code generation (MtimesLeftAdjoint.h) */
