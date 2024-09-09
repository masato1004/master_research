/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * makima.h
 *
 * Code generation for function 'makima'
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
void b_makima(const real_T x[20], const real_T y[20], const real_T xx[3],
              real_T output[3]);

real_T makima(const real_T x[20], const real_T y[20], real_T xx);

/* End of code generation (makima.h) */
