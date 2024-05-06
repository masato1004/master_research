/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpc_config__costFcnJacobianAD.h
 *
 * Code generation for function 'nlmpc_config__costFcnJacobianAD'
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
real_T nlmpc_config__costFcnJacobianAD(
    const real_T inputVariables[25], real_T extraParams_f1,
    const real_T extraParams_f2[196], real_T extraParams_f3,
    const real_T extraParams_f4[16], real_T extraParams_f5,
    const real_T extraParams_f6[9], const real_T params_p[323],
    real_T grad[25]);

/* End of code generation (nlmpc_config__costFcnJacobianAD.h) */
