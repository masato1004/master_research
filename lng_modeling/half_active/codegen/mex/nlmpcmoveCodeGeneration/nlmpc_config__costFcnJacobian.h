/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpc_config__costFcnJacobian.h
 *
 * Code generation for function 'nlmpc_config__costFcnJacobian'
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
void c_nlmpc_config__costFcnJacobian(void);

void nlmpc_config__costFcnJacobian(const real_T x[14], const real_T u[4],
                                   const real_T dmv[4], const real_T e[2],
                                   const real_T p[123], real_T Jx[14],
                                   real_T Ju[4], real_T Jdmv[4], real_T Je[2]);

/* End of code generation (nlmpc_config__costFcnJacobian.h) */
