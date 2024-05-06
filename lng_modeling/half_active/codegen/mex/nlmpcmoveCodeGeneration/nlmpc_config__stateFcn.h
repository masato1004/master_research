/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpc_config__stateFcn.h
 *
 * Code generation for function 'nlmpc_config__stateFcn'
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
void nlmpc_config__stateFcn(const real_T x[14], const real_T u[4],
                            const real_T p[309], real_T dxdt[14]);

void nlmpc_config__stateFcn_init(void);

/* End of code generation (nlmpc_config__stateFcn.h) */
