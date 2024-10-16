/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpc_config__ineqConFcn.h
 *
 * Code generation for function 'nlmpc_config__ineqConFcn'
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
void b_nlmpc_config__ineqConFcn(const real_T x[14], const real_T p[123],
                                real_T cineq[4]);

void nlmpc_config__ineqConFcn(const real_T x[14], const real_T e[2],
                              const real_T p[123], real_T cineq[4]);

void nlmpc_config__ineqConFcn_init(void);

/* End of code generation (nlmpc_config__ineqConFcn.h) */
