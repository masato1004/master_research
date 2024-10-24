/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mtimes.h
 *
 * Code generation for function 'mtimes'
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
void b_mtimes(const real_T A[196], const real_T B[56], real_T C[56]);

void c_mtimes(const real_T A[196], const real_T B[112], real_T C[112]);

void d_mtimes(const real_T A[112], const real_T B[196], real_T C[112]);

void e_mtimes(const real_T A[56], const real_T B[196], real_T C[56]);

void f_mtimes(const real_T A[196], const real_T B[196], real_T C[196]);

void mtimes(const real_T A[196], const real_T B[196], real_T C[196]);

/* End of code generation (mtimes.h) */
