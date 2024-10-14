/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * getHamiltonianDerivatives.h
 *
 * Code generation for function 'getHamiltonianDerivatives'
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
void b_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[2],
                                 const real_T d[4], const real_T muineq[4],
                                 const real_T mudummy[4], const real_T dmv[4],
                                 const real_T pv[123], real_T Hx[14],
                                 real_T Hu[10], real_T Cbds[4]);

real_T boundsToConstraints(real_T z, real_T zlb, real_T zub, real_T *cz);

void c_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[2],
                                 const real_T d[4], const real_T muineq[4],
                                 const real_T mudummy[4], const real_T dmv[4],
                                 const real_T pv[123], real_T Hx[14],
                                 real_T Hu[10], real_T Cbds[4]);

void d_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[2],
                                 const real_T d[4], const real_T muineq[4],
                                 const real_T mudummy[4], const real_T dmv[4],
                                 const real_T pv[123], real_T Hx[14],
                                 real_T Hu[10], real_T Cbds[4]);

void e_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[2],
                                 const real_T d[4], const real_T muineq[4],
                                 const real_T mudummy[4], const real_T dmv[4],
                                 const real_T pv[123], real_T Hx[14],
                                 real_T Hu[10], real_T Cbds[4]);

void evaluateJacobian(const real_T x[14], const real_T u[4],
                      const real_T dmv[4], const real_T e[2],
                      const real_T pvstage[123], real_T Gx[56], real_T Gmv[16],
                      real_T Gdmv[16], real_T Ge[8]);

void getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                               const real_T u[4], const real_T e[2],
                               const real_T d[4], const real_T muineq[4],
                               const real_T mudummy[4], const real_T dmv[4],
                               const real_T pv[123], real_T Hx[14],
                               real_T Hu[10], real_T Cbds[4]);

/* End of code generation (getHamiltonianDerivatives.h) */
