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
void ab_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void b_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void bb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void c_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void cb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void d_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void db_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void e_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void eb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void f_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void fb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void g_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void gb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                               const real_T u[4], const real_T e[3],
                               const real_T d[8], const real_T muineq[18],
                               const real_T mudummy[8], const real_T dmv[4],
                               const real_T pv[323], real_T Hx[14],
                               real_T Hu[15], real_T Cbds[8]);

void h_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void hb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void i_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void ib_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void j_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void jb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void k_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void kb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void l_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void lb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void m_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void mb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void n_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void nb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void o_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void ob_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void p_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void pb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void q_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void qb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void r_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void rb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void s_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void sb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void t_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void tb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void u_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void ub_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void v_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void vb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void w_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void wb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void x_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void xb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

void y_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                 const real_T u[4], const real_T e[3],
                                 const real_T d[8], const real_T muineq[18],
                                 const real_T mudummy[8], const real_T dmv[4],
                                 const real_T pv[323], real_T Hx[14],
                                 real_T Hu[15], real_T Cbds[8]);

void yb_getHamiltonianDerivatives(const real_T x[14], const real_T lbd[14],
                                  const real_T u[4], const real_T e[3],
                                  const real_T d[8], const real_T muineq[18],
                                  const real_T mudummy[8], const real_T dmv[4],
                                  const real_T pv[323], real_T Hx[14],
                                  real_T Hu[15], real_T Cbds[8]);

/* End of code generation (getHamiltonianDerivatives.h) */
