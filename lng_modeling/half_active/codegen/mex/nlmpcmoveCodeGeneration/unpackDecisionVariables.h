/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * unpackDecisionVariables.h
 *
 * Code generation for function 'unpackDecisionVariables'
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
void ab_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void ac_unpackDecisionVariables(const real_T Z[2071], const real_T lastMV[4],
                                real_T u[4], real_T mv[4], real_T dmv[4],
                                real_T e[3]);

void ad_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void ae_unpackDecisionVariables(const real_T Z[2071], real_T dmv[4],
                                real_T e[3], real_T muineq[18]);

void b_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void bb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void bc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void bd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void c_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void cb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void cc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void cd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void d_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void db_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void dc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void dd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void e_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void eb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void ec_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void ed_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void f_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void fb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void fc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void fd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void g_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void gb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void gc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void gd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void h_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void hb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void hc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void hd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void i_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void ib_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void ic_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void id_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void j_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void jb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void jc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void jd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void k_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void kb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void kc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void kd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void l_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void lb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void lc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void ld_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void m_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void mb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void mc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void md_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void n_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void nb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void nc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void nd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void o_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void ob_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void oc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void od_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void p_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void pb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void pc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void pd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void q_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void qb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void qc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void qd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void r_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void rb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void rc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void rd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void s_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void sb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void sc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void sd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void t_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void tb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void tc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void td_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void u_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void ub_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void uc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void ud_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                             real_T dmv[4], real_T e[3], real_T d[8],
                             real_T muineq[18], real_T mudummy[8]);

void v_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void vb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void vc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void vd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void w_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void wb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void wc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void wd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void x_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void xb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8]);

void xc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void xd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void y_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8]);

void yb_unpackDecisionVariables(const real_T Z[2071], const real_T lastMV[4],
                                real_T u[4], real_T mv[4], real_T dmv[4],
                                real_T e[3], real_T d[8], real_T muineq[18],
                                real_T mudummy[8]);

void yc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

void yd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3]);

/* End of code generation (unpackDecisionVariables.h) */
