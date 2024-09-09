/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpcmoveCodeGeneration_initialize.c
 *
 * Code generation for function 'nlmpcmoveCodeGeneration_initialize'
 *
 */

/* Include files */
#include "nlmpcmoveCodeGeneration_initialize.h"
#include "_coder_nlmpcmoveCodeGeneration_mex.h"
#include "nlmpc_config__costFcnJacobian.h"
#include "nlmpc_config__ineqConFcn.h"
#include "nlmpc_config__stateFcn.h"
#include "nlmpc_config__stateFcnJacobian.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void nlmpcmoveCodeGeneration_once(void);

/* Function Definitions */
static void nlmpcmoveCodeGeneration_once(void)
{
  mex_InitInfAndNan();
  nlmpc_config__stateFcn_init();
  c_nlmpc_config__stateFcnJacobia();
  nlmpc_config__ineqConFcn_init();
  c_nlmpc_config__costFcnJacobian();
}

void nlmpcmoveCodeGeneration_initialize(void)
{
  mexFunctionCreateRootTLS();
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, NULL);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    nlmpcmoveCodeGeneration_once();
  }
}

/* End of code generation (nlmpcmoveCodeGeneration_initialize.c) */
