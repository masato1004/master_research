/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpcmoveCodeGeneration_terminate.c
 *
 * Code generation for function 'nlmpcmoveCodeGeneration_terminate'
 *
 */

/* Include files */
#include "nlmpcmoveCodeGeneration_terminate.h"
#include "_coder_nlmpcmoveCodeGeneration_mex.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void nlmpcmoveCodeGeneration_atexit(void)
{
  mexFunctionCreateRootTLS();
  emlrtPushHeapReferenceStackR2021a(emlrtRootTLSGlobal, false, NULL,
                                    (void *)&emlrtExitTimeCleanupDtorFcn, NULL,
                                    NULL, NULL);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void nlmpcmoveCodeGeneration_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (nlmpcmoveCodeGeneration_terminate.c) */
