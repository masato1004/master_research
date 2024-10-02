/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_nlmpcmoveCodeGeneration_mex.c
 *
 * Code generation for function '_coder_nlmpcmoveCodeGeneration_mex'
 *
 */

/* Include files */
#include "_coder_nlmpcmoveCodeGeneration_mex.h"
#include "_coder_nlmpcmoveCodeGeneration_api.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_initialize.h"
#include "nlmpcmoveCodeGeneration_terminate.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&nlmpcmoveCodeGeneration_atexit);
  /* Module initialization. */
  nlmpcmoveCodeGeneration_initialize();
  /* Dispatch the entry-point. */
  unsafe_nlmpcmoveCodeGeneration_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  nlmpcmoveCodeGeneration_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "Shift_JIS", true);
  return emlrtRootTLSGlobal;
}

void unsafe_nlmpcmoveCodeGeneration_mexFunction(int32_T nlhs, mxArray *plhs[3],
                                                int32_T nrhs,
                                                const mxArray *prhs[3])
{
  const mxArray *outputs[3];
  int32_T i;
  /* Check for proper number of arguments. */
  if (nrhs > 3) {
    emlrtErrMsgIdAndTxt(
        emlrtRootTLSGlobal, "EMLRT:runTime:TooManyInputsConstants", 6, 4, 23,
        "nlmpcmoveCodeGeneration", 4, 23, "nlmpcmoveCodeGeneration");
  }
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal, "EMLRT:runTime:WrongNumberOfInputs",
                        5, 12, 3, 4, 23, "nlmpcmoveCodeGeneration");
  }
  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal,
                        "EMLRT:runTime:TooManyOutputArguments", 3, 4, 23,
                        "nlmpcmoveCodeGeneration");
  }
  /* Call the function. */
  nlmpcmoveCodeGeneration_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

/* End of code generation (_coder_nlmpcmoveCodeGeneration_mex.c) */
