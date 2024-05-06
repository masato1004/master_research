/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_nlmpcmoveCodeGeneration_api.c
 *
 * Code generation for function '_coder_nlmpcmoveCodeGeneration_api'
 *
 */

/* Include files */
#include "_coder_nlmpcmoveCodeGeneration_api.h"
#include "nlmpcmoveCodeGeneration.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[14];

static const mxArray *b_emlrt_marshallOut(const struct3_T *u);

static real_T (*c_emlrt_marshallIn(const mxArray *nullptr,
                                   const char_T *identifier))[4];

static const mxArray *c_emlrt_marshallOut(const struct4_T *u);

static real_T (*d_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4];

static void e_emlrt_marshallIn(const mxArray *nullptr, const char_T *identifier,
                               struct3_T *y);

static real_T (*emlrt_marshallIn(const mxArray *nullptr,
                                 const char_T *identifier))[14];

static const mxArray *emlrt_marshallOut(const real_T u[4]);

static void f_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct3_T *y);

static void g_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[309]);

static void h_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[16473]);

static void i_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[4156]);

static real_T (*j_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[14];

static real_T (*k_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4];

static void l_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret[309]);

static void m_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret[16473]);

static void n_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret[4156]);

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[14]
{
  real_T(*y)[14];
  y = j_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const struct3_T *u)
{
  static const int32_T i = 309;
  static const int32_T i1 = 16473;
  static const int32_T i2 = 4156;
  static const char_T *sv[3] = {"StateFcnParameter", "StageParameter",
                                "InitialGuess"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 3, (const char_T **)&sv[0]));
  b_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (b_i = 0; b_i < 309; b_i++) {
    pData[b_i] = u->StateFcnParameter[b_i];
  }
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "StateFcnParameter", b_y, 0);
  c_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (b_i = 0; b_i < 16473; b_i++) {
    pData[b_i] = u->StageParameter[b_i];
  }
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "StageParameter", c_y, 1);
  d_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i2, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (b_i = 0; b_i < 4156; b_i++) {
    pData[b_i] = u->InitialGuess[b_i];
  }
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, "InitialGuess", d_y, 2);
  return y;
}

static real_T (*c_emlrt_marshallIn(const mxArray *nullptr,
                                   const char_T *identifier))[4]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[4];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *c_emlrt_marshallOut(const struct4_T *u)
{
  static const int32_T b_iv[2] = {51, 4};
  static const int32_T b_iv1[2] = {51, 14};
  static const int32_T i1 = 51;
  static const int32_T i2 = 153;
  static const char_T *sv[7] = {"MVopt",      "Xopt", "Topt",    "Slack",
                                "Iterations", "Cost", "ExitFlag"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&sv[0]));
  b_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&b_iv[0], mxDOUBLE_CLASS,
                              mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 4; b_i++) {
    for (c_i = 0; c_i < 51; c_i++) {
      pData[i + c_i] = u->MVopt[c_i + 51 * b_i];
    }
    i += 51;
  }
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "MVopt", b_y, 0);
  c_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&b_iv1[0], mxDOUBLE_CLASS,
                              mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 14; b_i++) {
    for (c_i = 0; c_i < 51; c_i++) {
      pData[i + c_i] = u->Xopt[c_i + 51 * b_i];
    }
    i += 51;
  }
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "Xopt", c_y, 1);
  d_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (b_i = 0; b_i < 51; b_i++) {
    pData[b_i] = u->Topt[b_i];
  }
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, "Topt", d_y, 2);
  e_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i2, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (b_i = 0; b_i < 153; b_i++) {
    pData[b_i] = u->Slack[b_i];
  }
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "Slack", e_y, 3);
  f_y = NULL;
  m = emlrtCreateDoubleScalar(u->Iterations);
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, "Iterations", f_y, 4);
  g_y = NULL;
  m = emlrtCreateDoubleScalar(u->Cost);
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0, "Cost", g_y, 5);
  h_y = NULL;
  m = emlrtCreateDoubleScalar(u->ExitFlag);
  emlrtAssign(&h_y, m);
  emlrtSetFieldR2017b(y, 0, "ExitFlag", h_y, 6);
  return y;
}

static real_T (*d_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4]
{
  real_T(*y)[4];
  y = k_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void e_emlrt_marshallIn(const mxArray *nullptr, const char_T *identifier,
                               struct3_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

static real_T (*emlrt_marshallIn(const mxArray *nullptr,
                                 const char_T *identifier))[14]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[14];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[4])
{
  static const int32_T i = 0;
  static const int32_T i1 = 4;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct3_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[3] = {"StateFcnParameter", "StageParameter",
                                        "InitialGuess"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(emlrtRootTLSGlobal, parentId, u, 3,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "StateFcnParameter";
  g_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 0,
                                                    "StateFcnParameter")),
                     &thisId, y->StateFcnParameter);
  thisId.fIdentifier = "StageParameter";
  h_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 1,
                                                    "StageParameter")),
                     &thisId, y->StageParameter);
  thisId.fIdentifier = "InitialGuess";
  i_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 2,
                                                    "InitialGuess")),
                     &thisId, y->InitialGuess);
  emlrtDestroyArray(&u);
}

static void g_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[309])
{
  l_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void h_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[16473])
{
  m_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[4156])
{
  n_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T (*j_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[14]
{
  static const int32_T dims = 14;
  real_T(*ret)[14];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[14])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*k_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4]
{
  static const int32_T dims = 4;
  real_T(*ret)[4];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void l_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[309])
{
  static const int32_T dims = 309;
  real_T(*r)[309];
  int32_T i;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[309])emlrtMxGetData(src);
  for (i = 0; i < 309; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void m_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret[16473])
{
  static const int32_T dims = 16473;
  real_T(*r)[16473];
  int32_T i;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[16473])emlrtMxGetData(src);
  for (i = 0; i < 16473; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void n_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret[4156])
{
  static const int32_T dims = 4156;
  real_T(*r)[4156];
  int32_T i;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[4156])emlrtMxGetData(src);
  for (i = 0; i < 4156; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

void nlmpcmoveCodeGeneration_api(c_nlmpcmoveCodeGenerationStackD *SD,
                                 const mxArray *const prhs[3], int32_T nlhs,
                                 const mxArray *plhs[3])
{
  struct4_T info;
  real_T(*x)[14];
  real_T(*lastMV)[4];
  real_T(*mv)[4];
  mv = (real_T(*)[4])mxMalloc(sizeof(real_T[4]));
  /* Marshall function inputs */
  x = emlrt_marshallIn(emlrtAlias(prhs[0]), "x");
  lastMV = c_emlrt_marshallIn(emlrtAlias(prhs[1]), "lastMV");
  e_emlrt_marshallIn(emlrtAliasP(prhs[2]), "onlinedata", &SD->f2.onlinedata);
  /* Invoke the target function */
  nlmpcmoveCodeGeneration(SD, *x, *lastMV, &SD->f2.onlinedata, *mv, &info);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*mv);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(&SD->f2.onlinedata);
  }
  if (nlhs > 2) {
    plhs[2] = c_emlrt_marshallOut(&info);
  }
}

/* End of code generation (_coder_nlmpcmoveCodeGeneration_api.c) */
