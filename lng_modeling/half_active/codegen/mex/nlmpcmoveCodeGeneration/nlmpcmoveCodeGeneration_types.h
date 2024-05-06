/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpcmoveCodeGeneration_types.h
 *
 * Code generation for function 'nlmpcmoveCodeGeneration'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include <stddef.h>

/* Type Definitions */
#ifndef typedef_struct3_T
#define typedef_struct3_T
typedef struct {
  real_T StateFcnParameter[309];
  real_T StageParameter[16473];
  real_T InitialGuess[4156];
} struct3_T;
#endif /* typedef_struct3_T */

#ifndef typedef_struct4_T
#define typedef_struct4_T
typedef struct {
  real_T MVopt[204];
  real_T Xopt[714];
  real_T Topt[51];
  real_T Slack[153];
  real_T Iterations;
  real_T Cost;
  real_T ExitFlag;
} struct4_T;
#endif /* typedef_struct4_T */

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T
struct emxArray_boolean_T {
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_boolean_T */
#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T
typedef struct emxArray_boolean_T emxArray_boolean_T;
#endif /* typedef_emxArray_boolean_T */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T {
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_int32_T */
#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T
typedef struct emxArray_int32_T emxArray_int32_T;
#endif /* typedef_emxArray_int32_T */

#ifndef struct_emxArray_ptrdiff_t
#define struct_emxArray_ptrdiff_t
struct emxArray_ptrdiff_t {
  ptrdiff_t *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_ptrdiff_t */
#ifndef typedef_emxArray_ptrdiff_t
#define typedef_emxArray_ptrdiff_t
typedef struct emxArray_ptrdiff_t emxArray_ptrdiff_t;
#endif /* typedef_emxArray_ptrdiff_t */

#ifndef typedef_b_fdgmres
#define typedef_b_fdgmres
typedef struct {
  real_T v[43491];
  real_T v_[43491];
} b_fdgmres;
#endif /* typedef_b_fdgmres */

#ifndef c_typedef_b_nlmpcmoveCodeGenera
#define c_typedef_b_nlmpcmoveCodeGenera
typedef struct {
  real_T z[2071];
  real_T zdt[2071];
  real_T U_[2071];
} b_nlmpcmoveCodeGeneration;
#endif /* c_typedef_b_nlmpcmoveCodeGenera */

#ifndef d_typedef_b_nlmpcmoveCodeGenera
#define d_typedef_b_nlmpcmoveCodeGenera
typedef struct {
  struct3_T onlinedata;
} b_nlmpcmoveCodeGeneration_api;
#endif /* d_typedef_b_nlmpcmoveCodeGenera */

#ifndef c_typedef_c_nlmpcmoveCodeGenera
#define c_typedef_c_nlmpcmoveCodeGenera
typedef struct {
  b_fdgmres f0;
  b_nlmpcmoveCodeGeneration f1;
  b_nlmpcmoveCodeGeneration_api f2;
} c_nlmpcmoveCodeGenerationStackD;
#endif /* c_typedef_c_nlmpcmoveCodeGenera */

/* End of code generation (nlmpcmoveCodeGeneration_types.h) */
