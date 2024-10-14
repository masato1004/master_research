/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mpower.c
 *
 * Code generation for function 'mpower'
 *
 */

/* Include files */
#include "mpower.h"
#include "inv.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void mpower(real_T a[196], real_T b)
{
  real_T aBuffer[196];
  real_T b_a[196];
  real_T cBuffer[196];
  int32_T b_n;
  if (muDoubleScalarFloor(b) == b) {
    real_T e;
    e = muDoubleScalarAbs(b);
    if (e <= 2.147483647E+9) {
      int32_T n;
      int32_T nb;
      int32_T nbitson;
      memcpy(&b_a[0], &a[0], 196U * sizeof(real_T));
      n = (int32_T)e;
      b_n = n;
      nbitson = 0;
      nb = -2;
      while (b_n > 0) {
        nb++;
        if ((b_n & 1) != 0) {
          nbitson++;
        }
        b_n >>= 1;
      }
      if (e <= 2.0) {
        if (b == 2.0) {
          memcpy(&b_a[0], &a[0], 196U * sizeof(real_T));
          memcpy(&cBuffer[0], &a[0], 196U * sizeof(real_T));
          mtimes(b_a, cBuffer, a);
        } else if (!(b == 1.0)) {
          if (b == -1.0) {
            memcpy(&b_a[0], &a[0], 196U * sizeof(real_T));
            inv(b_a, a);
          } else if (b == -2.0) {
            mtimes(a, a, b_a);
            inv(b_a, a);
          } else {
            boolean_T firstmult;
            firstmult = false;
            for (b_n = 0; b_n < 196; b_n++) {
              if (firstmult || muDoubleScalarIsNaN(a[b_n])) {
                firstmult = true;
              }
            }
            if (firstmult) {
              for (b_n = 0; b_n < 196; b_n++) {
                a[b_n] = rtNaN;
              }
            } else {
              memset(&a[0], 0, 196U * sizeof(real_T));
              for (b_n = 0; b_n < 14; b_n++) {
                a[b_n + 14 * b_n] = 1.0;
              }
            }
          }
        }
      } else {
        boolean_T aBufferInUse;
        boolean_T first;
        boolean_T firstmult;
        first = true;
        aBufferInUse = false;
        firstmult = ((nbitson & 1) != 0);
        if ((firstmult && (b < 0.0)) || ((!firstmult) && (b >= 0.0))) {
          firstmult = true;
        } else {
          firstmult = false;
        }
        for (b_n = 0; b_n <= nb; b_n++) {
          if ((n & 1) != 0) {
            if (first) {
              first = false;
              if (firstmult) {
                if (aBufferInUse) {
                  memcpy(&cBuffer[0], &aBuffer[0], 196U * sizeof(real_T));
                } else {
                  memcpy(&cBuffer[0], &b_a[0], 196U * sizeof(real_T));
                }
              } else if (aBufferInUse) {
                memcpy(&a[0], &aBuffer[0], 196U * sizeof(real_T));
              } else {
                memcpy(&a[0], &b_a[0], 196U * sizeof(real_T));
              }
            } else {
              if (aBufferInUse) {
                if (firstmult) {
                  mtimes(cBuffer, aBuffer, a);
                } else {
                  mtimes(a, aBuffer, cBuffer);
                }
              } else if (firstmult) {
                mtimes(cBuffer, b_a, a);
              } else {
                mtimes(a, b_a, cBuffer);
              }
              firstmult = !firstmult;
            }
          }
          n >>= 1;
          if (aBufferInUse) {
            mtimes(aBuffer, aBuffer, b_a);
          } else {
            mtimes(b_a, b_a, aBuffer);
          }
          aBufferInUse = !aBufferInUse;
        }
        if (first) {
          if (b < 0.0) {
            if (aBufferInUse) {
              inv(aBuffer, a);
            } else {
              inv(b_a, a);
            }
          } else if (aBufferInUse) {
            memcpy(&a[0], &aBuffer[0], 196U * sizeof(real_T));
          } else {
            memcpy(&a[0], &b_a[0], 196U * sizeof(real_T));
          }
        } else if (b < 0.0) {
          if (aBufferInUse) {
            mtimes(a, aBuffer, cBuffer);
          } else {
            mtimes(a, b_a, cBuffer);
          }
          inv(cBuffer, a);
        } else if (aBufferInUse) {
          mtimes(cBuffer, aBuffer, a);
        } else {
          mtimes(cBuffer, b_a, a);
        }
      }
    } else {
      memcpy(&b_a[0], &a[0], 196U * sizeof(real_T));
      if (!muDoubleScalarIsInf(b)) {
        boolean_T firstmult;
        firstmult = true;
        real_T ed2;
        int32_T exitg1;
        do {
          exitg1 = 0;
          ed2 = muDoubleScalarFloor(e / 2.0);
          if (2.0 * ed2 != e) {
            if (firstmult) {
              memcpy(&a[0], &b_a[0], 196U * sizeof(real_T));
              firstmult = false;
            } else {
              memcpy(&cBuffer[0], &a[0], 196U * sizeof(real_T));
              mtimes(cBuffer, b_a, a);
            }
          }
          if (ed2 == 0.0) {
            exitg1 = 1;
          } else {
            e = ed2;
            memcpy(&cBuffer[0], &b_a[0], 196U * sizeof(real_T));
            mtimes(cBuffer, cBuffer, b_a);
          }
        } while (exitg1 == 0);
        if (b < 0.0) {
          memcpy(&b_a[0], &a[0], 196U * sizeof(real_T));
          inv(b_a, a);
        }
      } else {
        for (b_n = 0; b_n < 196; b_n++) {
          a[b_n] = rtNaN;
        }
      }
    }
  }
}

/* End of code generation (mpower.c) */
