#include "rtwtypes.h"
#include "sumColumnB_VwUfAZ2T.h"

real_T sumColumnB_VwUfAZ2T(const real_T x[24], int32_T col)
{
  real_T y;
  int32_T i0;
  int32_T k;
  i0 = (col - 1) << 3;
  y = x[i0];
  for (k = 0; k < 7; k++) {
    y += x[(i0 + k) + 1];
  }

  return y;
}
