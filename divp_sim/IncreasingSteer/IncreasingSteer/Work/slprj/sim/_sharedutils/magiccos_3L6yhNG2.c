#include "rtwtypes.h"
#include "magiccos_3L6yhNG2.h"
#include "mwmathutil.h"

void magiccos_3L6yhNG2(real_T C, const real_T B[4], const real_T E[4], real_T y
  [4])
{
  y[0] = muDoubleScalarCos(muDoubleScalarAtan(B[0] * 0.0 - (B[0] * 0.0 -
    muDoubleScalarAtan(B[0] * 0.0)) * E[0]) * C);
  y[1] = muDoubleScalarCos(muDoubleScalarAtan(B[1] * 0.0 - (B[1] * 0.0 -
    muDoubleScalarAtan(B[1] * 0.0)) * E[1]) * C);
  y[2] = muDoubleScalarCos(muDoubleScalarAtan(B[2] * 0.0 - (B[2] * 0.0 -
    muDoubleScalarAtan(B[2] * 0.0)) * E[2]) * C);
  y[3] = muDoubleScalarCos(muDoubleScalarAtan(B[3] * 0.0 - (B[3] * 0.0 -
    muDoubleScalarAtan(B[3] * 0.0)) * E[3]) * C);
}
