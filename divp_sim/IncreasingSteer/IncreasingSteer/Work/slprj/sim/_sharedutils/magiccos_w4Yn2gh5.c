#include "rtwtypes.h"
#include "magiccos_w4Yn2gh5.h"
#include "mwmathutil.h"

void magiccos_w4Yn2gh5(real_T C, const real_T B[4], const real_T E[4], const
  real_T u[4], real_T y[4])
{
  real_T y_p;
  y_p = B[0] * u[0];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * E[0];
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C;
  y[0] = muDoubleScalarCos(y_p);
  y_p = B[1] * u[1];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * E[1];
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C;
  y[1] = muDoubleScalarCos(y_p);
  y_p = B[2] * u[2];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * E[2];
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C;
  y[2] = muDoubleScalarCos(y_p);
  y_p = B[3] * u[3];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * E[3];
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C;
  y[3] = muDoubleScalarCos(y_p);
}
