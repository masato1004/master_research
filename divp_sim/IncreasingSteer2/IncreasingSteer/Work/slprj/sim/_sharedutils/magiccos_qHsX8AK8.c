#include "rtwtypes.h"
#include "magiccos_qHsX8AK8.h"
#include "mwmathutil.h"

void magiccos_qHsX8AK8(const real_T D[4], const real_T C[4], const real_T B[4],
  const real_T u[4], real_T y[4])
{
  real_T y_p;
  y_p = B[0] * u[0];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * 0.0;
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C[0];
  y_p = muDoubleScalarCos(y_p);
  y[0] = D[0] * y_p;
  y_p = B[1] * u[1];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * 0.0;
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C[1];
  y_p = muDoubleScalarCos(y_p);
  y[1] = D[1] * y_p;
  y_p = B[2] * u[2];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * 0.0;
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C[2];
  y_p = muDoubleScalarCos(y_p);
  y[2] = D[2] * y_p;
  y_p = B[3] * u[3];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * 0.0;
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C[3];
  y_p = muDoubleScalarCos(y_p);
  y[3] = D[3] * y_p;
}
