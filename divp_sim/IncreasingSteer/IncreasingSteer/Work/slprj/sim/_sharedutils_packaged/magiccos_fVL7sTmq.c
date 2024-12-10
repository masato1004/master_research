#include "rtwtypes.h"
#include "magiccos_fVL7sTmq.h"
#include "mwmathutil.h"

void magiccos_fVL7sTmq(const real_T D[4], real_T C, const real_T B[4], const
  real_T E[4], const real_T u[4], real_T y[4])
{
  real_T y_p;
  y_p = B[0] * u[0];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * E[0];
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C;
  y_p = muDoubleScalarCos(y_p);
  y[0] = D[0] * y_p;
  y_p = B[1] * u[1];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * E[1];
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C;
  y_p = muDoubleScalarCos(y_p);
  y[1] = D[1] * y_p;
  y_p = B[2] * u[2];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * E[2];
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C;
  y_p = muDoubleScalarCos(y_p);
  y[2] = D[2] * y_p;
  y_p = B[3] * u[3];
  y_p -= (y_p - muDoubleScalarAtan(y_p)) * E[3];
  y_p = muDoubleScalarAtan(y_p);
  y_p *= C;
  y_p = muDoubleScalarCos(y_p);
  y[3] = D[3] * y_p;
}
