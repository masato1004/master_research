#include "rtwtypes.h"
#include "rollingMoment_xZd8KFqg.h"
#include <emmintrin.h>
#include "mwmathutil.h"

void rollingMoment_xZd8KFqg(const real_T Fx[4], const real_T Vcx[4], const
  real_T Fz[4], const real_T press[4], const real_T b_gamma[4], real_T Vo,
  real_T Ro, real_T Fzo, real_T pio, real_T b_QSY1, real_T b_QSY2, real_T b_QSY3,
  real_T b_QSY4, real_T b_QSY5, real_T b_QSY6, real_T b_QSY7, real_T b_QSY8,
  const real_T lam_My[4], real_T My[4])
{
  __m128d tmp_p;
  real_T tmp[2];
  real_T Ro_p;
  Ro_p = -Ro * Fzo;
  tmp_p = _mm_set_pd(Vo, Fzo);
  _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(Vcx[0], Fz[0]), tmp_p));
  My[0] = ((((Fx[0] / Fzo * b_QSY2 + b_QSY1) + b_QSY3 * muDoubleScalarAbs(tmp[1]))
            + b_QSY4 * muDoubleScalarPower(tmp[1], 4.0)) + (b_QSY6 * tmp[0] +
            b_QSY5) * b_gamma[0] * b_gamma[0]) * (Ro_p * lam_My[0]) *
    (muDoubleScalarPower(muDoubleScalarMax(press[0] / pio, 0.05), b_QSY8) *
     muDoubleScalarPower(muDoubleScalarMax(tmp[0], 0.05), b_QSY7));
  _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(Vcx[1], Fz[1]), tmp_p));
  My[1] = ((((Fx[1] / Fzo * b_QSY2 + b_QSY1) + b_QSY3 * muDoubleScalarAbs(tmp[1]))
            + b_QSY4 * muDoubleScalarPower(tmp[1], 4.0)) + (b_QSY6 * tmp[0] +
            b_QSY5) * b_gamma[1] * b_gamma[1]) * (Ro_p * lam_My[1]) *
    (muDoubleScalarPower(muDoubleScalarMax(press[1] / pio, 0.05), b_QSY8) *
     muDoubleScalarPower(muDoubleScalarMax(tmp[0], 0.05), b_QSY7));
  _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(Vcx[2], Fz[2]), tmp_p));
  My[2] = ((((Fx[2] / Fzo * b_QSY2 + b_QSY1) + b_QSY3 * muDoubleScalarAbs(tmp[1]))
            + b_QSY4 * muDoubleScalarPower(tmp[1], 4.0)) + (b_QSY6 * tmp[0] +
            b_QSY5) * b_gamma[2] * b_gamma[2]) * (Ro_p * lam_My[2]) *
    (muDoubleScalarPower(muDoubleScalarMax(press[2] / pio, 0.05), b_QSY8) *
     muDoubleScalarPower(muDoubleScalarMax(tmp[0], 0.05), b_QSY7));
  _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(Vcx[3], Fz[3]), tmp_p));
  My[3] = ((((Fx[3] / Fzo * b_QSY2 + b_QSY1) + b_QSY3 * muDoubleScalarAbs(tmp[1]))
            + b_QSY4 * muDoubleScalarPower(tmp[1], 4.0)) + (b_QSY6 * tmp[0] +
            b_QSY5) * b_gamma[3] * b_gamma[3]) * (Ro_p * lam_My[3]) *
    (muDoubleScalarPower(muDoubleScalarMax(press[3] / pio, 0.05), b_QSY8) *
     muDoubleScalarPower(muDoubleScalarMax(tmp[0], 0.05), b_QSY7));
}
