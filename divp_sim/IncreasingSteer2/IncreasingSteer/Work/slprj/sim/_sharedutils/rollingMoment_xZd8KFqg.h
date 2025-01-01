#ifndef rollingMoment_xZd8KFqg_h_
#define rollingMoment_xZd8KFqg_h_
#include "rtwtypes.h"
#include "multiword_types.h"

extern void rollingMoment_xZd8KFqg(const real_T Fx[4], const real_T Vcx[4],
  const real_T Fz[4], const real_T press[4], const real_T b_gamma[4], real_T Vo,
  real_T Ro, real_T Fzo, real_T pio, real_T b_QSY1, real_T b_QSY2, real_T b_QSY3,
  real_T b_QSY4, real_T b_QSY5, real_T b_QSY6, real_T b_QSY7, real_T b_QSY8,
  const real_T lam_My[4], real_T My[4]);

#endif
