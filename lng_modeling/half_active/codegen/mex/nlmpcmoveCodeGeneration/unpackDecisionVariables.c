/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * unpackDecisionVariables.c
 *
 * Code generation for function 'unpackDecisionVariables'
 *
 */

/* Include files */
#include "unpackDecisionVariables.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void ab_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[984]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[943])));
  r = _mm_loadu_pd(&Z[986]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[945])));
  e[0] = Z[988];
  e[1] = Z[989];
  e[2] = Z[990];
  memcpy(&d[0], &Z[991], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[999], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1017], 8U * sizeof(real_T));
}

void ac_unpackDecisionVariables(const real_T Z[2071], const real_T lastMV[4],
                                real_T u[4], real_T mv[4], real_T dmv[4],
                                real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[0]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&lastMV[0])));
  r = _mm_loadu_pd(&Z[2]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&lastMV[2])));
  e[0] = Z[4];
  e[1] = Z[5];
  e[2] = Z[6];
}

void ad_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1025]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[984])));
  r = _mm_loadu_pd(&Z[1027]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[986])));
  e[0] = Z[1029];
  e[1] = Z[1030];
  e[2] = Z[1031];
}

void ae_unpackDecisionVariables(const real_T Z[2071], real_T dmv[4],
                                real_T e[3], real_T muineq[18])
{
  __m128d r;
  r = _mm_set1_pd(-1.0);
  _mm_storeu_pd(&dmv[0], _mm_mul_pd(_mm_loadu_pd(&Z[2009]), r));
  _mm_storeu_pd(&dmv[2], _mm_mul_pd(_mm_loadu_pd(&Z[2011]), r));
  e[0] = Z[2050];
  e[1] = Z[2051];
  e[2] = Z[2052];
  memcpy(&muineq[0], &Z[2053], 18U * sizeof(real_T));
}

void b_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1968]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1927])));
  r = _mm_loadu_pd(&Z[1970]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1929])));
  e[0] = Z[1972];
  e[1] = Z[1973];
  e[2] = Z[1974];
  memcpy(&d[0], &Z[1975], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1983], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[2001], 8U * sizeof(real_T));
}

void bb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[943]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[902])));
  r = _mm_loadu_pd(&Z[945]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[904])));
  e[0] = Z[947];
  e[1] = Z[948];
  e[2] = Z[949];
  memcpy(&d[0], &Z[950], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[958], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[976], 8U * sizeof(real_T));
}

void bc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[41]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[0])));
  r = _mm_loadu_pd(&Z[43]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[2])));
  e[0] = Z[45];
  e[1] = Z[46];
  e[2] = Z[47];
}

void bd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1066]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1025])));
  r = _mm_loadu_pd(&Z[1068]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1027])));
  e[0] = Z[1070];
  e[1] = Z[1071];
  e[2] = Z[1072];
}

void c_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1927]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1886])));
  r = _mm_loadu_pd(&Z[1929]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1888])));
  e[0] = Z[1931];
  e[1] = Z[1932];
  e[2] = Z[1933];
  memcpy(&d[0], &Z[1934], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1942], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1960], 8U * sizeof(real_T));
}

void cb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[902]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[861])));
  r = _mm_loadu_pd(&Z[904]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[863])));
  e[0] = Z[906];
  e[1] = Z[907];
  e[2] = Z[908];
  memcpy(&d[0], &Z[909], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[917], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[935], 8U * sizeof(real_T));
}

void cc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[82]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[41])));
  r = _mm_loadu_pd(&Z[84]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[43])));
  e[0] = Z[86];
  e[1] = Z[87];
  e[2] = Z[88];
}

void cd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1107]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1066])));
  r = _mm_loadu_pd(&Z[1109]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1068])));
  e[0] = Z[1111];
  e[1] = Z[1112];
  e[2] = Z[1113];
}

void d_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1886]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1845])));
  r = _mm_loadu_pd(&Z[1888]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1847])));
  e[0] = Z[1890];
  e[1] = Z[1891];
  e[2] = Z[1892];
  memcpy(&d[0], &Z[1893], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1901], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1919], 8U * sizeof(real_T));
}

void db_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[861]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[820])));
  r = _mm_loadu_pd(&Z[863]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[822])));
  e[0] = Z[865];
  e[1] = Z[866];
  e[2] = Z[867];
  memcpy(&d[0], &Z[868], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[876], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[894], 8U * sizeof(real_T));
}

void dc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[123]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[82])));
  r = _mm_loadu_pd(&Z[125]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[84])));
  e[0] = Z[127];
  e[1] = Z[128];
  e[2] = Z[129];
}

void dd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1148]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1107])));
  r = _mm_loadu_pd(&Z[1150]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1109])));
  e[0] = Z[1152];
  e[1] = Z[1153];
  e[2] = Z[1154];
}

void e_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1845]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1804])));
  r = _mm_loadu_pd(&Z[1847]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1806])));
  e[0] = Z[1849];
  e[1] = Z[1850];
  e[2] = Z[1851];
  memcpy(&d[0], &Z[1852], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1860], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1878], 8U * sizeof(real_T));
}

void eb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[820]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[779])));
  r = _mm_loadu_pd(&Z[822]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[781])));
  e[0] = Z[824];
  e[1] = Z[825];
  e[2] = Z[826];
  memcpy(&d[0], &Z[827], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[835], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[853], 8U * sizeof(real_T));
}

void ec_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[164]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[123])));
  r = _mm_loadu_pd(&Z[166]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[125])));
  e[0] = Z[168];
  e[1] = Z[169];
  e[2] = Z[170];
}

void ed_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1189]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1148])));
  r = _mm_loadu_pd(&Z[1191]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1150])));
  e[0] = Z[1193];
  e[1] = Z[1194];
  e[2] = Z[1195];
}

void f_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1804]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1763])));
  r = _mm_loadu_pd(&Z[1806]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1765])));
  e[0] = Z[1808];
  e[1] = Z[1809];
  e[2] = Z[1810];
  memcpy(&d[0], &Z[1811], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1819], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1837], 8U * sizeof(real_T));
}

void fb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[779]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[738])));
  r = _mm_loadu_pd(&Z[781]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[740])));
  e[0] = Z[783];
  e[1] = Z[784];
  e[2] = Z[785];
  memcpy(&d[0], &Z[786], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[794], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[812], 8U * sizeof(real_T));
}

void fc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[205]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[164])));
  r = _mm_loadu_pd(&Z[207]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[166])));
  e[0] = Z[209];
  e[1] = Z[210];
  e[2] = Z[211];
}

void fd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1230]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1189])));
  r = _mm_loadu_pd(&Z[1232]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1191])));
  e[0] = Z[1234];
  e[1] = Z[1235];
  e[2] = Z[1236];
}

void g_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1763]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1722])));
  r = _mm_loadu_pd(&Z[1765]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1724])));
  e[0] = Z[1767];
  e[1] = Z[1768];
  e[2] = Z[1769];
  memcpy(&d[0], &Z[1770], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1778], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1796], 8U * sizeof(real_T));
}

void gb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[738]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[697])));
  r = _mm_loadu_pd(&Z[740]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[699])));
  e[0] = Z[742];
  e[1] = Z[743];
  e[2] = Z[744];
  memcpy(&d[0], &Z[745], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[753], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[771], 8U * sizeof(real_T));
}

void gc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[246]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[205])));
  r = _mm_loadu_pd(&Z[248]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[207])));
  e[0] = Z[250];
  e[1] = Z[251];
  e[2] = Z[252];
}

void gd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1271]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1230])));
  r = _mm_loadu_pd(&Z[1273]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1232])));
  e[0] = Z[1275];
  e[1] = Z[1276];
  e[2] = Z[1277];
}

void h_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1722]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1681])));
  r = _mm_loadu_pd(&Z[1724]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1683])));
  e[0] = Z[1726];
  e[1] = Z[1727];
  e[2] = Z[1728];
  memcpy(&d[0], &Z[1729], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1737], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1755], 8U * sizeof(real_T));
}

void hb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[697]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[656])));
  r = _mm_loadu_pd(&Z[699]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[658])));
  e[0] = Z[701];
  e[1] = Z[702];
  e[2] = Z[703];
  memcpy(&d[0], &Z[704], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[712], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[730], 8U * sizeof(real_T));
}

void hc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[287]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[246])));
  r = _mm_loadu_pd(&Z[289]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[248])));
  e[0] = Z[291];
  e[1] = Z[292];
  e[2] = Z[293];
}

void hd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1312]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1271])));
  r = _mm_loadu_pd(&Z[1314]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1273])));
  e[0] = Z[1316];
  e[1] = Z[1317];
  e[2] = Z[1318];
}

void i_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1681]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1640])));
  r = _mm_loadu_pd(&Z[1683]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1642])));
  e[0] = Z[1685];
  e[1] = Z[1686];
  e[2] = Z[1687];
  memcpy(&d[0], &Z[1688], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1696], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1714], 8U * sizeof(real_T));
}

void ib_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[656]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[615])));
  r = _mm_loadu_pd(&Z[658]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[617])));
  e[0] = Z[660];
  e[1] = Z[661];
  e[2] = Z[662];
  memcpy(&d[0], &Z[663], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[671], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[689], 8U * sizeof(real_T));
}

void ic_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[328]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[287])));
  r = _mm_loadu_pd(&Z[330]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[289])));
  e[0] = Z[332];
  e[1] = Z[333];
  e[2] = Z[334];
}

void id_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1353]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1312])));
  r = _mm_loadu_pd(&Z[1355]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1314])));
  e[0] = Z[1357];
  e[1] = Z[1358];
  e[2] = Z[1359];
}

void j_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1640]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1599])));
  r = _mm_loadu_pd(&Z[1642]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1601])));
  e[0] = Z[1644];
  e[1] = Z[1645];
  e[2] = Z[1646];
  memcpy(&d[0], &Z[1647], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1655], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1673], 8U * sizeof(real_T));
}

void jb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[615]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[574])));
  r = _mm_loadu_pd(&Z[617]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[576])));
  e[0] = Z[619];
  e[1] = Z[620];
  e[2] = Z[621];
  memcpy(&d[0], &Z[622], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[630], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[648], 8U * sizeof(real_T));
}

void jc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[369]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[328])));
  r = _mm_loadu_pd(&Z[371]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[330])));
  e[0] = Z[373];
  e[1] = Z[374];
  e[2] = Z[375];
}

void jd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1394]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1353])));
  r = _mm_loadu_pd(&Z[1396]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1355])));
  e[0] = Z[1398];
  e[1] = Z[1399];
  e[2] = Z[1400];
}

void k_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1599]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1558])));
  r = _mm_loadu_pd(&Z[1601]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1560])));
  e[0] = Z[1603];
  e[1] = Z[1604];
  e[2] = Z[1605];
  memcpy(&d[0], &Z[1606], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1614], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1632], 8U * sizeof(real_T));
}

void kb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[574]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[533])));
  r = _mm_loadu_pd(&Z[576]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[535])));
  e[0] = Z[578];
  e[1] = Z[579];
  e[2] = Z[580];
  memcpy(&d[0], &Z[581], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[589], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[607], 8U * sizeof(real_T));
}

void kc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[410]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[369])));
  r = _mm_loadu_pd(&Z[412]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[371])));
  e[0] = Z[414];
  e[1] = Z[415];
  e[2] = Z[416];
}

void kd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1435]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1394])));
  r = _mm_loadu_pd(&Z[1437]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1396])));
  e[0] = Z[1439];
  e[1] = Z[1440];
  e[2] = Z[1441];
}

void l_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1558]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1517])));
  r = _mm_loadu_pd(&Z[1560]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1519])));
  e[0] = Z[1562];
  e[1] = Z[1563];
  e[2] = Z[1564];
  memcpy(&d[0], &Z[1565], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1573], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1591], 8U * sizeof(real_T));
}

void lb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[533]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[492])));
  r = _mm_loadu_pd(&Z[535]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[494])));
  e[0] = Z[537];
  e[1] = Z[538];
  e[2] = Z[539];
  memcpy(&d[0], &Z[540], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[548], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[566], 8U * sizeof(real_T));
}

void lc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[451]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[410])));
  r = _mm_loadu_pd(&Z[453]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[412])));
  e[0] = Z[455];
  e[1] = Z[456];
  e[2] = Z[457];
}

void ld_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1476]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1435])));
  r = _mm_loadu_pd(&Z[1478]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1437])));
  e[0] = Z[1480];
  e[1] = Z[1481];
  e[2] = Z[1482];
}

void m_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1517]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1476])));
  r = _mm_loadu_pd(&Z[1519]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1478])));
  e[0] = Z[1521];
  e[1] = Z[1522];
  e[2] = Z[1523];
  memcpy(&d[0], &Z[1524], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1532], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1550], 8U * sizeof(real_T));
}

void mb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[492]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[451])));
  r = _mm_loadu_pd(&Z[494]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[453])));
  e[0] = Z[496];
  e[1] = Z[497];
  e[2] = Z[498];
  memcpy(&d[0], &Z[499], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[507], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[525], 8U * sizeof(real_T));
}

void mc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[492]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[451])));
  r = _mm_loadu_pd(&Z[494]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[453])));
  e[0] = Z[496];
  e[1] = Z[497];
  e[2] = Z[498];
}

void md_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1517]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1476])));
  r = _mm_loadu_pd(&Z[1519]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1478])));
  e[0] = Z[1521];
  e[1] = Z[1522];
  e[2] = Z[1523];
}

void n_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1476]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1435])));
  r = _mm_loadu_pd(&Z[1478]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1437])));
  e[0] = Z[1480];
  e[1] = Z[1481];
  e[2] = Z[1482];
  memcpy(&d[0], &Z[1483], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1491], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1509], 8U * sizeof(real_T));
}

void nb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[451]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[410])));
  r = _mm_loadu_pd(&Z[453]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[412])));
  e[0] = Z[455];
  e[1] = Z[456];
  e[2] = Z[457];
  memcpy(&d[0], &Z[458], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[466], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[484], 8U * sizeof(real_T));
}

void nc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[533]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[492])));
  r = _mm_loadu_pd(&Z[535]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[494])));
  e[0] = Z[537];
  e[1] = Z[538];
  e[2] = Z[539];
}

void nd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1558]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1517])));
  r = _mm_loadu_pd(&Z[1560]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1519])));
  e[0] = Z[1562];
  e[1] = Z[1563];
  e[2] = Z[1564];
}

void o_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1435]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1394])));
  r = _mm_loadu_pd(&Z[1437]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1396])));
  e[0] = Z[1439];
  e[1] = Z[1440];
  e[2] = Z[1441];
  memcpy(&d[0], &Z[1442], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1450], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1468], 8U * sizeof(real_T));
}

void ob_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[410]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[369])));
  r = _mm_loadu_pd(&Z[412]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[371])));
  e[0] = Z[414];
  e[1] = Z[415];
  e[2] = Z[416];
  memcpy(&d[0], &Z[417], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[425], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[443], 8U * sizeof(real_T));
}

void oc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[574]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[533])));
  r = _mm_loadu_pd(&Z[576]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[535])));
  e[0] = Z[578];
  e[1] = Z[579];
  e[2] = Z[580];
}

void od_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1599]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1558])));
  r = _mm_loadu_pd(&Z[1601]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1560])));
  e[0] = Z[1603];
  e[1] = Z[1604];
  e[2] = Z[1605];
}

void p_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1394]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1353])));
  r = _mm_loadu_pd(&Z[1396]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1355])));
  e[0] = Z[1398];
  e[1] = Z[1399];
  e[2] = Z[1400];
  memcpy(&d[0], &Z[1401], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1409], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1427], 8U * sizeof(real_T));
}

void pb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[369]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[328])));
  r = _mm_loadu_pd(&Z[371]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[330])));
  e[0] = Z[373];
  e[1] = Z[374];
  e[2] = Z[375];
  memcpy(&d[0], &Z[376], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[384], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[402], 8U * sizeof(real_T));
}

void pc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[615]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[574])));
  r = _mm_loadu_pd(&Z[617]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[576])));
  e[0] = Z[619];
  e[1] = Z[620];
  e[2] = Z[621];
}

void pd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1640]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1599])));
  r = _mm_loadu_pd(&Z[1642]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1601])));
  e[0] = Z[1644];
  e[1] = Z[1645];
  e[2] = Z[1646];
}

void q_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1353]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1312])));
  r = _mm_loadu_pd(&Z[1355]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1314])));
  e[0] = Z[1357];
  e[1] = Z[1358];
  e[2] = Z[1359];
  memcpy(&d[0], &Z[1360], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1368], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1386], 8U * sizeof(real_T));
}

void qb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[328]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[287])));
  r = _mm_loadu_pd(&Z[330]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[289])));
  e[0] = Z[332];
  e[1] = Z[333];
  e[2] = Z[334];
  memcpy(&d[0], &Z[335], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[343], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[361], 8U * sizeof(real_T));
}

void qc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[656]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[615])));
  r = _mm_loadu_pd(&Z[658]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[617])));
  e[0] = Z[660];
  e[1] = Z[661];
  e[2] = Z[662];
}

void qd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1681]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1640])));
  r = _mm_loadu_pd(&Z[1683]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1642])));
  e[0] = Z[1685];
  e[1] = Z[1686];
  e[2] = Z[1687];
}

void r_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1312]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1271])));
  r = _mm_loadu_pd(&Z[1314]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1273])));
  e[0] = Z[1316];
  e[1] = Z[1317];
  e[2] = Z[1318];
  memcpy(&d[0], &Z[1319], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1327], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1345], 8U * sizeof(real_T));
}

void rb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[287]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[246])));
  r = _mm_loadu_pd(&Z[289]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[248])));
  e[0] = Z[291];
  e[1] = Z[292];
  e[2] = Z[293];
  memcpy(&d[0], &Z[294], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[302], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[320], 8U * sizeof(real_T));
}

void rc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[697]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[656])));
  r = _mm_loadu_pd(&Z[699]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[658])));
  e[0] = Z[701];
  e[1] = Z[702];
  e[2] = Z[703];
}

void rd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1722]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1681])));
  r = _mm_loadu_pd(&Z[1724]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1683])));
  e[0] = Z[1726];
  e[1] = Z[1727];
  e[2] = Z[1728];
}

void s_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1271]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1230])));
  r = _mm_loadu_pd(&Z[1273]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1232])));
  e[0] = Z[1275];
  e[1] = Z[1276];
  e[2] = Z[1277];
  memcpy(&d[0], &Z[1278], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1286], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1304], 8U * sizeof(real_T));
}

void sb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[246]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[205])));
  r = _mm_loadu_pd(&Z[248]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[207])));
  e[0] = Z[250];
  e[1] = Z[251];
  e[2] = Z[252];
  memcpy(&d[0], &Z[253], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[261], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[279], 8U * sizeof(real_T));
}

void sc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[738]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[697])));
  r = _mm_loadu_pd(&Z[740]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[699])));
  e[0] = Z[742];
  e[1] = Z[743];
  e[2] = Z[744];
}

void sd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1763]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1722])));
  r = _mm_loadu_pd(&Z[1765]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1724])));
  e[0] = Z[1767];
  e[1] = Z[1768];
  e[2] = Z[1769];
}

void t_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1230]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1189])));
  r = _mm_loadu_pd(&Z[1232]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1191])));
  e[0] = Z[1234];
  e[1] = Z[1235];
  e[2] = Z[1236];
  memcpy(&d[0], &Z[1237], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1245], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1263], 8U * sizeof(real_T));
}

void tb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[205]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[164])));
  r = _mm_loadu_pd(&Z[207]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[166])));
  e[0] = Z[209];
  e[1] = Z[210];
  e[2] = Z[211];
  memcpy(&d[0], &Z[212], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[220], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[238], 8U * sizeof(real_T));
}

void tc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[779]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[738])));
  r = _mm_loadu_pd(&Z[781]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[740])));
  e[0] = Z[783];
  e[1] = Z[784];
  e[2] = Z[785];
}

void td_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1804]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1763])));
  r = _mm_loadu_pd(&Z[1806]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1765])));
  e[0] = Z[1808];
  e[1] = Z[1809];
  e[2] = Z[1810];
}

void u_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1189]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1148])));
  r = _mm_loadu_pd(&Z[1191]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1150])));
  e[0] = Z[1193];
  e[1] = Z[1194];
  e[2] = Z[1195];
  memcpy(&d[0], &Z[1196], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1204], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1222], 8U * sizeof(real_T));
}

void ub_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[164]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[123])));
  r = _mm_loadu_pd(&Z[166]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[125])));
  e[0] = Z[168];
  e[1] = Z[169];
  e[2] = Z[170];
  memcpy(&d[0], &Z[171], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[179], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[197], 8U * sizeof(real_T));
}

void uc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[820]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[779])));
  r = _mm_loadu_pd(&Z[822]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[781])));
  e[0] = Z[824];
  e[1] = Z[825];
  e[2] = Z[826];
}

void ud_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1845]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1804])));
  r = _mm_loadu_pd(&Z[1847]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1806])));
  e[0] = Z[1849];
  e[1] = Z[1850];
  e[2] = Z[1851];
}

void unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                             real_T dmv[4], real_T e[3], real_T d[8],
                             real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[2009]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1968])));
  r = _mm_loadu_pd(&Z[2011]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1970])));
  e[0] = Z[2013];
  e[1] = Z[2014];
  e[2] = Z[2015];
  memcpy(&d[0], &Z[2016], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[2024], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[2042], 8U * sizeof(real_T));
}

void v_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1148]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1107])));
  r = _mm_loadu_pd(&Z[1150]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1109])));
  e[0] = Z[1152];
  e[1] = Z[1153];
  e[2] = Z[1154];
  memcpy(&d[0], &Z[1155], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1163], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1181], 8U * sizeof(real_T));
}

void vb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[123]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[82])));
  r = _mm_loadu_pd(&Z[125]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[84])));
  e[0] = Z[127];
  e[1] = Z[128];
  e[2] = Z[129];
  memcpy(&d[0], &Z[130], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[138], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[156], 8U * sizeof(real_T));
}

void vc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[861]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[820])));
  r = _mm_loadu_pd(&Z[863]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[822])));
  e[0] = Z[865];
  e[1] = Z[866];
  e[2] = Z[867];
}

void vd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1886]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1845])));
  r = _mm_loadu_pd(&Z[1888]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1847])));
  e[0] = Z[1890];
  e[1] = Z[1891];
  e[2] = Z[1892];
}

void w_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1107]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1066])));
  r = _mm_loadu_pd(&Z[1109]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1068])));
  e[0] = Z[1111];
  e[1] = Z[1112];
  e[2] = Z[1113];
  memcpy(&d[0], &Z[1114], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1122], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1140], 8U * sizeof(real_T));
}

void wb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[82]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[41])));
  r = _mm_loadu_pd(&Z[84]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[43])));
  e[0] = Z[86];
  e[1] = Z[87];
  e[2] = Z[88];
  memcpy(&d[0], &Z[89], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[97], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[115], 8U * sizeof(real_T));
}

void wc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[902]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[861])));
  r = _mm_loadu_pd(&Z[904]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[863])));
  e[0] = Z[906];
  e[1] = Z[907];
  e[2] = Z[908];
}

void wd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1927]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1886])));
  r = _mm_loadu_pd(&Z[1929]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1888])));
  e[0] = Z[1931];
  e[1] = Z[1932];
  e[2] = Z[1933];
}

void x_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1066]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1025])));
  r = _mm_loadu_pd(&Z[1068]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1027])));
  e[0] = Z[1070];
  e[1] = Z[1071];
  e[2] = Z[1072];
  memcpy(&d[0], &Z[1073], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1081], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1099], 8U * sizeof(real_T));
}

void xb_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3], real_T d[8],
                                real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[41]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[0])));
  r = _mm_loadu_pd(&Z[43]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[2])));
  e[0] = Z[45];
  e[1] = Z[46];
  e[2] = Z[47];
  memcpy(&d[0], &Z[48], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[56], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[74], 8U * sizeof(real_T));
}

void xc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[943]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[902])));
  r = _mm_loadu_pd(&Z[945]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[904])));
  e[0] = Z[947];
  e[1] = Z[948];
  e[2] = Z[949];
}

void xd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1968]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1927])));
  r = _mm_loadu_pd(&Z[1970]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1929])));
  e[0] = Z[1972];
  e[1] = Z[1973];
  e[2] = Z[1974];
}

void y_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                               real_T dmv[4], real_T e[3], real_T d[8],
                               real_T muineq[18], real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[1025]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[984])));
  r = _mm_loadu_pd(&Z[1027]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[986])));
  e[0] = Z[1029];
  e[1] = Z[1030];
  e[2] = Z[1031];
  memcpy(&d[0], &Z[1032], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[1040], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[1058], 8U * sizeof(real_T));
}

void yb_unpackDecisionVariables(const real_T Z[2071], const real_T lastMV[4],
                                real_T u[4], real_T mv[4], real_T dmv[4],
                                real_T e[3], real_T d[8], real_T muineq[18],
                                real_T mudummy[8])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[0]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&lastMV[0])));
  r = _mm_loadu_pd(&Z[2]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&lastMV[2])));
  e[0] = Z[4];
  e[1] = Z[5];
  e[2] = Z[6];
  memcpy(&d[0], &Z[7], 8U * sizeof(real_T));
  memcpy(&muineq[0], &Z[15], 18U * sizeof(real_T));
  memcpy(&mudummy[0], &Z[33], 8U * sizeof(real_T));
}

void yc_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[984]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[943])));
  r = _mm_loadu_pd(&Z[986]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[945])));
  e[0] = Z[988];
  e[1] = Z[989];
  e[2] = Z[990];
}

void yd_unpackDecisionVariables(const real_T Z[2071], real_T u[4], real_T mv[4],
                                real_T dmv[4], real_T e[3])
{
  __m128d r;
  r = _mm_loadu_pd(&Z[2009]);
  _mm_storeu_pd(&mv[0], r);
  _mm_storeu_pd(&u[0], r);
  _mm_storeu_pd(&dmv[0], _mm_sub_pd(r, _mm_loadu_pd(&Z[1968])));
  r = _mm_loadu_pd(&Z[2011]);
  _mm_storeu_pd(&mv[2], r);
  _mm_storeu_pd(&u[2], r);
  _mm_storeu_pd(&dmv[2], _mm_sub_pd(r, _mm_loadu_pd(&Z[1970])));
  e[0] = Z[2013];
  e[1] = Z[2014];
  e[2] = Z[2015];
}

/* End of code generation (unpackDecisionVariables.c) */
