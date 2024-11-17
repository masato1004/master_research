#ifndef PassVeh14DOF_h_
#define PassVeh14DOF_h_
#ifndef PassVeh14DOF_COMMON_INCLUDES_
#define PassVeh14DOF_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "sf_runtime/sfc_sdi.h"
#endif
#include "PassVeh14DOF_types.h"
#include <string.h>
#include "model_reference_types.h"
#include "rtw_modelmap_simtarget.h"
#include <stddef.h>
#include "zero_crossing_types.h"
typedef struct { real_T fxfndzz0d1 ; real_T p0unzg13wo [ 3 ] ; } eny4tltuc5 ;
typedef struct { real_T dzk2kdorhz [ 3 ] ; real_T k1a54miwbp ; real_T
cultywlytp [ 2 ] ; real_T f3i0qwmqhd [ 2 ] ; real_T jgd3rak15n [ 2 ] ; real_T
m1hwfgbxdk [ 2 ] ; real_T aljvt534lz [ 2 ] ; real_T jl0jvt0d0k ; real_T
gnzypp4b3v ; real_T czngd4n0iq ; real_T aeqvdqjce5 ; real_T nf2iekvwo0 [ 2 ]
; real_T hed5ppmy0v [ 2 ] ; real_T i2zdopflz1 [ 2 ] ; real_T dnzrihotbc ;
real_T aaynfu2j1d ; } kpt1nmykuu ; typedef struct { int_T bp3tgqqqct ; }
a3yiko2loe ; typedef struct { real_T j03uw5kkja ; real_T kgdldehodj [ 3 ] ;
real_T nmfyvjxfu0 ; real_T bg2wjmcuth ; real_T ojrtzxm4or ; } ibm535n4bu ;
typedef struct { real_T j03uw5kkja ; real_T kgdldehodj [ 3 ] ; real_T
nmfyvjxfu0 ; real_T bg2wjmcuth ; real_T ojrtzxm4or ; } onacud0tvk ; typedef
struct { boolean_T j03uw5kkja ; boolean_T kgdldehodj [ 3 ] ; boolean_T
nmfyvjxfu0 ; boolean_T bg2wjmcuth ; boolean_T ojrtzxm4or ; } kwqslyp4dt ;
typedef struct { real_T j03uw5kkja ; real_T kgdldehodj [ 3 ] ; real_T
nmfyvjxfu0 ; real_T bg2wjmcuth ; real_T ojrtzxm4or ; } orqpqw5hkj ; typedef
struct { real_T j03uw5kkja ; real_T kgdldehodj [ 3 ] ; real_T nmfyvjxfu0 ;
real_T bg2wjmcuth ; real_T ojrtzxm4or ; } oa1ao3rbfq ; typedef struct {
real_T j03uw5kkja ; real_T kgdldehodj [ 3 ] ; real_T nmfyvjxfu0 ; real_T
bg2wjmcuth ; real_T ojrtzxm4or ; } eygklkfwgw ; typedef struct { real_T
chdaveyo3u ; real_T imzjiegmhs ; real_T ljnyziehbg ; } ilk1lfdy0e ; typedef
struct { real_T nhrs1vhctg ; real_T nag5qn1hz0 ; } lspl30b1kv ; typedef
struct { real_T ciyjva0rcy ; real_T gjks211rsy ; } e2p0y1ulol ; typedef
struct { real_T mdsnhdf51w ; real_T f1ftrazw5c ; real_T mnvbfsrwem ; real_T
am2pmcobn1 ; real_T o1dvfpketx ; real_T a3ol5jrudl ; real_T ixc0rizmcy ;
real_T oxeqxpsxbe ; real_T bzyonmge1w ; real_T cyuj1aqqt0 ; real_T dul0gurj1a
; real_T ktm4fhohyl ; real_T djzykhf2ex ; real_T dmbq5t1f3o ; e2p0y1ulol
bqikbu1ussl ; lspl30b1kv pwrrlmf4ove ; } comxhf3paa ; typedef struct { real_T
g3ygzxsk0f ; real_T cl52p0joj0 ; real_T elz01z3xn2 ; real_T kyzinnoxxv [ 2 ]
; real_T egsi5yxkjj ; } k0so1grdtv ; typedef struct { real_T ib2zgdbvfg ;
real_T gd3wtbesqk ; real_T izqakrt5gt ; real_T bgcwdykgym ; real_T g2n0kzbxiu
; real_T kzylicai5z ; real_T lvprouuziq ; real_T ahpc0noijl ; real_T
jr40drn4zw ; real_T fkpftiga0j ; real_T eud2hsmgnp ; real_T fhomoj0xmn ;
real_T p10c2uraxe ; real_T hetg4atxab ; real_T p2n0vsd4t3 ; real_T mcovfqrdnv
; real_T m5dbvveclu ; real_T af2wlmmwjp ; e2p0y1ulol jvbvozzghf ; lspl30b1kv
iy2vqcsdml ; } gqox2rbg05 ; typedef struct { real_T djstpatcad ; real_T
ngneoxnud4 ; real_T ber1fygurn ; } ernqvedpm2 ; typedef struct { real_T
nqw2kn00fp ; real_T crhr1dlu5e ; int8_T dgn22zt0vt ; boolean_T b4qx2seyzb ; }
n3eocp3q4u ; typedef struct { real_T bv35wzh0dj ; real_T mi3hjywdfv ; }
f44j4j3cb0 ; typedef struct { ZCSigState dw5br0dopp ; } o4rstliyo3 ; typedef
struct { real_T b34veyerhx [ 9 ] ; } nafsdv5otth ; typedef struct { real_T
nr32p14f5w ; real_T dhrvzhjuvz ; real_T iuvadceyxv ; real_T jgbtyskarg ;
real_T phiwiraj1v ; real_T ibsxusclmn ; real_T j5usjvj1gp ; boolean_T
aefevy3zk3 ; boolean_T npyetp3zfm ; } fdrylszg0n ; typedef struct { real_T
grw1csad5p ; boolean_T e5aj011b32 ; int8_T fcvkt5cgws ; int8_T cmfxzild0x ;
int8_T m5fdvylywn ; int8_T nckwrz5x5b ; int8_T put5iez4wm ; int8_T pyx5ttqko4
; uint8_T hmaon0vm50 ; uint8_T pfff4mwrt2 ; boolean_T m4gj5oeqmm ; boolean_T
mr0a0xuwb0 ; } a5veu5vblh ; typedef struct { real_T pomzptkmfz ; } kprfhpv3i5
; typedef struct { real_T pomzptkmfz ; } ifmf1zr4nj ; typedef struct {
boolean_T pomzptkmfz ; } cjlkti24pb ; typedef struct { real_T pomzptkmfz ; }
nwasmcgr00 ; typedef struct { real_T pomzptkmfz ; } add10d52w2 ; typedef
struct { real_T pomzptkmfz ; } bmgtgxodvg ; typedef struct { fdrylszg0n
mlvobr1uwv ; } imdajxgu3t ; typedef struct { a5veu5vblh mlvobr1uwv ; }
d44b0chayz ; typedef struct { kprfhpv3i5 mlvobr1uwv ; } ebwmznemwo ; typedef
struct { ifmf1zr4nj mlvobr1uwv ; } gjgafjchyv ; typedef struct { cjlkti24pb
mlvobr1uwv ; } cnu1igm4r4 ; typedef struct { nwasmcgr00 mlvobr1uwv ; }
nqzbhadunr ; typedef struct { add10d52w2 mlvobr1uwv ; } nbxgb5vqhz ; typedef
struct { bmgtgxodvg mlvobr1uwv ; } oihjekixf3 ; typedef struct { real_T
ng3wyaph2m [ 9 ] ; } nafsdv5otthx ; typedef struct { real_T ie20kn2zle [ 3 ]
; real_T nkt5bzw4ye [ 3 ] ; real_T ksdmacdk50 [ 3 ] ; real_T oi0a0ve40h [ 3 ]
; real_T azqoy2jqj5 [ 3 ] ; real_T h3cwmgsp0a [ 3 ] ; real_T lr5fu550z3 [ 3 ]
; real_T opewauqkii [ 3 ] ; real_T g5k5uteaia [ 3 ] ; real_T kk1qr14biw [ 3 ]
; real_T lk5cnlzuia [ 4 ] ; real_T lkf4uwg5g4 [ 4 ] ; real_T c41k2sow55 [ 4 ]
; real_T gnm2ivskyj [ 4 ] ; real_T lzrjzdlalz [ 2 ] ; real_T psolppo1ju [ 2 ]
; real_T pf44foyw5m [ 3 ] ; real_T nxs4ffbfcl [ 3 ] ; real_T foigk5lvhe ;
real_T axikyi2e3z ; real_T fu0uua5jas ; real_T lnegytupwr [ 9 ] ; real_T
bqaqezeool [ 9 ] ; real_T efb0sxp14g [ 9 ] ; real_T fcuevjzath [ 3 ] ; real_T
e0og5bkv4o [ 2 ] ; real_T jbknabrmjr [ 6 ] ; real_T pm4hex2jt3 [ 2 ] ; real_T
cz1uf5rvyb [ 3 ] ; real_T meug4j03q5 [ 4 ] ; real_T ovwqxm5vyk [ 12 ] ;
real_T pjjqi31exq [ 4 ] ; real_T h33gukfxda [ 4 ] ; real_T j4bpzjo3ib [ 4 ] ;
real_T a3smujx3c0 [ 4 ] ; real_T h2kxws0nkj [ 4 ] ; real_T doffapjzcr ;
real_T ef343rbce3 [ 12 ] ; real_T kgbz2mwdii [ 3 ] ; real_T jwinehzl2t [ 3 ]
; real_T jb2faw4505 [ 3 ] ; real_T ekth21b3yv ; real_T oyqb25civ3 ; real_T
lqgl33u0bp [ 3 ] ; real_T lwmakdevs1 [ 3 ] ; real_T fseojumlh0 [ 4 ] ; real_T
esxvyxikjq [ 12 ] ; real_T bg12yoslse [ 2 ] ; real_T bm5vmfu32y [ 2 ] ;
boolean_T ffk1onm5vp ; nafsdv5otthx m42zhk1m0gmm [ 4 ] ; imdajxgu3t
miajmxmfzv [ 4 ] ; nafsdv5otth m42zhk1m0gm [ 4 ] ; ernqvedpm2 lv5ud0bqym [ 3
] ; gqox2rbg05 ahbq3r2d43 [ 2 ] ; k0so1grdtv kcekfle0ez [ 1 ] ; comxhf3paa
p2zcxdfgi4 [ 2 ] ; kpt1nmykuu ojcafh5j00 [ 1 ] ; eny4tltuc5 g5ko5f54co [ 2 ]
; } k0r514zhkn ; typedef struct { real_T aksobzdnda [ 2 ] ; real_T i2x5kkvua5
[ 9 ] ; int_T dclwhpnu1k [ 4 ] ; int_T bpqyogr4ln [ 4 ] ; int8_T ch154vt1i4 ;
boolean_T hslut0feh3 ; boolean_T f5qsuhrbeb ; d44b0chayz miajmxmfzv [ 4 ] ;
n3eocp3q4u lv5ud0bqym [ 3 ] ; a3yiko2loe ojcafh5j00 [ 1 ] ; } jb251fek03 ;
typedef struct { real_T dr15zamh5i [ 3 ] ; real_T oryzjrwk3h [ 3 ] ; real_T
phbigetiha [ 3 ] ; real_T jylnwljqhz [ 3 ] ; real_T cr2pxfinox [ 2 ] ; real_T
m0zwbzhb22 [ 8 ] ; real_T fwmyae1wzn [ 4 ] ; real_T dtjafqjnli [ 4 ] ; real_T
pbvhqqzyxg [ 4 ] ; real_T a5tsdaqepm [ 4 ] ; real_T dygqmcwtyc [ 12 ] ;
real_T cjz4grkxz5 [ 3 ] ; real_T flhfqq4whe [ 4 ] ; ebwmznemwo miajmxmfzv [ 4
] ; ibm535n4bu ojcafh5j00 [ 1 ] ; } h21fsrthfa ; typedef int_T hn13clrkfo [ 3
] ; typedef real_T ja0rmqcfdg [ 6 ] ; typedef struct { real_T dr15zamh5i [ 3
] ; real_T oryzjrwk3h [ 3 ] ; real_T phbigetiha [ 3 ] ; real_T jylnwljqhz [ 3
] ; real_T cr2pxfinox [ 2 ] ; real_T m0zwbzhb22 [ 8 ] ; real_T fwmyae1wzn [ 4
] ; real_T dtjafqjnli [ 4 ] ; real_T pbvhqqzyxg [ 4 ] ; real_T a5tsdaqepm [ 4
] ; real_T dygqmcwtyc [ 12 ] ; real_T cjz4grkxz5 [ 3 ] ; real_T flhfqq4whe [
4 ] ; gjgafjchyv miajmxmfzv [ 4 ] ; onacud0tvk ojcafh5j00 [ 1 ] ; }
ectmdkaxud ; typedef struct { boolean_T dr15zamh5i [ 3 ] ; boolean_T
oryzjrwk3h [ 3 ] ; boolean_T phbigetiha [ 3 ] ; boolean_T jylnwljqhz [ 3 ] ;
boolean_T cr2pxfinox [ 2 ] ; boolean_T m0zwbzhb22 [ 8 ] ; boolean_T
fwmyae1wzn [ 4 ] ; boolean_T dtjafqjnli [ 4 ] ; boolean_T pbvhqqzyxg [ 4 ] ;
boolean_T a5tsdaqepm [ 4 ] ; boolean_T dygqmcwtyc [ 12 ] ; boolean_T
cjz4grkxz5 [ 3 ] ; boolean_T flhfqq4whe [ 4 ] ; cnu1igm4r4 miajmxmfzv [ 4 ] ;
kwqslyp4dt ojcafh5j00 [ 1 ] ; } gy0s4k0fxb ; typedef struct { real_T
dr15zamh5i [ 3 ] ; real_T oryzjrwk3h [ 3 ] ; real_T phbigetiha [ 3 ] ; real_T
jylnwljqhz [ 3 ] ; real_T cr2pxfinox [ 2 ] ; real_T m0zwbzhb22 [ 8 ] ; real_T
fwmyae1wzn [ 4 ] ; real_T dtjafqjnli [ 4 ] ; real_T pbvhqqzyxg [ 4 ] ; real_T
a5tsdaqepm [ 4 ] ; real_T dygqmcwtyc [ 12 ] ; real_T cjz4grkxz5 [ 3 ] ;
real_T flhfqq4whe [ 4 ] ; nqzbhadunr miajmxmfzv [ 4 ] ; orqpqw5hkj ojcafh5j00
[ 1 ] ; } mq10j5tdpd ; typedef struct { real_T dr15zamh5i [ 3 ] ; real_T
oryzjrwk3h [ 3 ] ; real_T phbigetiha [ 3 ] ; real_T jylnwljqhz [ 3 ] ; real_T
cr2pxfinox [ 2 ] ; real_T m0zwbzhb22 [ 8 ] ; real_T fwmyae1wzn [ 4 ] ; real_T
dtjafqjnli [ 4 ] ; real_T pbvhqqzyxg [ 4 ] ; real_T a5tsdaqepm [ 4 ] ; real_T
dygqmcwtyc [ 12 ] ; real_T cjz4grkxz5 [ 3 ] ; real_T flhfqq4whe [ 4 ] ;
nbxgb5vqhz miajmxmfzv [ 4 ] ; oa1ao3rbfq ojcafh5j00 [ 1 ] ; } omszgc4jji ;
typedef struct { real_T dr15zamh5i [ 3 ] ; real_T oryzjrwk3h [ 3 ] ; real_T
phbigetiha [ 3 ] ; real_T jylnwljqhz [ 3 ] ; real_T cr2pxfinox [ 2 ] ; real_T
m0zwbzhb22 [ 8 ] ; real_T fwmyae1wzn [ 4 ] ; real_T dtjafqjnli [ 4 ] ; real_T
pbvhqqzyxg [ 4 ] ; real_T a5tsdaqepm [ 4 ] ; real_T dygqmcwtyc [ 12 ] ;
real_T cjz4grkxz5 [ 3 ] ; real_T flhfqq4whe [ 4 ] ; oihjekixf3 miajmxmfzv [ 4
] ; eygklkfwgw ojcafh5j00 [ 1 ] ; } bphav3l4td ; typedef struct { real_T
akigrtsbp4 [ 4 ] ; real_T pqsv1plyuc [ 4 ] ; f44j4j3cb0 lv5ud0bqym [ 3 ] ;
ilk1lfdy0e ojcafh5j00 [ 1 ] ; } beahf2xhqn ; typedef struct { o4rstliyo3
lv5ud0bqym [ 3 ] ; } hgp0bjgtdb ; typedef struct { const real_T lrds5yrgjy ;
const real_T hjtzlfxjv5 ; const real_T ddt3g0b1mn ; const real_T mtia2wgwed ;
const real_T mynjykptju ; const real_T dwwqqmzc0m ; } aawyhsu50a ; struct
b4r3m1sp45_ { real_T P_0 ; real_T P_1 ; real_T P_2 ; real_T P_3 ; } ; struct
elefnztwqk_ { real_T P_0 ; real_T P_1 ; real_T P_2 ; real_T P_3 ; real_T P_4
; real_T P_5 ; real_T P_6 ; real_T P_7 ; real_T P_8 ; real_T P_9 ; real_T
P_10 ; real_T P_11 ; real_T P_12 ; real_T P_13 ; real_T P_14 ; real_T P_15 ;
real_T P_16 ; real_T P_17 ; real_T P_18 ; real_T P_19 [ 3 ] ; real_T P_20 ; }
; struct kacurs2jv1_ { real_T P_0 ; real_T P_1 ; real_T P_2 ; real_T P_3 ;
real_T P_4 ; real_T P_5 [ 3 ] ; real_T P_6 [ 3 ] ; real_T P_7 ; } ; struct
hkp5vxtoev_ { real_T P_0 ; real_T P_1 ; real_T P_2 ; real_T P_3 ; real_T P_4
; real_T P_5 [ 3 ] ; real_T P_6 [ 3 ] ; real_T P_7 ; } ; struct bw4g1ckg0y_ {
real_T P_0 ; real_T P_1 ; real_T P_2 ; real_T P_3 ; real_T P_4 ; real_T P_5 ;
real_T P_6 ; real_T P_7 ; real_T P_8 ; real_T P_9 ; real_T P_10 ; real_T P_11
; real_T P_12 ; real_T P_13 ; real_T P_14 ; real_T P_15 ; real_T P_16 ;
real_T P_17 ; real_T P_18 ; real_T P_19 [ 2 ] ; real_T P_20 ; hkp5vxtoev
bqikbu1ussl ; kacurs2jv1 pwrrlmf4ove ; } ; struct pelpu4erbs_ { real_T P_0 ;
real_T P_1 ; real_T P_2 ; real_T P_3 [ 2 ] ; real_T P_4 [ 2 ] ; real_T P_5 ;
real_T P_6 [ 2 ] ; real_T P_7 [ 2 ] ; } ; struct ffogqh3wsz_ { real_T P_0 ;
real_T P_1 ; real_T P_2 ; real_T P_3 ; real_T P_4 ; real_T P_5 ; real_T P_6 ;
real_T P_7 ; real_T P_8 ; real_T P_9 ; real_T P_10 ; real_T P_11 ; real_T
P_12 ; real_T P_13 ; real_T P_14 ; real_T P_15 ; real_T P_16 ; real_T P_17 ;
real_T P_18 ; real_T P_19 ; real_T P_20 ; real_T P_21 ; real_T P_22 ; real_T
P_23 [ 2 ] ; hkp5vxtoev jvbvozzghf ; kacurs2jv1 iy2vqcsdml ; } ; struct
lo1uh5sdk1_ { real_T P_0 ; real_T P_1 ; real_T P_2 ; real_T P_3 ; real_T P_4
; } ; struct avgdmhuprh4_ { real_T P_0 [ 3 ] ; real_T P_1 [ 2 ] ; } ; struct
mh44323250_ { real_T P_0 ; real_T P_1 ; real_T P_2 ; real_T P_3 ; real_T P_4
; boolean_T P_5 ; boolean_T P_6 ; boolean_T P_7 ; boolean_T P_8 [ 8 ] ; } ;
struct p44mzhiemb_ { real_T P_0 ; mh44323250 mlvobr1uwv ; } ; struct
avgdmhuprh40_ { real_T P_0 [ 3 ] ; } ; struct l0o1fxj1vpq_ { real_T P_0 ;
real_T P_1 ; real_T P_2 ; real_T P_3 ; real_T P_4 ; real_T P_5 ; real_T P_6 ;
real_T P_7 ; real_T P_8 ; real_T P_9 [ 2 ] ; real_T P_10 [ 2 ] ; real_T P_11
; real_T P_12 ; real_T P_13 ; real_T P_14 ; real_T P_15 ; real_T P_16 ;
real_T P_17 ; real_T P_18 ; real_T P_19 ; real_T P_20 ; real_T P_21 ; real_T
P_22 ; real_T P_23 ; real_T P_24 ; real_T P_25 ; real_T P_26 [ 31 ] ; real_T
P_27 [ 31 ] ; real_T P_28 ; real_T P_29 ; real_T P_30 ; real_T P_31 ; real_T
P_32 ; real_T P_33 ; real_T P_34 ; real_T P_35 ; real_T P_36 ; real_T P_37 ;
real_T P_38 ; real_T P_39 ; real_T P_40 ; real_T P_41 [ 9 ] ; real_T P_42 ;
real_T P_43 ; real_T P_44 ; real_T P_45 ; real_T P_46 ; real_T P_47 ; real_T
P_48 ; real_T P_49 ; real_T P_50 ; real_T P_51 ; real_T P_52 ; real_T P_53 ;
real_T P_54 ; real_T P_55 ; real_T P_56 ; real_T P_57 ; real_T P_58 ; real_T
P_59 [ 6 ] ; real_T P_60 ; real_T P_61 ; real_T P_62 ; real_T P_63 ; real_T
P_64 [ 6 ] ; real_T P_65 ; real_T P_66 ; real_T P_67 ; real_T P_68 ; real_T
P_69 [ 2 ] ; real_T P_70 [ 2 ] ; real_T P_71 [ 3 ] ; real_T P_72 ; real_T
P_73 ; real_T P_74 [ 31 ] ; real_T P_75 ; real_T P_76 ; real_T P_77 ; real_T
P_78 ; real_T P_79 [ 3 ] ; real_T P_80 ; real_T P_81 ; real_T P_82 ; real_T
P_83 ; real_T P_84 ; real_T P_85 ; real_T P_86 ; real_T P_87 ; real_T P_88 ;
real_T P_89 [ 3 ] ; real_T P_90 ; real_T P_91 ; real_T P_92 ; real_T P_93 [ 2
] ; real_T P_94 ; real_T P_95 ; real_T P_96 ; real_T P_97 ; real_T P_98 [ 3 ]
; real_T P_99 ; real_T P_100 [ 9 ] ; real_T P_101 [ 3 ] ; real_T P_102 ;
real_T P_103 [ 9 ] ; real_T P_104 [ 3 ] ; real_T P_105 ; real_T P_106 [ 9 ] ;
real_T P_107 [ 3 ] ; real_T P_108 ; real_T P_109 [ 9 ] ; real_T P_110 [ 3 ] ;
real_T P_111 ; real_T P_112 [ 9 ] ; real_T P_113 [ 3 ] ; real_T P_114 ;
real_T P_115 [ 9 ] ; real_T P_116 [ 3 ] ; real_T P_117 ; real_T P_118 [ 9 ] ;
real_T P_119 [ 3 ] ; real_T P_120 ; real_T P_121 ; real_T P_122 ; real_T
P_123 ; real_T P_124 ; real_T P_125 ; real_T P_126 ; real_T P_127 ; real_T
P_128 ; real_T P_129 ; real_T P_130 ; real_T P_131 ; real_T P_132 ; real_T
P_133 ; real_T P_134 ; real_T P_135 ; real_T P_136 ; real_T P_137 ; real_T
P_138 ; real_T P_139 ; real_T P_140 ; real_T P_141 ; real_T P_142 ; real_T
P_143 ; real_T P_144 ; real_T P_145 ; real_T P_146 ; real_T P_147 ; real_T
P_148 ; real_T P_149 ; real_T P_150 ; real_T P_151 ; real_T P_152 ; real_T
P_153 ; real_T P_154 ; real_T P_155 ; real_T P_156 ; real_T P_157 ; real_T
P_158 ; real_T P_159 ; real_T P_160 ; real_T P_161 ; real_T P_162 ; real_T
P_163 ; real_T P_164 ; real_T P_165 ; real_T P_166 ; real_T P_167 ; real_T
P_168 ; real_T P_169 ; real_T P_170 ; real_T P_171 ; real_T P_172 ; real_T
P_173 ; real_T P_174 ; real_T P_175 ; real_T P_176 ; real_T P_177 ; real_T
P_178 ; real_T P_179 ; real_T P_180 ; real_T P_181 ; real_T P_182 ; real_T
P_183 ; real_T P_184 ; real_T P_185 ; real_T P_186 ; real_T P_187 ; real_T
P_188 ; real_T P_189 ; real_T P_190 ; real_T P_191 ; real_T P_192 ; real_T
P_193 ; real_T P_194 ; real_T P_195 ; real_T P_196 ; real_T P_197 ; real_T
P_198 ; real_T P_199 ; real_T P_200 ; real_T P_201 ; real_T P_202 ; real_T
P_203 ; real_T P_204 ; real_T P_205 ; real_T P_206 ; real_T P_207 ; real_T
P_208 ; real_T P_209 ; real_T P_210 ; real_T P_211 ; real_T P_212 ; real_T
P_213 ; real_T P_214 ; real_T P_215 ; real_T P_216 ; real_T P_217 ; real_T
P_218 ; real_T P_219 ; real_T P_220 ; real_T P_221 ; real_T P_222 ; real_T
P_223 ; real_T P_224 ; real_T P_225 ; real_T P_226 ; real_T P_227 ; real_T
P_228 ; real_T P_229 ; real_T P_230 ; real_T P_231 ; real_T P_232 ; real_T
P_233 ; real_T P_234 ; real_T P_235 ; real_T P_236 ; real_T P_237 ; real_T
P_238 ; real_T P_239 ; real_T P_240 ; real_T P_241 ; real_T P_242 ; real_T
P_243 ; real_T P_244 ; real_T P_245 ; real_T P_246 ; real_T P_247 ; real_T
P_248 ; real_T P_249 ; real_T P_250 ; real_T P_251 ; real_T P_252 ; real_T
P_253 ; real_T P_254 ; real_T P_255 ; real_T P_256 ; real_T P_257 ; real_T
P_258 ; real_T P_259 ; real_T P_260 ; real_T P_261 ; real_T P_262 ; real_T
P_263 ; real_T P_264 ; real_T P_265 ; real_T P_266 ; real_T P_267 ; real_T
P_268 ; real_T P_269 ; real_T P_270 ; real_T P_271 ; real_T P_272 ; real_T
P_273 ; real_T P_274 ; real_T P_275 ; real_T P_276 ; real_T P_277 ; real_T
P_278 ; real_T P_279 ; real_T P_280 ; real_T P_281 ; real_T P_282 ; real_T
P_283 ; real_T P_284 ; real_T P_285 ; real_T P_286 ; real_T P_287 ; real_T
P_288 ; real_T P_289 ; real_T P_290 ; real_T P_291 ; real_T P_292 ; real_T
P_293 ; real_T P_294 ; real_T P_295 ; real_T P_296 ; real_T P_297 ; real_T
P_298 ; real_T P_299 ; real_T P_300 ; real_T P_301 ; real_T P_302 [ 279 ] ;
real_T P_303 ; real_T P_304 ; real_T P_305 [ 3 ] ; real_T P_306 [ 2 ] ;
real_T P_307 [ 2 ] ; real_T P_308 ; real_T P_309 ; real_T P_310 ; real_T
P_311 ; real_T P_312 ; real_T P_313 ; real_T P_314 ; real_T P_315 [ 4 ] ;
real_T P_316 ; real_T P_317 ; real_T P_318 ; real_T P_319 ; real_T P_320 ;
real_T P_321 ; real_T P_322 ; real_T P_323 ; real_T P_324 ; real_T P_325 ;
real_T P_326 ; real_T P_327 ; real_T P_328 ; real_T P_329 ; real_T P_330 ;
real_T P_331 ; real_T P_332 ; real_T P_333 ; real_T P_334 ; real_T P_335 ;
real_T P_336 ; real_T P_337 ; real_T P_338 ; real_T P_339 ; real_T P_340 ;
real_T P_341 ; real_T P_342 ; real_T P_343 ; real_T P_344 ; real_T P_345 ;
real_T P_346 ; real_T P_347 ; real_T P_348 ; real_T P_349 ; real_T P_350 ;
real_T P_351 ; real_T P_352 ; real_T P_353 [ 12 ] ; real_T P_354 [ 3 ] ;
real_T P_355 ; real_T P_356 ; real_T P_357 [ 2 ] ; real_T P_358 [ 4 ] ;
real_T P_359 [ 4 ] ; real_T P_360 [ 12 ] ; real_T P_361 ; real_T P_362 ;
real_T P_363 [ 3 ] ; real_T P_364 [ 9 ] ; real_T P_365 ; real_T P_366 [ 4 ] ;
real_T P_367 ; real_T P_368 ; real_T P_369 ; real_T P_370 ; real_T P_371 [ 4
] ; real_T P_372 [ 4 ] ; real_T P_373 [ 8 ] ; real_T P_374 [ 4 ] ; real_T
P_375 [ 24 ] ; real_T P_376 [ 36 ] ; real_T P_377 [ 4 ] ; real_T P_378 [ 20 ]
; boolean_T P_379 ; boolean_T P_380 ; avgdmhuprh40 m42zhk1m0gmm ; p44mzhiemb
miajmxmfzv ; avgdmhuprh4 m42zhk1m0gm ; lo1uh5sdk1 lv5ud0bqym ; ffogqh3wsz
ahbq3r2d43 ; pelpu4erbs kcekfle0ez ; bw4g1ckg0y p2zcxdfgi4 ; elefnztwqk
ojcafh5j00 ; b4r3m1sp45 g5ko5f54co ; } ; struct mmo2vwjqw3 { struct
SimStruct_tag * _mdlRefSfcnS ; struct { real_T mr_nonContSig0 [ 9 ] ; real_T
mr_nonContSig1 [ 9 ] ; real_T mr_nonContSig2 [ 9 ] ; real_T mr_nonContSig3 [
1 ] ; real_T mr_nonContSig4 [ 1 ] ; real_T mr_nonContSig5 [ 1 ] ; real_T
mr_nonContSig6 [ 3 ] ; boolean_T mr_nonContSig7 [ 1 ] ; real_T mr_nonContSig8
[ 2 ] ; real_T mr_nonContSig9 [ 3 ] ; real_T mr_nonContSig10 [ 3 ] ; real_T
mr_nonContSig11 [ 3 ] ; real_T mr_nonContSig12 [ 3 ] ; real_T mr_nonContSig13
[ 3 ] ; real_T mr_nonContSig14 [ 3 ] ; boolean_T mr_nonContSig15 [ 1 ] ;
boolean_T mr_nonContSig16 [ 1 ] ; boolean_T mr_nonContSig17 [ 1 ] ; boolean_T
mr_nonContSig18 [ 1 ] ; boolean_T mr_nonContSig19 [ 1 ] ; boolean_T
mr_nonContSig20 [ 1 ] ; boolean_T mr_nonContSig21 [ 1 ] ; boolean_T
mr_nonContSig22 [ 1 ] ; real_T mr_nonContSig23 [ 12 ] ; real_T
mr_nonContSig24 [ 4 ] ; real_T mr_nonContSig25 [ 3 ] ; real_T mr_nonContSig26
[ 3 ] ; real_T mr_nonContSig27 [ 1 ] ; real_T mr_nonContSig28 [ 1 ] ; real_T
mr_nonContSig29 [ 1 ] ; real_T mr_nonContSig30 [ 1 ] ; real_T mr_nonContSig31
[ 1 ] ; real_T mr_nonContSig32 [ 1 ] ; real_T mr_nonContSig33 [ 1 ] ; real_T
mr_nonContSig34 [ 1 ] ; real_T mr_nonContSig35 [ 1 ] ; real_T mr_nonContSig36
[ 1 ] ; real_T mr_nonContSig37 [ 1 ] ; real_T mr_nonContSig38 [ 1 ] ; real_T
mr_nonContSig39 [ 1 ] ; real_T mr_nonContSig40 [ 1 ] ; real_T mr_nonContSig41
[ 1 ] ; real_T mr_nonContSig42 [ 1 ] ; real_T mr_nonContSig43 [ 1 ] ; real_T
mr_nonContSig44 [ 1 ] ; real_T mr_nonContSig45 [ 1 ] ; real_T mr_nonContSig46
[ 1 ] ; real_T mr_nonContSig47 [ 1 ] ; real_T mr_nonContSig48 [ 1 ] ; real_T
mr_nonContSig49 [ 1 ] ; real_T mr_nonContSig50 [ 1 ] ; real_T mr_nonContSig51
[ 1 ] ; real_T mr_nonContSig52 [ 1 ] ; real_T mr_nonContSig53 [ 1 ] ; real_T
mr_nonContSig54 [ 1 ] ; real_T mr_nonContSig55 [ 1 ] ; real_T mr_nonContSig56
[ 1 ] ; real_T mr_nonContSig57 [ 1 ] ; real_T mr_nonContSig58 [ 1 ] ; real_T
mr_nonContSig59 [ 1 ] ; real_T mr_nonContSig60 [ 1 ] ; real_T mr_nonContSig61
[ 1 ] ; real_T mr_nonContSig62 [ 1 ] ; real_T mr_nonContSig63 [ 1 ] ; real_T
mr_nonContSig64 [ 1 ] ; real_T mr_nonContSig65 [ 1 ] ; real_T mr_nonContSig66
[ 1 ] ; real_T mr_nonContSig67 [ 1 ] ; real_T mr_nonContSig68 [ 1 ] ; real_T
mr_nonContSig69 [ 1 ] ; real_T mr_nonContSig70 [ 1 ] ; real_T mr_nonContSig71
[ 2 ] ; real_T mr_nonContSig72 [ 1 ] ; real_T mr_nonContSig73 [ 2 ] ; real_T
mr_nonContSig74 [ 1 ] ; real_T mr_nonContSig75 [ 1 ] ; real_T mr_nonContSig76
[ 1 ] ; real_T mr_nonContSig77 [ 1 ] ; real_T mr_nonContSig78 [ 1 ] ; real_T
mr_nonContSig79 [ 1 ] ; real_T mr_nonContSig80 [ 1 ] ; real_T mr_nonContSig81
[ 1 ] ; real_T mr_nonContSig82 [ 1 ] ; real_T mr_nonContSig83 [ 1 ] ; real_T
mr_nonContSig84 [ 1 ] ; real_T mr_nonContSig85 [ 1 ] ; real_T mr_nonContSig86
[ 1 ] ; real_T mr_nonContSig87 [ 1 ] ; real_T mr_nonContSig88 [ 1 ] ; real_T
mr_nonContSig89 [ 1 ] ; real_T mr_nonContSig90 [ 1 ] ; real_T mr_nonContSig91
[ 1 ] ; real_T mr_nonContSig92 [ 1 ] ; real_T mr_nonContSig93 [ 1 ] ; real_T
mr_nonContSig94 [ 1 ] ; real_T mr_nonContSig95 [ 1 ] ; real_T mr_nonContSig96
[ 1 ] ; real_T mr_nonContSig97 [ 1 ] ; real_T mr_nonContSig98 [ 1 ] ; real_T
mr_nonContSig99 [ 1 ] ; real_T mr_nonContSig100 [ 1 ] ; real_T
mr_nonContSig101 [ 1 ] ; real_T mr_nonContSig102 [ 1 ] ; real_T
mr_nonContSig103 [ 1 ] ; real_T mr_nonContSig104 [ 1 ] ; real_T
mr_nonContSig105 [ 1 ] ; real_T mr_nonContSig106 [ 1 ] ; real_T
mr_nonContSig107 [ 1 ] ; real_T mr_nonContSig108 [ 1 ] ; real_T
mr_nonContSig109 [ 1 ] ; real_T mr_nonContSig110 [ 1 ] ; real_T
mr_nonContSig111 [ 2 ] ; real_T mr_nonContSig112 [ 2 ] ; real_T
mr_nonContSig113 [ 2 ] ; real_T mr_nonContSig114 [ 2 ] ; real_T
mr_nonContSig115 [ 2 ] ; real_T mr_nonContSig116 [ 3 ] ; real_T
mr_nonContSig117 [ 1 ] ; real_T mr_nonContSig118 [ 3 ] ; real_T
mr_nonContSig119 [ 1 ] ; } NonContDerivMemory ; ssNonContDerivSigInfo
nonContDerivSignal [ 120 ] ; const rtTimingBridge * timingBridge ; struct {
rtwCAPI_ModelMappingInfo mmi ; rtwCAPI_ModelMapLoggingInstanceInfo
mmiLogInstanceInfo ; void * dataAddress [ 13 ] ; int32_T * vardimsAddress [
13 ] ; RTWLoggingFcnPtr loggingPtrs [ 13 ] ; sysRanDType * systemRan [ 39 ] ;
int_T systemTid [ 39 ] ; } DataMapInfo ; struct { int_T mdlref_GlobalTID [ 3
] ; time_T tStart ; } Timing ; } ; typedef struct { k0r514zhkn rtb ;
jb251fek03 rtdw ; gwlxzditat rtm ; hgp0bjgtdb rtzce ; } egcgcay4tuh ; extern
void klg3zs3vu4 ( SimStruct * _mdlRefSfcnS , ssNonContDerivSigFeedingOutports
* * mr_nonContOutputArray , int_T mdlref_TID0 , int_T mdlref_TID1 , int_T
mdlref_TID2 , gwlxzditat * const ke3gqsjzkb , k0r514zhkn * localB ,
jb251fek03 * localDW , h21fsrthfa * localX , void * sysRanPtr , int
contextTid , rtwCAPI_ModelMappingInfo * rt_ParentMMI , const char_T *
rt_ChildPath , int_T rt_ChildMMIIdx , int_T rt_CSTATEIdx ) ; extern void
mr_PassVeh14DOF_MdlInfoRegFcn ( SimStruct * mdlRefSfcnS , char_T * modelName
, int_T * retVal ) ; extern mxArray * mr_PassVeh14DOF_GetDWork ( const
egcgcay4tuh * mdlrefDW ) ; extern void mr_PassVeh14DOF_SetDWork ( egcgcay4tuh
* mdlrefDW , const mxArray * ssDW ) ; extern void
mr_PassVeh14DOF_RegisterSimStateChecksum ( SimStruct * S ) ; extern mxArray *
mr_PassVeh14DOF_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * PassVeh14DOF_GetCAPIStaticMap ( void ) ;
extern void hfudby3zvk ( lspl30b1kv * localB ) ; extern void pwrrlmf4ov ( gwlxzditat * const ke3gqsjzkb , real_T awei5l4hcm , real_T kr5a0j4eo4 , real_T c4inh5tf5k , real_T hc4odgk0jj , real_T evms5paslx , real_T b1bbpkg05l , lspl30b1kv * localB , kacurs2jv1 * localP ) ; extern void eeylblckdd ( e2p0y1ulol * localB ) ; extern void bqikbu1uss ( gwlxzditat * const ke3gqsjzkb , real_T j54ttoqu3f , real_T h0dzew3ug0 , real_T gzfemaevkd , real_T pl0vghrvjm , real_T gxn5qwj5te , real_T itvorpvtnl , e2p0y1ulol * localB , hkp5vxtoev * localP ) ; extern void kolrbed2p5 ( real_T pji0kxfrm2 , fdrylszg0n * localB , a5veu5vblh * localDW , mh44323250 * localP , kprfhpv3i5 * localX ) ; extern void huifmwcmfw ( fdrylszg0n * localB , a5veu5vblh * localDW ) ; extern void p0mcbsbfai ( fdrylszg0n * localB ) ; extern void anzqjwxq4l ( fdrylszg0n * localB , a5veu5vblh * localDW , ifmf1zr4nj * localXdot ) ; extern void h1jl5u1zlh ( gwlxzditat * const ke3gqsjzkb , real_T icbykxvaey , real_T cixc1r45gp , real_T h5pbzq2s3k , real_T pji0kxfrm2 , real_T kukpbski0y , real_T kjclsak5ve , real_T lc253rtqa1 , fdrylszg0n * localB , a5veu5vblh * localDW , mh44323250 * localP , kprfhpv3i5 * localX ) ; extern void h1jl5u1zlhTID2 ( real_T kukpbski0y , fdrylszg0n * localB , mh44323250 * localP ) ; extern void m3kvjvuvj4 ( k0r514zhkn * localB , jb251fek03 * localDW , h21fsrthfa * localX ) ; extern void nqasp5wza1 ( k0r514zhkn * localB , jb251fek03 * localDW , h21fsrthfa * localX ) ; extern void g53loesncj ( k0r514zhkn * localB , jb251fek03 * localDW , hgp0bjgtdb * localZCE ) ; extern void kpl5p5meuz ( k0r514zhkn * localB , jb251fek03 * localDW , h21fsrthfa * localX , gy0s4k0fxb * localXdis , ectmdkaxud * localXdot ) ; extern void gahgwqe10m ( k0r514zhkn * localB , jb251fek03 * localDW , h21fsrthfa * localX , beahf2xhqn * localZCSV ) ; extern void jpl1qt0btr ( gwlxzditat * const ke3gqsjzkb , k0r514zhkn * localB , jb251fek03 * localDW , gy0s4k0fxb * localXdis , mq10j5tdpd * localXAbsTol ) ; extern void PassVeh14DOF ( gwlxzditat * const ke3gqsjzkb , const real_T bczv22mps0 [ 4 ] , const real_T hcd1y4n4my [ 4 ] , const real_T c3zd4dk1uf [ 4 ] , const real_T ezwtynlaay [ 3 ] , const real_T miqwpmctiu [ 4 ] , const real_T jwz44h4qyo [ 4 ] , const real_T ouxmszsjxp [ 36 ] , real_T * pggp52jwjk , real_T * lypor0xpj1 , real_T * ftjbqounxj , real_T * fwqgqivicv , real_T * h2ks3iyaq5 , real_T * mtvbpsemzz , real_T * jehzjpccgx , real_T * o2ichp5nle , real_T * nhtjsoj1c0 , real_T * ikcpl1bxkn , real_T * dnxeocsv4g , real_T * f0v2wgxza1 , real_T * p2rbija23c , real_T * ih1j3pjyoo , real_T * fvg3lzzjdo , real_T * f1l52yb4jq , real_T * gruxe1rlv2 , real_T * b4q0jmmbug , real_T * fjv5k1mcbc , real_T * kzcqf2vfsw , real_T * mvyjnd51e3 , real_T * bwcemexstv , real_T * acgkqvdrcw , real_T * jzifxelwed , real_T * h3g0ryofks , real_T * nt4yr0i0hj , real_T * euq0t2rwxj , real_T * br2bfjcrie , real_T * fubqlc0btn , real_T * cp4osmnkfu , real_T * oslcsqdauk , real_T * itriuucltl , real_T * jrziauh3hc , real_T * cb12umgqon , real_T * g1grudw5zi , real_T * jrfbqrdzbm , real_T * dilomkhgbi , real_T * mkgisvgwny , real_T * kpx1shkjby , real_T * hhus1j2ht4 , real_T * h24xfa4b2s , real_T * gyxdtsbsii , real_T * csvwbc1ngb , real_T * cphc5xqcaf , real_T * b3x5z4nffw , real_T * lr5bejms2p , real_T * kr43cb0av4 , real_T * pdg3pyyyt1 , real_T * i12ej03rlb , real_T * itvcmupd1d , real_T * ls4pgv0zuu , real_T * bpm3ytzg4f , real_T * mxgqr4usay , real_T * hquqidfxko , real_T * cme5swnmwl , real_T * ppoog3xxbt , real_T * azmjjziqsi , real_T * hznzo55035 , real_T * fou4lpiexq , real_T * aileu3tu2t , real_T * f1at3n5uo2 , real_T m1zcq4hwyf [ 9 ] , real_T * orr5wkftrw , real_T * mfh20vrwyy , real_T * n2hw2wfznp , real_T * fwlutfb3qo , real_T * gpoabq5xfo , real_T * gknadatvmm , real_T * c0ddnq33p1 , real_T * dfsavw5d4p , real_T * gkyz5csfv1 , real_T * kibmlqtxrn , real_T * khxqckm1yt , real_T * paixfwxgrm , real_T * kyouf3dlyr , real_T * msc0qqut3z , real_T * bdsbl3lls3 , real_T * mzoy0wcmum , real_T * imps2wzct5 , real_T * fcpecyfr1p , real_T * leyoipbphh , real_T * omlhx2pmiy , real_T * gafatnvo4t , real_T * f3bflzw334 , real_T * ifyydfciv0 , real_T * fpjcoztxho , real_T * fcxkbnwivz , real_T * ae54wnjufm , real_T * htyao0onrh , real_T * hnkdp4anrc , real_T * ld4nk0i2f3 , real_T * cjyx0ggofo , real_T * ppdzxftqd5 , real_T * brxr5nq2wv , real_T * ib2jvpohml , real_T * kp4bowcmx1 , real_T * dydkdbhqtq , real_T * bk1ungcsto , real_T * asnimghpuw , real_T * mdx3hm51e4 , real_T * pvpwfmy2zj , real_T * j12kdesvwd , real_T * fz1hvbd3ha , real_T * hwh5xrfjyc , real_T * gnpoipqnqm , real_T * khlmnda5td , real_T * ix0xmoriky , real_T * bmdwvpx0pt , real_T * iq1s4szpjx , real_T * f0dnsawuhf , real_T * mbaluqx304 , real_T * pgbnm0ztm0 , real_T * eollgbskep , real_T * avglojeqa1 , real_T * pdwpqebgoj , real_T * edgjymrhb4 , real_T * p1pag4do5w , real_T * pk1uikfk5s , real_T * nd2u2ddcwj , real_T * ahpvnbzdyu , real_T * ms3w1rdoko , real_T * lsasxjpadr , real_T * ms3o0pzip0 , real_T * oe3nlf4agu , real_T * psg2xwpfre , real_T * ggx5kmyw21 , real_T * oltyospixn , real_T * h1sviefwhh , real_T * id34i2kzzk , real_T * cjdaegizuz , real_T * nvcaux2psi , real_T nl23ksykzq [ 4 ] , real_T hebuqrelww [ 4 ] , real_T koq0sywfe0 [ 4 ] , real_T ehjrka3nm3 [ 4 ] , real_T j3jrsqvej5 [ 4 ] , real_T iluwdwgent [ 4 ] , real_T kbxarz1ybt [ 4 ] , real_T llnyhp4mi5 [ 4 ] , real_T h24rgjgomt [ 4 ] , real_T icto1de4jv [ 4 ] , real_T fz3aokj1g4 [ 4 ] , real_T gwhkpzoifm [ 4 ] , real_T dvo3guef0v [ 4 ] , real_T eneo0zj5f4 [ 4 ] , real_T myefcxugud [ 4 ] , k0r514zhkn * localB , jb251fek03 * localDW , h21fsrthfa * localX , hgp0bjgtdb * localZCE ) ; extern void PassVeh14DOFTID2 ( real_T * ott1f1jrfb , real_T * o0jceoxqtn , real_T * i1oz14f35t , real_T * gqri2wme2p , real_T * bzrb3pvul0 , real_T * ikseaxfuag , real_T * cigdfbj5ce , real_T * c4jeplja1s , real_T * g3ihp3v4wu , real_T * borh2qxquh , real_T * d0wonak4fa , real_T * ij40l25dpj , real_T * ditcefagkm , real_T * nqdvj32nx2 , real_T * dzn0kjppdh , k0r514zhkn * localB ) ; extern void apei0jh2jv ( gwlxzditat * const ke3gqsjzkb ) ;
#endif
