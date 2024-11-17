#ifndef SiMappedEngineV_h_
#define SiMappedEngineV_h_
#ifndef SiMappedEngineV_COMMON_INCLUDES_
#define SiMappedEngineV_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif
#include "SiMappedEngineV_types.h"
#include "rtw_modelmap_simtarget.h"
#include <string.h>
#include <stddef.h>
typedef struct { real_T guvbzoda22 ; real_T jv1i44zpz4 ; real_T nmqikn0wfy ;
real_T hln2r0vyrz [ 2 ] ; real_T haqwzftjaf [ 2 ] ; real_T bagmxejbfa [ 2 ] ;
real_T ozcmxjbphl [ 2 ] ; real_T cuyzaclebl ; real_T nz430vfzwk ; real_T
akvflxxm0n ; real_T gqznr3sow4 ; real_T hrrraf0pos ; real_T pcmsck1qnt ; }
puu5fcics1 ; typedef struct { int_T ex3gnslpxt ; } ow0cmqxpyk ; typedef
struct { real_T peid1wqkia ; real_T obuuvimtvu ; real_T h235wq4fsj ; real_T
lf5wmc502m ; } fpgmc1blog ; typedef int_T mxzhu33opg [ 1 ] ; typedef real_T
mquzxajhwn [ 2 ] ; typedef struct { real_T peid1wqkia ; real_T obuuvimtvu ;
real_T h235wq4fsj ; real_T lf5wmc502m ; } ih0mudozt1 ; typedef struct {
boolean_T peid1wqkia ; boolean_T obuuvimtvu ; boolean_T h235wq4fsj ;
boolean_T lf5wmc502m ; } dk2lj1ie3v ; typedef struct { real_T peid1wqkia ;
real_T obuuvimtvu ; real_T h235wq4fsj ; real_T lf5wmc502m ; } i3uom1vnc5 ;
typedef struct { real_T peid1wqkia ; real_T obuuvimtvu ; real_T h235wq4fsj ;
real_T lf5wmc502m ; } mtobhgowxj ; typedef struct { real_T peid1wqkia ;
real_T obuuvimtvu ; real_T h235wq4fsj ; real_T lf5wmc502m ; } jfimtdo5tu ;
typedef struct { real_T oww3vu3njk ; real_T j54vi5dxsi ; } jgubm4xidl ;
struct anoacnregqm_ { real_T P_0 [ 256 ] ; real_T P_1 [ 16 ] ; real_T P_2 [
16 ] ; real_T P_3 [ 16 ] ; real_T P_4 ; real_T P_5 ; real_T P_6 ; real_T P_7
; real_T P_8 ; real_T P_9 [ 2 ] ; real_T P_10 [ 2 ] ; real_T P_11 ; real_T
P_12 ; real_T P_13 ; real_T P_14 ; real_T P_15 ; real_T P_16 ; real_T P_17 ;
real_T P_18 ; real_T P_19 ; real_T P_20 ; real_T P_21 ; real_T P_22 ; real_T
P_23 ; uint32_T P_24 [ 2 ] ; } ; struct mrj2qlehgq { struct SimStruct_tag *
_mdlRefSfcnS ; struct { rtwCAPI_ModelMappingInfo mmi ;
rtwCAPI_ModelMapLoggingInstanceInfo mmiLogInstanceInfo ; void * dataAddress [
4 ] ; int32_T * vardimsAddress [ 4 ] ; RTWLoggingFcnPtr loggingPtrs [ 4 ] ;
sysRanDType * systemRan [ 3 ] ; int_T systemTid [ 3 ] ; } DataMapInfo ;
struct { int_T mdlref_GlobalTID [ 3 ] ; time_T tStart ; } Timing ; } ;
typedef struct { puu5fcics1 rtb ; ow0cmqxpyk rtdw ; kgxfljuibq rtm ; }
o2k4rz0uw5x ; extern void fbspkqemka ( SimStruct * _mdlRefSfcnS , int_T
mdlref_TID0 , int_T mdlref_TID1 , int_T mdlref_TID2 , kgxfljuibq * const
plpdajfsza , puu5fcics1 * localB , ow0cmqxpyk * localDW , fpgmc1blog * localX
, void * sysRanPtr , int contextTid , rtwCAPI_ModelMappingInfo * rt_ParentMMI
, const char_T * rt_ChildPath , int_T rt_ChildMMIIdx , int_T rt_CSTATEIdx ) ;
extern void mr_SiMappedEngineV_MdlInfoRegFcn ( SimStruct * mdlRefSfcnS ,
char_T * modelName , int_T * retVal ) ; extern mxArray *
mr_SiMappedEngineV_GetDWork ( const o2k4rz0uw5x * mdlrefDW ) ; extern void
mr_SiMappedEngineV_SetDWork ( o2k4rz0uw5x * mdlrefDW , const mxArray * ssDW )
; extern void mr_SiMappedEngineV_RegisterSimStateChecksum ( SimStruct * S ) ;
extern mxArray * mr_SiMappedEngineV_GetSimStateDisallowedBlocks ( ) ; extern
const rtwCAPI_ModelMappingStaticInfo * SiMappedEngineV_GetCAPIStaticMap ( void
) ; extern void f5rmroe5ej ( fpgmc1blog * localX ) ; extern void f4glzcjee3 ( fpgmc1blog * localX ) ; extern void bmpf100lpn ( kgxfljuibq * const plpdajfsza , const real_T * dhoq3itz5d , const real_T * gzpwimsfg2 , real_T * p2vuqhqrpz , puu5fcics1 * localB , ow0cmqxpyk * localDW , ih0mudozt1 * localXdot ) ; extern void a20sjwaowh ( kgxfljuibq * const plpdajfsza , const real_T * gzpwimsfg2 , puu5fcics1 * localB , ow0cmqxpyk * localDW , jgubm4xidl * localZCSV ) ; extern void pj3vtlmlns ( kgxfljuibq * const plpdajfsza , const real_T * gzpwimsfg2 , puu5fcics1 * localB , ow0cmqxpyk * localDW ) ; extern void SiMappedEngineV ( real_T * p2vuqhqrpz , puu5fcics1 * localB , fpgmc1blog * localX ) ; extern void SiMappedEngineVTID2 ( puu5fcics1 * localB ) ; extern void po5zlrzzyd ( kgxfljuibq * const plpdajfsza ) ;
#endif
