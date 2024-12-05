#include <stddef.h>
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "PassVeh14DOF_capi_host.h"
#define sizeof(...) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)
#ifndef SS_UINT64
#define SS_UINT64 21
#endif
#ifndef SS_INT64
#define SS_INT64 22
#endif
#else
#include "builtin_typeid_types.h"
#include "PassVeh14DOF.h"
#include "PassVeh14DOF_capi.h"
#include "PassVeh14DOF_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) ,
0 , 0 , 0 , 0 , 0 } } ; static rtwCAPI_States rtBlockStates [ ] = { { 0 , 30
, TARGET_STRING ( "PassVeh14DOF/Wheels and Tires/Cont LPF/Integrator1" ) ,
TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 , 1 , - 1 , 0
} , { 1 , 38 , TARGET_STRING ( "PassVeh14DOF/Wheels and Tires/Cont LPF1/Integrator1" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 , 1 , - 1 , 0 } , { 2 , 12 , TARGET_STRING ( "PassVeh14DOF/Vehicle/Variant Subsystem/6DOF/Suspension to Chasiss\nTransforms/Bushings/Integrator1" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 , 1 , - 1 , 0 } , { 3 , 9 , TARGET_STRING ( "PassVeh14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF/6 DOF Generic Vehicle Body/6DOF (Euler Angles)/p,q,r " ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 , 1 , - 1 , 0 } , { 4 , 6 , TARGET_STRING ( "PassVeh14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF/6 DOF Generic Vehicle Body/6DOF (Euler Angles)/ub,vb,wb" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 , 1 , - 1 , 0 } , { 5 , 0 , TARGET_STRING ( "PassVeh14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF/6 DOF Generic Vehicle Body/6DOF (Euler Angles)/xe,ye,ze" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 , 1 , - 1 , 0 } , { 6 , 50 , TARGET_STRING ( "PassVeh14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF/6 DOF Generic Vehicle Body/SignalCollection/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 , 1 , - 1 , 0 } , { 7 , 3 , TARGET_STRING ( "PassVeh14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF/6 DOF Generic Vehicle Body/6DOF (Euler Angles)/Calculate DCM &\nEuler Angles/phi\ntheta\npsi" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 , 1 , - 1 , 0 } , { 8 , 22 , TARGET_STRING ( "PassVeh14DOF/Wheels and Tires/VDBS/Tires/MF Tires Vector/Combined Slip Wheel 2DOF/Magic Tire Const Input/Fx Relaxation/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 , 1 , - 1 , 0 } , { 9 , 26 , TARGET_STRING ( "PassVeh14DOF/Wheels and Tires/VDBS/Tires/MF Tires Vector/Combined Slip Wheel 2DOF/Magic Tire Const Input/Fy Relaxation/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 , 1 , - 1 , 0 } , { 10 , 34 , TARGET_STRING ( "PassVeh14DOF/Wheels and Tires/VDBS/Tires/MF Tires Vector/Combined Slip Wheel 2DOF/Magic Tire Const Input/My Relaxation/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 , 1 , - 1 , 0 } , { 11 , 53 , TARGET_STRING ( "PassVeh14DOF/Wheels and Tires/VDBS/Tires/MF Tires Vector/Combined Slip Wheel 2DOF/Magic Tire Const Input/WhlTrq Relaxation/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 , 1 , - 1 , 0 } , { 12 , 14 , TARGET_STRING ( "PassVeh14DOF/Wheels and Tires/VDBS/Tires/MF Tires Vector/Combined Slip Wheel 2DOF/Vertical DOF/Vertical Wheel and Unsprung Mass Response/Integrator,\nSecond-Order" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 4 , 0 , 0 , 1 , - 1 , 0 } , { 0 , - 1 , ( NULL ) , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 , 0 , - 1 , 0 } } ; static int_T rt_LoggedStateIdxList [ ] = { 8 , 10 , 4 , 3 , 2 , 0 , 11 , 1 , 6 , 7 , 9 , 12 , 5 } ;
#ifndef HOST_CAPI_BUILD
static void PassVeh14DOF_InitializeDataAddr ( void * dataAddr [ ] ,
jb251fek03 * localDW , h21fsrthfa * localX ) { dataAddr [ 0 ] = ( void * ) ( &
localX -> pbvhqqzyxg [ 0 ] ) ; dataAddr [ 1 ] = ( void * ) ( & localX ->
dygqmcwtyc [ 0 ] ) ; dataAddr [ 2 ] = ( void * ) ( & localX -> cr2pxfinox [ 0
] ) ; dataAddr [ 3 ] = ( void * ) ( & localX -> phbigetiha [ 0 ] ) ; dataAddr
[ 4 ] = ( void * ) ( & localX -> oryzjrwk3h [ 0 ] ) ; dataAddr [ 5 ] = ( void
* ) ( & localX -> jylnwljqhz [ 0 ] ) ; dataAddr [ 6 ] = ( void * ) ( & localX
-> cjz4grkxz5 [ 0 ] ) ; dataAddr [ 7 ] = ( void * ) ( & localX -> dr15zamh5i
[ 0 ] ) ; dataAddr [ 8 ] = ( void * ) ( & localX -> fwmyae1wzn [ 0 ] ) ;
dataAddr [ 9 ] = ( void * ) ( & localX -> dtjafqjnli [ 0 ] ) ; dataAddr [ 10
] = ( void * ) ( & localX -> a5tsdaqepm [ 0 ] ) ; dataAddr [ 11 ] = ( void *
) ( & localX -> flhfqq4whe [ 0 ] ) ; dataAddr [ 12 ] = ( void * ) ( & localX
-> m0zwbzhb22 [ 0 ] ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void PassVeh14DOF_InitializeVarDimsAddr ( int32_T * vardimsAddr [ ] )
{ vardimsAddr [ 0 ] = ( NULL ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void PassVeh14DOF_InitializeLoggingFunctions ( RTWLoggingFcnPtr
loggingPtrs [ ] ) { loggingPtrs [ 0 ] = ( NULL ) ; loggingPtrs [ 1 ] = ( NULL
) ; loggingPtrs [ 2 ] = ( NULL ) ; loggingPtrs [ 3 ] = ( NULL ) ; loggingPtrs
[ 4 ] = ( NULL ) ; loggingPtrs [ 5 ] = ( NULL ) ; loggingPtrs [ 6 ] = ( NULL
) ; loggingPtrs [ 7 ] = ( NULL ) ; loggingPtrs [ 8 ] = ( NULL ) ; loggingPtrs
[ 9 ] = ( NULL ) ; loggingPtrs [ 10 ] = ( NULL ) ; loggingPtrs [ 11 ] = ( NULL
) ; loggingPtrs [ 12 ] = ( NULL ) ; }
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , ( uint8_T ) SS_DOUBLE , 0 , 0 , 0 } }
;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_VECTOR , 0 , 2 , 0 } , { rtwCAPI_VECTOR , 2 , 2 , 0 } , {
rtwCAPI_VECTOR , 4 , 2 , 0 } , { rtwCAPI_VECTOR , 6 , 2 , 0 } , {
rtwCAPI_VECTOR , 8 , 2 , 0 } } ; static uint_T rtDimensionArray [ ] = { 4 , 1
, 12 , 1 , 2 , 1 , 3 , 1 , 8 , 1 } ; static const real_T rtcapiStoredFloats [
] = { 0.0 } ; static rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL
) , rtwCAPI_FIX_RESERVED , 0 , 0 , ( boolean_T ) 0 } , } ; static
rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , ( int8_T ) 0 , ( uint8_T ) 0 } } ; static int_T rtContextSystems [ 39 ] ; static rtwCAPI_LoggingMetaInfo loggingMetaInfo [ ] = { { 0 , 0 , "" , 0 } } ; static rtwCAPI_ModelMapLoggingStaticInfo mmiStaticInfoLogging = { 39 , rtContextSystems , loggingMetaInfo , 0 , ( NULL ) , { 0 , ( NULL ) , ( NULL ) } , 0 , ( NULL ) } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals , 0 , ( NULL ) , 0 , ( NULL ) , 0 } , { ( NULL ) , 0 , ( NULL ) , 0 } , { rtBlockStates , 13 } , { rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float" , { 4049050945U , 4025042598U , 4079991450U , 1642321259U } , & mmiStaticInfoLogging , 0 , ( boolean_T ) 0 , rt_LoggedStateIdxList } ; const rtwCAPI_ModelMappingStaticInfo * PassVeh14DOF_GetCAPIStaticMap ( void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
static void PassVeh14DOF_InitializeSystemRan ( gwlxzditat * const ke3gqsjzkb
, sysRanDType * systemRan [ ] , jb251fek03 * localDW , int_T systemTid [ ] ,
void * rootSysRanPtr , int rootTid ) { UNUSED_PARAMETER ( ke3gqsjzkb ) ;
UNUSED_PARAMETER ( localDW ) ; systemRan [ 0 ] = ( sysRanDType * )
rootSysRanPtr ; systemRan [ 1 ] = ( NULL ) ; systemRan [ 2 ] = ( NULL ) ;
systemRan [ 3 ] = ( NULL ) ; systemRan [ 4 ] = ( NULL ) ; systemRan [ 5 ] = ( NULL ) ; systemRan [ 6 ] = ( NULL ) ; systemRan [ 7 ] = ( NULL ) ; systemRan [ 8 ] = ( NULL ) ; systemRan [ 9 ] = ( NULL ) ; systemRan [ 10 ] = ( NULL ) ; systemRan [ 11 ] = ( NULL ) ; systemRan [ 12 ] = ( NULL ) ; systemRan [ 13 ] = ( NULL ) ; systemRan [ 14 ] = ( NULL ) ; systemRan [ 15 ] = ( NULL ) ; systemRan [ 16 ] = ( NULL ) ; systemRan [ 17 ] = ( NULL ) ; systemRan [ 18 ] = ( NULL ) ; systemRan [ 19 ] = ( NULL ) ; systemRan [ 20 ] = ( NULL ) ; systemRan [ 21 ] = ( sysRanDType * ) & localDW -> lv5ud0bqym [ 2 ] . dgn22zt0vt ; systemRan [ 22 ] = ( NULL ) ; systemRan [ 23 ] = ( NULL ) ; systemRan [ 24 ] = ( NULL ) ; systemRan [ 25 ] = ( NULL ) ; systemRan [ 26 ] = ( NULL ) ; systemRan [ 27 ] = ( NULL ) ; systemRan [ 28 ] = ( sysRanDType * ) & localDW -> miajmxmfzv [ 3 ] . mlvobr1uwv . nckwrz5x5b ; systemRan [ 29 ] = ( sysRanDType * ) & localDW -> miajmxmfzv [ 3 ] . mlvobr1uwv . m5fdvylywn ; systemRan [ 30 ] = ( sysRanDType * ) & localDW -> miajmxmfzv [ 3 ] . mlvobr1uwv . put5iez4wm ; systemRan [ 31 ] = ( sysRanDType * ) & localDW -> miajmxmfzv [ 3 ] . mlvobr1uwv . pyx5ttqko4 ; systemRan [ 32 ] = ( NULL ) ; systemRan [ 33 ] = ( NULL ) ; systemRan [ 34 ] = ( NULL ) ; systemRan [ 35 ] = ( NULL ) ; systemRan [ 36 ] = ( NULL ) ; systemRan [ 37 ] = ( NULL ) ; systemRan [ 38 ] = ( NULL ) ; systemTid [ 1 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 2 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 3 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 4 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 5 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 6 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 12 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 7 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 13 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 8 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 9 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 10 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 11 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 14 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 15 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 16 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 17 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 18 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 19 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 1 ] ; systemTid [ 20 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 1 ] ; systemTid [ 21 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 1 ] ; systemTid [ 22 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 23 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 24 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 1 ] ; systemTid [ 25 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 26 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 27 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 31 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 1 ] ; systemTid [ 30 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 1 ] ; systemTid [ 28 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 29 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 32 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 33 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 34 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 35 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 36 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 37 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 38 ] = ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 0 ] = rootTid ; rtContextSystems [ 0 ] = 0 ; rtContextSystems [ 1 ] = 0 ; rtContextSystems [ 2 ] = 0 ; rtContextSystems [ 3 ] = 0 ; rtContextSystems [ 4 ] = 0 ; rtContextSystems [ 5 ] = 0 ; rtContextSystems [ 6 ] = 0 ; rtContextSystems [ 7 ] = 0 ; rtContextSystems [ 8 ] = 0 ; rtContextSystems [ 9 ] = 0 ; rtContextSystems [ 10 ] = 0 ; rtContextSystems [ 11 ] = 0 ; rtContextSystems [ 12 ] = 0 ; rtContextSystems [ 13 ] = 0 ; rtContextSystems [ 14 ] = 0 ; rtContextSystems [ 15 ] = 0 ; rtContextSystems [ 16 ] = 0 ; rtContextSystems [ 17 ] = 0 ; rtContextSystems [ 18 ] = 0 ; rtContextSystems [ 19 ] = 0 ; rtContextSystems [ 20 ] = 0 ; rtContextSystems [ 21 ] = 21 ; rtContextSystems [ 22 ] = 0 ; rtContextSystems [ 23 ] = 0 ; rtContextSystems [ 24 ] = 0 ; rtContextSystems [ 25 ] = 0 ; rtContextSystems [ 26 ] = 0 ; rtContextSystems [ 27 ] = 0 ; rtContextSystems [ 28 ] = 28 ; rtContextSystems [ 29 ] = 29 ; rtContextSystems [ 30 ] = 30 ; rtContextSystems [ 31 ] = 31 ; rtContextSystems [ 32 ] = 0 ; rtContextSystems [ 33 ] = 0 ; rtContextSystems [ 34 ] = 0 ; rtContextSystems [ 35 ] = 0 ; rtContextSystems [ 36 ] = 0 ; rtContextSystems [ 37 ] = 0 ; rtContextSystems [ 38 ] = 0 ; }
#endif
#ifndef HOST_CAPI_BUILD
void PassVeh14DOF_InitializeDataMapInfo ( gwlxzditat * const ke3gqsjzkb ,
jb251fek03 * localDW , h21fsrthfa * localX , void * sysRanPtr , int
contextTid ) { rtwCAPI_SetVersion ( ke3gqsjzkb -> DataMapInfo . mmi , 1 ) ;
rtwCAPI_SetStaticMap ( ke3gqsjzkb -> DataMapInfo . mmi , & mmiStatic ) ;
rtwCAPI_SetLoggingStaticMap ( ke3gqsjzkb -> DataMapInfo . mmi , &
mmiStaticInfoLogging ) ; PassVeh14DOF_InitializeDataAddr ( ke3gqsjzkb ->
DataMapInfo . dataAddress , localDW , localX ) ; rtwCAPI_SetDataAddressMap ( ke3gqsjzkb -> DataMapInfo . mmi , ke3gqsjzkb -> DataMapInfo . dataAddress ) ; PassVeh14DOF_InitializeVarDimsAddr ( ke3gqsjzkb -> DataMapInfo . vardimsAddress ) ; rtwCAPI_SetVarDimsAddressMap ( ke3gqsjzkb -> DataMapInfo . mmi , ke3gqsjzkb -> DataMapInfo . vardimsAddress ) ; rtwCAPI_SetPath ( ke3gqsjzkb -> DataMapInfo . mmi , ( NULL ) ) ; rtwCAPI_SetFullPath ( ke3gqsjzkb -> DataMapInfo . mmi , ( NULL ) ) ; PassVeh14DOF_InitializeLoggingFunctions ( ke3gqsjzkb -> DataMapInfo . loggingPtrs ) ; rtwCAPI_SetLoggingPtrs ( ke3gqsjzkb -> DataMapInfo . mmi , ke3gqsjzkb -> DataMapInfo . loggingPtrs ) ; rtwCAPI_SetInstanceLoggingInfo ( ke3gqsjzkb -> DataMapInfo . mmi , & ke3gqsjzkb -> DataMapInfo . mmiLogInstanceInfo ) ; rtwCAPI_SetChildMMIArray ( ke3gqsjzkb -> DataMapInfo . mmi , ( NULL ) ) ; rtwCAPI_SetChildMMIArrayLen ( ke3gqsjzkb -> DataMapInfo . mmi , 0 ) ; PassVeh14DOF_InitializeSystemRan ( ke3gqsjzkb , ke3gqsjzkb -> DataMapInfo . systemRan , localDW , ke3gqsjzkb -> DataMapInfo . systemTid , sysRanPtr , contextTid ) ; rtwCAPI_SetSystemRan ( ke3gqsjzkb -> DataMapInfo . mmi , ke3gqsjzkb -> DataMapInfo . systemRan ) ; rtwCAPI_SetSystemTid ( ke3gqsjzkb -> DataMapInfo . mmi , ke3gqsjzkb -> DataMapInfo . systemTid ) ; rtwCAPI_SetGlobalTIDMap ( ke3gqsjzkb -> DataMapInfo . mmi , & ke3gqsjzkb -> Timing . mdlref_GlobalTID [ 0 ] ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void PassVeh14DOF_host_InitializeDataMapInfo ( PassVeh14DOF_host_DataMapInfo_T
* dataMap , const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ;
rtwCAPI_SetDataAddressMap ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , ( NULL ) ) ; rtwCAPI_SetPath
( dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , ( NULL ) )
; rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
