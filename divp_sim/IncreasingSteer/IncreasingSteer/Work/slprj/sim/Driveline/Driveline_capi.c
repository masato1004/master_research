#include <stddef.h>
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "Driveline_capi_host.h"
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
#include "Driveline.h"
#include "Driveline_capi.h"
#include "Driveline_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) ,
0 , 0 , 0 , 0 , 0 } } ; static rtwCAPI_States rtBlockStates [ ] = { { 0 , 9 ,
TARGET_STRING ( "Driveline/Brake Hydraulics/Valve/Integrator1" ) ,
TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 , 1 , - 1 , 0
} , { 1 , 6 , TARGET_STRING ( "Driveline/Driveline Model/Rear Wheel Drive Active Differential/Active Differential/Active Slip Differential Superposition/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 , 1 , - 1 , 0 } , { 2 , 3 , TARGET_STRING ( "Driveline/Driveline Model/Rear Wheel Drive Active Differential/Torsional Compliance/Torsional Compliance Linear/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 , 1 , - 1 , 0 } , { 3 , 5 , TARGET_STRING ( "Driveline/Driveline Model/Rear Wheel Drive Active Differential/Torsional Compliance1/Torsional Compliance Linear/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 , 1 , - 1 , 0 } , { 4 , 1 , TARGET_STRING ( "Driveline/Transmission/Ideal/Prop Shaft/Torsional Compliance Linear/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 , 1 , - 1 , 0 } , { 5 , 2 , TARGET_STRING ( "Driveline/Driveline Model/Rear Wheel Drive Active Differential/Torsional Compliance/Torsional Compliance Linear/Cont LPF IC Dyn/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 , 1 , - 1 , 0 } , { 6 , 4 , TARGET_STRING ( "Driveline/Driveline Model/Rear Wheel Drive Active Differential/Torsional Compliance1/Torsional Compliance Linear/Cont LPF IC Dyn/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 , 1 , - 1 , 0 } , { 7 , 8 , TARGET_STRING ( "Driveline/Transmission/Ideal/Ideal Fixed Gear Transmission/Ideal Fixed Gear Transmission/Cont LPF IC Dyn/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 , 1 , - 1 , 0 } , { 8 , 15 , TARGET_STRING ( "Driveline/Transmission/Ideal/Ideal Fixed Gear Transmission/Ideal Fixed Gear Transmission/Locked/x" ) , TARGET_STRING ( "w" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 , 1 , - 1 , 0 } , { 9 , 13 , TARGET_STRING ( "Driveline/Transmission/Ideal/Ideal Fixed Gear Transmission/Ideal Fixed Gear Transmission/Unlocked/xe" ) , TARGET_STRING ( "we" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 , 1 , - 1 , 0 } , { 10 , 14 , TARGET_STRING ( "Driveline/Transmission/Ideal/Ideal Fixed Gear Transmission/Ideal Fixed Gear Transmission/Unlocked/xv" ) , TARGET_STRING ( "wv" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 , 1 , - 1 , 0 } , { 11 , 0 , TARGET_STRING ( "Driveline/Transmission/Ideal/Prop Shaft/Torsional Compliance Linear/Cont LPF IC Dyn/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 , 1 , - 1 , 0 } , { 0 , - 1 , ( NULL ) , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 , 0 , - 1 , 0 } } ; static int_T rt_LoggedStateIdxList [ ] = { 8 , 6 , 3 , 5 , 1 , 2 , 4 , 7 , 11 , 9 , 10 , 0 } ;
#ifndef HOST_CAPI_BUILD
static void Driveline_InitializeDataAddr ( void * dataAddr [ ] , bsz3qtxrxd *
localDW , oakdhkzk5b * localX ) { dataAddr [ 0 ] = ( void * ) ( & localX ->
nbshvx4uu4 [ 0 ] ) ; dataAddr [ 1 ] = ( void * ) ( & localX -> apjpgtpiwz [ 0
] ) ; dataAddr [ 2 ] = ( void * ) ( & localX -> aaxaf4jedn ) ; dataAddr [ 3 ]
= ( void * ) ( & localX -> jkxxbnfgvy ) ; dataAddr [ 4 ] = ( void * ) ( &
localX -> gmkd3jzxo3 ) ; dataAddr [ 5 ] = ( void * ) ( & localX -> hddpi2g224
) ; dataAddr [ 6 ] = ( void * ) ( & localX -> o2lamygnrv ) ; dataAddr [ 7 ] =
( void * ) ( & localX -> ioq10m20tp ) ; dataAddr [ 8 ] = ( void * ) ( &
localX -> ih5x40znwx ) ; dataAddr [ 9 ] = ( void * ) ( & localX -> lsght3lvcc
) ; dataAddr [ 10 ] = ( void * ) ( & localX -> oinmq3iuai ) ; dataAddr [ 11 ]
= ( void * ) ( & localX -> nkhwyywomp ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void Driveline_InitializeVarDimsAddr ( int32_T * vardimsAddr [ ] ) {
vardimsAddr [ 0 ] = ( NULL ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void Driveline_InitializeLoggingFunctions ( RTWLoggingFcnPtr
loggingPtrs [ ] ) { loggingPtrs [ 0 ] = ( NULL ) ; loggingPtrs [ 1 ] = ( NULL
) ; loggingPtrs [ 2 ] = ( NULL ) ; loggingPtrs [ 3 ] = ( NULL ) ; loggingPtrs
[ 4 ] = ( NULL ) ; loggingPtrs [ 5 ] = ( NULL ) ; loggingPtrs [ 6 ] = ( NULL
) ; loggingPtrs [ 7 ] = ( NULL ) ; loggingPtrs [ 8 ] = ( NULL ) ; loggingPtrs
[ 9 ] = ( NULL ) ; loggingPtrs [ 10 ] = ( NULL ) ; loggingPtrs [ 11 ] = ( NULL
) ; }
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
rtwCAPI_SCALAR , 4 , 2 , 0 } } ; static uint_T rtDimensionArray [ ] = { 4 , 1
, 2 , 1 , 1 , 1 } ; static const real_T rtcapiStoredFloats [ ] = { 0.0 } ;
static rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , ( boolean_T ) 0 } , } ; static
rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , ( int8_T ) 0 , ( uint8_T ) 0 } } ; static int_T rtContextSystems [ 8 ] ; static rtwCAPI_LoggingMetaInfo loggingMetaInfo [ ] = { { 0 , 0 , "" , 0 } } ; static rtwCAPI_ModelMapLoggingStaticInfo mmiStaticInfoLogging = { 8 , rtContextSystems , loggingMetaInfo , 0 , ( NULL ) , { 0 , ( NULL ) , ( NULL ) } , 0 , ( NULL ) } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals , 0 , ( NULL ) , 0 , ( NULL ) , 0 } , { ( NULL ) , 0 , ( NULL ) , 0 } , { rtBlockStates , 12 } , { rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float" , { 2213937493U , 1643732146U , 3163254840U , 502455336U } , & mmiStaticInfoLogging , 0 , ( boolean_T ) 0 , rt_LoggedStateIdxList } ; const rtwCAPI_ModelMappingStaticInfo * Driveline_GetCAPIStaticMap ( void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
static void Driveline_InitializeSystemRan ( dwsgrvz41y * const pubqvcmed4 ,
sysRanDType * systemRan [ ] , bsz3qtxrxd * localDW , int_T systemTid [ ] ,
void * rootSysRanPtr , int rootTid ) { UNUSED_PARAMETER ( pubqvcmed4 ) ;
UNUSED_PARAMETER ( localDW ) ; systemRan [ 0 ] = ( sysRanDType * )
rootSysRanPtr ; systemRan [ 1 ] = ( NULL ) ; systemRan [ 2 ] = ( NULL ) ;
systemRan [ 3 ] = ( NULL ) ; systemRan [ 4 ] = ( NULL ) ; systemRan [ 5 ] = ( sysRanDType * ) & localDW -> prlkxfgjhi ; systemRan [ 6 ] = ( sysRanDType * ) & localDW -> l14mkp5oex ; systemRan [ 7 ] = ( NULL ) ; systemTid [ 1 ] = pubqvcmed4 -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 2 ] = pubqvcmed4 -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 3 ] = pubqvcmed4 -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 4 ] = pubqvcmed4 -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 5 ] = pubqvcmed4 -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 6 ] = pubqvcmed4 -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 7 ] = pubqvcmed4 -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 0 ] = rootTid ; rtContextSystems [ 0 ] = 0 ; rtContextSystems [ 1 ] = 0 ; rtContextSystems [ 2 ] = 0 ; rtContextSystems [ 3 ] = 0 ; rtContextSystems [ 4 ] = 0 ; rtContextSystems [ 5 ] = 5 ; rtContextSystems [ 6 ] = 6 ; rtContextSystems [ 7 ] = 0 ; }
#endif
#ifndef HOST_CAPI_BUILD
void Driveline_InitializeDataMapInfo ( dwsgrvz41y * const pubqvcmed4 ,
bsz3qtxrxd * localDW , oakdhkzk5b * localX , void * sysRanPtr , int
contextTid ) { rtwCAPI_SetVersion ( pubqvcmed4 -> DataMapInfo . mmi , 1 ) ;
rtwCAPI_SetStaticMap ( pubqvcmed4 -> DataMapInfo . mmi , & mmiStatic ) ;
rtwCAPI_SetLoggingStaticMap ( pubqvcmed4 -> DataMapInfo . mmi , &
mmiStaticInfoLogging ) ; Driveline_InitializeDataAddr ( pubqvcmed4 ->
DataMapInfo . dataAddress , localDW , localX ) ; rtwCAPI_SetDataAddressMap ( pubqvcmed4 -> DataMapInfo . mmi , pubqvcmed4 -> DataMapInfo . dataAddress ) ; Driveline_InitializeVarDimsAddr ( pubqvcmed4 -> DataMapInfo . vardimsAddress ) ; rtwCAPI_SetVarDimsAddressMap ( pubqvcmed4 -> DataMapInfo . mmi , pubqvcmed4 -> DataMapInfo . vardimsAddress ) ; rtwCAPI_SetPath ( pubqvcmed4 -> DataMapInfo . mmi , ( NULL ) ) ; rtwCAPI_SetFullPath ( pubqvcmed4 -> DataMapInfo . mmi , ( NULL ) ) ; Driveline_InitializeLoggingFunctions ( pubqvcmed4 -> DataMapInfo . loggingPtrs ) ; rtwCAPI_SetLoggingPtrs ( pubqvcmed4 -> DataMapInfo . mmi , pubqvcmed4 -> DataMapInfo . loggingPtrs ) ; rtwCAPI_SetInstanceLoggingInfo ( pubqvcmed4 -> DataMapInfo . mmi , & pubqvcmed4 -> DataMapInfo . mmiLogInstanceInfo ) ; rtwCAPI_SetChildMMIArray ( pubqvcmed4 -> DataMapInfo . mmi , ( NULL ) ) ; rtwCAPI_SetChildMMIArrayLen ( pubqvcmed4 -> DataMapInfo . mmi , 0 ) ; Driveline_InitializeSystemRan ( pubqvcmed4 , pubqvcmed4 -> DataMapInfo . systemRan , localDW , pubqvcmed4 -> DataMapInfo . systemTid , sysRanPtr , contextTid ) ; rtwCAPI_SetSystemRan ( pubqvcmed4 -> DataMapInfo . mmi , pubqvcmed4 -> DataMapInfo . systemRan ) ; rtwCAPI_SetSystemTid ( pubqvcmed4 -> DataMapInfo . mmi , pubqvcmed4 -> DataMapInfo . systemTid ) ; rtwCAPI_SetGlobalTIDMap ( pubqvcmed4 -> DataMapInfo . mmi , & pubqvcmed4 -> Timing . mdlref_GlobalTID [ 0 ] ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void Driveline_host_InitializeDataMapInfo ( Driveline_host_DataMapInfo_T *
dataMap , const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
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
