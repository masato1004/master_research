#if !defined(S_FUNCTION_NAME)
#define S_FUNCTION_NAME PassVeh14DOF_msf
#endif
#define S_FUNCTION_LEVEL 2
#if !defined(RTW_GENERATED_S_FUNCTION)
#define RTW_GENERATED_S_FUNCTION
#endif
#include <stdio.h>
#include <math.h>
#include "simstruc.h"
#include "fixedpoint.h"
#include "mwstringutil.h"
#define rt_logging_h
#include "PassVeh14DOF_types.h"
#include "PassVeh14DOF.h"
#include "PassVeh14DOF_private.h"
extern const aawyhsu50a byh5mtyyyx ; const char *
rt_GetMatSignalLoggingFileName ( void ) { return NULL ; } const char *
rt_GetMatSigLogSelectorFileName ( void ) { return NULL ; } void *
rt_GetOSigstreamManager ( void ) { return NULL ; } void * rt_slioCatalogue ( void ) { return NULL ; } void * rtwGetPointerFromUniquePtr ( void * uniquePtr ) { return NULL ; } void * CreateDiagnosticAsVoidPtr ( const char * id , int nargs , ... ) { void * voidPtrDiagnostic = NULL ; va_list args ; va_start ( args , nargs ) ; slmrCreateDiagnostic ( id , nargs , args , & voidPtrDiagnostic ) ; va_end ( args ) ; return voidPtrDiagnostic ; } void rt_ssSet_slErrMsg ( void * S , void * diag ) { SimStruct * simStrcut = ( SimStruct * ) S ; if ( ! _ssIsErrorStatusAslErrMsg ( simStrcut ) ) { _ssSet_slLocalErrMsg ( simStrcut , diag ) ; } else { _ssDiscardDiagnostic ( simStrcut , diag ) ; } } void rt_ssReportDiagnosticAsWarning ( void * S , void * diag ) { _ssReportDiagnosticAsWarning ( ( SimStruct * ) S , diag ) ; } void rt_ssReportDiagnosticAsInfo ( void * S , void * diag ) { _ssReportDiagnosticAsInfo ( ( SimStruct * ) S , diag ) ; } const char * rt_CreateFullPathToTop ( const char * toppath , const char * subpath ) { char * fullpath = NULL ; slmrCreateFullPathToTop ( toppath , subpath , & fullpath ) ; return fullpath ; } boolean_T slIsRapidAcceleratorSimulating ( void ) { return false ; } void rt_RAccelReplaceFromFilename ( const char * blockpath , char * fileName ) { ( void ) blockpath ; ( void ) fileName ; } void rt_RAccelReplaceToFilename ( const char * blockpath , char * fileName ) { ( void ) blockpath ; ( void ) fileName ; } void * slsa_malloc ( size_t s ) { return malloc ( s ) ; } void slsa_free ( void * ptr ) { free ( ptr ) ; } void slsaCacheDWorkPointerForSimTargetOP ( void * ss , void * * ptr ) { ( void ) ss ; ( void ) ptr ; } void slsaCacheDWorkDataForSimTargetOP ( void * ss , void * ptr , unsigned int sizeInBytes ) { ( void ) ss ; ( void ) ptr ; ( void ) sizeInBytes ; } void slsaSaveRawMemoryForSimTargetOP ( void * ss , const char * key , void * * ptr , unsigned int sizeInBytes , void * ( * customOPSaveFcn ) ( void * dworkPtr , unsigned int * sizeInBytes ) , void ( * customOPRestoreFcn ) ( void * dworkPtr , const void * data , unsigned int sizeInBytes ) ) { ( void ) ss ; ( void ) key ; ( void ) ptr ; ( void ) sizeInBytes ; ( void ) customOPSaveFcn ; ( void ) customOPRestoreFcn ; }
#define MDL_PROCESS_PARAMETERS
#if defined(MATLAB_MEX_FILE)
static void mdlProcessParameters ( SimStruct * S ) { h21fsrthfa * localX = ( h21fsrthfa * ) ssGetContStates ( S ) ; }
#endif
static void mdlInitializeConditions ( SimStruct * S ) { egcgcay4tuh * dw = ( egcgcay4tuh * ) ssGetDWork ( S , 0 ) ; h21fsrthfa * localX = ( h21fsrthfa * ) ssGetContStates ( S ) ; m3kvjvuvj4 ( & ( dw -> rtb ) , & ( dw -> rtdw ) , localX ) ; ssSetPeriodicContState ( S , 0 , 0 , - 3.1415926535897931 , 3.1415926535897931 ) ; ssSetPeriodicContState ( S , 1 , 1 , - 3.1415926535897931 , 3.1415926535897931 ) ; ssSetPeriodicContState ( S , 2 , 2 , - 3.1415926535897931 , 3.1415926535897931 ) ; } static void mdlReset ( SimStruct * S ) { egcgcay4tuh * dw = ( egcgcay4tuh * ) ssGetDWork ( S , 0 ) ; h21fsrthfa * localX = ( h21fsrthfa * ) ssGetContStates ( S ) ; nqasp5wza1 ( & ( dw -> rtb ) , & ( dw -> rtdw ) , localX ) ; } static void mdlPeriodicOutputUpdate ( SimStruct * S , int_T tid ) { egcgcay4tuh * dw = ( egcgcay4tuh * ) ssGetDWork ( S , 0 ) ; real_T const * i_krp1l3jf4p = ( real_T * ) ssGetInputPortSignal ( S , 0 ) ; real_T const * i_jb1g1kp3ja = ( real_T * ) ssGetInputPortSignal ( S , 1 ) ; real_T const * i_mcau1kqq0y = ( real_T * ) ssGetInputPortSignal ( S , 2 ) ; real_T const * i_eics31md2u = ( real_T * ) ssGetInputPortSignal ( S , 3 ) ; real_T const * i_itd21vjha2 = ( real_T * ) ssGetInputPortSignal ( S , 4 ) ; real_T const * i_dvme42dmxf = ( real_T * ) ssGetInputPortSignal ( S , 5 ) ; real_T const * i_onroqiiasl = ( real_T * ) ssGetInputPortSignal ( S , 6 ) ; real_T * o_B_36_1 = ( real_T * ) ssGetOutputPortSignal ( S , 0 ) ; real_T * o_B_36_2 = ( real_T * ) ssGetOutputPortSignal ( S , 1 ) ; real_T * o_B_36_3 = ( real_T * ) ssGetOutputPortSignal ( S , 2 ) ; real_T * o_B_36_4 = ( real_T * ) ssGetOutputPortSignal ( S , 3 ) ; real_T * o_B_36_5 = ( real_T * ) ssGetOutputPortSignal ( S , 4 ) ; real_T * o_B_36_6 = ( real_T * ) ssGetOutputPortSignal ( S , 5 ) ; real_T * o_B_36_7 = ( real_T * ) ssGetOutputPortSignal ( S , 6 ) ; real_T * o_B_36_8 = ( real_T * ) ssGetOutputPortSignal ( S , 7 ) ; real_T * o_B_36_9 = ( real_T * ) ssGetOutputPortSignal ( S , 8 ) ; real_T * o_B_36_10 = ( real_T * ) ssGetOutputPortSignal ( S , 9 ) ; real_T * o_B_36_11 = ( real_T * ) ssGetOutputPortSignal ( S , 10 ) ; real_T * o_B_36_12 = ( real_T * ) ssGetOutputPortSignal ( S , 11 ) ; real_T * o_B_36_13 = ( real_T * ) ssGetOutputPortSignal ( S , 12 ) ; real_T * o_B_36_14 = ( real_T * ) ssGetOutputPortSignal ( S , 13 ) ; real_T * o_B_36_15 = ( real_T * ) ssGetOutputPortSignal ( S , 14 ) ; real_T * o_B_36_16 = ( real_T * ) ssGetOutputPortSignal ( S , 15 ) ; real_T * o_B_36_17 = ( real_T * ) ssGetOutputPortSignal ( S , 16 ) ; real_T * o_B_36_18 = ( real_T * ) ssGetOutputPortSignal ( S , 17 ) ; real_T * o_B_36_19 = ( real_T * ) ssGetOutputPortSignal ( S , 18 ) ; real_T * o_B_36_20 = ( real_T * ) ssGetOutputPortSignal ( S , 19 ) ; real_T * o_B_36_21 = ( real_T * ) ssGetOutputPortSignal ( S , 20 ) ; real_T * o_B_36_22 = ( real_T * ) ssGetOutputPortSignal ( S , 21 ) ; real_T * o_B_36_23 = ( real_T * ) ssGetOutputPortSignal ( S , 22 ) ; real_T * o_B_36_24 = ( real_T * ) ssGetOutputPortSignal ( S , 23 ) ; real_T * o_B_36_25 = ( real_T * ) ssGetOutputPortSignal ( S , 24 ) ; real_T * o_B_36_26 = ( real_T * ) ssGetOutputPortSignal ( S , 25 ) ; real_T * o_B_36_27 = ( real_T * ) ssGetOutputPortSignal ( S , 26 ) ; real_T * o_B_36_28 = ( real_T * ) ssGetOutputPortSignal ( S , 27 ) ; real_T * o_B_36_29 = ( real_T * ) ssGetOutputPortSignal ( S , 28 ) ; real_T * o_B_36_30 = ( real_T * ) ssGetOutputPortSignal ( S , 29 ) ; real_T * o_B_36_31 = ( real_T * ) ssGetOutputPortSignal ( S , 30 ) ; real_T * o_B_36_32 = ( real_T * ) ssGetOutputPortSignal ( S , 31 ) ; real_T * o_B_36_33 = ( real_T * ) ssGetOutputPortSignal ( S , 32 ) ; real_T * o_B_36_34 = ( real_T * ) ssGetOutputPortSignal ( S , 33 ) ; real_T * o_B_36_35 = ( real_T * ) ssGetOutputPortSignal ( S , 34 ) ; real_T * o_B_36_36 = ( real_T * ) ssGetOutputPortSignal ( S , 35 ) ; real_T * o_B_36_37 = ( real_T * ) ssGetOutputPortSignal ( S , 36 ) ; real_T * o_B_36_38 = ( real_T * ) ssGetOutputPortSignal ( S , 37 ) ; real_T * o_B_36_39 = ( real_T * ) ssGetOutputPortSignal ( S , 38 ) ; real_T * o_B_36_40 = ( real_T * ) ssGetOutputPortSignal ( S , 39 ) ; real_T * o_B_36_41 = ( real_T * ) ssGetOutputPortSignal ( S , 40 ) ; real_T * o_B_36_42 = ( real_T * ) ssGetOutputPortSignal ( S , 41 ) ; real_T * o_B_36_43 = ( real_T * ) ssGetOutputPortSignal ( S , 42 ) ; real_T * o_B_36_44 = ( real_T * ) ssGetOutputPortSignal ( S , 43 ) ; real_T * o_B_36_45 = ( real_T * ) ssGetOutputPortSignal ( S , 44 ) ; real_T * o_B_36_46 = ( real_T * ) ssGetOutputPortSignal ( S , 45 ) ; real_T * o_B_36_47 = ( real_T * ) ssGetOutputPortSignal ( S , 46 ) ; real_T * o_B_36_48 = ( real_T * ) ssGetOutputPortSignal ( S , 47 ) ; real_T * o_B_36_49 = ( real_T * ) ssGetOutputPortSignal ( S , 48 ) ; real_T * o_B_36_50 = ( real_T * ) ssGetOutputPortSignal ( S , 49 ) ; real_T * o_B_36_51 = ( real_T * ) ssGetOutputPortSignal ( S , 50 ) ; real_T * o_B_36_52 = ( real_T * ) ssGetOutputPortSignal ( S , 51 ) ; real_T * o_B_36_53 = ( real_T * ) ssGetOutputPortSignal ( S , 52 ) ; real_T * o_B_36_54 = ( real_T * ) ssGetOutputPortSignal ( S , 53 ) ; real_T * o_B_36_55 = ( real_T * ) ssGetOutputPortSignal ( S , 54 ) ; real_T * o_B_36_56 = ( real_T * ) ssGetOutputPortSignal ( S , 55 ) ; real_T * o_B_36_57 = ( real_T * ) ssGetOutputPortSignal ( S , 56 ) ; real_T * o_B_36_58 = ( real_T * ) ssGetOutputPortSignal ( S , 57 ) ; real_T * o_B_36_59 = ( real_T * ) ssGetOutputPortSignal ( S , 58 ) ; real_T * o_B_36_60 = ( real_T * ) ssGetOutputPortSignal ( S , 59 ) ; real_T * o_B_36_61 = ( real_T * ) ssGetOutputPortSignal ( S , 60 ) ; real_T * o_B_36_62 = ( real_T * ) ssGetOutputPortSignal ( S , 61 ) ; real_T * o_B_36_63 = ( real_T * ) ssGetOutputPortSignal ( S , 62 ) ; real_T * o_B_36_64 = ( real_T * ) ssGetOutputPortSignal ( S , 63 ) ; real_T * o_B_36_65 = ( real_T * ) ssGetOutputPortSignal ( S , 64 ) ; real_T * o_B_36_66 = ( real_T * ) ssGetOutputPortSignal ( S , 65 ) ; real_T * o_B_36_67 = ( real_T * ) ssGetOutputPortSignal ( S , 66 ) ; real_T * o_B_36_68 = ( real_T * ) ssGetOutputPortSignal ( S , 67 ) ; real_T * o_B_36_69 = ( real_T * ) ssGetOutputPortSignal ( S , 68 ) ; real_T * o_B_36_70 = ( real_T * ) ssGetOutputPortSignal ( S , 69 ) ; real_T * o_B_36_71 = ( real_T * ) ssGetOutputPortSignal ( S , 70 ) ; real_T * o_B_36_72 = ( real_T * ) ssGetOutputPortSignal ( S , 71 ) ; real_T * o_B_36_73 = ( real_T * ) ssGetOutputPortSignal ( S , 72 ) ; real_T * o_B_36_74 = ( real_T * ) ssGetOutputPortSignal ( S , 73 ) ; real_T * o_B_36_75 = ( real_T * ) ssGetOutputPortSignal ( S , 74 ) ; real_T * o_B_36_76 = ( real_T * ) ssGetOutputPortSignal ( S , 75 ) ; real_T * o_B_36_77 = ( real_T * ) ssGetOutputPortSignal ( S , 76 ) ; real_T * o_B_36_78 = ( real_T * ) ssGetOutputPortSignal ( S , 77 ) ; real_T * o_B_36_79 = ( real_T * ) ssGetOutputPortSignal ( S , 78 ) ; real_T * o_B_36_80 = ( real_T * ) ssGetOutputPortSignal ( S , 79 ) ; real_T * o_B_36_81 = ( real_T * ) ssGetOutputPortSignal ( S , 95 ) ; real_T * o_B_36_82 = ( real_T * ) ssGetOutputPortSignal ( S , 96 ) ; real_T * o_B_36_83 = ( real_T * ) ssGetOutputPortSignal ( S , 97 ) ; real_T * o_B_36_84 = ( real_T * ) ssGetOutputPortSignal ( S , 98 ) ; real_T * o_B_36_85 = ( real_T * ) ssGetOutputPortSignal ( S , 99 ) ; real_T * o_B_36_86 = ( real_T * ) ssGetOutputPortSignal ( S , 100 ) ; real_T * o_B_36_87 = ( real_T * ) ssGetOutputPortSignal ( S , 101 ) ; real_T * o_B_36_88 = ( real_T * ) ssGetOutputPortSignal ( S , 102 ) ; real_T * o_B_36_89 = ( real_T * ) ssGetOutputPortSignal ( S , 103 ) ; real_T * o_B_36_90 = ( real_T * ) ssGetOutputPortSignal ( S , 104 ) ; real_T * o_B_36_91 = ( real_T * ) ssGetOutputPortSignal ( S , 105 ) ; real_T * o_B_36_92 = ( real_T * ) ssGetOutputPortSignal ( S , 106 ) ; real_T * o_B_36_93 = ( real_T * ) ssGetOutputPortSignal ( S , 113 ) ; real_T * o_B_36_94 = ( real_T * ) ssGetOutputPortSignal ( S , 114 ) ; real_T * o_B_36_95 = ( real_T * ) ssGetOutputPortSignal ( S , 115 ) ; real_T * o_B_36_96 = ( real_T * ) ssGetOutputPortSignal ( S , 116 ) ; real_T * o_B_36_97 = ( real_T * ) ssGetOutputPortSignal ( S , 117 ) ; real_T * o_B_36_98 = ( real_T * ) ssGetOutputPortSignal ( S , 118 ) ; real_T * o_B_36_99 = ( real_T * ) ssGetOutputPortSignal ( S , 119 ) ; real_T * o_B_36_100 = ( real_T * ) ssGetOutputPortSignal ( S , 120 ) ; real_T * o_B_36_101 = ( real_T * ) ssGetOutputPortSignal ( S , 121 ) ; real_T * o_B_36_102 = ( real_T * ) ssGetOutputPortSignal ( S , 122 ) ; real_T * o_B_36_103 = ( real_T * ) ssGetOutputPortSignal ( S , 123 ) ; real_T * o_B_36_104 = ( real_T * ) ssGetOutputPortSignal ( S , 124 ) ; real_T * o_B_36_105 = ( real_T * ) ssGetOutputPortSignal ( S , 125 ) ; real_T * o_B_36_106 = ( real_T * ) ssGetOutputPortSignal ( S , 126 ) ; real_T * o_B_36_107 = ( real_T * ) ssGetOutputPortSignal ( S , 127 ) ; real_T * o_B_36_108 = ( real_T * ) ssGetOutputPortSignal ( S , 128 ) ; real_T * o_B_36_109 = ( real_T * ) ssGetOutputPortSignal ( S , 129 ) ; real_T * o_B_36_110 = ( real_T * ) ssGetOutputPortSignal ( S , 130 ) ; real_T * o_B_36_111 = ( real_T * ) ssGetOutputPortSignal ( S , 131 ) ; real_T * o_B_36_112 = ( real_T * ) ssGetOutputPortSignal ( S , 132 ) ; real_T * o_B_36_113 = ( real_T * ) ssGetOutputPortSignal ( S , 133 ) ; real_T * o_B_36_114 = ( real_T * ) ssGetOutputPortSignal ( S , 134 ) ; real_T * o_B_36_115 = ( real_T * ) ssGetOutputPortSignal ( S , 135 ) ; real_T * o_B_36_116 = ( real_T * ) ssGetOutputPortSignal ( S , 136 ) ; real_T * o_B_36_117 = ( real_T * ) ssGetOutputPortSignal ( S , 137 ) ; real_T * o_B_36_118 = ( real_T * ) ssGetOutputPortSignal ( S , 138 ) ; real_T * o_B_36_119 = ( real_T * ) ssGetOutputPortSignal ( S , 139 ) ; real_T * o_B_36_120 = ( real_T * ) ssGetOutputPortSignal ( S , 140 ) ; real_T * o_B_36_121 = ( real_T * ) ssGetOutputPortSignal ( S , 141 ) ; real_T * o_B_36_122 = ( real_T * ) ssGetOutputPortSignal ( S , 142 ) ; real_T * o_B_36_123 = ( real_T * ) ssGetOutputPortSignal ( S , 143 ) ; real_T * o_B_36_124 = ( real_T * ) ssGetOutputPortSignal ( S , 144 ) ; real_T * o_B_36_125 = ( real_T * ) ssGetOutputPortSignal ( S , 145 ) ; real_T * o_B_36_126 = ( real_T * ) ssGetOutputPortSignal ( S , 146 ) ; real_T * o_B_36_127 = ( real_T * ) ssGetOutputPortSignal ( S , 147 ) ; real_T * o_B_36_128 = ( real_T * ) ssGetOutputPortSignal ( S , 148 ) ; real_T * o_B_36_129 = ( real_T * ) ssGetOutputPortSignal ( S , 149 ) ; real_T * o_B_36_130 = ( real_T * ) ssGetOutputPortSignal ( S , 150 ) ; real_T * o_B_36_131 = ( real_T * ) ssGetOutputPortSignal ( S , 151 ) ; real_T * o_B_36_132 = ( real_T * ) ssGetOutputPortSignal ( S , 152 ) ; real_T * o_B_36_133 = ( real_T * ) ssGetOutputPortSignal ( S , 153 ) ; real_T * o_B_36_134 = ( real_T * ) ssGetOutputPortSignal ( S , 154 ) ; real_T * o_B_36_135 = ( real_T * ) ssGetOutputPortSignal ( S , 155 ) ; real_T * o_B_36_136 = ( real_T * ) ssGetOutputPortSignal ( S , 156 ) ; real_T * o_B_36_137 = ( real_T * ) ssGetOutputPortSignal ( S , 157 ) ; real_T * o_B_36_138 = ( real_T * ) ssGetOutputPortSignal ( S , 158 ) ; real_T * o_B_36_139 = ( real_T * ) ssGetOutputPortSignal ( S , 159 ) ; real_T * o_B_36_140 = ( real_T * ) ssGetOutputPortSignal ( S , 160 ) ; real_T * o_B_36_141 = ( real_T * ) ssGetOutputPortSignal ( S , 161 ) ; real_T * o_B_36_142 = ( real_T * ) ssGetOutputPortSignal ( S , 162 ) ; real_T * o_B_36_143 = ( real_T * ) ssGetOutputPortSignal ( S , 163 ) ; real_T * o_B_36_144 = ( real_T * ) ssGetOutputPortSignal ( S , 164 ) ; real_T * o_B_36_145 = ( real_T * ) ssGetOutputPortSignal ( S , 165 ) ; real_T * o_B_36_146 = ( real_T * ) ssGetOutputPortSignal ( S , 166 ) ; h21fsrthfa * localX = ( h21fsrthfa * ) ssGetContStates ( S ) ; if ( tid == 0 ) { PassVeh14DOF ( & ( dw -> rtm ) , i_krp1l3jf4p , i_jb1g1kp3ja , i_mcau1kqq0y , i_eics31md2u , i_itd21vjha2 , i_dvme42dmxf , i_onroqiiasl , o_B_36_1 , o_B_36_2 , o_B_36_3 , o_B_36_4 , o_B_36_5 , o_B_36_6 , o_B_36_7 , o_B_36_8 , o_B_36_9 , o_B_36_10 , o_B_36_11 , o_B_36_12 , o_B_36_13 , o_B_36_14 , o_B_36_15 , o_B_36_16 , o_B_36_17 , o_B_36_18 , o_B_36_19 , o_B_36_20 , o_B_36_21 , o_B_36_22 , o_B_36_23 , o_B_36_24 , o_B_36_25 , o_B_36_26 , o_B_36_27 , o_B_36_28 , o_B_36_29 , o_B_36_30 , o_B_36_31 , o_B_36_32 , o_B_36_33 , o_B_36_34 , o_B_36_35 , o_B_36_36 , o_B_36_37 , o_B_36_38 , o_B_36_39 , o_B_36_40 , o_B_36_41 , o_B_36_42 , o_B_36_43 , o_B_36_44 , o_B_36_45 , o_B_36_46 , o_B_36_47 , o_B_36_48 , o_B_36_49 , o_B_36_50 , o_B_36_51 , o_B_36_52 , o_B_36_53 , o_B_36_54 , o_B_36_55 , o_B_36_56 , o_B_36_57 , o_B_36_58 , o_B_36_59 , o_B_36_60 , o_B_36_61 , o_B_36_62 , o_B_36_63 , o_B_36_64 , o_B_36_65 , o_B_36_66 , o_B_36_67 , o_B_36_68 , o_B_36_69 , o_B_36_70 , o_B_36_71 , o_B_36_72 , o_B_36_73 , o_B_36_74 , o_B_36_75 , o_B_36_76 , o_B_36_77 , o_B_36_78 , o_B_36_79 , o_B_36_80 , o_B_36_81 , o_B_36_82 , o_B_36_83 , o_B_36_84 , o_B_36_85 , o_B_36_86 , o_B_36_87 , o_B_36_88 , o_B_36_89 , o_B_36_90 , o_B_36_91 , o_B_36_92 , o_B_36_93 , o_B_36_94 , o_B_36_95 , o_B_36_96 , o_B_36_97 , o_B_36_98 , o_B_36_99 , o_B_36_100 , o_B_36_101 , o_B_36_102 , o_B_36_103 , o_B_36_104 , o_B_36_105 , o_B_36_106 , o_B_36_107 , o_B_36_108 , o_B_36_109 , o_B_36_110 , o_B_36_111 , o_B_36_112 , o_B_36_113 , o_B_36_114 , o_B_36_115 , o_B_36_116 , o_B_36_117 , o_B_36_118 , o_B_36_119 , o_B_36_120 , o_B_36_121 , o_B_36_122 , o_B_36_123 , o_B_36_124 , o_B_36_125 , o_B_36_126 , o_B_36_127 , o_B_36_128 , o_B_36_129 , o_B_36_130 , o_B_36_131 , o_B_36_132 , o_B_36_133 , o_B_36_134 , o_B_36_135 , o_B_36_136 , o_B_36_137 , o_B_36_138 , o_B_36_139 , o_B_36_140 , o_B_36_141 , o_B_36_142 , o_B_36_143 , o_B_36_144 , o_B_36_145 , o_B_36_146 , & ( dw -> rtb ) , & ( dw -> rtdw ) , localX , & ( dw -> rtzce ) ) ; jpl1qt0btr ( & ( dw -> rtm ) , & ( dw -> rtb ) , & ( dw -> rtdw ) ) ; } } static void mdlInitializeSizes ( SimStruct * S ) { if ( ( S -> mdlInfo -> genericFcn != ( NULL ) ) && ( ! ( S -> mdlInfo -> genericFcn ) ( S , GEN_FCN_CHK_MODELREF_SFUN_HAS_MODEL_BLOCK , - 1 , ( NULL ) ) ) ) { return ; } ssSetNumSFcnParams ( S , 0 ) ; ssFxpSetU32BitRegionCompliant ( S , 1 ) ; if ( S -> mdlInfo -> genericFcn != ( NULL ) ) { _GenericFcn fcn = S -> mdlInfo -> genericFcn ; } ssSetRTWGeneratedSFcn ( S , 2 ) ; ssSetNumContStates ( S , 68 ) ; ssSetNumDiscStates ( S , 0 ) ; ssSetNumPeriodicContStates ( S , 3 ) ; ssSetSymbolicDimsSupport ( S , true ) ; slmrInitializeIOPortDataVectors ( S , 7 , 167 ) ; if ( ! ssSetNumInputPorts ( S , 7 ) ) return ; if ( ! ssSetInputPortVectorDimension ( S , 0 , 4 ) ) return ; ssSetInputPortDimensionsMode ( S , 0 , FIXED_DIMS_MODE ) ; ssSetInputPortFrameData ( S , 0 , FRAME_NO ) ; if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetInputPortDataType ( S , 0 , SS_DOUBLE ) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetInputPortUnit ( S , 0 , unitIdReg ) ;
#endif
} ssSetInputPortDirectFeedThrough ( S , 0 , 1 ) ;
ssSetInputPortRequiredContiguous ( S , 0 , 1 ) ; ssSetInputPortOptimOpts ( S
, 0 , SS_NOT_REUSABLE_AND_LOCAL ) ; ssSetInputPortOverWritable ( S , 0 ,
false ) ; ssSetInputPortSampleTime ( S , 0 , 0.0 ) ; ssSetInputPortOffsetTime
( S , 0 , 0.0 ) ; if ( ! ssSetInputPortVectorDimension ( S , 1 , 4 ) ) return
; ssSetInputPortDimensionsMode ( S , 1 , FIXED_DIMS_MODE ) ;
ssSetInputPortFrameData ( S , 1 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetInputPortDataType ( S , 1 , SS_DOUBLE ) ;
} if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetInputPortUnit ( S , 1 , unitIdReg ) ;
#endif
} ssSetInputPortDirectFeedThrough ( S , 1 , 1 ) ;
ssSetInputPortRequiredContiguous ( S , 1 , 1 ) ; ssSetInputPortOptimOpts ( S
, 1 , SS_REUSABLE_AND_LOCAL ) ; ssSetInputPortOverWritable ( S , 1 , false )
; ssSetInputPortSampleTime ( S , 1 , 0.0 ) ; ssSetInputPortOffsetTime ( S , 1
, 0.0 ) ; if ( ! ssSetInputPortVectorDimension ( S , 2 , 4 ) ) return ;
ssSetInputPortDimensionsMode ( S , 2 , FIXED_DIMS_MODE ) ;
ssSetInputPortFrameData ( S , 2 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetInputPortDataType ( S , 2 , SS_DOUBLE ) ;
} if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "Pa" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetInputPortUnit ( S , 2 , unitIdReg ) ;
#endif
} ssSetInputPortDirectFeedThrough ( S , 2 , 1 ) ;
ssSetInputPortRequiredContiguous ( S , 2 , 1 ) ; ssSetInputPortOptimOpts ( S
, 2 , SS_REUSABLE_AND_LOCAL ) ; ssSetInputPortOverWritable ( S , 2 , false )
; ssSetInputPortSampleTime ( S , 2 , 0.0 ) ; ssSetInputPortOffsetTime ( S , 2
, 0.0 ) ; if ( ! ssSetInputPortVectorDimension ( S , 3 , 3 ) ) return ;
ssSetInputPortDimensionsMode ( S , 3 , FIXED_DIMS_MODE ) ;
ssSetInputPortFrameData ( S , 3 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetInputPortDataType ( S , 3 , SS_DOUBLE ) ;
} if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetInputPortUnit ( S , 3 , unitIdReg ) ;
#endif
} ssSetInputPortDirectFeedThrough ( S , 3 , 1 ) ;
ssSetInputPortRequiredContiguous ( S , 3 , 1 ) ; ssSetInputPortOptimOpts ( S
, 3 , SS_REUSABLE_AND_LOCAL ) ; ssSetInputPortOverWritable ( S , 3 , false )
; ssSetInputPortSampleTime ( S , 3 , 0.0 ) ; ssSetInputPortOffsetTime ( S , 3
, 0.0 ) ; if ( ! ssSetInputPortVectorDimension ( S , 4 , 4 ) ) return ;
ssSetInputPortDimensionsMode ( S , 4 , FIXED_DIMS_MODE ) ;
ssSetInputPortFrameData ( S , 4 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetInputPortDataType ( S , 4 , SS_DOUBLE ) ;
} if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetInputPortUnit ( S , 4 , unitIdReg ) ;
#endif
} ssSetInputPortDirectFeedThrough ( S , 4 , 1 ) ;
ssSetInputPortRequiredContiguous ( S , 4 , 1 ) ; ssSetInputPortOptimOpts ( S
, 4 , SS_REUSABLE_AND_LOCAL ) ; ssSetInputPortOverWritable ( S , 4 , false )
; ssSetInputPortSampleTime ( S , 4 , 0.0 ) ; ssSetInputPortOffsetTime ( S , 4
, 0.0 ) ; if ( ! ssSetInputPortVectorDimension ( S , 5 , 4 ) ) return ;
ssSetInputPortDimensionsMode ( S , 5 , FIXED_DIMS_MODE ) ;
ssSetInputPortFrameData ( S , 5 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetInputPortDataType ( S , 5 , SS_DOUBLE ) ;
} if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "1" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetInputPortUnit ( S , 5 , unitIdReg ) ;
#endif
} ssSetInputPortDirectFeedThrough ( S , 5 , 1 ) ;
ssSetInputPortRequiredContiguous ( S , 5 , 1 ) ; ssSetInputPortOptimOpts ( S
, 5 , SS_REUSABLE_AND_LOCAL ) ; ssSetInputPortOverWritable ( S , 5 , false )
; ssSetInputPortSampleTime ( S , 5 , 0.0 ) ; ssSetInputPortOffsetTime ( S , 5
, 0.0 ) ; { DimsInfo_T inDims ; int_T inDimsVals [ 3 ] = { 3 , 3 , 4 } ;
inDims . width = 36 ; inDims . numDims = 3 ; inDims . dims = inDimsVals ;
inDims . nextSigDims = ( NULL ) ; if ( ! ssSetInputPortDimensionInfo ( S , 6
, & inDims ) ) return ; } ssSetInputPortDimensionsMode ( S , 6 ,
FIXED_DIMS_MODE ) ; ssSetInputPortFrameData ( S , 6 , FRAME_NO ) ; if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetInputPortDataType ( S , 6 , SS_DOUBLE ) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetInputPortUnit ( S , 6 , unitIdReg ) ;
#endif
} ssSetInputPortDirectFeedThrough ( S , 6 , 1 ) ;
ssSetInputPortRequiredContiguous ( S , 6 , 1 ) ; ssSetInputPortOptimOpts ( S
, 6 , SS_NOT_REUSABLE_AND_GLOBAL ) ; ssSetInputPortOverWritable ( S , 6 ,
false ) ; ssSetInputPortSampleTime ( S , 6 , 0.0 ) ; ssSetInputPortOffsetTime
( S , 6 , 0.0 ) ; if ( ! ssSetNumOutputPorts ( S , 167 ) ) return ; if ( !
ssSetOutputPortVectorDimension ( S , 0 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 0 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 0 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 0 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 0 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 0 , 0.0 ) ; ssSetOutputPortOffsetTime ( S ,
0 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 0 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 0 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 0 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 0 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 1 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 1 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 1 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 1 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 1 , 0.0 ) ; ssSetOutputPortOffsetTime ( S ,
1 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 1 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 1 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 1 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 1 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 2 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 2 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 2 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 2 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 2 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 2 , 0.0 ) ; ssSetOutputPortOffsetTime ( S ,
2 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 2 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 2 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 2 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 2 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 3 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 3 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 3 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 3 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 3 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 3 , 0.0 ) ; ssSetOutputPortOffsetTime ( S ,
3 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 3 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 3 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 3 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 3 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 4 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 4 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 4 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 4 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 4 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 4 , 0.0 ) ; ssSetOutputPortOffsetTime ( S ,
4 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 4 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 4 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 4 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 4 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 5 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 5 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 5 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 5 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 5 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 5 , 0.0 ) ; ssSetOutputPortOffsetTime ( S ,
5 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 5 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 5 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 5 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 5 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 6 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 6 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 6 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 6 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 6 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 6 , 0.0 ) ; ssSetOutputPortOffsetTime ( S ,
6 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 6 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 6 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 6 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 6 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 7 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 7 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 7 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 7 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 7 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 7 , 0.0 ) ; ssSetOutputPortOffsetTime ( S ,
7 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 7 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 7 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 7 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 7 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 8 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 8 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 8 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 8 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 8 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 8 , 0.0 ) ; ssSetOutputPortOffsetTime ( S ,
8 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 8 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 8 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 8 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 8 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 9 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 9 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 9 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 9 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 9 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 9 , 0.0 ) ; ssSetOutputPortOffsetTime ( S ,
9 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 9 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 9 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 9 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 9 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 10 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 10 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 10 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 10 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 10 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 10 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 10 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 10 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 10 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 10 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 10 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 11 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 11 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 11 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 11 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 11 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 11 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 11 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 11 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 11 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 11 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 11 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 12 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 12 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 12 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 12 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 12 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 12 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 12 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 12 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 12 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 12 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 12 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 13 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 13 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 13 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 13 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 13 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 13 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 13 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 13 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 13 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 13 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 13 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 14 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 14 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 14 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 14 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 14 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 14 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 14 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 14 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 14 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 14 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 14 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 15 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 15 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 15 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 15 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 15 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 15 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 15 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 15 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 15 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 15 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 15 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 16 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 16 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 16 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 16 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 16 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 16 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 16 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 16 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 16 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 16 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 16 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 17 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 17 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 17 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 17 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 17 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 17 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 17 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 17 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 17 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 17 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 17 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 18 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 18 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 18 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 18 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 18 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 18 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 18 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 18 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 18 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 18 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 18 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 19 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 19 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 19 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 19 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 19 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 19 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 19 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 19 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 19 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 19 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 19 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 20 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 20 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 20 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 20 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 20 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 20 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 20 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 20 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 20 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 20 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 20 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 21 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 21 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 21 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 21 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 21 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 21 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 21 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 21 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 21 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 21 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 21 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 22 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 22 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 22 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 22 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 22 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 22 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 22 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 22 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 22 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 22 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 22 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 23 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 23 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 23 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 23 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 23 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 23 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 23 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 23 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 23 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 23 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 23 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 24 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 24 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 24 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 24 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 24 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 24 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 24 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 24 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 24 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 24 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 24 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 25 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 25 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 25 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 25 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 25 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 25 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 25 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 25 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 25 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 25 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 25 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 26 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 26 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 26 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 26 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 26 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 26 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 26 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 26 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 26 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 26 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 26 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 27 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 27 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 27 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 27 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 27 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 27 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 27 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 27 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 27 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 27 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 27 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 28 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 28 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 28 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 28 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 28 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 28 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 28 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 28 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 28 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 28 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 28 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 29 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 29 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 29 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 29 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 29 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 29 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 29 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 29 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 29 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 29 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 29 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 30 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 30 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 30 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 30 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 30 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 30 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 30 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 30 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 30 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 30 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 30 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 31 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 31 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 31 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 31 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 31 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 31 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 31 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 31 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 31 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 31 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 31 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 32 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 32 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 32 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 32 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 32 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 32 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 32 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 32 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 32 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 32 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 32 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 33 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 33 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 33 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 33 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 33 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 33 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 33 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 33 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 33 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 33 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 33 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 34 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 34 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 34 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 34 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 34 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 34 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 34 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 34 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 34 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 34 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 34 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 35 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 35 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 35 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 35 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 35 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 35 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 35 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 35 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 35 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 35 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 35 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 36 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 36 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 36 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 36 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 36 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 36 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 36 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 36 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 36 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 36 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 36 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 37 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 37 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 37 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 37 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 37 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 37 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 37 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 37 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 37 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 37 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 37 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 38 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 38 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 38 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 38 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 38 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 38 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 38 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 38 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 38 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 38 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 38 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 39 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 39 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 39 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 39 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 39 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 39 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 39 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 39 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 39 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 39 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 39 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 40 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 40 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 40 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 40 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 40 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 40 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 40 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 40 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 40 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 40 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 40 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 41 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 41 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 41 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 41 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 41 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 41 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 41 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 41 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 41 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 41 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 41 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 42 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 42 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 42 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 42 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 42 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 42 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 42 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 42 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 42 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 42 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 42 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 43 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 43 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 43 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 43 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 43 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 43 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 43 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 43 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 43 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 43 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 43 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 44 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 44 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 44 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 44 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 44 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 44 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 44 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 44 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 44 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 44 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 44 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 45 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 45 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 45 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 45 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 45 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 45 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 45 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 45 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 45 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 45 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 45 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 46 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 46 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 46 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 46 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 46 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 46 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 46 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 46 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 46 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 46 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 46 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 47 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 47 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 47 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 47 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 47 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 47 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 47 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 47 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 47 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 47 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 47 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 48 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 48 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 48 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 48 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 48 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 48 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 48 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 48 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 48 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 48 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 48 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 49 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 49 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 49 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 49 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad/s" , & unitIdReg ) ; if
( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 49 ,
unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 49 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 49 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 49 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 49 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 49 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 49 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 50 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 50 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 50 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 50 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad/s" , & unitIdReg ) ; if
( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 50 ,
unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 50 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 50 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 50 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 50 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 50 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 50 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 51 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 51 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 51 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 51 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad/s" , & unitIdReg ) ; if
( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 51 ,
unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 51 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 51 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 51 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 51 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 51 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 51 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 52 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 52 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 52 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 52 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "gn" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 52 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 52 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 52 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 52 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 52 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 52 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 52 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 53 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 53 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 53 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 53 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "gn" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 53 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 53 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 53 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 53 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 53 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 53 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 53 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 54 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 54 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 54 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 54 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "gn" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 54 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 54 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 54 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 54 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 54 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 54 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 54 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 55 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 55 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 55 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 55 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s^2" , & unitIdReg ) ; if
( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 55 ,
unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 55 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 55 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 55 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 55 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 55 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 55 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 56 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 56 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 56 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 56 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s^2" , & unitIdReg ) ; if
( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 56 ,
unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 56 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 56 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 56 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 56 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 56 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 56 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 57 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 57 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 57 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 57 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s^2" , & unitIdReg ) ; if
( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 57 ,
unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 57 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 57 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 57 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 57 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 57 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 57 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 58 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 58 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 58 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 58 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad/s^2" , & unitIdReg ) ;
if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 58 ,
unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 58 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 58 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 58 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 58 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 58 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 58 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 59 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 59 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 59 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 59 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad/s^2" , & unitIdReg ) ;
if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 59 ,
unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 59 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 59 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 59 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 59 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 59 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 59 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 60 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 60 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 60 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 60 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad/s^2" , & unitIdReg ) ;
if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 60 ,
unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 60 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 60 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 60 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 60 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 60 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 60 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 61 , 3 , 3 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 61 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 61 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 61 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "1" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 61 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 61 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 61 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 61 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 61 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 61 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 61 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 62 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 62 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 62 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 62 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 62 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 62 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 62 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 62 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 62 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 62 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 62 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 63 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 63 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 63 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 63 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 63 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 63 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 63 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 63 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 63 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 63 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 63 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 64 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 64 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 64 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 64 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 64 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 64 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 64 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 64 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 64 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 64 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 64 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 65 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 65 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 65 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 65 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 65 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 65 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 65 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 65 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 65 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 65 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 65 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 66 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 66 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 66 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 66 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 66 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 66 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 66 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 66 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 66 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 66 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 66 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 67 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 67 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 67 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 67 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 67 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 67 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 67 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 67 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 67 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 67 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 67 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 68 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 68 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 68 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 68 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 68 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 68 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 68 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 68 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 68 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 68 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 68 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 69 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 69 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 69 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 69 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 69 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 69 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 69 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 69 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 69 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 69 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 69 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 70 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 70 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 70 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 70 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 70 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 70 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 70 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 70 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 70 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 70 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 70 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 71 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 71 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 71 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 71 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 71 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 71 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 71 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 71 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 71 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 71 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 71 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 72 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 72 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 72 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 72 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 72 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 72 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 72 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 72 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 72 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 72 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 72 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 73 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 73 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 73 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 73 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 73 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 73 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 73 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 73 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 73 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 73 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 73 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 74 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 74 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 74 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 74 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 74 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 74 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 74 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 74 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 74 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 74 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 74 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 75 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 75 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 75 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 75 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 75 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 75 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 75 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 75 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 75 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 75 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 75 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 76 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 76 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 76 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 76 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 76 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 76 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 76 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 76 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 76 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 76 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 76 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 77 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 77 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 77 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 77 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 77 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 77 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 77 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 77 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 77 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 77 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 77 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 78 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 78 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 78 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 78 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 78 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 78 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 78 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 78 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 78 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 78 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 78 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 79 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 79 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 79 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 79 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 79 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 79 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 79 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 79 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 79 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 79 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 79 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 80 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 80 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 80 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 80 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 80 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 80 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 80 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 80 , 0 ) ; ssSetOutputPortOkToMerge
( S , 80 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
80 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 80 ,
SS_NOT_REUSABLE_AND_GLOBAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 81
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 81 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 81 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 81 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 81 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 81 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 81 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 81 , 0 ) ; ssSetOutputPortOkToMerge
( S , 81 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
81 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 81 ,
SS_NOT_REUSABLE_AND_GLOBAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 82
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 82 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 82 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 82 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 82 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 82 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 82 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 82 , 0 ) ; ssSetOutputPortOkToMerge
( S , 82 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
82 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 82 ,
SS_NOT_REUSABLE_AND_GLOBAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 83
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 83 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 83 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 83 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 83 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 83 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 83 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 83 , 0 ) ; ssSetOutputPortOkToMerge
( S , 83 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
83 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 83 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 84
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 84 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 84 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 84 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 84 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 84 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 84 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 84 , 0 ) ; ssSetOutputPortOkToMerge
( S , 84 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
84 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 84 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 85
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 85 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 85 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 85 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 85 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 85 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 85 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 85 , 0 ) ; ssSetOutputPortOkToMerge
( S , 85 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
85 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 85 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 86
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 86 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 86 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 86 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 86 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 86 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 86 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 86 , 0 ) ; ssSetOutputPortOkToMerge
( S , 86 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
86 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 86 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 87
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 87 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 87 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 87 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 87 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 87 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 87 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 87 , 0 ) ; ssSetOutputPortOkToMerge
( S , 87 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
87 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 87 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 88
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 88 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 88 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 88 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 88 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 88 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 88 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 88 , 0 ) ; ssSetOutputPortOkToMerge
( S , 88 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
88 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 88 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 89
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 89 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 89 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 89 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 89 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 89 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 89 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 89 , 0 ) ; ssSetOutputPortOkToMerge
( S , 89 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
89 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 89 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 90
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 90 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 90 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 90 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 90 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 90 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 90 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 90 , 0 ) ; ssSetOutputPortOkToMerge
( S , 90 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
90 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 90 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 91
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 91 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 91 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 91 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 91 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 91 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 91 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 91 , 0 ) ; ssSetOutputPortOkToMerge
( S , 91 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
91 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 91 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 92
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 92 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 92 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 92 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 92 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 92 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 92 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 92 , 0 ) ; ssSetOutputPortOkToMerge
( S , 92 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
92 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 92 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 93
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 93 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 93 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 93 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 93 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 93 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 93 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 93 , 0 ) ; ssSetOutputPortOkToMerge
( S , 93 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
93 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 93 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 94
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 94 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 94 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 94 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 94 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 94 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 94 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 94 , 0 ) ; ssSetOutputPortOkToMerge
( S , 94 , SS_OK_TO_MERGE_CONDITIONAL ) ; ssSetOutputPortICAttributes ( S ,
94 , false , false , false ) ; ssSetOutputPortOptimOpts ( S , 94 ,
SS_NOT_REUSABLE_AND_LOCAL ) ; if ( ! ssSetOutputPortVectorDimension ( S , 95
, 1 ) ) return ; ssSetOutputPortDimensionsMode ( S , 95 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 95 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 95 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 95 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 95 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 95 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 95 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 95 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 95 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 95 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 96 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 96 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 96 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 96 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 96 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 96 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 96 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 96 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 96 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 96 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 96 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 97 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 97 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 97 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 97 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 97 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 97 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 97 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 97 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 97 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 97 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 97 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 98 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 98 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 98 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 98 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 98 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 98 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 98 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 98 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 98 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 98 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 98 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 99 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 99 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 99 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 99 , SS_DOUBLE )
; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 99 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 99 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 99 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 99 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 99 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 99 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 99 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 100 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 100 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 100 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 100 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 100 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 100 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 100 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 100 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 100 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 100 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 100 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 101 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 101 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 101 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 101 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 101 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 101 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 101 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 101 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 101 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 101 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 101 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 102 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 102 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 102 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 102 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 102 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 102 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 102 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 102 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 102 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 102 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 102 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 103 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 103 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 103 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 103 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 103 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 103 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 103 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 103 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 103 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 103 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 103 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 104 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 104 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 104 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 104 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 104 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 104 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 104 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 104 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 104 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 104 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 104 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 105 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 105 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 105 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 105 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 105 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 105 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 105 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 105 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 105 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 105 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 105 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 106 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 106 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 106 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 106 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 106 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 106 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 106 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 106 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 106 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 106 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 106 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 107 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 107 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 107 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 107 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 107 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 107 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 107 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 107 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 107 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 107 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 107 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 108 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 108 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 108 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 108 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 108 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 108 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 108 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 108 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 108 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 108 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 108 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 109 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 109 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 109 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 109 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 109 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 109 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 109 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 109 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 109 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 109 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 109 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 110 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 110 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 110 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 110 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 110 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 110 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 110 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 110 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 110 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 110 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 110 , SS_NOT_REUSABLE_AND_GLOBAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 111 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 111 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 111 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 111 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 111 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 111 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 111 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 111 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 111 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 111 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 111 , SS_NOT_REUSABLE_AND_GLOBAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 112 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 112 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 112 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 112 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 112 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 112 , mxGetInf ( ) ) ;
ssSetOutputPortOffsetTime ( S , 112 , 0 ) ;
ssSetOutputPortDiscreteValuedOutput ( S , 112 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 112 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 112 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 112 , SS_NOT_REUSABLE_AND_GLOBAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 113 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 113 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 113 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 113 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 113 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 113 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 113 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 113 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 113 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 113 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 113 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 114 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 114 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 114 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 114 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 114 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 114 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 114 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 114 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 114 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 114 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 114 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 115 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 115 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 115 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 115 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 115 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 115 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 115 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 115 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 115 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 115 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 115 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 116 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 116 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 116 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 116 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 116 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 116 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 116 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 116 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 116 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 116 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 116 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 117 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 117 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 117 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 117 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 117 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 117 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 117 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 117 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 117 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 117 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 117 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 118 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 118 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 118 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 118 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 118 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 118 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 118 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 118 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 118 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 118 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 118 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 119 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 119 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 119 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 119 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 119 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 119 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 119 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 119 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 119 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 119 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 119 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 120 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 120 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 120 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 120 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 120 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 120 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 120 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 120 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 120 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 120 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 120 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 121 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 121 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 121 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 121 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 121 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 121 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 121 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 121 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 121 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 121 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 121 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 122 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 122 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 122 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 122 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 122 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 122 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 122 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 122 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 122 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 122 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 122 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 123 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 123 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 123 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 123 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 123 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 123 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 123 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 123 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 123 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 123 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 123 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 124 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 124 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 124 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 124 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 124 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 124 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 124 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 124 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 124 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 124 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 124 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 125 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 125 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 125 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 125 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 125 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 125 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 125 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 125 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 125 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 125 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 125 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 126 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 126 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 126 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 126 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 126 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 126 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 126 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 126 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 126 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 126 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 126 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 127 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 127 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 127 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 127 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 127 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 127 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 127 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 127 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 127 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 127 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 127 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 128 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 128 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 128 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 128 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 128 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 128 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 128 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 128 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 128 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 128 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 128 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 129 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 129 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 129 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 129 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 129 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 129 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 129 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 129 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 129 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 129 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 129 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 130 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 130 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 130 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 130 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 130 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 130 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 130 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 130 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 130 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 130 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 130 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 131 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 131 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 131 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 131 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 131 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 131 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 131 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 131 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 131 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 131 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 131 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 132 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 132 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 132 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 132 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 132 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 132 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 132 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 132 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 132 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 132 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 132 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 133 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 133 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 133 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 133 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 133 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 133 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 133 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 133 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 133 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 133 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 133 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 134 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 134 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 134 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 134 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 134 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 134 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 134 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 134 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 134 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 134 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 134 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 135 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 135 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 135 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 135 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 135 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 135 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 135 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 135 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 135 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 135 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 135 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 136 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 136 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 136 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 136 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 136 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 136 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 136 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 136 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 136 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 136 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 136 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 137 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 137 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 137 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 137 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 137 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 137 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 137 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 137 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 137 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 137 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 137 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 138 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 138 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 138 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 138 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 138 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 138 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 138 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 138 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 138 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 138 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 138 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 139 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 139 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 139 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 139 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 139 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 139 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 139 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 139 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 139 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 139 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 139 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 140 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 140 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 140 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 140 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 140 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 140 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 140 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 140 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 140 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 140 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 140 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 141 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 141 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 141 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 141 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 141 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 141 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 141 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 141 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 141 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 141 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 141 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 142 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 142 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 142 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 142 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 142 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 142 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 142 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 142 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 142 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 142 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 142 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 143 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 143 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 143 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 143 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "W" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 143 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 143 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 143 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 143 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 143 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 143 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 143 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 144 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 144 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 144 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 144 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "W" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 144 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 144 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 144 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 144 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 144 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 144 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 144 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 145 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 145 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 145 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 145 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 145 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 145 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 145 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 145 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 145 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 145 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 145 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 146 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 146 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 146 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 146 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 146 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 146 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 146 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 146 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 146 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 146 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 146 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortMatrixDimensions ( S , 147 , 1 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 147 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 147 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 147 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 147 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 147 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 147 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 147 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 147 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 147 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 147 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 148 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 148 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 148 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 148 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 148 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 148 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 148 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 148 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 148 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 148 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 148 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 149 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 149 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 149 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 149 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 149 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 149 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 149 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 149 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 149 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 149 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 149 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 150 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 150 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 150 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 150 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 150 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 150 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 150 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 150 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 150 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 150 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 150 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 151 , 1 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 151 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 151 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 151 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 151 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 151 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 151 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 151 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 151 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 151 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 151 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 152 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 152 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 152 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 152 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad/s" , & unitIdReg ) ; if
( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 152 ,
unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 152 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 152 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 152 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 152 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 152 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 152 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 153 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 153 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 153 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 153 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 153 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 153 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 153 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 153 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 153 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 153 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 153 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 154 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 154 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 154 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 154 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 154 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 154 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 154 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 154 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 154 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 154 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 154 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 155 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 155 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 155 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 155 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 155 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 155 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 155 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 155 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 155 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 155 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 155 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 156 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 156 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 156 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 156 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 156 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 156 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 156 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 156 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 156 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 156 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 156 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 157 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 157 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 157 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 157 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 157 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 157 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 157 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 157 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 157 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 157 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 157 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 158 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 158 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 158 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 158 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "N*m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 158 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 158 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 158 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 158 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 158 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 158 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 158 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 159 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 159 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 159 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 159 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 159 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 159 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 159 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 159 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 159 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 159 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 159 , SS_NOT_REUSABLE_AND_GLOBAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 160 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 160 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 160 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 160 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 160 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 160 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 160 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 160 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 160 , SS_OK_TO_MERGE_CONDITIONAL ) ;
ssSetOutputPortICAttributes ( S , 160 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 160 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 161 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 161 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 161 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 161 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 161 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 161 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 161 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 161 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 161 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 161 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 161 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 162 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 162 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 162 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 162 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "m/s" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 162 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 162 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 162 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 162 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 162 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 162 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 162 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 163 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 163 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 163 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 163 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "1" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 163 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 163 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 163 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 163 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 163 , SS_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 163 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 163 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 164 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 164 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 164 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 164 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 164 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 164 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 164 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 164 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 164 , SS_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 164 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 164 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 165 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 165 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 165 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 165 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 165 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 165 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 165 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 165 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 165 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 165 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 165 , SS_NOT_REUSABLE_AND_LOCAL ) ; if ( !
ssSetOutputPortVectorDimension ( S , 166 , 4 ) ) return ;
ssSetOutputPortDimensionsMode ( S , 166 , FIXED_DIMS_MODE ) ;
ssSetOutputPortFrameData ( S , 166 , FRAME_NO ) ; if ( ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { ssSetOutputPortDataType ( S , 166 , SS_DOUBLE
) ; } if ( ssGetSimMode ( S ) != SS_SIMMODE_SIZES_CALL_ONLY ) {
#if defined (MATLAB_MEX_FILE)
UnitId unitIdReg ; ssRegisterUnitFromExpr ( S , "rad" , & unitIdReg ) ; if ( unitIdReg == INVALID_UNIT_ID ) return ; ssSetOutputPortUnit ( S , 166 , unitIdReg ) ;
#endif
} ssSetOutputPortSampleTime ( S , 166 , 0.0 ) ; ssSetOutputPortOffsetTime ( S
, 166 , 0.0 ) ; ssSetOutputPortDiscreteValuedOutput ( S , 166 , 0 ) ;
ssSetOutputPortOkToMerge ( S , 166 , SS_NOT_OK_TO_MERGE ) ;
ssSetOutputPortICAttributes ( S , 166 , false , false , false ) ;
ssSetOutputPortOptimOpts ( S , 166 , SS_NOT_REUSABLE_AND_LOCAL ) ;
ssSetSimStateCompliance ( S , USE_CUSTOM_SIM_STATE ) ;
mr_PassVeh14DOF_RegisterSimStateChecksum ( S ) ; ssSetNumSampleTimes ( S , 4
) ; ssSetParameterTuningCompliance ( S , true ) ; ssSetNumRWork ( S , 0 ) ;
ssSetNumIWork ( S , 0 ) ; ssSetNumPWork ( S , 0 ) ; ssSetNumModes ( S , 0 ) ;
{ int_T zcsIdx = 0 ; zcsIdx = ssCreateAndAddZcSignalInfo ( S ) ;
ssSetZcSignalWidth ( S , zcsIdx , 1 ) ; ssSetZcSignalName ( S , zcsIdx ,
"Trig" ) ; ssSetZcSignalType ( S , zcsIdx , SL_ZCS_TYPE_DISC ) ;
ssSetZcSignalZcEventType ( S , zcsIdx , SL_ZCS_EVENT_ALL_UP ) ;
ssSetZcSignalNeedsEventNotification ( S , zcsIdx , 0 ) ; zcsIdx =
ssCreateAndAddZcSignalInfo ( S ) ; ssSetZcSignalWidth ( S , zcsIdx , 1 ) ;
ssSetZcSignalName ( S , zcsIdx , "Trig" ) ; ssSetZcSignalType ( S , zcsIdx ,
SL_ZCS_TYPE_DISC ) ; ssSetZcSignalZcEventType ( S , zcsIdx ,
SL_ZCS_EVENT_ALL_UP ) ; ssSetZcSignalNeedsEventNotification ( S , zcsIdx , 0
) ; zcsIdx = ssCreateAndAddZcSignalInfo ( S ) ; ssSetZcSignalWidth ( S ,
zcsIdx , 1 ) ; ssSetZcSignalName ( S , zcsIdx , "Trig" ) ; ssSetZcSignalType
( S , zcsIdx , SL_ZCS_TYPE_DISC ) ; ssSetZcSignalZcEventType ( S , zcsIdx ,
SL_ZCS_EVENT_ALL_UP ) ; ssSetZcSignalNeedsEventNotification ( S , zcsIdx , 0
) ; } ssSetOutputPortIsNonContinuous ( S , 0 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 0 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 1 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 1 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 2 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 2 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 3 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 3 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 4 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 4 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 5 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 5 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 6 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 6 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 7 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 7 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 8 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 8 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 9 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 9 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 10 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 10 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 11 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 11 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 12 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 12 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 13 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 13 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 14 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 14 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 15 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 15 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 16 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 16 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 17 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 17 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 18 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 18 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 19 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 19 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 20 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 20 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 21 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 21 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 22 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 22 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 23 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 23 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 24 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 24 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 25 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 25 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 26 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 26 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 27 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 27 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 28 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 28 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 29 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 29 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 30 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 30 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 31 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 31 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 32 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 32 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 33 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 33 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 34 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 34 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 35 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 35 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 36 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 36 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 37 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 37 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 38 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 38 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 39 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 39 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 40 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 40 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 41 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 41 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 42 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 42 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 43 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 43 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 44 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 44 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 45 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 45 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 46 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 46 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 47 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 47 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 48 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 48 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 49 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 49 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 50 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 50 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 51 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 51 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 52 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 52 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 53 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 53 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 54 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 54 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 55 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 55 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 56 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 56 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 57 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 57 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 58 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 58 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 59 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 59 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 60 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 60 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 61 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 61 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 62 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 62 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 63 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 63 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 64 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 64 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 65 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 65 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 66 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 66 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 67 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 67 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 68 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 68 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 69 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 69 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 70 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 70 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 71 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 71 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 72 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 72 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 73 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 73 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 74 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 74 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 75 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 75 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 76 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 76 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 77 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 77 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 78 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 78 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 79 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 79 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 80 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 80 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 81 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 81 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 82 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 82 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 83 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 83 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 84 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 84 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 85 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 85 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 86 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 86 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 87 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 87 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 88 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 88 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 89 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 89 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 90 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 90 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 91 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 91 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 92 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 92 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 93 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 93 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 94 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 94 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 95 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 95 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 96 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 96 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 97 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 97 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 98 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 98 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 99 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 99 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 100 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 100 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 101 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 101 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 102 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 102 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 103 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 103 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 104 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 104 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 105 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 105 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 106 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 106 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 107 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 107 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 108 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 108 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 109 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 109 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 110 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 110 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 111 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 111 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 112 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 112 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 113 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 113 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 114 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 114 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 115 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 115 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 116 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 116 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 117 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 117 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 118 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 118 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 119 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 119 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 120 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 120 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 121 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 121 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 122 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 122 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 123 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 123 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 124 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 124 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 125 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 125 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 126 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 126 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 127 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 127 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 128 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 128 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 129 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 129 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 130 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 130 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 131 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 131 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 132 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 132 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 133 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 133 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 134 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 134 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 135 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 135 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 136 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 136 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 137 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 137 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 138 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 138 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 139 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 139 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 140 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 140 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 141 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 141 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 142 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 142 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 143 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 143 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 144 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 144 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 145 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 145 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 146 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 146 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 147 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 147 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 148 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 148 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 149 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 149 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 150 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 150 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 151 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 151 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 152 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 152 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 153 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 153 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 154 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 154 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 155 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 155 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 156 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 156 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 157 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 157 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 158 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 158 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 159 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 159 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 160 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 160 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 161 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 161 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 162 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 162 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 163 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 163 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 164 , 1 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 164 , 1 ) ;
ssSetOutputPortIsNonContinuous ( S , 165 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 165 , 0 ) ;
ssSetOutputPortIsNonContinuous ( S , 166 , 0 ) ;
ssSetOutputPortIsFedByBlockWithModesNoZCs ( S , 166 , 0 ) ;
ssSetInputPortIsNotDerivPort ( S , 0 , 0 ) ; ssSetInputPortIsNotDerivPort ( S
, 1 , 0 ) ; ssSetInputPortIsNotDerivPort ( S , 2 , 0 ) ;
ssSetInputPortIsNotDerivPort ( S , 3 , 0 ) ; ssSetInputPortIsNotDerivPort ( S
, 4 , 0 ) ; ssSetInputPortIsNotDerivPort ( S , 5 , 0 ) ;
ssSetInputPortIsNotDerivPort ( S , 6 , 0 ) ;
ssSetModelReferenceSampleTimeInheritanceRule ( S ,
DISALLOW_SAMPLE_TIME_INHERITANCE ) ; ssSetAcceptsFcnCallInputs ( S ) ;
ssSetModelReferenceNormalModeSupport ( S ,
MDL_START_AND_MDL_PROCESS_PARAMS_OK ) ; ssSupportsMultipleExecInstances ( S ,
true ) ; ssHasStateInsideForEachSS ( S , true ) ; ssSetOptions ( S ,
SS_OPTION_ALLOW_CONSTANT_PORT_SAMPLE_TIME |
SS_OPTION_PORT_SAMPLE_TIMES_ASSIGNED | SS_OPTION_SUPPORTS_ALIAS_DATA_TYPES |
SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME | SS_OPTION_EXCEPTION_FREE_CODE |
SS_OPTION_WORKS_WITH_CODE_REUSE ) ;
#if SS_SFCN_FOR_SIM
if ( S -> mdlInfo -> genericFcn != ( NULL ) && ssGetSimMode ( S ) !=
SS_SIMMODE_SIZES_CALL_ONLY ) { int_T retVal = 1 ;
mr_PassVeh14DOF_MdlInfoRegFcn ( S , "PassVeh14DOF" , & retVal ) ; if ( !
retVal ) return ; }
#endif
#if SS_SFCN_FOR_SIM
if ( ssSetNumDWork ( S , 1 ) ) { int mdlrefDWTypeId ; ssRegMdlRefDWorkType ( S
, & mdlrefDWTypeId ) ; if ( mdlrefDWTypeId == INVALID_DTYPE_ID ) return ; if
( ! ssSetDataTypeSize ( S , mdlrefDWTypeId , sizeof ( egcgcay4tuh ) ) )
return ; ssSetDWorkDataType ( S , 0 , mdlrefDWTypeId ) ; ssSetDWorkWidth ( S
, 0 , 1 ) ; }
#else
if ( ! ssSetNumDWork ( S , 1 ) ) { return ; }
#endif
slmrRegisterSystemInitializeMethod ( S , mdlInitializeConditions ) ;
slmrRegisterSystemResetMethod ( S , mdlReset ) ;
slmrRegisterPeriodicOutputUpdateMethod ( S , mdlPeriodicOutputUpdate ) ;
ssSetSimulinkVersionGeneratedIn ( S , "24.2" ) ; ssSetNeedAbsoluteTime ( S ,
1 ) ; } static void mdlInitializeSampleTimes ( SimStruct * S ) {
ssSetSampleTime ( S , 0 , 0 ) ; ssSetOffsetTime ( S , 0 , 0 ) ;
ssSetSampleTime ( S , 1 , 0.001 ) ; ssSetOffsetTime ( S , 1 , 0 ) ;
ssSetSampleTime ( S , 2 , mxGetInf ( ) ) ; ssSetOffsetTime ( S , 2 , 0 ) ;
ssSetSampleTime ( S , 3 , rtInf ) ; ssSetOffsetTime ( S , 3 , rtInf ) ;
return ; }
#define MDL_SET_WORK_WIDTHS
static void mdlSetWorkWidths ( SimStruct * S ) { if ( S -> mdlInfo ->
genericFcn != ( NULL ) ) { _GenericFcn fcn = S -> mdlInfo -> genericFcn ;
ssSetSignalSizesComputeType ( S , SS_VARIABLE_SIZE_FROM_INPUT_VALUE_AND_SIZE
) ; } { static const char * toFileNames [ ] = { "" } ; static const char *
fromFileNames [ ] = { "" } ; if ( ! ssSetModelRefFromFiles ( S , 0 ,
fromFileNames ) ) return ; if ( ! ssSetModelRefToFiles ( S , 0 , toFileNames
) ) return ; } }
#define MDL_SETUP_RUNTIME_RESOURCES
static void mdlSetupRuntimeResources ( SimStruct * S ) { egcgcay4tuh * dw = ( egcgcay4tuh * ) ssGetDWork ( S , 0 ) ; h21fsrthfa * localX = ( h21fsrthfa * ) ssGetContStates ( S ) ; ssNonContDerivSigFeedingOutports mr_nonContOutput6 [ 3 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput7 [ 3 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput8 [ 3 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput9 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput10 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput11 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput12 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput13 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput14 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput15 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput16 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput17 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput18 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput19 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput20 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput21 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput22 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput23 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput24 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput25 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput26 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput27 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput28 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput29 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput30 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput31 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput32 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput33 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput34 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput35 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput36 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput37 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput38 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput39 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput40 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput41 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput42 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput43 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput44 [ 4 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput52 [ 59 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput53 [ 59 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput54 [ 59 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput55 [ 59 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput56 [ 59 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput57 [ 59 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput58 [ 72 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput59 [ 72 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput60 [ 72 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput62 [ 55 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput63 [ 55 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput64 [ 55 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput68 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput69 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput70 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput71 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput72 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput73 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput74 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput75 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput76 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput77 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput78 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput79 [ 54 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput98 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput99 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput100 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput101 [ 69 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput102 [ 69 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput103 [ 69 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput113 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput114 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput115 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput116 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput117 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput118 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput119 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput120 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput121 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput122 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput123 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput124 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput125 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput126 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput127 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput128 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput129 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput130 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput131 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput132 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput133 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput134 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput135 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput136 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput137 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput138 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput139 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput140 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput141 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput142 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput143 [ 71 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput145 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput146 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput147 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput148 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput149 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput150 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput151 [ 1 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput152 [ 8 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput156 [ 12 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput158 [ 12 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput159 [ 12 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput160 [ 7 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput163 [ 12 ] ; ssNonContDerivSigFeedingOutports mr_nonContOutput164 [ 12 ] ; ssNonContDerivSigFeedingOutports * mr_nonContOutputArray [ 167 ] ; void * sysRanPtr = ( NULL ) ; int sysTid = 0 ; ssGetContextSysRanBCPtr ( S , & sysRanPtr ) ; ssGetContextSysTid ( S , & sysTid ) ; if ( sysTid == CONSTANT_TID ) { sysTid = 0 ; } mr_nonContOutputArray [ 0 ] = ( NULL ) ; mr_nonContOutputArray [ 1 ] = ( NULL ) ; mr_nonContOutputArray [ 2 ] = ( NULL ) ; mr_nonContOutputArray [ 3 ] = ( NULL ) ; mr_nonContOutputArray [ 4 ] = ( NULL ) ; mr_nonContOutputArray [ 5 ] = ( NULL ) ; mr_nonContOutputArray [ 6 ] = mr_nonContOutput6 ; mr_nonContOutputArray [ 7 ] = mr_nonContOutput7 ; mr_nonContOutputArray [ 8 ] = mr_nonContOutput8 ; mr_nonContOutputArray [ 9 ] = mr_nonContOutput9 ; mr_nonContOutputArray [ 10 ] = mr_nonContOutput10 ; mr_nonContOutputArray [ 11 ] = mr_nonContOutput11 ; mr_nonContOutputArray [ 12 ] = mr_nonContOutput12 ; mr_nonContOutputArray [ 13 ] = mr_nonContOutput13 ; mr_nonContOutputArray [ 14 ] = mr_nonContOutput14 ; mr_nonContOutputArray [ 15 ] = mr_nonContOutput15 ; mr_nonContOutputArray [ 16 ] = mr_nonContOutput16 ; mr_nonContOutputArray [ 17 ] = mr_nonContOutput17 ; mr_nonContOutputArray [ 18 ] = mr_nonContOutput18 ; mr_nonContOutputArray [ 19 ] = mr_nonContOutput19 ; mr_nonContOutputArray [ 20 ] = mr_nonContOutput20 ; mr_nonContOutputArray [ 21 ] = mr_nonContOutput21 ; mr_nonContOutputArray [ 22 ] = mr_nonContOutput22 ; mr_nonContOutputArray [ 23 ] = mr_nonContOutput23 ; mr_nonContOutputArray [ 24 ] = mr_nonContOutput24 ; mr_nonContOutputArray [ 25 ] = mr_nonContOutput25 ; mr_nonContOutputArray [ 26 ] = mr_nonContOutput26 ; mr_nonContOutputArray [ 27 ] = mr_nonContOutput27 ; mr_nonContOutputArray [ 28 ] = mr_nonContOutput28 ; mr_nonContOutputArray [ 29 ] = mr_nonContOutput29 ; mr_nonContOutputArray [ 30 ] = mr_nonContOutput30 ; mr_nonContOutputArray [ 31 ] = mr_nonContOutput31 ; mr_nonContOutputArray [ 32 ] = mr_nonContOutput32 ; mr_nonContOutputArray [ 33 ] = mr_nonContOutput33 ; mr_nonContOutputArray [ 34 ] = mr_nonContOutput34 ; mr_nonContOutputArray [ 35 ] = mr_nonContOutput35 ; mr_nonContOutputArray [ 36 ] = mr_nonContOutput36 ; mr_nonContOutputArray [ 37 ] = mr_nonContOutput37 ; mr_nonContOutputArray [ 38 ] = mr_nonContOutput38 ; mr_nonContOutputArray [ 39 ] = mr_nonContOutput39 ; mr_nonContOutputArray [ 40 ] = mr_nonContOutput40 ; mr_nonContOutputArray [ 41 ] = mr_nonContOutput41 ; mr_nonContOutputArray [ 42 ] = mr_nonContOutput42 ; mr_nonContOutputArray [ 43 ] = mr_nonContOutput43 ; mr_nonContOutputArray [ 44 ] = mr_nonContOutput44 ; mr_nonContOutputArray [ 45 ] = ( NULL ) ; mr_nonContOutputArray [ 46 ] = ( NULL ) ; mr_nonContOutputArray [ 47 ] = ( NULL ) ; mr_nonContOutputArray [ 48 ] = ( NULL ) ; mr_nonContOutputArray [ 49 ] = ( NULL ) ; mr_nonContOutputArray [ 50 ] = ( NULL ) ; mr_nonContOutputArray [ 51 ] = ( NULL ) ; mr_nonContOutputArray [ 52 ] = mr_nonContOutput52 ; mr_nonContOutputArray [ 53 ] = mr_nonContOutput53 ; mr_nonContOutputArray [ 54 ] = mr_nonContOutput54 ; mr_nonContOutputArray [ 55 ] = mr_nonContOutput55 ; mr_nonContOutputArray [ 56 ] = mr_nonContOutput56 ; mr_nonContOutputArray [ 57 ] = mr_nonContOutput57 ; mr_nonContOutputArray [ 58 ] = mr_nonContOutput58 ; mr_nonContOutputArray [ 59 ] = mr_nonContOutput59 ; mr_nonContOutputArray [ 60 ] = mr_nonContOutput60 ; mr_nonContOutputArray [ 61 ] = ( NULL ) ; mr_nonContOutputArray [ 62 ] = mr_nonContOutput62 ; mr_nonContOutputArray [ 63 ] = mr_nonContOutput63 ; mr_nonContOutputArray [ 64 ] = mr_nonContOutput64 ; mr_nonContOutputArray [ 65 ] = ( NULL ) ; mr_nonContOutputArray [ 66 ] = ( NULL ) ; mr_nonContOutputArray [ 67 ] = ( NULL ) ; mr_nonContOutputArray [ 68 ] = mr_nonContOutput68 ; mr_nonContOutputArray [ 69 ] = mr_nonContOutput69 ; mr_nonContOutputArray [ 70 ] = mr_nonContOutput70 ; mr_nonContOutputArray [ 71 ] = mr_nonContOutput71 ; mr_nonContOutputArray [ 72 ] = mr_nonContOutput72 ; mr_nonContOutputArray [ 73 ] = mr_nonContOutput73 ; mr_nonContOutputArray [ 74 ] = mr_nonContOutput74 ; mr_nonContOutputArray [ 75 ] = mr_nonContOutput75 ; mr_nonContOutputArray [ 76 ] = mr_nonContOutput76 ; mr_nonContOutputArray [ 77 ] = mr_nonContOutput77 ; mr_nonContOutputArray [ 78 ] = mr_nonContOutput78 ; mr_nonContOutputArray [ 79 ] = mr_nonContOutput79 ; mr_nonContOutputArray [ 80 ] = ( NULL ) ; mr_nonContOutputArray [ 81 ] = ( NULL ) ; mr_nonContOutputArray [ 82 ] = ( NULL ) ; mr_nonContOutputArray [ 83 ] = ( NULL ) ; mr_nonContOutputArray [ 84 ] = ( NULL ) ; mr_nonContOutputArray [ 85 ] = ( NULL ) ; mr_nonContOutputArray [ 86 ] = ( NULL ) ; mr_nonContOutputArray [ 87 ] = ( NULL ) ; mr_nonContOutputArray [ 88 ] = ( NULL ) ; mr_nonContOutputArray [ 89 ] = ( NULL ) ; mr_nonContOutputArray [ 90 ] = ( NULL ) ; mr_nonContOutputArray [ 91 ] = ( NULL ) ; mr_nonContOutputArray [ 92 ] = ( NULL ) ; mr_nonContOutputArray [ 93 ] = ( NULL ) ; mr_nonContOutputArray [ 94 ] = ( NULL ) ; mr_nonContOutputArray [ 95 ] = ( NULL ) ; mr_nonContOutputArray [ 96 ] = ( NULL ) ; mr_nonContOutputArray [ 97 ] = ( NULL ) ; mr_nonContOutputArray [ 98 ] = mr_nonContOutput98 ; mr_nonContOutputArray [ 99 ] = mr_nonContOutput99 ; mr_nonContOutputArray [ 100 ] = mr_nonContOutput100 ; mr_nonContOutputArray [ 101 ] = mr_nonContOutput101 ; mr_nonContOutputArray [ 102 ] = mr_nonContOutput102 ; mr_nonContOutputArray [ 103 ] = mr_nonContOutput103 ; mr_nonContOutputArray [ 104 ] = ( NULL ) ; mr_nonContOutputArray [ 105 ] = ( NULL ) ; mr_nonContOutputArray [ 106 ] = ( NULL ) ; mr_nonContOutputArray [ 107 ] = ( NULL ) ; mr_nonContOutputArray [ 108 ] = ( NULL ) ; mr_nonContOutputArray [ 109 ] = ( NULL ) ; mr_nonContOutputArray [ 110 ] = ( NULL ) ; mr_nonContOutputArray [ 111 ] = ( NULL ) ; mr_nonContOutputArray [ 112 ] = ( NULL ) ; mr_nonContOutputArray [ 113 ] = mr_nonContOutput113 ; mr_nonContOutputArray [ 114 ] = mr_nonContOutput114 ; mr_nonContOutputArray [ 115 ] = mr_nonContOutput115 ; mr_nonContOutputArray [ 116 ] = mr_nonContOutput116 ; mr_nonContOutputArray [ 117 ] = mr_nonContOutput117 ; mr_nonContOutputArray [ 118 ] = mr_nonContOutput118 ; mr_nonContOutputArray [ 119 ] = mr_nonContOutput119 ; mr_nonContOutputArray [ 120 ] = mr_nonContOutput120 ; mr_nonContOutputArray [ 121 ] = mr_nonContOutput121 ; mr_nonContOutputArray [ 122 ] = mr_nonContOutput122 ; mr_nonContOutputArray [ 123 ] = mr_nonContOutput123 ; mr_nonContOutputArray [ 124 ] = mr_nonContOutput124 ; mr_nonContOutputArray [ 125 ] = mr_nonContOutput125 ; mr_nonContOutputArray [ 126 ] = mr_nonContOutput126 ; mr_nonContOutputArray [ 127 ] = mr_nonContOutput127 ; mr_nonContOutputArray [ 128 ] = mr_nonContOutput128 ; mr_nonContOutputArray [ 129 ] = mr_nonContOutput129 ; mr_nonContOutputArray [ 130 ] = mr_nonContOutput130 ; mr_nonContOutputArray [ 131 ] = mr_nonContOutput131 ; mr_nonContOutputArray [ 132 ] = mr_nonContOutput132 ; mr_nonContOutputArray [ 133 ] = mr_nonContOutput133 ; mr_nonContOutputArray [ 134 ] = mr_nonContOutput134 ; mr_nonContOutputArray [ 135 ] = mr_nonContOutput135 ; mr_nonContOutputArray [ 136 ] = mr_nonContOutput136 ; mr_nonContOutputArray [ 137 ] = mr_nonContOutput137 ; mr_nonContOutputArray [ 138 ] = mr_nonContOutput138 ; mr_nonContOutputArray [ 139 ] = mr_nonContOutput139 ; mr_nonContOutputArray [ 140 ] = mr_nonContOutput140 ; mr_nonContOutputArray [ 141 ] = mr_nonContOutput141 ; mr_nonContOutputArray [ 142 ] = mr_nonContOutput142 ; mr_nonContOutputArray [ 143 ] = mr_nonContOutput143 ; mr_nonContOutputArray [ 144 ] = ( NULL ) ; mr_nonContOutputArray [ 145 ] = mr_nonContOutput145 ; mr_nonContOutputArray [ 146 ] = mr_nonContOutput146 ; mr_nonContOutputArray [ 147 ] = mr_nonContOutput147 ; mr_nonContOutputArray [ 148 ] = mr_nonContOutput148 ; mr_nonContOutputArray [ 149 ] = mr_nonContOutput149 ; mr_nonContOutputArray [ 150 ] = mr_nonContOutput150 ; mr_nonContOutputArray [ 151 ] = mr_nonContOutput151 ; mr_nonContOutputArray [ 152 ] = mr_nonContOutput152 ; mr_nonContOutputArray [ 153 ] = ( NULL ) ; mr_nonContOutputArray [ 154 ] = ( NULL ) ; mr_nonContOutputArray [ 155 ] = ( NULL ) ; mr_nonContOutputArray [ 156 ] = mr_nonContOutput156 ; mr_nonContOutputArray [ 157 ] = ( NULL ) ; mr_nonContOutputArray [ 158 ] = mr_nonContOutput158 ; mr_nonContOutputArray [ 159 ] = mr_nonContOutput159 ; mr_nonContOutputArray [ 160 ] = mr_nonContOutput160 ; mr_nonContOutputArray [ 161 ] = ( NULL ) ; mr_nonContOutputArray [ 162 ] = ( NULL ) ; mr_nonContOutputArray [ 163 ] = mr_nonContOutput163 ; mr_nonContOutputArray [ 164 ] = mr_nonContOutput164 ; mr_nonContOutputArray [ 165 ] = ( NULL ) ; mr_nonContOutputArray [ 166 ] = ( NULL ) ; klg3zs3vu4 ( S , mr_nonContOutputArray , slmrGetTopTidFromMdlRefChildTid ( S , 0 , false ) , slmrGetTopTidFromMdlRefChildTid ( S , 1 , false ) , slmrGetTopTidFromMdlRefChildTid ( S , 2 , true ) , & ( dw -> rtm ) , & ( dw -> rtb ) , & ( dw -> rtdw ) , localX , sysRanPtr , sysTid , ( ( NULL ) ) , ( ( NULL ) ) , 0 , - 1 ) ; ssSetModelMappingInfoPtr ( S , & ( dw -> rtm . DataMapInfo . mmi ) ) ; if ( S -> mdlInfo -> genericFcn != ( NULL ) ) { _GenericFcn fcn = S -> mdlInfo -> genericFcn ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 6 , mr_nonContOutput6 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 7 , mr_nonContOutput7 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 8 , mr_nonContOutput8 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 9 , mr_nonContOutput9 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 10 , mr_nonContOutput10 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 11 , mr_nonContOutput11 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 12 , mr_nonContOutput12 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 13 , mr_nonContOutput13 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 14 , mr_nonContOutput14 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 15 , mr_nonContOutput15 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 16 , mr_nonContOutput16 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 17 , mr_nonContOutput17 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 18 , mr_nonContOutput18 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 19 , mr_nonContOutput19 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 20 , mr_nonContOutput20 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 21 , mr_nonContOutput21 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 22 , mr_nonContOutput22 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 23 , mr_nonContOutput23 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 24 , mr_nonContOutput24 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 25 , mr_nonContOutput25 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 26 , mr_nonContOutput26 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 27 , mr_nonContOutput27 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 28 , mr_nonContOutput28 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 29 , mr_nonContOutput29 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 30 , mr_nonContOutput30 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 31 , mr_nonContOutput31 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 32 , mr_nonContOutput32 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 33 , mr_nonContOutput33 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 34 , mr_nonContOutput34 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 35 , mr_nonContOutput35 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 36 , mr_nonContOutput36 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 37 , mr_nonContOutput37 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 38 , mr_nonContOutput38 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 39 , mr_nonContOutput39 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 40 , mr_nonContOutput40 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 41 , mr_nonContOutput41 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 42 , mr_nonContOutput42 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 43 , mr_nonContOutput43 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 44 , mr_nonContOutput44 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 52 , mr_nonContOutput52 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 53 , mr_nonContOutput53 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 54 , mr_nonContOutput54 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 55 , mr_nonContOutput55 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 56 , mr_nonContOutput56 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 57 , mr_nonContOutput57 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 58 , mr_nonContOutput58 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 59 , mr_nonContOutput59 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 60 , mr_nonContOutput60 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 62 , mr_nonContOutput62 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 63 , mr_nonContOutput63 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 64 , mr_nonContOutput64 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 68 , mr_nonContOutput68 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 69 , mr_nonContOutput69 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 70 , mr_nonContOutput70 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 71 , mr_nonContOutput71 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 72 , mr_nonContOutput72 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 73 , mr_nonContOutput73 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 74 , mr_nonContOutput74 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 75 , mr_nonContOutput75 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 76 , mr_nonContOutput76 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 77 , mr_nonContOutput77 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 78 , mr_nonContOutput78 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 79 , mr_nonContOutput79 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 98 , mr_nonContOutput98 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 99 , mr_nonContOutput99 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 100 , mr_nonContOutput100 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 101 , mr_nonContOutput101 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 102 , mr_nonContOutput102 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 103 , mr_nonContOutput103 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 113 , mr_nonContOutput113 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 114 , mr_nonContOutput114 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 115 , mr_nonContOutput115 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 116 , mr_nonContOutput116 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 117 , mr_nonContOutput117 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 118 , mr_nonContOutput118 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 119 , mr_nonContOutput119 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 120 , mr_nonContOutput120 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 121 , mr_nonContOutput121 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 122 , mr_nonContOutput122 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 123 , mr_nonContOutput123 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 124 , mr_nonContOutput124 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 125 , mr_nonContOutput125 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 126 , mr_nonContOutput126 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 127 , mr_nonContOutput127 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 128 , mr_nonContOutput128 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 129 , mr_nonContOutput129 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 130 , mr_nonContOutput130 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 131 , mr_nonContOutput131 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 132 , mr_nonContOutput132 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 133 , mr_nonContOutput133 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 134 , mr_nonContOutput134 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 135 , mr_nonContOutput135 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 136 , mr_nonContOutput136 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 137 , mr_nonContOutput137 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 138 , mr_nonContOutput138 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 139 , mr_nonContOutput139 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 140 , mr_nonContOutput140 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 141 , mr_nonContOutput141 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 142 , mr_nonContOutput142 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 143 , mr_nonContOutput143 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 145 , mr_nonContOutput145 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 146 , mr_nonContOutput146 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 147 , mr_nonContOutput147 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 148 , mr_nonContOutput148 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 149 , mr_nonContOutput149 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 150 , mr_nonContOutput150 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 151 , mr_nonContOutput151 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 152 , mr_nonContOutput152 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 156 , mr_nonContOutput156 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 158 , mr_nonContOutput158 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 159 , mr_nonContOutput159 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 160 , mr_nonContOutput160 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 163 , mr_nonContOutput163 ) ) return ; if ( ! ( fcn ) ( S , GEN_FCN_REG_MODELREF_NONCONTSIGS , 164 , mr_nonContOutput164 ) ) return ; } }
#define MDL_START
static void mdlStart ( SimStruct * S ) { egcgcay4tuh * dw = ( egcgcay4tuh * )
ssGetDWork ( S , 0 ) ; g53loesncj ( & ( dw -> rtb ) , & ( dw -> rtdw ) , & ( dw
-> rtzce ) ) ; } static void mdlOutputs ( SimStruct * S , int_T tid ) {
egcgcay4tuh * dw = ( egcgcay4tuh * ) ssGetDWork ( S , 0 ) ; real_T const *
i_krp1l3jf4p = ( real_T * ) ssGetInputPortSignal ( S , 0 ) ; real_T const *
i_jb1g1kp3ja = ( real_T * ) ssGetInputPortSignal ( S , 1 ) ; real_T const *
i_mcau1kqq0y = ( real_T * ) ssGetInputPortSignal ( S , 2 ) ; real_T const *
i_eics31md2u = ( real_T * ) ssGetInputPortSignal ( S , 3 ) ; real_T const *
i_itd21vjha2 = ( real_T * ) ssGetInputPortSignal ( S , 4 ) ; real_T const *
i_dvme42dmxf = ( real_T * ) ssGetInputPortSignal ( S , 5 ) ; real_T const *
i_onroqiiasl = ( real_T * ) ssGetInputPortSignal ( S , 6 ) ; real_T *
o_B_36_1 = ( real_T * ) ssGetOutputPortSignal ( S , 0 ) ; real_T * o_B_36_2 =
( real_T * ) ssGetOutputPortSignal ( S , 1 ) ; real_T * o_B_36_3 = ( real_T *
) ssGetOutputPortSignal ( S , 2 ) ; real_T * o_B_36_4 = ( real_T * )
ssGetOutputPortSignal ( S , 3 ) ; real_T * o_B_36_5 = ( real_T * )
ssGetOutputPortSignal ( S , 4 ) ; real_T * o_B_36_6 = ( real_T * )
ssGetOutputPortSignal ( S , 5 ) ; real_T * o_B_36_7 = ( real_T * )
ssGetOutputPortSignal ( S , 6 ) ; real_T * o_B_36_8 = ( real_T * )
ssGetOutputPortSignal ( S , 7 ) ; real_T * o_B_36_9 = ( real_T * )
ssGetOutputPortSignal ( S , 8 ) ; real_T * o_B_36_10 = ( real_T * )
ssGetOutputPortSignal ( S , 9 ) ; real_T * o_B_36_11 = ( real_T * )
ssGetOutputPortSignal ( S , 10 ) ; real_T * o_B_36_12 = ( real_T * )
ssGetOutputPortSignal ( S , 11 ) ; real_T * o_B_36_13 = ( real_T * )
ssGetOutputPortSignal ( S , 12 ) ; real_T * o_B_36_14 = ( real_T * )
ssGetOutputPortSignal ( S , 13 ) ; real_T * o_B_36_15 = ( real_T * )
ssGetOutputPortSignal ( S , 14 ) ; real_T * o_B_36_16 = ( real_T * )
ssGetOutputPortSignal ( S , 15 ) ; real_T * o_B_36_17 = ( real_T * )
ssGetOutputPortSignal ( S , 16 ) ; real_T * o_B_36_18 = ( real_T * )
ssGetOutputPortSignal ( S , 17 ) ; real_T * o_B_36_19 = ( real_T * )
ssGetOutputPortSignal ( S , 18 ) ; real_T * o_B_36_20 = ( real_T * )
ssGetOutputPortSignal ( S , 19 ) ; real_T * o_B_36_21 = ( real_T * )
ssGetOutputPortSignal ( S , 20 ) ; real_T * o_B_36_22 = ( real_T * )
ssGetOutputPortSignal ( S , 21 ) ; real_T * o_B_36_23 = ( real_T * )
ssGetOutputPortSignal ( S , 22 ) ; real_T * o_B_36_24 = ( real_T * )
ssGetOutputPortSignal ( S , 23 ) ; real_T * o_B_36_25 = ( real_T * )
ssGetOutputPortSignal ( S , 24 ) ; real_T * o_B_36_26 = ( real_T * )
ssGetOutputPortSignal ( S , 25 ) ; real_T * o_B_36_27 = ( real_T * )
ssGetOutputPortSignal ( S , 26 ) ; real_T * o_B_36_28 = ( real_T * )
ssGetOutputPortSignal ( S , 27 ) ; real_T * o_B_36_29 = ( real_T * )
ssGetOutputPortSignal ( S , 28 ) ; real_T * o_B_36_30 = ( real_T * )
ssGetOutputPortSignal ( S , 29 ) ; real_T * o_B_36_31 = ( real_T * )
ssGetOutputPortSignal ( S , 30 ) ; real_T * o_B_36_32 = ( real_T * )
ssGetOutputPortSignal ( S , 31 ) ; real_T * o_B_36_33 = ( real_T * )
ssGetOutputPortSignal ( S , 32 ) ; real_T * o_B_36_34 = ( real_T * )
ssGetOutputPortSignal ( S , 33 ) ; real_T * o_B_36_35 = ( real_T * )
ssGetOutputPortSignal ( S , 34 ) ; real_T * o_B_36_36 = ( real_T * )
ssGetOutputPortSignal ( S , 35 ) ; real_T * o_B_36_37 = ( real_T * )
ssGetOutputPortSignal ( S , 36 ) ; real_T * o_B_36_38 = ( real_T * )
ssGetOutputPortSignal ( S , 37 ) ; real_T * o_B_36_39 = ( real_T * )
ssGetOutputPortSignal ( S , 38 ) ; real_T * o_B_36_40 = ( real_T * )
ssGetOutputPortSignal ( S , 39 ) ; real_T * o_B_36_41 = ( real_T * )
ssGetOutputPortSignal ( S , 40 ) ; real_T * o_B_36_42 = ( real_T * )
ssGetOutputPortSignal ( S , 41 ) ; real_T * o_B_36_43 = ( real_T * )
ssGetOutputPortSignal ( S , 42 ) ; real_T * o_B_36_44 = ( real_T * )
ssGetOutputPortSignal ( S , 43 ) ; real_T * o_B_36_45 = ( real_T * )
ssGetOutputPortSignal ( S , 44 ) ; real_T * o_B_36_46 = ( real_T * )
ssGetOutputPortSignal ( S , 45 ) ; real_T * o_B_36_47 = ( real_T * )
ssGetOutputPortSignal ( S , 46 ) ; real_T * o_B_36_48 = ( real_T * )
ssGetOutputPortSignal ( S , 47 ) ; real_T * o_B_36_49 = ( real_T * )
ssGetOutputPortSignal ( S , 48 ) ; real_T * o_B_36_50 = ( real_T * )
ssGetOutputPortSignal ( S , 49 ) ; real_T * o_B_36_51 = ( real_T * )
ssGetOutputPortSignal ( S , 50 ) ; real_T * o_B_36_52 = ( real_T * )
ssGetOutputPortSignal ( S , 51 ) ; real_T * o_B_36_53 = ( real_T * )
ssGetOutputPortSignal ( S , 52 ) ; real_T * o_B_36_54 = ( real_T * )
ssGetOutputPortSignal ( S , 53 ) ; real_T * o_B_36_55 = ( real_T * )
ssGetOutputPortSignal ( S , 54 ) ; real_T * o_B_36_56 = ( real_T * )
ssGetOutputPortSignal ( S , 55 ) ; real_T * o_B_36_57 = ( real_T * )
ssGetOutputPortSignal ( S , 56 ) ; real_T * o_B_36_58 = ( real_T * )
ssGetOutputPortSignal ( S , 57 ) ; real_T * o_B_36_59 = ( real_T * )
ssGetOutputPortSignal ( S , 58 ) ; real_T * o_B_36_60 = ( real_T * )
ssGetOutputPortSignal ( S , 59 ) ; real_T * o_B_36_61 = ( real_T * )
ssGetOutputPortSignal ( S , 60 ) ; real_T * o_B_36_62 = ( real_T * )
ssGetOutputPortSignal ( S , 61 ) ; real_T * o_B_36_63 = ( real_T * )
ssGetOutputPortSignal ( S , 62 ) ; real_T * o_B_36_64 = ( real_T * )
ssGetOutputPortSignal ( S , 63 ) ; real_T * o_B_36_65 = ( real_T * )
ssGetOutputPortSignal ( S , 64 ) ; real_T * o_B_36_66 = ( real_T * )
ssGetOutputPortSignal ( S , 65 ) ; real_T * o_B_36_67 = ( real_T * )
ssGetOutputPortSignal ( S , 66 ) ; real_T * o_B_36_68 = ( real_T * )
ssGetOutputPortSignal ( S , 67 ) ; real_T * o_B_36_69 = ( real_T * )
ssGetOutputPortSignal ( S , 68 ) ; real_T * o_B_36_70 = ( real_T * )
ssGetOutputPortSignal ( S , 69 ) ; real_T * o_B_36_71 = ( real_T * )
ssGetOutputPortSignal ( S , 70 ) ; real_T * o_B_36_72 = ( real_T * )
ssGetOutputPortSignal ( S , 71 ) ; real_T * o_B_36_73 = ( real_T * )
ssGetOutputPortSignal ( S , 72 ) ; real_T * o_B_36_74 = ( real_T * )
ssGetOutputPortSignal ( S , 73 ) ; real_T * o_B_36_75 = ( real_T * )
ssGetOutputPortSignal ( S , 74 ) ; real_T * o_B_36_76 = ( real_T * )
ssGetOutputPortSignal ( S , 75 ) ; real_T * o_B_36_77 = ( real_T * )
ssGetOutputPortSignal ( S , 76 ) ; real_T * o_B_36_78 = ( real_T * )
ssGetOutputPortSignal ( S , 77 ) ; real_T * o_B_36_79 = ( real_T * )
ssGetOutputPortSignal ( S , 78 ) ; real_T * o_B_36_80 = ( real_T * )
ssGetOutputPortSignal ( S , 79 ) ; real_T * o_B_36_81 = ( real_T * )
ssGetOutputPortSignal ( S , 95 ) ; real_T * o_B_36_82 = ( real_T * )
ssGetOutputPortSignal ( S , 96 ) ; real_T * o_B_36_83 = ( real_T * )
ssGetOutputPortSignal ( S , 97 ) ; real_T * o_B_36_84 = ( real_T * )
ssGetOutputPortSignal ( S , 98 ) ; real_T * o_B_36_85 = ( real_T * )
ssGetOutputPortSignal ( S , 99 ) ; real_T * o_B_36_86 = ( real_T * )
ssGetOutputPortSignal ( S , 100 ) ; real_T * o_B_36_87 = ( real_T * )
ssGetOutputPortSignal ( S , 101 ) ; real_T * o_B_36_88 = ( real_T * )
ssGetOutputPortSignal ( S , 102 ) ; real_T * o_B_36_89 = ( real_T * )
ssGetOutputPortSignal ( S , 103 ) ; real_T * o_B_36_90 = ( real_T * )
ssGetOutputPortSignal ( S , 104 ) ; real_T * o_B_36_91 = ( real_T * )
ssGetOutputPortSignal ( S , 105 ) ; real_T * o_B_36_92 = ( real_T * )
ssGetOutputPortSignal ( S , 106 ) ; real_T * o_B_36_93 = ( real_T * )
ssGetOutputPortSignal ( S , 113 ) ; real_T * o_B_36_94 = ( real_T * )
ssGetOutputPortSignal ( S , 114 ) ; real_T * o_B_36_95 = ( real_T * )
ssGetOutputPortSignal ( S , 115 ) ; real_T * o_B_36_96 = ( real_T * )
ssGetOutputPortSignal ( S , 116 ) ; real_T * o_B_36_97 = ( real_T * )
ssGetOutputPortSignal ( S , 117 ) ; real_T * o_B_36_98 = ( real_T * )
ssGetOutputPortSignal ( S , 118 ) ; real_T * o_B_36_99 = ( real_T * )
ssGetOutputPortSignal ( S , 119 ) ; real_T * o_B_36_100 = ( real_T * )
ssGetOutputPortSignal ( S , 120 ) ; real_T * o_B_36_101 = ( real_T * )
ssGetOutputPortSignal ( S , 121 ) ; real_T * o_B_36_102 = ( real_T * )
ssGetOutputPortSignal ( S , 122 ) ; real_T * o_B_36_103 = ( real_T * )
ssGetOutputPortSignal ( S , 123 ) ; real_T * o_B_36_104 = ( real_T * )
ssGetOutputPortSignal ( S , 124 ) ; real_T * o_B_36_105 = ( real_T * )
ssGetOutputPortSignal ( S , 125 ) ; real_T * o_B_36_106 = ( real_T * )
ssGetOutputPortSignal ( S , 126 ) ; real_T * o_B_36_107 = ( real_T * )
ssGetOutputPortSignal ( S , 127 ) ; real_T * o_B_36_108 = ( real_T * )
ssGetOutputPortSignal ( S , 128 ) ; real_T * o_B_36_109 = ( real_T * )
ssGetOutputPortSignal ( S , 129 ) ; real_T * o_B_36_110 = ( real_T * )
ssGetOutputPortSignal ( S , 130 ) ; real_T * o_B_36_111 = ( real_T * )
ssGetOutputPortSignal ( S , 131 ) ; real_T * o_B_36_112 = ( real_T * )
ssGetOutputPortSignal ( S , 132 ) ; real_T * o_B_36_113 = ( real_T * )
ssGetOutputPortSignal ( S , 133 ) ; real_T * o_B_36_114 = ( real_T * )
ssGetOutputPortSignal ( S , 134 ) ; real_T * o_B_36_115 = ( real_T * )
ssGetOutputPortSignal ( S , 135 ) ; real_T * o_B_36_116 = ( real_T * )
ssGetOutputPortSignal ( S , 136 ) ; real_T * o_B_36_117 = ( real_T * )
ssGetOutputPortSignal ( S , 137 ) ; real_T * o_B_36_118 = ( real_T * )
ssGetOutputPortSignal ( S , 138 ) ; real_T * o_B_36_119 = ( real_T * )
ssGetOutputPortSignal ( S , 139 ) ; real_T * o_B_36_120 = ( real_T * )
ssGetOutputPortSignal ( S , 140 ) ; real_T * o_B_36_121 = ( real_T * )
ssGetOutputPortSignal ( S , 141 ) ; real_T * o_B_36_122 = ( real_T * )
ssGetOutputPortSignal ( S , 142 ) ; real_T * o_B_36_123 = ( real_T * )
ssGetOutputPortSignal ( S , 143 ) ; real_T * o_B_36_124 = ( real_T * )
ssGetOutputPortSignal ( S , 144 ) ; real_T * o_B_36_125 = ( real_T * )
ssGetOutputPortSignal ( S , 145 ) ; real_T * o_B_36_126 = ( real_T * )
ssGetOutputPortSignal ( S , 146 ) ; real_T * o_B_36_127 = ( real_T * )
ssGetOutputPortSignal ( S , 147 ) ; real_T * o_B_36_128 = ( real_T * )
ssGetOutputPortSignal ( S , 148 ) ; real_T * o_B_36_129 = ( real_T * )
ssGetOutputPortSignal ( S , 149 ) ; real_T * o_B_36_130 = ( real_T * )
ssGetOutputPortSignal ( S , 150 ) ; real_T * o_B_36_131 = ( real_T * )
ssGetOutputPortSignal ( S , 151 ) ; real_T * o_B_36_132 = ( real_T * )
ssGetOutputPortSignal ( S , 152 ) ; real_T * o_B_36_133 = ( real_T * )
ssGetOutputPortSignal ( S , 153 ) ; real_T * o_B_36_134 = ( real_T * )
ssGetOutputPortSignal ( S , 154 ) ; real_T * o_B_36_135 = ( real_T * )
ssGetOutputPortSignal ( S , 155 ) ; real_T * o_B_36_136 = ( real_T * )
ssGetOutputPortSignal ( S , 156 ) ; real_T * o_B_36_137 = ( real_T * )
ssGetOutputPortSignal ( S , 157 ) ; real_T * o_B_36_138 = ( real_T * )
ssGetOutputPortSignal ( S , 158 ) ; real_T * o_B_36_139 = ( real_T * )
ssGetOutputPortSignal ( S , 159 ) ; real_T * o_B_36_140 = ( real_T * )
ssGetOutputPortSignal ( S , 160 ) ; real_T * o_B_36_141 = ( real_T * )
ssGetOutputPortSignal ( S , 161 ) ; real_T * o_B_36_142 = ( real_T * )
ssGetOutputPortSignal ( S , 162 ) ; real_T * o_B_36_143 = ( real_T * )
ssGetOutputPortSignal ( S , 163 ) ; real_T * o_B_36_144 = ( real_T * )
ssGetOutputPortSignal ( S , 164 ) ; real_T * o_B_36_145 = ( real_T * )
ssGetOutputPortSignal ( S , 165 ) ; real_T * o_B_36_146 = ( real_T * )
ssGetOutputPortSignal ( S , 166 ) ; h21fsrthfa * localX = ( h21fsrthfa * )
ssGetContStates ( S ) ; real_T * o_B_36_147 = ( real_T * )
ssGetOutputPortSignal ( S , 83 ) ; real_T * o_B_36_148 = ( real_T * )
ssGetOutputPortSignal ( S , 84 ) ; real_T * o_B_36_149 = ( real_T * )
ssGetOutputPortSignal ( S , 85 ) ; real_T * o_B_36_150 = ( real_T * )
ssGetOutputPortSignal ( S , 86 ) ; real_T * o_B_36_151 = ( real_T * )
ssGetOutputPortSignal ( S , 87 ) ; real_T * o_B_36_152 = ( real_T * )
ssGetOutputPortSignal ( S , 88 ) ; real_T * o_B_36_153 = ( real_T * )
ssGetOutputPortSignal ( S , 89 ) ; real_T * o_B_36_154 = ( real_T * )
ssGetOutputPortSignal ( S , 90 ) ; real_T * o_B_36_155 = ( real_T * )
ssGetOutputPortSignal ( S , 91 ) ; real_T * o_B_36_156 = ( real_T * )
ssGetOutputPortSignal ( S , 92 ) ; real_T * o_B_36_157 = ( real_T * )
ssGetOutputPortSignal ( S , 93 ) ; real_T * o_B_36_158 = ( real_T * )
ssGetOutputPortSignal ( S , 94 ) ; real_T * o_B_36_159 = ( real_T * )
ssGetOutputPortSignal ( S , 107 ) ; real_T * o_B_36_160 = ( real_T * )
ssGetOutputPortSignal ( S , 108 ) ; real_T * o_B_36_161 = ( real_T * )
ssGetOutputPortSignal ( S , 109 ) ; if ( tid == PARAMETER_TUNING_TID ) {
PassVeh14DOFTID2 ( o_B_36_147 , o_B_36_148 , o_B_36_149 , o_B_36_150 ,
o_B_36_151 , o_B_36_152 , o_B_36_153 , o_B_36_154 , o_B_36_155 , o_B_36_156 ,
o_B_36_157 , o_B_36_158 , o_B_36_159 , o_B_36_160 , o_B_36_161 , & ( dw ->
rtb ) ) ; } if ( tid != CONSTANT_TID && tid != PARAMETER_TUNING_TID ) { if ( ssIsSampleHit ( S , 0 , tid ) || ssIsMinorTimeStep ( S ) ) { PassVeh14DOF ( & ( dw -> rtm ) , i_krp1l3jf4p , i_jb1g1kp3ja , i_mcau1kqq0y , i_eics31md2u , i_itd21vjha2 , i_dvme42dmxf , i_onroqiiasl , o_B_36_1 , o_B_36_2 , o_B_36_3 , o_B_36_4 , o_B_36_5 , o_B_36_6 , o_B_36_7 , o_B_36_8 , o_B_36_9 , o_B_36_10 , o_B_36_11 , o_B_36_12 , o_B_36_13 , o_B_36_14 , o_B_36_15 , o_B_36_16 , o_B_36_17 , o_B_36_18 , o_B_36_19 , o_B_36_20 , o_B_36_21 , o_B_36_22 , o_B_36_23 , o_B_36_24 , o_B_36_25 , o_B_36_26 , o_B_36_27 , o_B_36_28 , o_B_36_29 , o_B_36_30 , o_B_36_31 , o_B_36_32 , o_B_36_33 , o_B_36_34 , o_B_36_35 , o_B_36_36 , o_B_36_37 , o_B_36_38 , o_B_36_39 , o_B_36_40 , o_B_36_41 , o_B_36_42 , o_B_36_43 , o_B_36_44 , o_B_36_45 , o_B_36_46 , o_B_36_47 , o_B_36_48 , o_B_36_49 , o_B_36_50 , o_B_36_51 , o_B_36_52 , o_B_36_53 , o_B_36_54 , o_B_36_55 , o_B_36_56 , o_B_36_57 , o_B_36_58 , o_B_36_59 , o_B_36_60 , o_B_36_61 , o_B_36_62 , o_B_36_63 , o_B_36_64 , o_B_36_65 , o_B_36_66 , o_B_36_67 , o_B_36_68 , o_B_36_69 , o_B_36_70 , o_B_36_71 , o_B_36_72 , o_B_36_73 , o_B_36_74 , o_B_36_75 , o_B_36_76 , o_B_36_77 , o_B_36_78 , o_B_36_79 , o_B_36_80 , o_B_36_81 , o_B_36_82 , o_B_36_83 , o_B_36_84 , o_B_36_85 , o_B_36_86 , o_B_36_87 , o_B_36_88 , o_B_36_89 , o_B_36_90 , o_B_36_91 , o_B_36_92 , o_B_36_93 , o_B_36_94 , o_B_36_95 , o_B_36_96 , o_B_36_97 , o_B_36_98 , o_B_36_99 , o_B_36_100 , o_B_36_101 , o_B_36_102 , o_B_36_103 , o_B_36_104 , o_B_36_105 , o_B_36_106 , o_B_36_107 , o_B_36_108 , o_B_36_109 , o_B_36_110 , o_B_36_111 , o_B_36_112 , o_B_36_113 , o_B_36_114 , o_B_36_115 , o_B_36_116 , o_B_36_117 , o_B_36_118 , o_B_36_119 , o_B_36_120 , o_B_36_121 , o_B_36_122 , o_B_36_123 , o_B_36_124 , o_B_36_125 , o_B_36_126 , o_B_36_127 , o_B_36_128 , o_B_36_129 , o_B_36_130 , o_B_36_131 , o_B_36_132 , o_B_36_133 , o_B_36_134 , o_B_36_135 , o_B_36_136 , o_B_36_137 , o_B_36_138 , o_B_36_139 , o_B_36_140 , o_B_36_141 , o_B_36_142 , o_B_36_143 , o_B_36_144 , o_B_36_145 , o_B_36_146 , & ( dw -> rtb ) , & ( dw -> rtdw ) , localX , & ( dw -> rtzce ) ) ; } } }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) { egcgcay4tuh * dw = ( egcgcay4tuh * ) ssGetDWork ( S , 0 ) ; jpl1qt0btr ( & ( dw -> rtm ) , & ( dw -> rtb ) , & ( dw -> rtdw ) ) ; return ; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { egcgcay4tuh * dw = ( egcgcay4tuh
* ) ssGetDWork ( S , 0 ) ; h21fsrthfa * localX = ( h21fsrthfa * )
ssGetContStates ( S ) ; ectmdkaxud * localXdot = ( ectmdkaxud * ) ssGetdX ( S
) ; kpl5p5meuz ( & ( dw -> rtb ) , & ( dw -> rtdw ) , localX , localXdot ) ;
} static void mdlTerminate ( SimStruct * S ) { egcgcay4tuh * dw = ( egcgcay4tuh
* ) ssGetDWork ( S , 0 ) ; apei0jh2jv ( & ( dw -> rtm ) ) ; return ; }
#define MDL_CLEANUP_RUNTIME_RESOURCES
static void mdlCleanupRuntimeResources ( SimStruct * S ) { }
#if !defined(MDL_SIM_STATE)
#define MDL_SIM_STATE
#endif
static mxArray * mdlGetSimState ( SimStruct * S ) { static const char *
simStateFieldNames [ 6 ] = { "localX" , "mdlrefDW" , "disallowedStateData" ,
"tNext" , "tNextTid" , "nonContDerivSigInfoPrevVal" , } ; mxArray * ss =
mxCreateStructMatrix ( 1 , 1 , 6 , simStateFieldNames ) ; { const h21fsrthfa
* localX = ( const h21fsrthfa * ) ssGetContStates ( S ) ; const size_t
numBytes = sizeof ( h21fsrthfa ) ; mxArray * storedX = mxCreateNumericMatrix
( 1 , numBytes , mxUINT8_CLASS , mxREAL ) ; UINT8_T * rawData = ( UINT8_T * )
mxGetData ( storedX ) ; memcpy ( & rawData [ 0 ] , localX , numBytes ) ;
mxSetFieldByNumber ( ss , 0 , 0 , storedX ) ; } { mxArray * mdlrefDW =
mr_PassVeh14DOF_GetDWork ( ssGetDWork ( S , 0 ) ) ; mxSetFieldByNumber ( ss ,
0 , 1 , mdlrefDW ) ; } { mxArray * data =
mr_PassVeh14DOF_GetSimStateDisallowedBlocks ( ) ; mxSetFieldByNumber ( ss , 0
, 2 , data ) ; } ; mxSetFieldByNumber ( ss , 0 , 3 , mxCreateDoubleScalar ( ( double ) ssGetTNext ( S ) ) ) ; mxSetFieldByNumber ( ss , 0 , 4 , mxCreateDoubleScalar ( ( double ) ssGetTNextTid ( S ) ) ) ; return ss ; }
#if !defined(MDL_SIM_STATE)
#define MDL_SIM_STATE
#endif
static void mdlSetSimState ( SimStruct * S , const mxArray * ss ) { {
h21fsrthfa * localX = ( h21fsrthfa * ) ssGetContStates ( S ) ; const size_t
numBytes = sizeof ( h21fsrthfa ) ; const mxArray * storedX =
mxGetFieldByNumber ( ss , 0 , 0 ) ; const UINT8_T * rawData = ( const UINT8_T
* ) mxGetData ( storedX ) ; memcpy ( localX , & rawData [ 0 ] , numBytes ) ;
} mr_PassVeh14DOF_SetDWork ( ssGetDWork ( S , 0 ) , mxGetFieldByNumber ( ss ,
0 , 1 ) ) ; ssSetTNext ( S , ( time_T ) mxGetScalar ( mxGetFieldByNumber ( ss
, 0 , 3 ) ) ) ; ssSetTNextTid ( S , ( int_T ) mxGetScalar ( mxGetFieldByNumber
( ss , 0 , 4 ) ) ) ; }
#ifdef MATLAB_MEX_FILE 
#include "simulink.c"
#include "fixedpoint.c"
#else
#error Assertion failed: file must be compiled as a MEX-file
#endif
