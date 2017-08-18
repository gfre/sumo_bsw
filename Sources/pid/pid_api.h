/***********************************************************************************************//**
 * @file		pid_api.h
 * @ingroup		pid
 * @brief 		API of the SWC @a PID
 *
 * This API provides a BSW-internal interface of the SWC @ref pid. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	27.04.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef PID_API_H_
#define PID_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"
#include "Platform.h"
#include "nvm_api.h"

#ifdef MASTER_pid_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup pid
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef  StdRtn_t ReadValFct_t(int32_t*);
typedef  StdRtn_t NVMReadFct_t(NVM_PidCfg_t*);
typedef  StdRtn_t NVMSaveValFct_t(const NVM_PidCfg_t *);

typedef enum PID_ItmType_e
{
	PID_LFT_MTR_SPD = 0,
	PID_RGHT_MTR_SPD,
	PID_LFT_MTR_POS,
	PID_RGHT_MTR_POS,
	PID_NUM_OF_ITMS,
}PID_ItmType_t;

typedef enum PID_SatType_e
{
	PID_NEG_SAT = -1,
	PID_NO_SAT,
	PID_POS_SAT,
}PID_SatType_t;

typedef struct PID_PrmCfg_s
{
	uint32_t Factor_KP_scld;
	uint32_t Factor_KI_scld;
	uint32_t Factor_KD_scld;
	uint16_t Scale;
	uint32_t SaturationVal;
}PID_PrmCfg_t;

typedef struct PID_Itm_s
{
	char_t          *pItmName;
	PID_ItmType_t   ItmType;
	PID_PrmCfg_t    *Config;
	PID_SatType_t	Saturation;
	int32_t 		lastError;
	int32_t 		integralVal;
	ReadValFct_t    *pCurValFct;
	ReadValFct_t    *pTrgtValFct;
	NVMReadFct_t 	*pNVMReadValFct;
	NVMReadFct_t	*pNVMReadDfltValFct;
	NVMSaveValFct_t *pNVMSaveValFct;
}PID_Itm_t;

typedef struct PID_Cfg_s
{
	PID_Itm_t    *pItmTbl;
	int8_t       NumOfItms;
}PID_Cfg_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/

EXTERNAL_ StdRtn_t PID(PID_Itm_t* plant_, int32_t* result_);

EXTERNAL_ PID_Cfg_t* Get_pPidCfg(void);

/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !PID_API_H_ */
