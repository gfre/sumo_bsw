/***************************************************************************************************
 * @brief 	Iinterface of the Non-Volatile-Memory (NVM) storage.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This provides an interface of the NVM software component to other software components
 *
 * =================================================================================================
 */

#ifndef NVM_TYPES_H_
#define NVM_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Acon_Types.h"


#ifdef MASTER_nvm_Types_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
/* #define TMPL_MACRO (0xFFu) */



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct NVM_PidCfg_s
{
	uint16_t pGain100;			/**< proportional gain */
	uint16_t iGain100;			/**< integral gain */
	uint16_t dGain100;			/**< differential gain */
	uint16_t  maxSpdPerc;		/**< maximum speed command in percent */
	uint32_t iAntiWindup;		/**< maximum integral value for anti windup procedure */
} NVM_PidCfg_t;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ uint8_t NVM_Get_NvmVer(void);

EXTERNAL_ StdRtn_t NVM_Save_PIDpGainPos(const uint16_t pGain_);
EXTERNAL_ StdRtn_t NVM_Save_PIDiGainPos(const uint16_t iGain_);
EXTERNAL_ StdRtn_t NVM_Save_PIDdGainPos(const uint16_t dGain_);
EXTERNAL_ StdRtn_t NVM_Save_PIDiAntiWindUpPos(const uint32_t iAntiWindUp_);
EXTERNAL_ StdRtn_t NVM_Save_PIDMaxSpdPercPos(const uint16_t maxSpdPerc_);

EXTERNAL_ StdRtn_t NVM_Save_PIDPosCfg(const NVM_PidCfg_t *posCfg_);
EXTERNAL_ StdRtn_t NVM_Read_PIDPosCfg(NVM_PidCfg_t *posCfg_);
EXTERNAL_ StdRtn_t NVM_Read_Dflt_PIDPosCfg(NVM_PidCfg_t *posCfg_);


EXTERNAL_ StdRtn_t NVM_Save_PIDpGainSpdLe(const uint16_t pGain_);
EXTERNAL_ StdRtn_t NVM_Save_PIDiGainSpdLe(const uint16_t iGain_);
EXTERNAL_ StdRtn_t NVM_Save_PIDdGainSpdLe(const uint16_t dGain_);
EXTERNAL_ StdRtn_t NVM_Save_PIDiAntiWindUpSpdLe(const uint32_t iAntiWindUp_);
EXTERNAL_ StdRtn_t NVM_Save_PIDMaxSpdPercSpdLe(const uint16_t maxSpdPerc_);

EXTERNAL_ StdRtn_t NVM_Save_PIDSpdLeCfg(const NVM_PidCfg_t *posCfg_);
EXTERNAL_ StdRtn_t NVM_Read_PIDSpdLeCfg(NVM_PidCfg_t *spdCfg_);
EXTERNAL_ StdRtn_t NVM_Read_Dflt_PIDSpdLeCfg(NVM_PidCfg_t *spdCfg_);


EXTERNAL_ StdRtn_t NVM_Save_PIDpGainSpdRi(const uint16_t pGain_);
EXTERNAL_ StdRtn_t NVM_Save_PIDiGainSpdRi(const uint16_t iGain_);
EXTERNAL_ StdRtn_t NVM_Save_PIDdGainSpdRi(const uint16_t dGain_);
EXTERNAL_ StdRtn_t NVM_Save_PIDMaxSpdPercSpdRi(const uint16_t maxSpdPerc_);
EXTERNAL_ StdRtn_t NVM_Save_PIDiAntiWindUpSpdRi(const uint32_t iAntiWindUp_);

EXTERNAL_ StdRtn_t NVM_Save_PIDSpdRiCfg(const NVM_PidCfg_t *posCfg_);
EXTERNAL_ StdRtn_t NVM_Read_PIDSpdRiCfg(NVM_PidCfg_t *spdCfg_);
EXTERNAL_ StdRtn_t NVM_Read_Dflt_PIDSpdRiCfg(NVM_PidCfg_t *spdCfg_);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !NVM_TYPES_H_ */
