/***********************************************************************************************//**
 * @file		sh_cfg.h
 * @ingroup		sh
 * @brief 		SWC-internal configuration interface of the SWC @a Shell
 *
 * This header file provides an internal interface within the software component SWC @ref sh
 * for the configuration of callback functions, which parse CLS commands of the corresponding BSW
 * components.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	03.02.2017
 *  
 * @note Interface for SWC-specific use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef SH_CFG_H_
#define SH_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_sh_cfg_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup sh
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @typedef SH_IODesc_t
 * @brief Data type definition of the structure SH_IODesc_s
 *
 * @struct SH_IODesc_s
 * @brief This structure defines a descriptor for the  standard I/O streams of the shell
 */
typedef struct SH_IODesc_s{
  unsigned char *buf;
  const uint8 bufSize;
  CLS1_ConstStdIOType *stdio;
} SH_IODesc_t;

/**
 * @typedef SH_IOCfg_t
 * @brief Data type definition of the structure SH_IOCfg_s
 *
 * @struct SH_IOCfg_s
 * @brief This structure defines the configuration of the shell I/Os.
 */
typedef struct SH_IOCfg_s
{
	const SH_IODesc_t *ios;
	uint8 ioSize;
} SH_IOCfg_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Interface function which returns the command parser table
 * @return reference to the command parser table.
 */
EXTERNAL_ const CLS1_ParseCommandCallback *Get_CmdParserTbl();

/**
 * @brief Interface function which returns the shell I/O configuration
 * @return reference to the shell I/O table.
 */
EXTERNAL_ const SH_IOCfg_t *Get_ShIOCfg();



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !SH_CFG_H_ */
