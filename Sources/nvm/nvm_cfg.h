/***********************************************************************************************//**
 * @file		nvm_cfg.h
 * @ingroup		nvm
 * @brief 		SWC-internal configuration interface of the SWC @a NVM
 *
 * This header file provides an internal interface within the software component SWC @ref nvm
 * for the configuration of the data storage within the internal non-volatile memory
 * of the MCU.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @note Interface for SWC-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef NVM_CFG_H_
#define NVM_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_NVM_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup nvm
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !NVM_CFG_H_ */
