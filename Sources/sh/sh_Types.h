/***************************************************************************************************
 * @brief 	This is an interface of the shell software component.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, University Kiel
 * @date 	09.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file sh_Types.h
 * 
 *==================================================================================================
 */


#ifndef SH_TYPES_H_
#define SH_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_sh_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Sends a string to the shell/console stdout
 * @param msg_ Zero terminated string to write
 */
EXTERNAL_ void SH_SendStr(unsigned char *msg_);

/*!
 * @brief Sends a error string to the shell/console stdout
 * @param msg_ Zero terminated string to write
 */
EXTERNAL_ void SH_SendErrStr(unsigned char *msg_);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !SH_TYPES_H_ */
