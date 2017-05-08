/***********************************************************************************************//**
 * @file		sh_clshdlr.c
 * @ingroup		sh
 * @brief 		Implementation of the command line shell handler for the SWC @a sh
 *
 * This module implements the internal interface of the SWC @ref sh which allows to debug the over-
 * laying @a shell component by itself. It introduces application specific commands for requests
 * of status information via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.02.2017
 *  
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#define MASTER_sh_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "sh_clshdlr.h"
#include "Platform.h"
#include "Acon_Types.h"
#include "task_api.h"
#include "FRTOS1.h"



/*======================================= >> #DEFINES << =========================================*/
#define SH_CMD_EXIT   ("exit")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static StdRtn_t PrintHelp(const CLS1_StdIOType *io_);
static StdRtn_t PrintStatus(const CLS1_StdIOType *io_);
static void ExitDbgTask(void);



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static StdRtn_t PrintHelp(const CLS1_StdIOType *io_) {
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != io_)
	{
		CLS1_SendHelpStr((unsigned char*)"shell", (unsigned char*)"Group of shell commands\r\n", io_->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows shell help or status\r\n", io_->stdOut);
		retVal = ERR_OK;
	}
	return retVal;
}

static StdRtn_t PrintStatus(const CLS1_StdIOType *io_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != io_)
	{
		CLS1_SendStatusStr((unsigned char*)"shell", (unsigned char*)"\r\n", io_->stdOut);
		CLS1_SendStatusStr((unsigned char*)"  connections", NULL, io_->stdOut);
		CLS1_SendStr((unsigned char*)"DEFAULT", io_->stdOut);
		CLS1_SendStr((unsigned char*)"   +RTT", io_->stdOut);
		CLS1_SendStr((unsigned char*)"\r\n", io_->stdOut);
		retVal = ERR_OK;
	}

  return retVal;
}

static void ExitDbgTask(void)
{
	TASK_Hdl_t applTaskHdl = NULL;
	BaseType_t higherPriorityTaskWoken = pdFALSE;


	if ( ERR_OK != TASK_Read_ApplTaskHdl(&applTaskHdl) )
	{
		FRTOS1_xTaskNotifyFromISR( applTaskHdl,
				KEY_PRESSED_LONG_NOTIFICATION_VALUE,
				eSetBits,
				&higherPriorityTaskWoken );
		portYIELD_FROM_ISR( higherPriorityTaskWoken );
	}
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t SH_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
	uint8_t retVal = ERR_OK;

	*handled_ = FALSE;
	if( ( NULL != cmd_ ) && (NULL != io_ ) )
	{
		if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_HELP)==ERR_OK || UTIL1_strcmp((char*)cmd_, (char*)"shell help")==ERR_OK)
		{
			*handled_ = TRUE;
			retVal |= PrintHelp(io_);
		}
		else if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_STATUS)==ERR_OK || UTIL1_strcmp((char*)cmd_, (char*)"shell status")==ERR_OK)
		{
			*handled_ = TRUE;
			retVal |= PrintStatus(io_);
		}
		else if (ERR_OK == UTIL1_strcmp((char*)cmd_, (char*)SH_CMD_EXIT))
		{
			ExitDbgTask();
			*handled_ = TRUE;
		}
	}
	else
	{
		retVal = ERR_PARAM_ADDRESS;
	}
	return retVal;
}



#ifdef MASTER_sh_clshdlr_C_
#undef MASTER_sh_clshdlr_C_

#endif /* !MASTER_sh_clshdlr_C_ */
