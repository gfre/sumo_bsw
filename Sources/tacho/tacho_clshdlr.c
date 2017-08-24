/***********************************************************************************************//**
 * @file		rnet_clshdlr.c
 * @ingroup		rnet
 * @brief 		Implementation of the command line shell handler for the SWC @a Tacho
 *
 * This module implements the interface of the SWC @ref tacho which is addressed to
 * the SWC @ref sh. It introduces application specific commands for requests
 * of status information via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	30.03.2017
 *  
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#define MASTER_tacho_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tacho_clshdlr.h"
#include "tacho_cfg.h"



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
/*!
 * \brief Prints the system low power status
 * \param io I/O channel to use for printing status
 */
static void TACHO_PrintStatus(const CLS1_StdIOType *io) {
	uint8_t i = 0u;
	bool enabled = FALSE;
	CLS1_SendStatusStr((unsigned char*)"Tacho", (unsigned char*)"\r\n", io->stdOut);
	CLS1_SendStr((unsigned char*)"Speeds:\r\n", io->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  L speed", (unsigned char*)"", io->stdOut);
	//CLS1_SendNum32s(Get_pTachoCfg()->pFilterTable[TACHO_Get_FltrType()].pGetSpeedFct(TRUE), io->stdOut);
	CLS1_SendStr((unsigned char*)" steps/sec\r\n", io->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  R speed", (unsigned char*)"", io->stdOut);
	//CLS1_SendNum32s(Get_pTachoCfg()->pFilterTable[TACHO_Get_FltrType()].pGetSpeedFct(FALSE), io->stdOut);
	CLS1_SendStr((unsigned char*)" steps/sec\r\n", io->stdOut);

	CLS1_SendStatusStr((unsigned char*)"Filters:", (unsigned char*)"\r\n", io->stdOut);
	for(i = 0; i < Get_pTachoCfg()->NumOfFilters; i++)
	{
		//if(Get_pTachoCfg()->pFilterTable[i].FilterType == Get_pTachoCfg()->pFilterTable[TACHO_Get_FltrType()].FilterType) enabled = TRUE;
		//else enabled = FALSE;
		CLS1_SendStr((unsigned char*)"  ", io->stdOut);
		CLS1_SendStatusStr((unsigned char*)"filter name", (unsigned char*)Get_pTachoCfg()->pFilterTable[i].pFilterName, io->stdOut);
		CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
		CLS1_SendStatusStr((unsigned char*)"   enabled", (enabled)?(unsigned char*)"Yes\r\n":(unsigned char*)"No\r\n", io->stdOut);
	}
}

/*!
 * \brief Prints the help text to the console
 * \param io I/O channel to be used
 */
static void TACHO_PrintHelp(const CLS1_StdIOType *io) {
	uint8_t i = 0u;
	CLS1_SendHelpStr((unsigned char*)"tacho", (unsigned char*)"Group of tacho commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows tacho help or status\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  filter", (unsigned char*)"Change filter type for tacho component\r\n", io->stdOut);
	for(i = 0; i < Get_pTachoCfg()->NumOfFilters; i++){
		CLS1_SendHelpStr((unsigned char*)Get_pTachoCfg()->pFilterTable[i].pFilterName, (unsigned char*)"Enables this filter to calculate speed\r\n", io->stdOut);
	}
}

static uint8_t TACHO_ParseParameter(TACHO_Cfg_t* config_, const unsigned char* cmd_, bool* handled_, const CLS1_StdIOType* io_)
{
	int8_t retVal = ERR_FAILED;
	uint8_t i = 0u;
	for(i = 0u; i < config_->NumOfFilters; i++)
	{
		if(UTIL1_strcmp((char*)cmd_, config_->pFilterTable[i].pFilterName) == 0)
		{
			//TACHO_Set_FltrType(i);
			*handled_ = TRUE;
			retVal = ERR_OK;
			break;
		}
	}
	if(ERR_OK != retVal)
	{
		CLS1_SendStr((unsigned char*)"Wrong argument\r\n ->Using Moving Average Filter\r\n", io_->stdErr);
		//TACHO_Set_FltrType(MOVING_AVERAGE_FILTER);
	}
	return retVal;
}


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t TACHO_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"tacho help")==0) {
		TACHO_PrintHelp(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"tacho status")==0) {
		TACHO_PrintStatus(io);
		*handled = TRUE;
	} else if(UTIL1_strncmp((char*)cmd, (char*)"tacho filter ", sizeof("tacho filter ")-1) == 0)
	{
		TACHO_ParseParameter(Get_pTachoCfg(), cmd+sizeof("tacho filter ")-1, handled, io);
	}
	return ERR_OK;
}



#ifdef MASTER_tacho_clshdlr_C_
#undef MASTER_tacho_clshdlr_C_
#endif /* !MASTER_tacho_clshdlr_C_ */
