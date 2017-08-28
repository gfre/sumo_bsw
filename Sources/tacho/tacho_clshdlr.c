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
static void Print_TachoStatus(const CLS1_StdIOType *io_);
static void Print_TachoHelp(const CLS1_StdIOType *io_);
static uint8_t Parse_TachoParam(TACHO_FltrItmTbl_t* config_, const uchar_t* cmd_, bool* handled_, const CLS1_StdIOType* io_);


/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
/*!
 * \brief Prints the system low power status
 * \param io_ I/O channel to use for printing status
 */
static void Print_TachoStatus(const CLS1_StdIOType *io_)
{
	uint8_t i = 0u;
	int16_t spdTmp = 0;
	uchar_t buf[13] = {"        \0"};
	TACHO_FltrItmTbl_t *pFltrTbl = Get_pFltrTbl();

	CLS1_SendStatusStr((uchar_t*)TACHO_SWC_STRING, (uchar_t*)"\r\n", io_->stdOut);
	CLS1_SendStatusStr((uchar_t*)" speeds", (uchar_t *)"\r\n", io_->stdOut);

	CLS1_SendStatusStr((uchar_t*)"   L speed", (uchar_t*)"", io_->stdOut);
	(void)TACHO_Read_SpdLft((int32_t *)&spdTmp);
	CLS1_SendNum32s(spdTmp, io_->stdOut);
	CLS1_SendStr((uchar_t*)" steps/sec\r\n", io_->stdOut);

	CLS1_SendStatusStr((uchar_t*)"   R speed", (uchar_t*)"", io_->stdOut);
	(void)TACHO_Read_SpdRght((int32_t *)&spdTmp);
	CLS1_SendNum32s(spdTmp, io_->stdOut);
	CLS1_SendStr((uchar_t*)" steps/sec\r\n", io_->stdOut);

	CLS1_SendStatusStr((uchar_t*)" Filters", (uchar_t*)"\r\n", io_->stdOut);
	for(i = 0; i < Get_pFltrTbl()->numFltrs; i++)
	{
		if( ( NULL != pFltrTbl ) && ( NULL != pFltrTbl->aFltrs ) )
		{
			for( i = 0u; i < pFltrTbl->numFltrs; i++)
			{
				if( i == TACHO_Get_ActFltrIdx() )
				{
					UTIL1_strcat(buf, sizeof(buf), (const uchar_t *)"-->");
					CLS1_SendStatusStr((uchar_t*)buf, (uchar_t*)pFltrTbl->aFltrs[i].aFltrName, io_->stdOut);
				}
				else
				{
					CLS1_SendStatusStr((uchar_t*)"", (uchar_t*)pFltrTbl->aFltrs[i].aFltrName, io_->stdOut);
				}
				CLS1_SendStr((uchar_t*)"\r\n", io_->stdOut);
			}
		}




	}
}

/*!
 * \brief Prints the help text to the console
 * \param io_ I/O channel to be used
 */
static void Print_TachoHelp(const CLS1_StdIOType *io_)
{
	uint8_t i = 0u;
	CLS1_SendHelpStr((uchar_t*)"TACHO", (uchar_t*)"Group of tacho commands\r\n", io_->stdOut);
	CLS1_SendHelpStr((uchar_t*)"  help|status", (uchar_t*)"Shows tacho help or status\r\n", io_->stdOut);
	CLS1_SendHelpStr((uchar_t*)"  filter", (uchar_t*)"Change filter type for tacho component\r\n", io_->stdOut);
	for(i = 0; i < Get_pFltrTbl()->numFltrs; i++){
		CLS1_SendHelpStr((uchar_t*)Get_pFltrTbl()->aFltrs[i].aFltrName, (uchar_t*)"Enables this filter to calculate speed\r\n", io_->stdOut);
	}
}

static uint8_t Parse_TachoParam(TACHO_FltrItmTbl_t* config_, const uchar_t* cmd_, bool* handled_, const CLS1_StdIOType* io_)
{
	int8_t retVal = ERR_FAILED;
	uint8_t i = 0u;
	for(i = 0u; i < config_->numFltrs; i++)
	{
		if(UTIL1_strcmp((char*)cmd_, config_->aFltrs[i].aFltrName) == 0)
		{
			TACHO_Set_FltrReq(i);
			*handled_ = TRUE;
			retVal = ERR_OK;
			break;
		}
	}
	if(ERR_OK != retVal)
	{
		CLS1_SendStr((uchar_t*)"Wrong argument\r\n ->Using Kalman Filter\r\n", io_->stdErr);
		TACHO_Set_FltrReq(0);
	}
	return retVal;
}


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t TACHO_ParseCommand(const uchar_t *cmd, bool *handled, const CLS1_StdIOType *io_)
{
	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"tacho help")==0)
	{
		Print_TachoHelp(io_);
		*handled = TRUE;
	}
	else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"tacho status")==0)
	{
		Print_TachoStatus(io_);
		*handled = TRUE;
	}
	else if(UTIL1_strncmp((char*)cmd, (char*)"tacho filter ", sizeof("tacho filter ")-1) == 0)
	{
		Parse_TachoParam(Get_pFltrTbl(), cmd+sizeof("tacho filter ")-1, handled, io_);
	}
	else
	{

	}
	return ERR_OK;
}



#ifdef MASTER_tacho_clshdlr_C_
#undef MASTER_tacho_clshdlr_C_
#endif /* !MASTER_tacho_clshdlr_C_ */
