/***********************************************************************************************//**
 * @file		tl_clshdlr.c
 * @ingroup		tl
 * @brief 		Implementation of the command line shell handler for the SWC @a TL
 *
 * This module implements the interface of the SWC @ref tl which is addressed to
 * the SWC @ref sh. It introduces application specific commands for requests of status information,
 * changing PID controller parameters for the Tracking loop, or restoring them from NVM via command line shell (@b CLS).
 * The changed parameters are immediately saved to the @ref nvm.
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	18.08.2017
 *  
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_tl_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tl_clshdlr.h"
#include "pid_api.h"
#include "nvm_api.h"
#include "tl_api.h"


/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef StdRtn_t ReadPIDCfg_t(NVM_PidCfg_t *);
typedef StdRtn_t SavePIDCfg_t(const NVM_PidCfg_t *);
//typedef PID_Config_t *GetPIDConfig_t(void);



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void TL_PrintHelp(const CLS1_StdIOType *io);
static void TL_PrintStatus(const CLS1_StdIOType *io);
static void PrintTLstatus(PID_Itm_t *itm_, const unsigned char *kindStr, const CLS1_StdIOType *io);
static uint8_t ParseTLParameter(PID_Itm_t *itm_, const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);
static StdRtn_t TL_restoreCfg(ReadPIDCfg_t *readDfltCfg_, SavePIDCfg_t *saveCfg_, PID_Itm_t *itm_);
static StdRtn_t TL_saveCfg(SavePIDCfg_t *saveCfg_, PID_Itm_t *itm_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void TL_PrintHelp(const CLS1_StdIOType *io)
{
	CLS1_SendHelpStr((unsigned char*)"tl", (unsigned char*)"Group of Tracking Loop commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows TL help or status\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed (L|R) (p|i|d|w) <value>", (unsigned char*)"Sets P, I, D or anti-Windup speed value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed (L|R) scale <value>", (unsigned char*)"Scaling value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed (L|R) restore", (unsigned char*)"Restores and saves default parameters for (L|R) speed control to NVM\r\n", io->stdOut);
}

static void PrintTLstatus(PID_Itm_t* itm_, const unsigned char *kindStr, const CLS1_StdIOType *io)
{
	unsigned char buf[48];
	unsigned char kindBuf[16];

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" TL");
	UTIL1_strcpy(buf, sizeof(buf), (unsigned char*)"p: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), itm_->Config->Factor_KP_scld);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" i: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), itm_->Config->Factor_KI_scld);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" d: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), itm_->Config->Factor_KD_scld);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" sat");
	UTIL1_Num32sToStr(buf, sizeof(buf), itm_->Config->SaturationVal);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" err");
	UTIL1_Num32sToStr(buf, sizeof(buf), itm_->lastError);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" int");
	UTIL1_Num32sToStr(buf, sizeof(buf), itm_->integralVal);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" scale");
	UTIL1_Num8uToStr(buf, sizeof(buf), itm_->Config->Scale);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);
}

static void TL_PrintStatus(const CLS1_StdIOType *io)
{
	uint8_t i = 0u;
	CLS1_SendStatusStr((unsigned char*)"TL", (unsigned char*)"\r\n", io->stdOut);
	for(i = 0; i < Get_pTLCfg()->NumOfItms; i++)
	{
		PrintTLstatus(&(Get_pTLCfg()->pItmTbl[i]), Get_pTLCfg()->pItmTbl[i].pItmName, io);
	}

}

static uint8_t ParseTLParameter(PID_Itm_t* itm_, const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
	const unsigned char *p;
	uint32_t val32u;
	uint8_t val8u;
	uint8_t res = ERR_OK;

	if (UTIL1_strncmp((char*)cmd, (char*)"p ", sizeof("p ")-1)==0) {
		p = cmd+sizeof("p");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			itm_->Config->Factor_KP_scld = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"i ", sizeof("i ")-1)==0) {
		p = cmd+sizeof("i");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			itm_->Config->Factor_KI_scld = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"d ", sizeof("d ")-1)==0) {
		p = cmd+sizeof("d");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			itm_->Config->Factor_KD_scld = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"w ", sizeof("w ")-1)==0) {
		p = cmd+sizeof("w");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			itm_->Config->SaturationVal = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"scale ", sizeof("scale ")-1)==0) {
		p = cmd+sizeof("scale");
		if (UTIL1_ScanDecimal8uNumber(&p, &val8u)==ERR_OK && val8u<=100) {
			itm_->Config->Scale = val8u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	}
	return res;
}



static StdRtn_t TL_restoreCfg(ReadPIDCfg_t *readDfltCfg_, SavePIDCfg_t *saveCfg_, PID_Itm_t* itm_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	NVM_PidCfg_t tmp = {0u};

	if ( (NULL != readDfltCfg_) && (NULL != saveCfg_) && (NULL != itm_ ) )
	{
		if ( ERR_OK == readDfltCfg_(&tmp) )
		{
			itm_->Config->Factor_KP_scld = (int32_t)tmp.KP_scld;
			itm_->Config->Factor_KI_scld = (int32_t)tmp.KI_scld;
			itm_->Config->Factor_KD_scld = (int32_t)tmp.KD_scld;
			itm_->Config->Scale 		 = (int16_t)tmp.Scale;
			itm_->Config->SaturationVal  = (int32_t)tmp.SaturationVal;
			if (ERR_OK == saveCfg_(&tmp))
			{
				retVal = ERR_OK;
			}
		}
	}
	return retVal;
}


static StdRtn_t TL_saveCfg(SavePIDCfg_t *saveCfg_, PID_Itm_t *itm_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	NVM_PidCfg_t tmp = {0u};

	if ((NULL != saveCfg_) && (NULL != itm_ ))
	{
		tmp.KP_scld = itm_->Config->Factor_KP_scld;
		tmp.KI_scld = itm_->Config->Factor_KI_scld;
		tmp.KD_scld = itm_->Config->Factor_KD_scld;
		tmp.Scale   = itm_->Config->Scale;
		tmp.SaturationVal = itm_->Config->SaturationVal;
		if (ERR_OK == saveCfg_(&tmp))
		{
			retVal = ERR_OK;
		}
	}
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t TL_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io)
{
	uint8_t res = ERR_OK;

	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"tl help")==0) {
		TL_PrintHelp(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"tl status")==0) {
		TL_PrintStatus(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)"tl speed L restore")==0) {
		if( ERR_OK == TL_restoreCfg(Get_pTLCfg()->pItmTbl[TL_LFT_SPD_EST].pNVMReadDfltValFct, Get_pTLCfg()->pItmTbl[TL_LFT_SPD_EST].pNVMSaveValFct, &Get_pTLCfg()->pItmTbl[TL_LFT_SPD_EST]) )
		{
			*handled = TRUE;
		}
	} else if (UTIL1_strcmp((char*)cmd, (char*)"tl speed R restore")==0) {
		if( ERR_OK == TL_restoreCfg(Get_pTLCfg()->pItmTbl[TL_RGHT_SPD_EST].pNVMReadDfltValFct, Get_pTLCfg()->pItmTbl[TL_RGHT_SPD_EST].pNVMSaveValFct, &Get_pTLCfg()->pItmTbl[TL_RGHT_SPD_EST]) )
		{
			*handled = TRUE;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"tl speed L ", sizeof("tl speed L ")-1)==0) {
		res = ParseTLParameter( &(Get_pTLCfg()->pItmTbl[TL_LFT_SPD_EST]), cmd+sizeof("tl speed L ")-1, handled, io);

		if( ERR_OK != TL_saveCfg(Get_pTLCfg()->pItmTbl[TL_LFT_SPD_EST].pNVMSaveValFct, &(Get_pTLCfg()->pItmTbl[TL_LFT_SPD_EST])) )
		{
			/* error handling */
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"tl speed R ", sizeof("tl speed R ")-1)==0) {
		res = ParseTLParameter( &(Get_pTLCfg()->pItmTbl[TL_RGHT_SPD_EST]), cmd+sizeof("tl speed R ")-1, handled, io);

		if( ERR_OK != TL_saveCfg(Get_pTLCfg()->pItmTbl[TL_RGHT_SPD_EST].pNVMSaveValFct, &(Get_pTLCfg()->pItmTbl[TL_RGHT_SPD_EST])) )
		{
			/* error handling */
		}
	}
	return res;
}



#ifdef MASTER_tl_clshdlr_C_
#undef MASTER_tl_clshdlr_C_
#endif /* !MASTER_tl_clshdlr_C_ */
