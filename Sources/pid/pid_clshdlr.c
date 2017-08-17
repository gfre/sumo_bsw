/***********************************************************************************************//**
 * @file		pid_clshdlr.c
 * @ingroup		pid
 * @brief 		Implementation of the command line shell handler for the SWC @a PID
 *
 * This module implements the interface of the SWC @ref pid which is addressed to
 * the SWC @ref sh. It introduces application specific commands for requests of status information,
 * changing PID controller parameters, or restoring them from NVM via command line shell (@b CLS).
 * The changed parameters are immediately saved to the @ref nvm.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	28.02.2017
 *  
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_pid_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "pid_clshdlr.h"
#include "pid_api.h"
#include "nvm_api.h"



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef StdRtn_t ReadPIDCfg_t(NVM_PidCfg_t *);
typedef StdRtn_t SavePIDCfg_t(const NVM_PidCfg_t *);
//typedef PID_Config_t *GetPIDConfig_t(void);



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void PID_PrintHelp(const CLS1_StdIOType *io);
static void PID_PrintStatus(const CLS1_StdIOType *io);
static void PrintPIDstatus(PID_Itm_t *itm_, const unsigned char *kindStr, const CLS1_StdIOType *io);
static uint8_t ParsePidParameter(PID_Itm_t *itm_, const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);
static StdRtn_t PID_restoreCfg(ReadPIDCfg_t *readDfltCfg_, SavePIDCfg_t *saveCfg_, PID_Itm_t *itm_);
static StdRtn_t PID_saveCfg(SavePIDCfg_t *saveCfg_, PID_Itm_t *itm_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void PID_PrintHelp(const CLS1_StdIOType *io)
{
	CLS1_SendHelpStr((unsigned char*)"pid", (unsigned char*)"Group of PID commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows PID help or status\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  pos (p|i|d|w) <value>", (unsigned char*)"Sets P, I, D or anti-Windup position value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  pos scale <value>", (unsigned char*)"Scaling value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed (L|R) (p|i|d|w) <value>", (unsigned char*)"Sets P, I, D or anti-Windup speed value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed (L|R) scale <value>", (unsigned char*)"Scaling value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  tl (L|R) (p|i|d|w) <value>", (unsigned char*)"Sets P, I, D or anti-Windup tracking loop value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  tl (L|R) scale <value>", (unsigned char*)"Scaling value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  pos restore", (unsigned char*)"Restores and saves default parameters for position control to NVM\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed (L|R) restore", (unsigned char*)"Restores and saves default parameters for (L|R) speed control to NVM\r\n", io->stdOut);
}

static void PrintPIDstatus(PID_Itm_t* itm_, const unsigned char *kindStr, const CLS1_StdIOType *io)
{
	unsigned char buf[48];
	unsigned char kindBuf[16];

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" PID");
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
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" windup");
	UTIL1_Num32sToStr(buf, sizeof(buf), itm_->Config->SaturationVal);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" error");
	UTIL1_Num32sToStr(buf, sizeof(buf), itm_->lastError);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" integral");
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

static void PID_PrintStatus(const CLS1_StdIOType *io)
{
	uint8_t i = 0u;
	CLS1_SendStatusStr((unsigned char*)"pid", (unsigned char*)"\r\n", io->stdOut);
	for(i = 0; i < Get_pPidCfg()->NumOfItms; i++)
	{
		PrintPIDstatus(&(Get_pPidCfg()->pItmTbl[i]), Get_pPidCfg()->pItmTbl[i].pItmName, io);
	}

}

static uint8_t ParsePidParameter(PID_Itm_t* itm_, const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
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



static StdRtn_t PID_restoreCfg(ReadPIDCfg_t *readDfltCfg_, SavePIDCfg_t *saveCfg_, PID_Itm_t* itm_)
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
			itm_->Config->Scale 		   = (int32_t)tmp.Scale;
			itm_->Config->SaturationVal  = (int32_t)tmp.SaturationVal;
			if (ERR_OK == saveCfg_(&tmp))
			{
				retVal = ERR_OK;
			}
		}
	}
	return retVal;
}


static StdRtn_t PID_saveCfg(SavePIDCfg_t *saveCfg_, PID_Itm_t *itm_)
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
uint8_t PID_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io)
{
	uint8_t res = ERR_OK;

	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"pid help")==0) {
		PID_PrintHelp(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"pid status")==0) {
		PID_PrintStatus(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)"pid pos restore")==0) {
		if( ( ERR_OK == PID_restoreCfg(NVM_Read_Dflt_PIDPosCfg, NVM_Save_PIDPosCfg, &(Get_pPidCfg()->pItmTbl[PID_LFT_MTR_POS]) ) )
		&&	( ERR_OK == PID_restoreCfg(NVM_Read_Dflt_PIDPosCfg, NVM_Save_PIDPosCfg, &(Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_POS]) ) ) )
		{
			*handled = TRUE;
		}
	} else if (UTIL1_strcmp((char*)cmd, (char*)"pid speed L restore")==0) {
		if( ERR_OK == PID_restoreCfg(NVM_Read_Dflt_PIDSpdLeCfg, NVM_Save_PIDSpdLeCfg, &(Get_pPidCfg()->pItmTbl[PID_LFT_MTR_SPD]) ) )
		{
			*handled = TRUE;
		}
	} else if (UTIL1_strcmp((char*)cmd, (char*)"pid speed R restore")==0) {
		if( ERR_OK == PID_restoreCfg(NVM_Read_Dflt_PIDSpdRiCfg, NVM_Save_PIDSpdRiCfg, &(Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_SPD]) ) )
		{
			*handled = TRUE;
		}

	} else if (UTIL1_strncmp((char*)cmd, (char*)"pid pos ", sizeof("pid pos ")-1)==0) {
		res = ParsePidParameter( &(Get_pPidCfg()->pItmTbl[PID_LFT_MTR_POS]), cmd+sizeof("pid pos ")-1, handled, io);
		if (res==ERR_OK)
		{
			res = ParsePidParameter( &(Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_POS]), cmd+sizeof("pid pos ")-1, handled, io);
		}

		if (res==ERR_OK)
		{
			if( ( ERR_OK != PID_saveCfg(NVM_Save_PIDPosCfg, &(Get_pPidCfg()->pItmTbl[PID_LFT_MTR_POS]) ) )
			||	( ERR_OK != PID_saveCfg(NVM_Save_PIDPosCfg, &(Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_POS]) ) ) )
			{
				/* error handling */
			}
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"pid speed L ", sizeof("pid speed L ")-1)==0) {
		res = ParsePidParameter( &(Get_pPidCfg()->pItmTbl[PID_LFT_MTR_SPD]), cmd+sizeof("pid speed L ")-1, handled, io);

		if( ERR_OK != PID_saveCfg(NVM_Save_PIDSpdLeCfg, &(Get_pPidCfg()->pItmTbl[PID_LFT_MTR_SPD]) ) )
		{
			/* error handling */
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"pid speed R ", sizeof("pid speed R ")-1)==0) {
		res = ParsePidParameter( &(Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_SPD]), cmd+sizeof("pid speed R ")-1, handled, io);

		if( ERR_OK != PID_saveCfg(NVM_Save_PIDSpdRiCfg, &(Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_SPD])) )
		{
			/* error handling */
		}
	}
	return res;
}



#ifdef MASTER_pid_clshdlr_C_
#undef MASTER_pid_clshdlr_C_
#endif /* !MASTER_pid_clshdlr_C_ */
