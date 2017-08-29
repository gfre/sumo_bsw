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
#include "pid_cfg.h"
#include "pid_api.h"
#include "nvm_api.h"



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef StdRtn_t ReadPIDCfg_t(NVM_PidCfg_t *);
typedef StdRtn_t SavePIDCfg_t(const NVM_PidCfg_t *);
//typedef PID_Config_t *GetPIDConfig_t(void);



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void PID_PrintHelp(const CLS1_StdIOType *io_);
static void PID_PrintStatus(const CLS1_StdIOType *io_);
static void PrintPIDstatus(const PID_Gain_t *gain_, const PID_Data_t *rtData_, const uchar_t *kindStr, const CLS1_StdIOType *io_);
static uint8_t ParsePidParameter(PID_Gain_t* gain_, const uchar_t *cmd_, bool *handled_, const CLS1_StdIOType *io_);
static StdRtn_t PID_restoreCfg(ReadPIDCfg_t *readDfltCfg_, SavePIDCfg_t *saveCfg_, PID_Gain_t* gain_);
static StdRtn_t PID_saveCfg(SavePIDCfg_t *saveCfg_, PID_Gain_t *gain_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void PID_PrintHelp(const CLS1_StdIOType *io_)
{
	CLS1_SendHelpStr((uchar_t*)"pid", (uchar_t*)"Group of PID commands\r\n", io_->stdOut);
	CLS1_SendHelpStr((uchar_t*)"  help|status", (uchar_t*)"Shows PID help or status\r\n", io_->stdOut);
	CLS1_SendHelpStr((uchar_t*)"  pos (p|i|d|w) <value>", (uchar_t*)"Sets P, I, D or anti-Windup position value\r\n", io_->stdOut);
	CLS1_SendHelpStr((uchar_t*)"  pos scale <value>", (uchar_t*)"Scaling value\r\n", io_->stdOut);
	CLS1_SendHelpStr((uchar_t*)"  speed (L|R) (p|i|d|w) <value>", (uchar_t*)"Sets P, I, D or anti-Windup speed value\r\n", io_->stdOut);
	CLS1_SendHelpStr((uchar_t*)"  speed (L|R) scale <value>", (uchar_t*)"Scaling value\r\n", io_->stdOut);
	CLS1_SendHelpStr((uchar_t*)"  pos restore", (uchar_t*)"Restores and saves default parameters for position control to NVM\r\n", io_->stdOut);
	CLS1_SendHelpStr((uchar_t*)"  speed (L|R) restore", (uchar_t*)"Restores and saves default parameters for (L|R) speed control to NVM\r\n", io_->stdOut);
}

static void PrintPIDstatus(const PID_Gain_t *gain_, const PID_Data_t *rtData_, const uchar_t *kindStr, const CLS1_StdIOType *io_)
{
	uchar_t buf[48];
	uchar_t kindBuf[16];

	UTIL1_strcpy(kindBuf, sizeof(buf), (uchar_t*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (uchar_t*)" PID");
	UTIL1_strcpy(buf, sizeof(buf), (uchar_t*)"p: ");
	UTIL1_strcatNum16u(buf, sizeof(buf), gain_->kP_scld);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)" i: ");
	UTIL1_strcatNum16u(buf, sizeof(buf), gain_->kI_scld);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)" d: ");
	UTIL1_strcatNum16u(buf, sizeof(buf), gain_->kD_scld);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io_->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (uchar_t*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (uchar_t*)" saturation");
	UTIL1_Num32uToStr(buf, sizeof(buf), gain_->intSatVal);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io_->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (uchar_t*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (uchar_t*)" scale");
	UTIL1_Num16uToStr(buf, sizeof(buf), gain_->nScale);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io_->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (uchar_t*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (uchar_t*)" error");
	UTIL1_Num32sToStr(buf, sizeof(buf), rtData_->prevErr);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io_->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (uchar_t*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (uchar_t*)" integral");
	UTIL1_Num32sToStr(buf, sizeof(buf), rtData_->intVal);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io_->stdOut);
}

static void PID_PrintStatus(const CLS1_StdIOType *io_)
{
	uint8_t i = 0u;
	PID_ItmTbl_t *pPidTbl = Get_pPidItmTbl();

	if( ( NULL != pPidTbl ) && ( NULL != pPidTbl->aPids ) )
	{
		CLS1_SendStatusStr((uchar_t*)"pid", (uchar_t*)"\r\n", io_->stdOut);
		for(i = 0u; i < pPidTbl->numPids; i++)
		{
			PrintPIDstatus(&pPidTbl->aPids[i].cfg.gain, &pPidTbl->aPids[i].data, pPidTbl->aPids[i].cfg.pItmName, io_);
		}
	}
	else
	{
		/* error handling */
	}


}

static uint8_t ParsePidParameter(PID_Gain_t* gain_, const uchar_t *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
	const uchar_t *p;
	uint16_t val16u;
	uint8_t val8u;
	uint8_t res = ERR_OK;

	if (UTIL1_strncmp((char*)cmd_, (char*)"p ", sizeof("p ")-1)==0) {
		p = cmd_+sizeof("p");
		if (UTIL1_ScanDecimal16uNumber(&p, &val16u)==ERR_OK) {
			gain_->kP_scld = val16u;
			*handled_ = TRUE;
		} else {
			CLS1_SendStr((uchar_t*)"Wrong argument\r\n", io_->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd_, (char*)"i ", sizeof("i ")-1)==0) {
		p = cmd_+sizeof("i");
		if (UTIL1_ScanDecimal16uNumber(&p, &val16u)==ERR_OK) {
			gain_->kI_scld = val16u;
			*handled_ = TRUE;
		} else {
			CLS1_SendStr((uchar_t*)"Wrong argument\r\n", io_->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd_, (char*)"d ", sizeof("d ")-1)==0) {
		p = cmd_+sizeof("d");
		if (UTIL1_ScanDecimal16uNumber(&p, &val16u)==ERR_OK) {
			gain_->kD_scld = val16u;
			*handled_ = TRUE;
		} else {
			CLS1_SendStr((uchar_t*)"Wrong argument\r\n", io_->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd_, (char*)"w ", sizeof("w ")-1)==0) {
		p = cmd_+sizeof("w");
		if (UTIL1_ScanDecimal16uNumber(&p, &val16u)==ERR_OK) {
			gain_->intSatVal = val16u;
			*handled_ = TRUE;
		} else {
			CLS1_SendStr((uchar_t*)"Wrong argument\r\n", io_->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd_, (char*)"scale ", sizeof("scale ")-1)==0) {
		p = cmd_+sizeof("scale");
		if (UTIL1_ScanDecimal8uNumber(&p, &val8u)==ERR_OK && val8u<=100) {
			gain_->nScale = val8u;
			*handled_ = TRUE;
		} else {
			CLS1_SendStr((uchar_t*)"Wrong argument\r\n", io_->stdErr);
			res = ERR_FAILED;
		}
	}
	return res;
}



static StdRtn_t PID_restoreCfg(ReadPIDCfg_t *readDfltCfg_, SavePIDCfg_t *saveCfg_, PID_Gain_t* gain_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	NVM_PidCfg_t tmp = {0u};

	if ( (NULL != readDfltCfg_) && (NULL != saveCfg_) && (NULL != gain_ ) )
	{
		if ( ERR_OK == readDfltCfg_(&tmp) )
		{
			gain_->kP_scld = tmp.KP_scld;
			gain_->kI_scld = tmp.KI_scld;
			gain_->kD_scld = tmp.KD_scld;
			gain_->nScale  = tmp.Scale;
			gain_->intSatVal  = tmp.SaturationVal;
			if (ERR_OK == saveCfg_(&tmp))
			{
				retVal = ERR_OK;
			}
		}
	}
	return retVal;
}


static StdRtn_t PID_saveCfg(SavePIDCfg_t *saveCfg_, PID_Gain_t *gain_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	NVM_PidCfg_t tmp = {0u};

	if ((NULL != saveCfg_) && (NULL != gain_ ))
	{
		tmp.KP_scld = gain_->kP_scld;
		tmp.KI_scld = gain_->kI_scld;
		tmp.KD_scld = gain_->kD_scld;
		tmp.Scale   = gain_->nScale;
		tmp.SaturationVal = gain_->intSatVal;
		if (ERR_OK == saveCfg_(&tmp))
		{
			retVal = ERR_OK;
		}
	}
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t PID_ParseCommand(const uchar_t *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
	uint8_t res = ERR_OK;
	PID_ItmTbl_t *pPidTbl = Get_pPidItmTbl();

	if( ( NULL != pPidTbl ) && ( NULL != pPidTbl->aPids ) )
	{

		if (ERR_OK == UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_HELP) || ERR_OK == UTIL1_strcmp((char*)cmd_, (char*)"pid help") )
		{
			PID_PrintHelp(io_);
			*handled_ = TRUE;
		}
		else if (ERR_OK == UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_STATUS) || ERR_OK == UTIL1_strcmp((char*)cmd_, (char*)"pid status") )
		{
			PID_PrintStatus(io_);
			*handled_ = TRUE;
		}
		else if (ERR_OK == UTIL1_strcmp((char*)cmd_, (char*)"pid pos restore") )
		{
			if( ( ERR_OK == PID_restoreCfg(pPidTbl->aPids[2].cfg.nvm.readDfltFct, pPidTbl->aPids[2].cfg.nvm.saveFct, &pPidTbl->aPids[2].cfg.gain) )
			&&	( ERR_OK == PID_restoreCfg(pPidTbl->aPids[3].cfg.nvm.readDfltFct, pPidTbl->aPids[3].cfg.nvm.saveFct, &pPidTbl->aPids[3].cfg.gain ) ) )
			{
				*handled_ = TRUE;
			}
		} else if (ERR_OK == UTIL1_strcmp((char*)cmd_, (char*)"pid speed L restore") )
		{
			if( ERR_OK == PID_restoreCfg(pPidTbl->aPids[0].cfg.nvm.readDfltFct, pPidTbl->aPids[0].cfg.nvm.saveFct, &pPidTbl->aPids[0].cfg.gain) )
			{
				*handled_ = TRUE;
			}
		}
		else if (ERR_OK == UTIL1_strcmp((char*)cmd_, (char*)"pid speed R restore") )
		{
			if( ERR_OK == PID_restoreCfg(pPidTbl->aPids[1].cfg.nvm.readDfltFct, pPidTbl->aPids[1].cfg.nvm.saveFct, &pPidTbl->aPids[1].cfg.gain) )
			{
				*handled_ = TRUE;
			}

		}
		else if ( ERR_OK == UTIL1_strncmp((char*)cmd_, (char*)"pid pos ", sizeof("pid pos ")-1) )
		{
			res = ParsePidParameter( &pPidTbl->aPids[2].cfg.gain, cmd_+sizeof("pid pos ")-1, handled_, io_);
			if (res==ERR_OK)
			{
				res = ParsePidParameter( &pPidTbl->aPids[3].cfg.gain, cmd_+sizeof("pid pos ")-1, handled_, io_);
			}

			if (res==ERR_OK)
			{
				if( ( ERR_OK != PID_saveCfg(pPidTbl->aPids[2].cfg.nvm.saveFct, &pPidTbl->aPids[2].cfg.gain) )
				||	( ERR_OK != PID_saveCfg(pPidTbl->aPids[3].cfg.nvm.saveFct, &pPidTbl->aPids[3].cfg.gain) ) )
				{
					/* error handling */
				}
			}
		}
		else if (UTIL1_strncmp((char*)cmd_, (char*)"pid speed L ", sizeof("pid speed L ")-1)==0)
		{
			res = ParsePidParameter( &pPidTbl->aPids[0].cfg.gain, cmd_+sizeof("pid speed L ")-1, handled_, io_);

			if( ERR_OK != PID_saveCfg(pPidTbl->aPids[0].cfg.nvm.saveFct, &pPidTbl->aPids[0].cfg.gain) )
			{
				/* error handling */
			}
		}
		else if (UTIL1_strncmp((char*)cmd_, (char*)"pid speed R ", sizeof("pid speed R ")-1)==0)
		{
			res = ParsePidParameter( &pPidTbl->aPids[1].cfg.gain, cmd_+sizeof("pid speed R ")-1, handled_, io_);

			if( ERR_OK != PID_saveCfg(pPidTbl->aPids[1].cfg.nvm.saveFct, &pPidTbl->aPids[1].cfg.gain) )
			{
				/* error handling */
			}
		}
		else
		{
			/* error handling */
		}
	}
	return res;
}



#ifdef MASTER_pid_clshdlr_C_
#undef MASTER_pid_clshdlr_C_
#endif /* !MASTER_pid_clshdlr_C_ */
