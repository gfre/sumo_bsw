/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	28.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file pid_clshdlr.c
 * 
 *==================================================================================================
 */

#define MASTER_pid_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "pid_clshdlr.h"
#include "pid.h"
#include "nvm_Types.h"

/*======================================= >> #DEFINES << =========================================*/
/* #define TMPL_MACRO (0x01u) */



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef StdRtn_t ReadPIDCfg_t(NVM_PidCfg_t *);
typedef StdRtn_t SavePIDCfg_t(const NVM_PidCfg_t *);
typedef PID_Config *GetPIDConfig_t(void);



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void PID_PrintHelp(const CLS1_StdIOType *io);
static void PID_PrintStatus(const CLS1_StdIOType *io);
static void PrintPIDstatus(PID_Config *config, const unsigned char *kindStr, const CLS1_StdIOType *io);


/*=================================== >> GLOBAL VARIABLES << =====================================*/
/* static TmplType_t tmplArray[STUD_MACRO] = {0u,TRUE,FALSE};



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void PID_PrintHelp(const CLS1_StdIOType *io)
{
	CLS1_SendHelpStr((unsigned char*)"pid", (unsigned char*)"Group of PID commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows PID help or status\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  pos (p|i|d|w) <value>", (unsigned char*)"Sets P, I, D or anti-Windup position value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  pos speed <value>", (unsigned char*)"Maximum speed % value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed (L|R) (p|i|d|w) <value>", (unsigned char*)"Sets P, I, D or anti-Windup position value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed (L|R) speed <value>", (unsigned char*)"Maximum speed % value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  pos restore", (unsigned char*)"Restores and saves default parameters for position control to NVM\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed (L|R) restore", (unsigned char*)"Restores and saves default parameters for (L|R) speed control to NVM\r\n", io->stdOut);
}

static void PrintPIDstatus(PID_Config *config, const unsigned char *kindStr, const CLS1_StdIOType *io)
{
	unsigned char buf[48];
	unsigned char kindBuf[16];

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" PID");
	UTIL1_strcpy(buf, sizeof(buf), (unsigned char*)"p: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), config->pFactor100);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" i: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), config->iFactor100);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" d: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), config->dFactor100);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" windup");
	UTIL1_Num32sToStr(buf, sizeof(buf), config->iAntiWindup);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" error");
	UTIL1_Num32sToStr(buf, sizeof(buf), config->lastError);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" integral");
	UTIL1_Num32sToStr(buf, sizeof(buf), config->integral);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" speed");
	UTIL1_Num8uToStr(buf, sizeof(buf), config->maxSpeedPercent);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"%\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);
}

static void PID_PrintStatus(const CLS1_StdIOType *io)
{
	CLS1_SendStatusStr((unsigned char*)"pid", (unsigned char*)"\r\n", io->stdOut);
	PrintPIDstatus(PID_Get_PosLeCfg(), (unsigned char*)"pos L", io);
	PrintPIDstatus(PID_Get_PosRiCfg(), (unsigned char*)"pos R", io);
	PrintPIDstatus(PID_Get_SpdLeCfg(), (unsigned char*)"speed L", io);
	PrintPIDstatus(PID_Get_SpdRiCfg(), (unsigned char*)"speed R", io);
}

static uint8_t ParsePidParameter(PID_Config *config, const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
	const unsigned char *p;
	uint32_t val32u;
	uint8_t val8u;
	uint8_t res = ERR_OK;

	if (UTIL1_strncmp((char*)cmd, (char*)"p ", sizeof("p ")-1)==0) {
		p = cmd+sizeof("p");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			config->pFactor100 = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"i ", sizeof("i ")-1)==0) {
		p = cmd+sizeof("i");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			config->iFactor100 = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"d ", sizeof("d ")-1)==0) {
		p = cmd+sizeof("d");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			config->dFactor100 = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"w ", sizeof("w ")-1)==0) {
		p = cmd+sizeof("w");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			config->iAntiWindup = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"speed ", sizeof("speed ")-1)==0) {
		p = cmd+sizeof("speed");
		if (UTIL1_ScanDecimal8uNumber(&p, &val8u)==ERR_OK && val8u<=100) {
			config->maxSpeedPercent = val8u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	}
	return res;
}



static StdRtn_t PID_restoreCfg(ReadPIDCfg_t *readDfltCfg_, SavePIDCfg_t *saveCfg_, PID_Config *config_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	NVM_PidCfg_t tmp = {0u};

	if ( (NULL != readDfltCfg_) && (NULL != saveCfg_) && (NULL != config_ ) )
	{
		if ( ERR_OK == readDfltCfg_(&tmp) )
		{

			config_->pFactor100 = (int32_t)tmp.pGain100;
			config_->iFactor100 = (int32_t)tmp.iGain100;
			config_->dFactor100 = (int32_t)tmp.dGain100;
			config_->iAntiWindup = (int32_t)tmp.iAntiWindup;
			config_->maxSpeedPercent = (int32_t)tmp.maxSpdPerc;
			if (ERR_OK == saveCfg_(&tmp))
			{
				retVal = ERR_OK;
			}
		}
	}
	return retVal;
}


static StdRtn_t PID_saveCfg(SavePIDCfg_t *saveCfg_, PID_Config *config_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	NVM_PidCfg_t tmp = {0u};

	if ((NULL != saveCfg_) && (NULL != config_ ))
	{
		tmp.pGain100 = config_->pFactor100;
		tmp.iGain100 = config_->iFactor100;
		tmp.dGain100 = config_->dFactor100;
		tmp.iAntiWindup = config_->iAntiWindup;
		tmp.maxSpdPerc = config_->maxSpeedPercent;
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
		if( ( ERR_OK == PID_restoreCfg(NVM_Read_Dflt_PIDPosCfg, NVM_Save_PIDPosCfg, PID_Get_PosLeCfg()) )
		&&	( ERR_OK == PID_restoreCfg(NVM_Read_Dflt_PIDPosCfg, NVM_Save_PIDPosCfg, PID_Get_PosRiCfg()) ) )
		{
			*handled = TRUE;
		}
	} else if (UTIL1_strcmp((char*)cmd, (char*)"pid speed L restore")==0) {
		if( ERR_OK == PID_restoreCfg(NVM_Read_Dflt_PIDSpdLeCfg, NVM_Save_PIDSpdLeCfg, PID_Get_SpdLeCfg()) )
		{
			*handled = TRUE;
		}
	} else if (UTIL1_strcmp((char*)cmd, (char*)"pid speed R restore")==0) {
		if( ERR_OK == PID_restoreCfg(NVM_Read_Dflt_PIDSpdRiCfg, NVM_Save_PIDSpdRiCfg, PID_Get_SpdRiCfg()) )
		{
			*handled = TRUE;
		}

	} else if (UTIL1_strncmp((char*)cmd, (char*)"pid pos ", sizeof("pid pos ")-1)==0) {
		res = ParsePidParameter(PID_Get_PosLeCfg(), cmd+sizeof("pid pos ")-1, handled, io);
		if (res==ERR_OK)
		{
			res = ParsePidParameter(PID_Get_PosRiCfg(), cmd+sizeof("pid pos ")-1, handled, io);
		}

		if (res==ERR_OK)
		{
			if( ( ERR_OK != PID_saveCfg(NVM_Save_PIDPosCfg, PID_Get_PosLeCfg()) )
			||	( ERR_OK != PID_saveCfg(NVM_Save_PIDPosCfg, PID_Get_PosRiCfg()) ) )
			{
				/* TODO ERROR handling */
			}
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"pid speed L ", sizeof("pid speed L ")-1)==0) {
		res = ParsePidParameter(PID_Get_SpdLeCfg(), cmd+sizeof("pid speed L ")-1, handled, io);

		if( ERR_OK != PID_saveCfg(NVM_Save_PIDSpdLeCfg, PID_Get_SpdLeCfg()) )
		{
			/* TODO ERROR handling */
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"pid speed R ", sizeof("pid speed R ")-1)==0) {
		res = ParsePidParameter(PID_Get_SpdRiCfg(), cmd+sizeof("pid speed R ")-1, handled, io);

		if( ERR_OK != PID_saveCfg(NVM_Save_PIDSpdRiCfg, PID_Get_SpdRiCfg()) )
		{
			/* TODO ERROR handling */
		}
	}
	return res;
}




#ifdef MASTER_pid_clshdlr_C_
#undef MASTER_pid_clshdlr_C_
#endif /* !MASTER_pid_clshdlr_C_ */
