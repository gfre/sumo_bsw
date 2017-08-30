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
 **************************************************************************************************/

#define MASTER_pid_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "pid_clshdlr.h"
#include "pid_cfg.h"
#include "pid_api.h"
#include "nvm_api.h"



/*======================================= >> #DEFINES << =========================================*/
#define PID_ID_INVALID (0xFFu)

#define PID_ID_DUMP (0xFEu)

#define PID_SHORT_STRING ("pid")

#define PID_ID_PREFIX ('#')

#define CLS_SEND_ERR_ID \
	( CLS1_SendStr((uchar_t*)"*** ERROR: Invalid or non existing #ID ***\r\n", io_->stdErr) )

#define CLS_SEND_ERR_TBL \
	( CLS1_SendStr((uchar_t*)"*** ERROR: Invalid or non existing PID item table ***\r\n", io_->stdErr) )

#define CLS_SEND_ERR_NVM_DATATYPE \
	( CLS1_SendStr((uchar_t*)"*** ERROR: Writing to NVM failed - NVM configuration data type invalid ***\r\n", io_->stdErr) )


/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 *
 * @param
 * @return
 */
typedef StdRtn_t ReadPIDCfg_t(NVM_PidCfg_t *);

/**
 *
 * @param
 * @return
 */
typedef StdRtn_t SavePIDCfg_t(const NVM_PidCfg_t *);



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void Print_PidHelp(const CLS1_StdIOType *io_);
static void Print_PidStatus(uint8_t id_, const CLS1_StdIOType *io_);
static void Print_PidItmStatus(const PID_Gain_t* gain_, const PID_Data_t *data_,
		const uchar_t *kindStr_, uint8_t id_, const CLS1_StdIOType *io_);
static uint8_t Parse_PidParam(PID_Gain_t* gain_, const uchar_t *cmd_, bool *handled_, const CLS1_StdIOType *io_);

static StdRtn_t PID_restoreCfg(ReadPIDCfg_t *readDfltCfg_, SavePIDCfg_t *saveCfg_, PID_Gain_t* gain_);
static StdRtn_t PID_saveCfg(SavePIDCfg_t *saveCfg_, PID_Gain_t *gain_);




/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void Print_PidHelp(const CLS1_StdIOType *io_)
{
	CLS1_SendHelpStr((uchar_t*)PID_SHORT_STRING, (uchar_t*)"Group of PID commands\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)" [#ID] help|status", (unsigned char*)"Shows PID help or status\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  #ID set (p|i|d) <value>", (unsigned char*)"Sets new P-, I-, or D-gain PID #ID and saves it to the NVM\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  #ID set aw-up <value>", (unsigned char*)"Sets new Anti-Windup value for PID #ID and saves it to the NVM\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  #ID set N <value>", (unsigned char*)"Sets new 2^N scaling for PID #ID and saves it to the NVM\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  #ID restore", (unsigned char*)"Restores default parameters for PID #ID and saves them to the NVM\r\n", io_->stdOut);
}



static void Print_PidStatus(uint8_t id_, const CLS1_StdIOType *io_)
{
	uint8_t i = 0u;
	PID_ItmTbl_t *pTbl = Get_pPidItmTbl();

	if( (NULL != pTbl) && ( NULL != pTbl->aPids) )
	{
		if (PID_ID_DUMP == id_ )
		{
				for(i = 0u; i < pTbl->numPids; i++)
				{
					Print_PidItmStatus(&(pTbl->aPids[i].cfg.gain), &(pTbl->aPids[i].data),
							pTbl->aPids[i].cfg.pItmName, i, io_);
				}
		}
		else
		{
			if(id_ < pTbl->numPids)
			{
				Print_PidItmStatus(&(pTbl->aPids[id_].cfg.gain), &(pTbl->aPids[id_].data),
						pTbl->aPids[id_].cfg.pItmName, id_, io_);
			}
			else
			{
				CLS_SEND_ERR_ID;
			}
		}
	}
	else
	{
		CLS_SEND_ERR_TBL;
	}
}

static void Print_PidItmStatus(const PID_Gain_t* gain_, const PID_Data_t *data_,
		const uchar_t *kindStr_, uint8_t id_, const CLS1_StdIOType *io_)
{
	uchar_t buf[48];

	UTIL1_strcpy(buf,sizeof(buf),(uchar_t*)PID_SHORT_STRING);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)" #");
	UTIL1_strcatNum8u(buf,sizeof(buf),id_);
	CLS1_SendStatusStr(buf, (uchar_t*)"\r\n", io_->stdOut);

	CLS1_SendStatusStr((uchar_t*)"  assign. to", (uchar_t*)kindStr_, io_->stdOut);
	CLS1_SendStr((uchar_t*)"\r\n", io_->stdOut);

	buf[0] = '\0';
	UTIL1_strcpy(buf, sizeof(buf), (uchar_t*)"kp: ");
	UTIL1_strcatNum16u(buf, sizeof(buf), gain_->kP_scld);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"  ki: ");
	UTIL1_strcatNum16u(buf, sizeof(buf), gain_->kI_scld);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"  kd: ");
	UTIL1_strcatNum16u(buf, sizeof(buf), gain_->kD_scld);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr((uchar_t*)"  gains", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_Num8uToStr(buf, sizeof(buf), gain_->nScale);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr("  2^N scale", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_strcpy(buf, sizeof(buf), (uchar_t*)"0x");
	UTIL1_strcatNum32Hex(buf, sizeof(buf), gain_->intSatVal);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"u\r\n");
	CLS1_SendStatusStr("  aw-up val", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_strcatNum32s(buf, sizeof(buf), data_->sat);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"u\r\n");
	CLS1_SendStatusStr("  sat. st", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_Num32sToStr(buf, sizeof(buf), data_->intVal);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr("  int. val", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_Num32sToStr(buf, sizeof(buf), data_->prevErr);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr("  err. val", buf, io_->stdOut);

	return;
}


static uint8_t Parse_PidParam(PID_Gain_t* gain_, const uchar_t *cmd_, bool *handled_, const CLS1_StdIOType *io_)
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
	uint8_t retVal = ERR_OK;
	PID_ItmTbl_t *pPidTbl = Get_pPidItmTbl();
	uint8_t pidID = 0u;
	const uchar_t *p = NULL;
	uchar_t buf[sizeof("pid #ID set aw-up 0xFFFFFFFFu")]={'\0'};

	p = cmd_+sizeof(PID_SHORT_STRING)-1u;

	while(' ' == *p || PID_ID_PREFIX == *p)
	{
		p++;
	}
	if(ERR_FAILED == UTIL1_ScanDecimal8uNumber(&p,&pidID))
	{
		pidID = PID_ID_DUMP;
	}
	while(' ' == *p )
	{
		p++;
	}

	UTIL1_strcpy(buf,sizeof(buf),p);
	if( ( NULL != pPidTbl ) && ( NULL != pPidTbl->aPids ) )
	{

		if (ERR_OK == UTIL1_strcmp((const char_t *)cmd_, (const char_t *)CLS1_CMD_HELP) || ERR_OK == UTIL1_strcmp((const char_t *)buf, (const char_t *)CLS1_CMD_HELP) )
		{
			Print_PidHelp(io_);
			*handled_ = TRUE;
		}
		else if (ERR_OK == UTIL1_strcmp((const char_t *)cmd_, (const char_t *)CLS1_CMD_STATUS) || ERR_OK == UTIL1_strcmp((const char_t *)buf, (const char_t *)CLS1_CMD_STATUS) )
		{
			Print_PidStatus(pidID, io_);
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
			retVal = Parse_PidParam( &pPidTbl->aPids[2].cfg.gain, cmd_+sizeof("pid pos ")-1, handled_, io_);
			if (retVal==ERR_OK)
			{
				retVal = Parse_PidParam( &pPidTbl->aPids[3].cfg.gain, cmd_+sizeof("pid pos ")-1, handled_, io_);
			}

			if (retVal==ERR_OK)
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
			retVal = Parse_PidParam( &pPidTbl->aPids[0].cfg.gain, cmd_+sizeof("pid speed L ")-1, handled_, io_);

			if( ERR_OK != PID_saveCfg(pPidTbl->aPids[0].cfg.nvm.saveFct, &pPidTbl->aPids[0].cfg.gain) )
			{
				/* error handling */
			}
		}
		else if (UTIL1_strncmp((char*)cmd_, (char*)"pid speed R ", sizeof("pid speed R ")-1)==0)
		{
			retVal = Parse_PidParam( &pPidTbl->aPids[1].cfg.gain, cmd_+sizeof("pid speed R ")-1, handled_, io_);

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
	return retVal;
}



#ifdef MASTER_pid_clshdlr_C_
#undef MASTER_pid_clshdlr_C_
#endif /* !MASTER_pid_clshdlr_C_ */
