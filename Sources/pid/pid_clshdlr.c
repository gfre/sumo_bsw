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

#define CLS_SEND_ERR_TBL \
	( CLS1_SendStr((uchar_t*)"*** ERROR: Invalid or non existing tracking loop table ***\r\n", io_->stdErr) )

#define CLS_SEND_ERR_ID_NOT_SPECIFIED \
	( CLS1_SendStr((uchar_t*)"*** ERROR: Invalid argument - #ID not specified ***\r\n", io_->stdErr) )

#define CLS_SEND_ERR_ID_NOT_FOUND \
	( CLS1_SendStr((uchar_t*)"*** ERROR: Invalid argument - #ID not found ***\r\n", io_->stdErr) )

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
static StdRtn_t Parse_PidID(const uchar_t *cmd_, const uchar_t **p_, uint8_t *id_);
static void Print_PidHelp(const CLS1_StdIOType *io_);
static void Print_PidItmStatus(const PID_Gain_t* gain_, const PID_Data_t *data_,
		const uchar_t *kindStr_, uint8_t id_, const CLS1_StdIOType *io_);
static void Print_PidStatus(uint8_t id_, const CLS1_StdIOType *io_);
static void Restore_PidGainCfg(ReadPIDCfg_t *readDfltCfg_, SavePIDCfg_t *saveCfg_,PID_Gain_t* gain_,
		const CLS1_StdIOType *io_);
static uint8_t Parse_PidGainArgs(PID_Gain_t* gain_, const uchar_t *cmd_, bool *handled_,
		const CLS1_StdIOType *io_);
static StdRtn_t Save_PidGainCfg(SavePIDCfg_t *saveCfg_, PID_Gain_t *gain_,
		const CLS1_StdIOType *io_);




/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static StdRtn_t Parse_PidID(const uchar_t *cmd_, const uchar_t **p_, uint8_t *id_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if( (NULL != p_) && (NULL != id_) )
	{
		retVal = ERR_OK;
		/* set pointer to string position after "pid" */
		*p_ = cmd_+sizeof(PID_SHORT_STRING)-1u;

		/* eat up spaces and hashtag */
		while(' ' == **p_ || PID_ID_PREFIX == **p_)
		{
			(*p_)++;
		}

		/* Scan ID - set to DUMP if there is no ID */
		if(ERR_FAILED == UTIL1_ScanDecimal8uNumber(p_,id_))
		{
			*id_ = PID_ID_DUMP;
		}

		/* eat up spaces */
		while(' ' == **p_ )
		{
			(*p_)++;
		}
		/* go back one space */
		(*p_)--;
	}
	return retVal;
}


static void Print_PidHelp(const CLS1_StdIOType *io_)
{
	CLS1_SendHelpStr((uchar_t*)PID_SHORT_STRING, (uchar_t*)"Group of PID commands\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)" [#ID] help|status", (unsigned char*)"Shows PID help or status\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  #ID set (p|i|d) <value>", (unsigned char*)"Sets a new P-, I-, or D-gain value for #ID and saves it to the NVM\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  #ID set bp <0...15>", (unsigned char*)"Sets a new binary point for the gains of #ID and saves it to the NVM\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  #ID set a-wup <value>", (unsigned char*)"Sets a new anti-windup value for #ID and saves it to the NVM\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  #ID restore", (unsigned char*)"Restores default parameters for #ID and saves them to the NVM\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  restore all", (unsigned char*)"Restores default parameters for all #IDs and saves them to the NVM\r\n", io_->stdOut);
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
	UTIL1_strcpy(buf, sizeof(buf), (uchar_t*)"p: ");
	UTIL1_strcatNum16u(buf, sizeof(buf), gain_->kP_scld);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"  i: ");
	UTIL1_strcatNum16u(buf, sizeof(buf), gain_->kI_scld);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"  d: ");
	UTIL1_strcatNum16u(buf, sizeof(buf), gain_->kD_scld);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr((uchar_t*)"  gains", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_Num8uToStr(buf, sizeof(buf), gain_->nScale);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr("  bin. point", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_strcpy(buf, sizeof(buf), (uchar_t*)"0x");
	UTIL1_strcatNum32Hex(buf, sizeof(buf), gain_->intSatVal);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"u\r\n");
	CLS1_SendStatusStr("  a-wup value", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_strcatNum32s(buf, sizeof(buf), data_->sat);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"u\r\n");
	CLS1_SendStatusStr("  sat. state", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_Num32sToStr(buf, sizeof(buf), data_->intVal);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr("  int. value", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_Num32sToStr(buf, sizeof(buf), data_->prevErr);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr("  err. value", buf, io_->stdOut);

	return;
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
				CLS_SEND_ERR_ID_NOT_FOUND;
			}
		}
	}
	else
	{
		CLS_SEND_ERR_TBL;
	}
}


static void Restore_PidGainCfg(ReadPIDCfg_t *readDfltCfg_, SavePIDCfg_t *saveCfg_, PID_Gain_t* gain_,
		const CLS1_StdIOType *io_)
{
	NVM_PidCfg_t tmp = {0u};

	if( (NULL != gain_ ) )
	{
		if ( (NULL != readDfltCfg_) && (NULL != saveCfg_) )
		{
			if ( ERR_OK == readDfltCfg_(&tmp) )
			{
				gain_->kP_scld = tmp.KP_scld;
				gain_->kI_scld = tmp.KI_scld;
				gain_->kD_scld = tmp.KD_scld;
				gain_->nScale  = tmp.Scale;
				gain_->intSatVal  = tmp.SaturationVal;
				CLS1_SendStr((uchar_t*)">>> Restoring successful...\r\n", io_->stdOut);
				if (ERR_OK == saveCfg_(&tmp))
				{
					CLS1_SendStr((uchar_t*)">>> Saving to NVM successful...\r\n", io_->stdOut);
				}
				else
				{
					CLS_SEND_ERR_NVM_DATATYPE;
				}
			}
			else
			{
				CLS1_SendStr((uchar_t*)"*** ERROR: Restoring failed - "
						"NVM configuration data type invalid ***\r\n", io_->stdErr);
			}
		}
		else
		{
			CLS1_SendStr((uchar_t*)"*** ERROR: Restoring failed - "
					"Invalid READ and/or WRITE Function(s) ***\r\n", io_->stdErr);
		}
	}
	else
	{
		CLS1_SendStr((uchar_t*)"*** ERROR: Restoring failed - "
				"Invalid reference to gain configuration ***\r\n", io_->stdErr);
	}

	return;
}


static uint8_t Parse_PidGainArgs(PID_Gain_t* gain_, const uchar_t *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
	const uchar_t *p;
	uint32_t val32u = 0u;
	uint16_t val16u = 0u;
	uint8_t val8u = 0u;
	uint8_t retVal = ERR_PARAM_ADDRESS;

	if(NULL != gain_)
	{
		retVal = ERR_PARAM_DATA;
		while( ' ' == *cmd_ )
		{
			cmd_++;
		}

		if ( ERR_OK == UTIL1_strncmp((char*)cmd_, (char*)"p ", sizeof("p ")-1) )
		{
			*handled_ = TRUE;
			p = cmd_+sizeof("p");
			if (UTIL1_ScanDecimal16uNumber(&p, &val16u)==ERR_OK)
			{
				gain_->kP_scld = val16u;

				retVal = ERR_OK;
			}
			else
			{
				CLS1_SendStr((uchar_t*)"Wrong argument\r\n", io_->stdErr);
			}
		}
		else if ( ERR_OK == UTIL1_strncmp((char*)cmd_, (char*)"i ", sizeof("i ")-1) )
		{
			*handled_ = TRUE;
			p = cmd_+sizeof("i");
			if (UTIL1_ScanDecimal16uNumber(&p, &val16u)==ERR_OK)
			{
				gain_->kI_scld = val16u;
				retVal = ERR_OK;
			}
			else
			{
				CLS1_SendStr((uchar_t*)"Wrong argument\r\n", io_->stdErr);
			}
		}
		else if ( ERR_OK == UTIL1_strncmp((char*)cmd_, (char*)"d ", sizeof("d ")-1) )
		{
			*handled_ = TRUE;
			p = cmd_+sizeof("d");
			if (UTIL1_ScanDecimal16uNumber(&p, &val16u)==ERR_OK)
			{
				gain_->kD_scld = val16u;
				retVal = ERR_OK;
			}
			else
			{
				CLS1_SendStr((uchar_t*)"Wrong argument\r\n", io_->stdErr);
			}
		}
		else if ( ERR_OK == UTIL1_strncmp((char*)cmd_, (char*)"a-wup ", sizeof("a-wup ")-1) )
		{
			*handled_ = TRUE;
			p = cmd_+sizeof("a-wup");
			if (UTIL1_ScanHex32uNumber(&p, &val32u)==ERR_OK) {
				gain_->intSatVal = val32u;

				retVal = ERR_OK;
			}
			else
			{
				CLS1_SendStr((uchar_t*)"Wrong argument\r\n", io_->stdErr);
			}
		}
		else if ( ERR_OK == UTIL1_strncmp((char*)cmd_, (char*)"bp ", sizeof("bp ")-1) )
		{
			*handled_ = TRUE;
			p = cmd_+sizeof("bp");
			if (UTIL1_ScanDecimal8uNumber(&p, &val8u)==ERR_OK && val8u<=100)
			{
				gain_->nScale = val8u;
				retVal = ERR_OK;
			}
			else
			{
				CLS1_SendStr((uchar_t*)"Wrong argument\r\n", io_->stdErr);
			}
		}
		else
		{
			*handled_ = FALSE;
		}
	}
	else
	{
		CLS1_SendStr((uchar_t*)"*** ERROR: Setting new parameters failed - "
				"Invalid reference to gain configuration ***\r\n", io_->stdErr);
	}
	return retVal;
}


static StdRtn_t Save_PidGainCfg(SavePIDCfg_t *saveCfg_, PID_Gain_t *gain_, const CLS1_StdIOType *io_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	NVM_PidCfg_t tmp = {0u};

	if( (NULL != gain_ ) )
	{
		if ( NULL != saveCfg_ )
		{
			tmp.KP_scld = gain_->kP_scld;
			tmp.KI_scld = gain_->kI_scld;
			tmp.KD_scld = gain_->kD_scld;
			tmp.Scale   = gain_->nScale;
			tmp.SaturationVal = gain_->intSatVal;
			if (ERR_OK == saveCfg_(&tmp))
			{
				retVal = ERR_OK;
				CLS1_SendStr((uchar_t*)">>> Saving to NVM successful...\r\n", io_->stdOut);
			}
			else
			{
				CLS_SEND_ERR_NVM_DATATYPE;
			}
		}
		else
		{
			CLS1_SendStr((uchar_t*)"*** ERROR: Saving to NVM failed - "
					"Invalid WRITE Function ***\r\n", io_->stdErr);
		}
	}
	else
	{
		CLS1_SendStr((uchar_t*)"*** ERROR: Setting new parameters failed - "
						"Invalid reference to gain configuration ***\r\n", io_->stdErr);
	}
}


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t PID_ParseCommand(const uchar_t *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
	uint8_t retVal = ERR_OK;
	PID_ItmTbl_t *pPidTbl = Get_pPidItmTbl();
	uint8_t pidID = 0u;
	const uchar_t *p = NULL;
	uchar_t buf[sizeof("pid #ID set a-wup 0xFFFFFFFFu")]={'\0'};

	(void)Parse_PidID(cmd_,&p,&pidID);
	UTIL1_strcpy(buf,sizeof(PID_SHORT_STRING),cmd_);
	UTIL1_strcat(buf,sizeof(buf),p);

	if( ( NULL != pPidTbl ) && ( NULL != pPidTbl->aPids ) )
	{
		if (ERR_OK == UTIL1_strcmp((const char_t *)cmd_, (const char_t *)CLS1_CMD_HELP) || ERR_OK == UTIL1_strcmp((const char_t *)buf, (const char_t *)"pid help") )
		{
			Print_PidHelp(io_);
			*handled_ = TRUE;
		}
		else if (ERR_OK == UTIL1_strcmp((const char_t *)cmd_, (const char_t *)CLS1_CMD_STATUS) || ERR_OK == UTIL1_strcmp((const char_t *)buf, (const char_t *)"pid status") )
		{
			*handled_ = TRUE;
			if( (pidID < pPidTbl->numPids) || (PID_ID_DUMP == pidID) )
			{
				Print_PidStatus(pidID, io_);
			}
			else
			{
				CLS_SEND_ERR_ID_NOT_FOUND;
			}
		}
		else if (ERR_OK == UTIL1_strcmp((char*)buf, (char*)"pid restore") )
		{
			*handled_ = TRUE;
			if( pidID < pPidTbl->numPids )
			{
				Restore_PidGainCfg(pPidTbl->aPids[pidID].cfg.nvm.readDfltFct, pPidTbl->aPids[pidID].cfg.nvm.saveFct, &pPidTbl->aPids[pidID].cfg.gain, io_);
			}
			else if (PID_ID_DUMP == pidID)
			{
				CLS_SEND_ERR_ID_NOT_SPECIFIED;
			}
			else
			{
				CLS_SEND_ERR_ID_NOT_FOUND;
			}
		}
		else if( (ERR_OK == UTIL1_strcmp((char*)buf, (char*)"pid restore all") ) && (PID_ID_DUMP == pidID) )
		{
			*handled_ = TRUE;
			for(pidID = 0u; pidID < pPidTbl->numPids; pidID++ )
			{
				Restore_PidGainCfg(pPidTbl->aPids[pidID].cfg.nvm.readDfltFct, pPidTbl->aPids[pidID].cfg.nvm.saveFct, &pPidTbl->aPids[pidID].cfg.gain, io_);
			}
		}
		else if (UTIL1_strncmp((char*)buf, (char*)"pid set", sizeof("pid set")-1)==0)
		{
			if( pidID < pPidTbl->numPids )
			{
				if( ERR_OK == Parse_PidGainArgs( &pPidTbl->aPids[pidID].cfg.gain, buf+sizeof("pid set")-1u, handled_, io_) )
				{
					CLS1_SendStr((uchar_t*)">>> Setting new parameters successful...\r\n", io_->stdOut);
					if( ERR_OK != Save_PidGainCfg(pPidTbl->aPids[pidID].cfg.nvm.saveFct, &pPidTbl->aPids[pidID].cfg.gain, io_) )
					{
						/* error handling */
					}
				}
			}
			else if (PID_ID_DUMP == pidID)
			{
				*handled_ = TRUE;
				CLS_SEND_ERR_ID_NOT_SPECIFIED;
			}
			else
			{
				*handled_ = TRUE;
				CLS_SEND_ERR_ID_NOT_FOUND;
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
