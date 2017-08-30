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
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	18.08.2017
 *  
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_tl_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tl_clshdlr.h"
#include "tl_cfg.h"
#if TL_USES_NVM
#include "nvm_api.h"
#endif

/*======================================= >> #DEFINES << =========================================*/
#define TL_ID_INVALID (0xFFu)

#define TL_ID_DUMP (0xFEu)

#define TL_SHORT_STRING ("tl")

#define CLS_SEND_ERR_ID \
	( CLS1_SendStr((uchar_t*)"*** ERROR: Invalid or non existing #ID ***\r\n", io_->stdErr) )

#define CLS_SEND_ERR_TBL \
	( CLS1_SendStr((uchar_t*)"*** ERROR: Invalid or non existing tracking loop table ***\r\n", io_->stdErr) )

#define CLS_SEND_ERR_NVM_DATATYPE \
	( CLS1_SendStr((uchar_t*)"*** ERROR: Writing to NVM failed - NVM configuration data type invalid ***\r\n", io_->stdErr) )



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef StdRtn_t ReadPIDCfg_t(NVM_PidCfg_t *);
typedef StdRtn_t SavePIDCfg_t(const NVM_PidCfg_t *);



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void Print_TLHelp(const CLS1_StdIOType *io_);
static void Print_TLStatus(uint8_t id_, const CLS1_StdIOType *io_);
static void Print_TLItmStatus(const PID_Gain_t* gain_, const PID_Data_t *data_,
		const uchar_t *kindStr_, uint8_t id_, const CLS1_StdIOType *io_);
#if TL_USES_NVM
static void Restore_TLItmCfg(uint8_t id_, const CLS1_StdIOType *io_);
static void Set_TLItmCfg(uint8_t id_, const uchar_t *cmd_, bool *handled_, const CLS1_StdIOType *io_);
#endif
static uint8_t Parse_TLParam(PID_Gain_t *itm_, const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void Print_TLHelp(const CLS1_StdIOType *io_)
{
	CLS1_SendHelpStr((unsigned char*)TL_SHORT_STRING, (unsigned char*)"Group of Tracking Loop commands\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)" [#ID] help|status", (unsigned char*)"Shows TL help or status\r\n", io_->stdOut);
#if TL_USES_NVM
	CLS1_SendHelpStr((unsigned char*)"  #ID set (p|i|w) <value>", (unsigned char*)"Sets new P, I or Anti-Windup value for TL #ID and saves it to the NVM\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  #ID set scaling <value>", (unsigned char*)"Sets new scaling value for TL #ID and saves it to the NVM\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  #ID restore", (unsigned char*)"Restores default parameters for TL #ID and saves them to the NVM\r\n", io_->stdOut);
#endif
}

static void Print_TLStatus(uint8_t id_, const CLS1_StdIOType *io_)
{
	uint8_t i = 0u;
	TL_ItmTbl_t *pTbl = Get_pTlItmTbl();

	if( (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		if (TL_ID_DUMP == id_ )
		{
				for(i = 0u; i < pTbl->numTls; i++)
				{
					Print_TLItmStatus(&(pTbl->aTls[i].cfg.pid), &(pTbl->aTls[i].data.pid),
							pTbl->aTls[i].cfg.pItmName, i, io_);
				}
		}
		else
		{
			if(id_ < pTbl->numTls)
			{
				Print_TLItmStatus(&(pTbl->aTls[i].cfg.pid), &pTbl->aTls[i].data.pid,
						pTbl->aTls[id_].cfg.pItmName, id_, io_);
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

static void Print_TLItmStatus(const PID_Gain_t* gain_, const PID_Data_t *data_,
		const uchar_t *kindStr_, uint8_t id_, const CLS1_StdIOType *io_)
{
	uchar_t buf[48];

	UTIL1_strcpy(buf,sizeof(buf),(uchar_t*)TL_SHORT_STRING);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)" #");
	UTIL1_strcatNum8u(buf,sizeof(buf),id_);
	CLS1_SendStatusStr(buf, (uchar_t*)"\r\n", io_->stdOut);

	CLS1_SendStatusStr((uchar_t*)"  assigned to", (uchar_t*)kindStr_, io_->stdOut);
	CLS1_SendStr((uchar_t*)"\r\n", io_->stdOut);

	buf[0] = '\0';
	UTIL1_strcpy(buf, sizeof(buf), (uchar_t*)"kp: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), gain_->kP_scld);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"  ki: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), gain_->kI_scld);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr((uchar_t*)"  pi gain", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_strcpy(buf, sizeof(buf), (uchar_t*)"0x");
	UTIL1_strcatNum16Hex(buf, sizeof(buf), gain_->intSatVal);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"u\r\n");
	CLS1_SendStatusStr("  pi satVal", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_Num8uToStr(buf, sizeof(buf), gain_->nScale);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr("  pi scaling", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_Num32sToStr(buf, sizeof(buf), data_->prevErr);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr("  pi prevErr", buf, io_->stdOut);

	buf[0] = '\0';
	UTIL1_Num32sToStr(buf, sizeof(buf), data_->intVal);
	UTIL1_strcat(buf, sizeof(buf), (uchar_t*)"\r\n");
	CLS1_SendStatusStr("  pi intVal", buf, io_->stdOut);

	return;
}

#if TL_USES_NVM
static void Restore_TLItmCfg(uint8_t id_, const CLS1_StdIOType *io_)
{
	uint8_t i = 0u;
	NVM_PidCfg_t tmp = {0u};
	TL_ItmTbl_t *pTbl = Get_pTlItmTbl();

	if( (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		if ( id_ < pTbl->numTls )
		{

			if ( (NULL != pTbl->aTls[id_].cfg.pNVMReadDfltValFct) && (NULL != pTbl->aTls[id_].cfg.pNVMSaveValFct) )
			{
				if ( ERR_OK == pTbl->aTls[id_].cfg.pNVMReadDfltValFct(&tmp) )
				{
					pTbl->aTls[id_].cfg.Config->Factor_KP_scld = (int32_t)tmp.KP_scld;
					pTbl->aTls[id_].cfg.Config->Factor_KI_scld = (int32_t)tmp.KI_scld;
					pTbl->aTls[id_].cfg.Config->Factor_KD_scld = 0u;
					pTbl->aTls[id_].cfg.Config->Scale 		 = (int16_t)tmp.Scale;
					pTbl->aTls[id_].cfg.Config->SaturationVal  = (int32_t)tmp.SaturationVal;
					CLS1_SendStr((uchar_t*)">>> Restoring successful...\r\n", io_->stdOut);

					if (ERR_OK != pTbl->aTls[id_].cfg.pNVMSaveValFct(&tmp))
					{
						CLS_SEND_ERR_NVM_DATATYPE;
					}
				}
				else
				{
					CLS1_SendStr((uchar_t*)"*** ERROR: Restoring failed - NVM configuration data type invalid ***\r\n", io_->stdErr);
				}
			}
			else
			{
				CLS1_SendStr((uchar_t*)"*** ERROR: Restoring failed - Invalid READ and/or WRITE Function(s) ***\r\n", io_->stdErr);
			}

		}
		else
		{
			CLS_SEND_ERR_ID;
		}
	}
	else
	{
		CLS_SEND_ERR_TBL;
	}
}

static void Set_TLItmCfg(uint8_t id_, const uchar_t *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
	uint8_t i = 0u;
	NVM_PidCfg_t tmp = {0u};
	TL_ItmTbl_t *pTbl = Get_pTlItmTbl();
	if( (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		if ( id_ < pTbl->numTls )
		{
			if( ERR_OK == Parse_TLParam( pTbl->aTls[id_].cfg.Config, cmd_, handled_, io_ ) )
			{
				CLS1_SendStr((uchar_t*)">>> Setting new parameters successful...\r\n", io_->stdOut);
				if ( NULL != pTbl->aTls[id_].cfg.pNVMSaveValFct )
				{
					tmp.KP_scld = pTbl->aTls[id_].cfg.Config->Factor_KP_scld;
					tmp.KI_scld = pTbl->aTls[id_].cfg.Config->Factor_KI_scld;
					tmp.KD_scld = 0u;
					tmp.Scale   = pTbl->aTls[id_].cfg.Config->Scale;
					tmp.SaturationVal = pTbl->aTls[id_].cfg.Config->SaturationVal;
					if ( ERR_OK == pTbl->aTls[id_].cfg.pNVMSaveValFct(&tmp) )
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
					CLS1_SendStr((uchar_t*)"*** ERROR: Saving to NVM failed - Invalid WRITE Function ***\r\n", io_->stdErr);
				}
			}
		}
		else
		{
			CLS_SEND_ERR_ID;
		}
	}
	else
	{
		CLS_SEND_ERR_TBL;
	}
}

static uint8_t Parse_TLParam(PID_Gain_t* itm_, const uchar_t *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
	const uchar_t *p;
	uint32_t val32u;
	uint8_t val8u;
	uint8_t res = ERR_OK;

	if (UTIL1_strncmp((char*)cmd_, (char*)"p ", sizeof("p ")-1)==0)
	{
		p = cmd_+sizeof("p");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK)
		{
			itm_->Factor_KP_scld = val32u;
			*handled_ = TRUE;
		}
		else
		{
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io_->stdErr);
			res = ERR_FAILED;
		}
	}
	else if (UTIL1_strncmp((char*)cmd_, (char*)"i ", sizeof("i ")-1)==0)
	{
		p = cmd_+sizeof("i");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK)
		{
			itm_->Factor_KI_scld = val32u;
			*handled_ = TRUE;
		}
		else
		{
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io_->stdErr);
			res = ERR_FAILED;
		}
	}
	else if (UTIL1_strncmp((char*)cmd_, (char*)"w ", sizeof("w ")-1)==0)
	{
		p = cmd_+sizeof("w");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK)
		{
			itm_->SaturationVal = val32u;
			*handled_ = TRUE;
		}
		else
		{
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io_->stdErr);
			res = ERR_FAILED;
		}
	}
	else if (UTIL1_strncmp((char*)cmd_, (char*)"scaling ", sizeof("scaling ")-1)==0)
	{
		p = cmd_+sizeof("scaling");
		if (UTIL1_ScanDecimal8uNumber(&p, &val8u)==ERR_OK && val8u<=100)
		{
			itm_->Scale = val8u;
			*handled_ = TRUE;
		}
		else
		{
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io_->stdErr);
			res = ERR_FAILED;
		}
	}
	else
	{
		CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io_->stdErr);
		res = ERR_FAILED;
	}
	return res;

}

#endif


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t TL_ParseCommand(const uchar_t *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
	uint8_t res = ERR_OK;
	uint8_t tlID = 0u;
	const uchar_t *p = NULL;
	uchar_t buf[sizeof("tl #ID set scaling 65535")]={'\0'};

	p = cmd_+sizeof("tl")-1u;
	if(ERR_FAILED == UTIL1_ScanDecimal8uNumber(&p,&tlID))
	{
		tlID = TL_ID_DUMP;
	}
	UTIL1_strcpy(buf,sizeof("tl"),cmd_);
	UTIL1_strcat(buf,sizeof(buf),p);

	if ( ERR_OK == UTIL1_strcmp((const char_t*)cmd_, (uchar_t*)CLS1_CMD_HELP) || ERR_OK == UTIL1_strcmp((const char_t*)buf, (const char*)"tl help") ) {
		Print_TLHelp(io_);
		*handled_ = TRUE;
	}
	else if (ERR_OK == UTIL1_strcmp((const char_t*)cmd_, (const char_t*)CLS1_CMD_STATUS) || ERR_OK == UTIL1_strcmp((const char_t*)buf, (const char_t*)"tl status") )
	{
		*handled_ = TRUE;
		Print_TLStatus(tlID, io_);
	}
#if TL_USES_NVM
	PID_PrmCfg_t tmpCfg = {0};
	else if (ERR_OK == UTIL1_strcmp((const char*)buf, (char*)"tl restore") )
	{
		*handled_ = TRUE;
		Restore_TLItmCfg(tlID, io_);
	}
	else if ( ERR_OK == UTIL1_strncmp((const char*)buf, (const char*)"tl set ", sizeof("tl set ")-1u) )
	{
		Set_TLItmCfg(tlID, buf+sizeof("tl set ")-1u, handled_, io_);
	}
#endif
	else
	{

	}

	return res;
}



#ifdef MASTER_tl_clshdlr_C_
#undef MASTER_tl_clshdlr_C_
#endif /* !MASTER_tl_clshdlr_C_ */
