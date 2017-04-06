/***********************************************************************************************//**
 * @file		rte.c
 * @ingroup		rte
 * @brief 		Implementation of the RTE
 *
 * The *Real-Time Environment* (@b RTE) is the application interface for application software development
 * within the ACon Sumo Robot Project. The firmware and basic software of the Sumo Robot is abstracted
 * to allow hardware independent development of C99-compliant application software
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_RTE_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "ind_Types.h"
#include "KEY1.h"
#include "buz.h"
#include "rte.h"
#include "tacho.h"
#include "drv_Types.h"
#include "RApp.h"
#include "rnet_Types.h"
#include "sh.h"
#include "sh_Types.h"
#include "id_Types.h"
#include "appl_Types.h"



/*======================================= >> #DEFINES << =========================================*/
#define USER_SWITCH_MASK 	(0x01u)
#define RTE_ERR_MSG_ADDRESS ("ERROR: Invliad pointer or address")



/*=================================== >> GLOBAL VARIABLES << =====================================*/
RTE_STREAM *RTE_stderr = NULL;
RTE_STREAM *RTE_stdout = NULL;



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void RTE_Init(void)
{
#ifndef ASW_STREAM_T
	RTE_stdout = (RTE_STREAM *)CLS1_GetStdio()->stdOut;
	RTE_stderr = (RTE_STREAM *)CLS1_GetStdio()->stdErr;
#endif
}

/*================================================================================================*/


/**
 * Interface to applicaton state machine
 */
void RTE_ReInitAppl(void)
{
	return APPL_Set_ReInitAppl();
}

StdRtn_t RTE_Write_HoldOnEnterNormal(const uint8_t holdOn_)
{
	return Set_HoldOnEnter(APPL_STATE_NORMAL, holdOn_);
}

StdRtn_t RTE_Write_HoldOnEnterIdle(const uint8_t holdOn_)
{
	return Set_HoldOnEnter(APPL_STATE_IDLE, holdOn_);
}

StdRtn_t RTE_Release_HoldOnEnterNormal(void)
{
	return Set_ReleaseEnter(APPL_STATE_NORMAL);
}

StdRtn_t RTE_Release_HoldOnEnterIdle(void)
{
	return Set_ReleaseEnter(APPL_STATE_IDLE);
}

/*================================================================================================*/


/**
 * Interface implementation for the left LED
 */
StdRtn_t RTE_Write_LedLeOn()
{
	return IND_Set_LED1On();
}

StdRtn_t RTE_Write_LedLeOff()
{
	return IND_Set_LED1Off();
}

StdRtn_t RTE_Write_LedLeNeg()
{
	return IND_Set_LED1Toggle();
}

StdRtn_t RTE_Write_LedLeSt(uint8_t state_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(FALSE==state_)
	{
		retVal = IND_Set_LED1Off();
	}
	else
	{
		retVal = IND_Set_LED1On();
	}
	return retVal;
}

StdRtn_t RTE_Write_LedLeFlshWithPerMS(uint16_t perMS_)
{
	return IND_Flash_LED1WithPerMS(perMS_);
}

StdRtn_t RTE_Read_LedLeSt(uint8_t *state_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL!=state_)
	{
		*state_ = IND_Get_LED1St();
		retVal = ERR_OK;
	}
	return retVal;
}

/*================================================================================================*/


/**
 * Interface implementation for the right LED
 */
StdRtn_t RTE_Write_LedRiOn()
{
	return IND_Set_LED2On();
}

StdRtn_t RTE_Write_LedRiOff()
{
	return IND_Set_LED2Off();
}

StdRtn_t RTE_Write_LedRiNeg()
{
	return IND_Set_LED2Toggle();
}

StdRtn_t RTE_Write_LedRiSt(uint8_t state_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(FALSE==state_)
	{
		retVal = IND_Set_LED2Off();
	}
	else
	{
		retVal = IND_Set_LED2On();
	}
	return retVal;
}

StdRtn_t RTE_Write_LedRiFlshWithPerMS(uint16_t perMS_)
{
	return IND_Flash_LED2WithPerMS(perMS_);
}

StdRtn_t RTE_Read_LedRiSt(uint8_t *state_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL!=state_)
	{
		*state_ = IND_Get_LED2St();
		retVal = ERR_OK;
	}
	return retVal;
}

/*================================================================================================*/


/**
 * Interface implementation for the user switch
 */
typedef struct CbFctTab_s{
	EvntCbFct_t *cbFctOnPrsd;
	EvntCbFct_t *cbFctOnLngPrsd;
	EvntCbFct_t *cbFctOnRlsd;
	EvntCbFct_t *cbFctOnLngRlsd;
}CbFctTab_t;

static CbFctTab_t cbFctTab={NULL};

StdRtn_t RTE_Read_BtnSt(uint8_t *state_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL!=state_)
	{
		*state_ = (uint8_t)KEY1_GetKeys() & USER_SWITCH_MASK;
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_BtnOnPrsdCbFct(const EvntCbFct_t *cbFct_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != cbFct_)
	{
		cbFctTab.cbFctOnPrsd = cbFct_;
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_BtnOnLngPrsdCbFct(const EvntCbFct_t *cbFct_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != cbFct_)
	{
		cbFctTab.cbFctOnLngPrsd = cbFct_;
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_BtnOnRlsdCbFct(const EvntCbFct_t *cbFct_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != cbFct_)
	{
		cbFctTab.cbFctOnRlsd = cbFct_;
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_BtnOnLngRlsdCbFct(const EvntCbFct_t *cbFct_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != cbFct_)
	{
		cbFctTab.cbFctOnLngRlsd = cbFct_;
		retVal = ERR_OK;
	}
	return retVal;
}

EvntCbFct_t *RTE_Get_BtnOnPrsdCbFct(void)
{
	return cbFctTab.cbFctOnPrsd;
}

EvntCbFct_t *RTE_Get_BtnOnLngPrsdCbFct(void)
{
	return cbFctTab.cbFctOnLngPrsd;
}

EvntCbFct_t *RTE_Get_BtnOnRlsdCbFct(void)
{
	return cbFctTab.cbFctOnRlsd;
}

EvntCbFct_t *RTE_Get_BtnOnLngRlsdCbFct(void)
{
	return cbFctTab.cbFctOnLngRlsd;
}

/*================================================================================================*/


/**
 * Interface implementation for the buzzer
 */
StdRtn_t RTE_Write_BuzPlayTune(BUZ_Tunes_t tune_)
{
	StdRtn_t retVal = ERR_PARAM_VALUE;
	if(BUZ_TUNE_NOF_TUNES > tune_)
	{
		retVal = (StdRtn_t)BUZ_PlayTune(tune_);
	}
	return retVal;
}


StdRtn_t RTE_Play_BuzBeep(uint16 freqHz_, uint16 durMs_)
{
	return (StdRtn_t)BUZ_Beep(freqHz_, durMs_);
}

/*================================================================================================*/


/**
 * Interface implementation for the speedometer
 */
#define LEFT   (TRUE)
#define RIGHT  (FALSE)

StdRtn_t RTE_Read_SpdoVelLe(uint16 *vel_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != vel_)
	{
		*vel_ = TACHO_GetSpeed(LEFT);
		retVal = ERR_OK;
	}
	return retVal;
}


StdRtn_t RTE_Read_SpdoVelRi(uint16 *vel_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != vel_)
	{
		*vel_ = TACHO_GetSpeed(RIGHT);
		retVal = ERR_OK;
	}
	return retVal;
}

#ifdef LEFT
#undef LEFT
#endif

#ifdef RIGHT
#undef RIGHT
#endif

/*================================================================================================*/


/**
 * Interface implementation for the drive component
 */
StdRtn_t RTE_Write_DrvVel(int32 velLe_, int32 velRi_)
{
	return (StdRtn_t)DRV_SetSpeed(velLe_, velRi_);
}

StdRtn_t RTE_Write_DrvPos(int32 posLe_, int32 posRi_)
{
	return (StdRtn_t)DRV_SetSpeed(posLe_, posRi_);
}

StdRtn_t RTE_Write_DrvMode(DrvMode_t mode_)
{
	StdRtn_t retVal = ERR_PARAM_VALUE;
	if((DRV_MODE_INVALID > mode_) && (DRV_MODE_NONE <= mode_))
	{
		retVal = (StdRtn_t)DRV_SetMode((DRV_Mode_t)(mode_));
	}
	return retVal;
}

StdRtn_t RTE_Read_DrvMode(DrvMode_t *mode_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != mode_)
	{
		*mode_ = (DrvMode_t)DRV_GetMode();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Read_DrvIsDrvgBkwd(uint8_t *isDrvgBkwd_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != isDrvgBkwd_)
	{
		*isDrvgBkwd_ = DRV_IsDrivingBackward();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Read_DrvHasStpd(uint8_t *hasStpd_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != hasStpd_)
	{
		*hasStpd_ =  DRV_IsStopped();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Read_DrvHasRvsd(uint8_t *hasRvsd_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != hasRvsd_)
	{
		*hasRvsd_ =  DRV_HasTurned();
		retVal = ERR_OK;
	}
	return retVal;
}

/*================================================================================================*/


/**
 * Interface implementation for the radio application layer
 */
StdRtn_t RTE_Write_RFSendDataBlk(const uint8_t *payload_, uint8_t payloadSize_, RTE_RF_MSG_TYPE_T msgType_,  uint8_t dstAddr_, uint8_t flags_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != payload_)
	{
		retVal = (StdRtn_t)RAPP_SendPayloadDataBlock((uint8_t *)payload_, payloadSize_, msgType_,  (RAPP_ShortAddrType)dstAddr_,  (RAPP_FlagsType)flags_);
	}
	return retVal;
}

StdRtn_t RTE_Write_RFRxMsgCbFct(const RFRxMsgCbFct_t *cbFct_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != cbFct_)
	{
		RNET_SetRTERxMsgCbFct((const RAPP_RxMsg_CbFct *)cbFct_);
	}
	return retVal;
}

RFRxMsgCbFct_t *RTE_Get_RFRxMsgCbFct(void)
{
	return RNET_GetRTERxMsgCbFct();
}



StdRtn_t RTE_Read_RFSniffPkt(RFPktDes_t *pkt_, uint8_t isTx_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	RAPP_PacketDesc pkt={0u};
	if((NULL != pkt_) && (NULL != pkt_->data))
	{
		isTx_ &= TRUE;
		pkt.flags  = (RPHY_FlagsType)pkt_->flags;
		pkt.phySize = pkt_->size;
		pkt.phyData = pkt_->data;
		pkt.rxtx    = pkt_->rxtx;
		RAPP_SniffPacket(&pkt, (bool)isTx_);
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Read_RFSrcAddr(uint8_t *addr_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != addr_)
	{
		*addr_ = (uint8_t)RAPP_GetThisNodeAddr();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_RFSrcAddr(uint8_t addr_)
{
	return (StdRtn_t)RAPP_SetThisNodeAddr((RAPP_ShortAddrType)addr_);
}

StdRtn_t RTE_Read_RFDstAddr(uint8_t *addr_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != addr_)
	{
		*addr_ = (uint8_t)RNET_GetDstAddr();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_RFDstAddr(uint8_t addr_)
{
	RNET_SetDstAddr((RAPP_ShortAddrType)addr_);
	return ERR_OK;
}

/*================================================================================================*/


/**
 * Interface implementation for shell debugging
 */
unsigned int RTE_fprintf(RTE_STREAM *stream_ ,unsigned char *fmt_, ...)
{
	va_list args;
	unsigned int count = 0u;

	if ( ( NULL != fmt_ ) && ( NULL != stream_ ) )
	{
		  va_start(args,fmt_);
		  if ( RTE_stdout == stream_ )
		  {
			  count = SH_FPRINTF(stdOut, fmt_, args);
		  }
		  else if ( RTE_stderr == stream_ )
		  {
			  count = SH_FPRINTF(stdErr, fmt_, args);
		  }
		  else
		  {
			  SH_SENDERRSTR(RTE_ERR_MSG_ADDRESS);
		  }
		  va_end(args);
	}
	else
	{
		SH_SENDERRSTR(RTE_ERR_MSG_ADDRESS);
	}
	return count;
}

unsigned int RTE_printf(unsigned char *fmt_, ...)
{
	va_list args;
	unsigned int count = 0u;

	if( NULL != fmt_ )
	{
		va_start(args,fmt_);
		count = SH_PRINTF(fmt_, args);
		va_end(args);
	}
	else
	{
		SH_SENDERRSTR(RTE_ERR_MSG_ADDRESS);
	}
	return count;
}



StdRtn_t RTE_puts(const uint8_t *msg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != msg_)
	{
		SH_SENDSTR(msg_);
		retVal = ERR_OK;
	}
	else
	{
		SH_SENDERRSTR(RTE_ERR_MSG_ADDRESS);
	}
	return retVal;
}

StdRtn_t RTE_putsErr(const uint8_t *errMsg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != errMsg_)
	{
		SH_SENDERRSTR(errMsg_);
		retVal = ERR_OK;
	}
	else
	{
		SH_SENDERRSTR(RTE_ERR_MSG_ADDRESS);
	}
	return retVal;
}

/*================================================================================================*/


/**
 * Interface implementation for the sumo ID
 */
ID_Sumo_t RTE_GetSumoID(void)
{
	return Get_SumoID();
}

/*================================================================================================*/


#ifdef MASTER_RTE_C_
#undef MASTER_RTE_C_
#endif /* !MASTER_RTE_C_ */
