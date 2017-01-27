/*******************************************************************************
 * @brief 	This is the interface entrance layer for students.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date	 	10.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 *
 * ==============================================================================
 */

#define MASTER_RTE_C_
#include "LED1.h"
#include "LED2.h"
#include "KEY1.h"
#include "Buzzer.h"
#include "rte.h"
#include "tacho.h"
#include "drive.h"
#include "RApp.h"
#include "rnet.h"

#define USER_SWITCH_MASK (0x01u)

/**
 * Interface implementation for the right LED
 */
StdRtn_t RTE_Write_LedRiOn()
{
	LED1_On();
	return RTN_OK;
}

StdRtn_t RTE_Write_LedRiOff()
{
	LED1_Off();
	return RTN_OK;
}

StdRtn_t RTE_Write_LedRiNeg()
{
	LED1_Neg();
	return RTN_OK;
}

StdRtn_t RTE_Write_LedRiSt(uint8_t state)
{
	if(0u==state)
	{
		LED1_Put(FALSE);
	}
	else
	{
		LED1_Put(TRUE);
	}
	return RTN_OK;
}

StdRtn_t RTE_Read_LedRiSt(uint8 *state_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL!=state_)
	{
		*state_ = (uint8)LED1_Get();
		retVal = RTN_OK;
	}
	return retVal;
}
/*========================================================*/


/**
 * Interface implementation for the right LED
 */
StdRtn_t RTE_Write_LedLeOn()
{
	LED2_On();
	return RTN_OK;
}

StdRtn_t RTE_Write_LedLeOff()
{
	LED2_Off();
	return RTN_OK;
}

StdRtn_t RTE_Write_LedLeNeg()
{
	LED2_Neg();
	return RTN_OK;
}

StdRtn_t RTE_Write_LedLeSt(uint8 state_)
{
	if(0u==state_)
	{
		LED2_Put(FALSE);
	}
	else
	{
		LED2_Put(TRUE);
	}
	return RTN_OK;
}

StdRtn_t RTE_Read_LedLeSt(uint8 *state_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL!=state_)
	{
		*state_ = (uint8)LED2_Get();
		retVal = RTN_OK;
	}
	return retVal;
}
/*========================================================*/


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

StdRtn_t RTE_Read_SwtSt(uint8 *state_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL!=state_)
	{
		*state_ = (uint8)KEY1_GetKeys() & USER_SWITCH_MASK;
		retVal = RTN_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_SwtOnPrsdCbFct(const EvntCbFct_t *cbFct_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL != cbFct_)
	{
		cbFctTab.cbFctOnPrsd = cbFct_;
		retVal = RTN_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_SwtOnLngPrsdCbFct(const EvntCbFct_t *cbFct_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL != cbFct_)
	{
		cbFctTab.cbFctOnLngPrsd = cbFct_;
		retVal = RTN_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_SwtOnRlsdCbFct(const EvntCbFct_t *cbFct_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL != cbFct_)
	{
		cbFctTab.cbFctOnRlsd = cbFct_;
		retVal = RTN_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_SwtOnLngRlsdCbFct(const EvntCbFct_t *cbFct_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL != cbFct_)
	{
		cbFctTab.cbFctOnLngRlsd = cbFct_;
		retVal = RTN_OK;
	}
	return retVal;
}

EvntCbFct_t *RTE_Get_SwtOnPrsdCbFct(void)
{
	return cbFctTab.cbFctOnPrsd;
}

EvntCbFct_t *RTE_Get_SwtOnLngPrsdCbFct(void)
{
	return cbFctTab.cbFctOnLngPrsd;
}

EvntCbFct_t *RTE_Get_SwtOnRlsdCbFct(void)
{
	return cbFctTab.cbFctOnRlsd;
}

EvntCbFct_t *RTE_Get_SwtOnLngRlsdCbFct(void)
{
	return cbFctTab.cbFctOnLngRlsd;
}
/*========================================================*/


/**
 * Interface implementation for the buzzer
 */
static inline BUZ_Tunes Trsnlte_TuneRTE2BUZ(RTE_BuzTune_t tune_);


static inline BUZ_Tunes Trsnlte_TuneRTE2BUZ(RTE_BuzTune_t mode_)
{
	switch(mode_)
	{
		case RTE_BUZ_TUNE_WELCOME:     return BUZ_TUNE_WELCOME;
		case RTE_BUZ_TUNE_BUTTON:      return BUZ_TUNE_BUTTON;
		case RTE_BUZ_TUNE_BUTTON_LONG: return BUZ_TUNE_BUTTON_LONG;
		default:
		case RTE_BUZ_TUNE_NOF_TUNES:   return BUZ_TUNE_NOF_TUNES;
	}
	return BUZ_TUNE_NOF_TUNES;
}

StdRtn_t RTE_Write_BuzPlayTune(RTE_BuzTune_t tune_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(BUZ_TUNE_NOF_TUNES > tune_)
	{
		retVal &= (StdRtn_t)BUZ_PlayTune(Trsnlte_TuneRTE2BUZ(tune_));
	}
	return retVal;
}


StdRtn_t RTE_Play_BuzBeep(uint16 freqHz_, uint16 durMs_)
{
	return (StdRtn_t)BUZ_Beep(freqHz_, durMs_);
}
/*========================================================*/


/**
 * Interface implementation for the speedometer
 */
#define LEFT   (TRUE)
#define RIGHT  (FALSE)

StdRtn_t RTE_Read_SpdoVelLe(uint16 *vel_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL != vel_)
	{
		*vel_ = TACHO_GetSpeed(TRUE);
		retVal = RTN_INVALID;
	}
	return retVal;
}


StdRtn_t RTE_Read_SpdoVelRi(uint16 *vel_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL != vel_)
	{
		*vel_ = TACHO_GetSpeed(FALSE);
		retVal = RTN_INVALID;
	}
	return retVal;
}
/*========================================================*/


/**
 * Interface implementation for the drive component
 */
static inline DRV_Mode Trsnlte_ModeRTE2DRV(RTE_DrvMode_t mode_);
static inline RTE_DrvMode_t Trsnlte_ModeDRV2RTE(DRV_Mode mode_);

static inline DRV_Mode Trsnlte_ModeRTE2DRV(RTE_DrvMode_t mode_)
{
	switch(mode_)
	{
		default:
		case RTE_DRV_MODE_NONE:  return DRV_MODE_NONE;
		case RTE_DRV_MODE_STOP:  return DRV_MODE_STOP;
		case RTE_DRV_MODE_SPEED: return DRV_MODE_SPEED;
		case RTE_DRV_MODE_POS:   return DRV_MODE_POS;
	}
	return DRV_MODE_NONE;
}

static inline RTE_DrvMode_t Trsnlte_ModeDRV2RTE(DRV_Mode mode_)
{
	switch(mode_)
	{
		case DRV_MODE_NONE:  return RTE_DRV_MODE_NONE;
		case DRV_MODE_STOP:  return RTE_DRV_MODE_STOP;
		case DRV_MODE_SPEED: return RTE_DRV_MODE_SPEED;
		case DRV_MODE_POS:   return RTE_DRV_MODE_POS;
		default:             return RTE_DRV_MODE_INVALID;
	}
	return RTE_DRV_MODE_INVALID;
}

StdRtn_t RTE_Write_DrvVel(int32 velLe_, int32 velRi_)
{
	return (StdRtn_t)DRV_SetSpeed(velLe_, velRi_);
}

StdRtn_t RTE_Write_DrvPos(int32 posLe_, int32 posRi_)
{
	return (StdRtn_t)DRV_SetSpeed(posLe_, posRi_);
}

StdRtn_t RTE_Write_DrvMode(RTE_DrvMode_t mode_)
{
	StdRtn_t retVal = RTN_INVALID;
	if((RTE_DRV_MODE_INVALID > mode_) && (RTE_DRV_MODE_NONE <= mode_))
	{
		retVal &= (StdRtn_t)DRV_SetMode(Trsnlte_ModeRTE2DRV(mode_));
	}
	return retVal;
}

StdRtn_t RTE_Read_DrvMode(RTE_DrvMode_t *mode_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL != mode_)
	{
		*mode_ = Trsnlte_ModeDRV2RTE(DRV_GetMode());
		retVal = RTN_OK;
	}
	return retVal;
}

StdRtn_t RTE_Read_DrvIsDrvgBkwd(uint8 *isDrvgBkwd_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL != isDrvgBkwd_)
	{
		*isDrvgBkwd_ = DRV_IsDrivingBackward();
		retVal = RTN_OK;
	}
	return retVal;
}

StdRtn_t RTE_Read_DrvHasStpd(uint8 *hasStpd_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL != hasStpd_)
	{
		*hasStpd_ =  DRV_IsStopped();
		retVal = RTN_OK;
	}
	return retVal;
}

StdRtn_t RTE_Read_DrvHasRvsd(uint8 *hasRvsd_)
{
	StdRtn_t retVal = RTN_INVALID;
	if(NULL != hasRvsd_)
	{
		*hasRvsd_ =  DRV_HasTurned();
		retVal = RTN_OK;
	}
	return retVal;
}
/*========================================================*/


/**
 * Interface implementation for the radio application layer
 */
static const RTE_RFRxMsgCbFct_t *RFRxMsgCbFct = NULL;

StdRtn_t RTE_Write_RFSendDataBlk(const uint8 *payload_, uint8 payloadSize_, RTE_RFMsgType_t msgType_,  uint8 dstAddr_, uint8 flags_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != payload_)
	{
		retVal = (StdRtn_t)RAPP_SendPayloadDataBlock((uint8_t *)payload_, (uint8_t)payloadSize_, (uint8_t)msgType_,  (RAPP_ShortAddrType)dstAddr_,  (RAPP_FlagsType)flags_);
	}
	return retVal;
}

StdRtn_t RTE_Write_RFRxMsgCbFctTbl(const RTE_RFRxMsgCbFct_t *cbFct_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != cbFct_)
	{
		RFRxMsgCbFct = cbFct_;
	}
	return retVal;
}

const RTE_RFRxMsgCbFct_t *RTE_Get_RFRxMsgCbFct(void)
{
	return RFRxMsgCbFct;
}

StdRtn_t RTE_Read_RFSniffPkt(RTE_RFPktDes_t *pkt_, uint8 isTx_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	RAPP_PacketDesc pkt={0u};
	if((NULL != pkt_) && (NULL != pkt_->data))
	{
		isTx_ &= TRUE;
		pkt.flags  = (RPHY_FlagsType)pkt_->flags;
		pkt.phySize = (uint8_t)pkt_->size;
		pkt.phyData = (uint8_t *)pkt_->data;
		pkt.rxtx    = (uint8_t *)pkt_->rxtx;
		RAPP_SniffPacket(&pkt, (bool)isTx_);
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Read_RFSrcAddr(uint8 *addr_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != addr_)
	{
		*addr_ = (uint8)RAPP_GetThisNodeAddr();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_RFSrcAddr(uint8 addr_)
{
	return (StdRtn_t)RAPP_SetThisNodeAddr((RAPP_ShortAddrType)addr_);
}

StdRtn_t RTE_Read_RFDstAddr(uint8 *addr_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != addr_)
	{
		*addr_ = (uint8)RNET_GetDstAddr();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t RTE_Write_RFDstAddr(uint8 addr_)
{
	RNET_SetDstAddr((RAPP_ShortAddrType)addr_);
	return ERR_OK;
}
/*================================================================================================*/

#ifdef MASTER_RTE_C_
#undef MASTER_RTE_C_
#endif
