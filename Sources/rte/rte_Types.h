/***********************************************************************************************//**
 * @file		rte_Types.h
 * @ingroup		rte
 * @brief 		Customisable and Non-Customisable RTE - Data Types for Application Software
 * 				Development.
 *
 * within the ACon Sumo Robot Project. This header file provides the customisable and non-customisable RTE-
 * data types for the development of hardware-independent application software.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.02.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef RTE_TYPES_H_
#define RTE_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"



#ifdef MASTER_rte_Types_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup rte
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
#ifdef ASW_STREAM_T
#define RTE_STREAM             ASW_STREAM_T
#else
#define RTE_STREAM             Stream_t
#endif

#ifdef ASW_RF_MSG_TYPE_T
#define RTE_RF_MSG_TYPE_T		ASW_RF_MSG_TYPE_T
#else
#define RTE_RF_MSG_TYPE_T		RFMsgType_t
#endif

#if defined( ASW_NVM_UNIT_SIZE )
	#if ASW_NVM_UNIT_SIZE
	#define RTE_NVM_UNIT_SIZE_ASW		ASW_NVM_UNIT_SIZE
	#else
	#undef ASW_NVM_UNIT_SIZE
	#endif
#endif

#ifndef NUM_OF_SUMOS
#define NUM_OF_SUMOS    (25)
#endif

#define MAX_ID_OF_SUMOS        (NUM_OF_SUMOS-(1))

#define DRV_MODE_T DrvMode_t



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @brief ASW-customisable data type for application message type for RF messages.
 *
 * > *How to customise:*\n
 * > 1. define @c ASW_STREAM with custom stream type in asw.h\n
 * > 2. include asw.h before rte.h
 */
typedef void Stream_t;

/**
 * @brief ASW-customisable data type for application message type for RF messages
 * >**How to customise:**
 * >define ASW_RF_MSG_TYPE_T with custom message type in asw.h
 * >include asw.h before rte.h
 */
typedef enum RFMsgType_e {
  MSG_TYPE_TESTDATA = 0xFE,
  MSG_TYPE_INVALID  = 0xFF,
} RFMsgType_t;


/**
 * @brief Non-customizeable data type for packet descriptor of a RF message
 */
typedef struct RFPktDes_e {
  uint8_t flags;
  uint8_t size;
  uint8_t *data;
  uint8_t *rxtx;
} RFPktDes_t;

/**
 * @brief Non-customizeable data type for a pointer to callback function for receiving RF messages
 */
typedef StdRtn_t RFRxMsgCbFct_t(RTE_RF_MSG_TYPE_T type_, uint8_t size_, const uint8_t *data_, uint8_t srcAddr_, uint8_t *handled_, const RFPktDes_t *pktDes_);

/**
 * @brief Non-customizeable data type for buzzer tunes
 */
typedef enum BUZ_Tunes_e {
  BUZ_TUNE_WELCOME = 0//!< BUZ_TUNE_WELCOME
 ,BUZ_TUNE_BUTTON     //!< BUZ_TUNE_BUTTON
 ,BUZ_TUNE_ACCEPT     //!< BUZ_TUNE_ACCEPT
 ,BUZ_TUNE_DECLINE    //!< BUZ_TUNE_DECLINE
 ,BUZ_TUNE_BUTTON_LONG//!< BUZ_TUNE_BUTTON_LONG
 ,BUZ_TUNE_NOF_TUNES  //!< BUZ_TUNE_NOF_TUNES
} BUZ_Tunes_t;

/**
 * @brief Non-customizeable data type for sumo IDs
 */
typedef enum ID_Sumo_e { /* do *not* change order of enumeration, they are used internally for a table index */
	 ID_SUMO_MIN  = 0						/**< min custom ID of sumo */
	,ID_SUMO_MAX  = MAX_ID_OF_SUMOS         /**< max custom ID of sumo */
	,ID_SUMO_UNKNOWN       					/**< unknown robot, unknown custom ID */
	,ID_SUMO_NONE          					/**< initialization value, used internally */
} ID_Sumo_t;

/**
 * @brief Non-customizeable data type for driving modes
 */
typedef enum DrvMode_e {
  DRV_MODE_NONE = 0,
  DRV_MODE_STOP,
  DRV_MODE_SPEED,
  DRV_MODE_POS,
  DRV_MODE_INVALID,
} DrvMode_t;

/**
 * @brief
 */
typedef enum RF_OutpPwr_e {
	 RF_PWR_0dB = 0         //!< RTE_RF_0dB
	,RF_PWR_minus10dB = -10 //!< RTE_RF_minus10dB
	,RF_PWR_minus12dB = -12 //!< RTE_RF_minus12dB
	,RF_PWR_minus18dB = -18 //!< RTE_RF_minus18dB
} RF_OutpPwr_t;

/**
 * @brief
 */
typedef enum RF_DataRate_e {
	 RF_DATA_RATE_250kbps = 250//!< RF_DATA_RATE_250kbps
	,RF_DATA_RATE_1Mbps = 1000 //!< RF_DATA_RATE_1Mbps
	,RF_DATA_RATE_2Mbps = 2000 //!< RF_DATA_RATE_2Mbps
} RF_DataRate_t;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Initializes the RTE interface
 */
EXTERNAL_ void RTE_Init(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !RTE_TYPES_H_ */
