/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	06.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file rte_Types.h
 * 
 *==================================================================================================
 */


#ifndef RTE_TYPES_H_
#define RTE_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"


#ifdef MASTER_rte_Types_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
#ifdef ASW_STREAM_T
#define RTE_STREAM             ASW_STREAM_T
#else
#define RTE_STREAM             Stream_t
#endif

#ifdef ASW_RF_MSG_TYPE_T
#define RTE_RF_MSG_TYPE_T      ASW_RF_MSG_TYPE_T
#else
#define RTE_RF_MSG_TYPE_T      RFMsgType_t
#endif

#ifndef NUM_OF_SUMOS
#define NUM_OF_SUMOS    (25)
#endif
#define MAX_ID_OF_SUMOS        (NUM_OF_SUMOS-(1))

/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @brief ASW-customisable data type for application message type for RF messages
 * How to customise:
 * >> #define ASW_STREAM with custom stream type in asw.h
 * >> include asw.h before rte.h
 */
typedef void Stream_t;

/**
 * @brief ASW-customisable data type for application message type for RF messages
 * How to customise:
 * >> #define ASW_RF_MSG_TYPE_T with custom message type in asw.h
 * >> include asw.h before rte.h
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


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Initializes the RTE interface
 */
EXTERNAL_ void RTE_Init(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !RTE_TYPES_H_ */
