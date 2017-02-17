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
#define RTE_STREAM RTE_Stream_t



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum BUZ_Tunes_e {
  BUZ_TUNE_WELCOME = 0
 ,BUZ_TUNE_BUTTON
 ,BUZ_TUNE_ACCEPT
 ,BUZ_TUNE_DECLINE
 ,BUZ_TUNE_BUTTON_LONG
 ,BUZ_TUNE_NOF_TUNES
} BUZ_Tunes_t;


typedef void * RTE_Stream_t;


/*! type ID's for application messages */
typedef enum RFMsgType_e {
  MSG_TYPE_TESTDATA = 0xFE,
  MSG_TYPE_INVALID = 0xFF,
} RFMsgType_t;

typedef struct RFPktDes_e {
  uint8 flags;
  uint8 size;
  uint8 *data;
  uint8 *rxtx;
} RFPktDes_t;


typedef StdRtn_t RFRxMsgCbFct_t(RFMsgType_t type_, uint8 size_, const uint8 *data_, uint8 srcAddr_, bool *handled_, const RFPktDes_t *pkt_);

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ void RTE_Init(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !RTE_TYPES_H_ */
