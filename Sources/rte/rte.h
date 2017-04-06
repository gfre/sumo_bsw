/***********************************************************************************************//**
 * @file		rte.h
 * @ingroup		rte
 * @brief 		Application Interface of the RTE
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

#ifndef RTE_H_
#define RTE_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "rte_Types.h"



#ifdef MASTER_RTE_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup rte
 * @{
 */
/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef void EvntCbFct_t(uint8_t);



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief RTE interface to turn the left LED ON
 * @param *state_ pointer to the LED state (call by reference)
 * @return Error code, always ERR_OK
 */
EXTERNAL_ StdRtn_t RTE_Write_LedLeOn();

/**
 * @brief RTE interface to turn the left LED OFF
 * @return Error code, always ERR_OK
 */
EXTERNAL_ StdRtn_t RTE_Write_LedLeOff();

/**
 * @brief RTE interface to toggle the state the left LED
 * @return Error code, always ERR_OK
 */
EXTERNAL_ StdRtn_t RTE_Write_LedLeNeg();

/**
 * @brief RTE interface to write the state of the left LED
 * @param state_ desired state of the LED
 * @return Error code, always ERR_OK
 */
EXTERNAL_ StdRtn_t RTE_Write_LedLeSt(uint8_t state_);

/**
 * @brief RTE interface to flash the left LED with a certain period
 * @param perMS_ desired flashing period of the left LED in milliseconds
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Write_LedLeFlshWithPerMS(uint16_t perMS_);

/**
 * @brief RTE interface to read the state of the right LED
 * @param *state_ pointer to the LED state (call by reference)
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_LedLeSt(uint8_t *state_);

/**
 * @brief RTE interface to turn the right LED ON
 * @return Error code, always ERR_OK
 */
EXTERNAL_ StdRtn_t RTE_Write_LedRiOn();

/**
 * @brief RTE interface to turn the right LED OFF
 * @return Error code, always ERR_OK
 */
EXTERNAL_ StdRtn_t RTE_Write_LedRiOff();

/**
 * @brief RTE interface to toggle the state the right LED
 * @return Error code, always ERR_OK
 */
EXTERNAL_ StdRtn_t RTE_Write_LedRiNeg();

/**
 * @brief RTE interface to write the state of the right LED
 * @param state_ desired state of the LED
 * @return Error code, always ERR_OK
 */
EXTERNAL_ StdRtn_t RTE_Write_LedRiSt(uint8_t state_);

/**
 * @brief RTE interface to flash the right LED with a certain period
 * @param perMS_ desired flashing period of the right LED in milliseconds
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Write_LedRiFlshWithPerMS(uint16_t perMS_);

/**
 * @brief RTE interface to read the state of the right LED
 * @param *state_ output: pointer to the LED state
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_LedRiSt(uint8_t *state_);

/*================================================================================================*/


/**
 * @brief RTE interface to read the state of the button
 * @param *state_ pointer to the button state (call by reference)
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_BtnSt(uint8_t *state_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the button is pressed shortly
 * @param *cbFct_ pointer to the callback function
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Write_BtnOnPrsdCbFct(const EvntCbFct_t *cbFct_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the button is pressed for a longer time
 * @param *cbFct_ pointer to the callback function
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Write_BtnOnLngPrsdCbFct(const EvntCbFct_t *cbFct_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the button is released after a short press
 * @param *cbFct_ pointer to the callback function
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Write_BtnOnRlsdCbFct(const EvntCbFct_t *cbFct_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the button is released after a long press
 * @param *cbFct_ pointer to the callback function
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Write_BtnOnLngRlsdCbFct(const EvntCbFct_t *cbFct_);

/**
 * @brief RTE interface to get the pointer to a callback function
 * which is called when the button is pressed shortly
 * @return pointer to the callback function
 */
EXTERNAL_ EvntCbFct_t *RTE_Get_BtnOnPrsdCbFct(void);

/**
 * @brief RTE interface to get the pointer to a callback function
 * which is called when the button is pressed for a longer time
 * @return pointer to the callback function
 */
EXTERNAL_ EvntCbFct_t *RTE_Get_BtnOnLngPrsdCbFct(void);

/**
 * @brief RTE interface to get the pointer to a callback function
 * which is called when the button is released after a short press
 * @return pointer to the callback function
 */
EXTERNAL_ EvntCbFct_t *RTE_Get_BtnOnRlsdCbFct(void);

/**
 * @brief RTE interface to get the pointer to a callback function
 * which is called when the button is released after a long press
 * @return pointer to the callback function
 */
EXTERNAL_ EvntCbFct_t *RTE_Get_BtnOnLngRlsdCbFct(void);


/*================================================================================================*/


/**
 * @brief RTE interface to play a buzzer tune
 * @param  tune_ enumeration to select a tune
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_VALUE otherwise
 */
EXTERNAL_ StdRtn_t RTE_Write_BuzPlayTune(BUZ_Tunes_t tune_);

/**
 * @brief RTE interface to play a buzzer beep
 * @param  freqHz_ Frequncy of the Beep in Hertz
 *         durMs_  Duratoin of the Beep in milli seconds
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_BUSY otherwise
 */
EXTERNAL_ StdRtn_t RTE_Write_BuzBeep(uint16_t freqHz_, uint16_t durMs_);


/*================================================================================================*/


/**
 * @brief RTE interface to read the velocity of the left wheels
 * @param  *vel_ pointer to the current velocity in steps/sec (call by reference)
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_SpdoVelLe(uint16_t *vel_);


/**
 * @brief RTE interface to read the velocity of the right wheels
 * @param  *vel_ pointer to the current velocity in steps/sec (call by reference)
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_SpdoVelRi(uint16_t *vel_);


/*================================================================================================*/


/**
 * @brief RTE interface to write the desired velocity in speed mode
 * @param  velLe_ desired velocity of left wheels in steps/sec
 * @param  velRi_ desired velocity of right wheels in steps/sec
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_FAILED otherwise
 */
EXTERNAL_ StdRtn_t RTE_Write_DrvVel(int32_t velLe_, int32_t velRi_);

/**
 * @brief RTE interface to write the target postion in position mode
 * @param  posLe_ desired position of left wheels in steps/sec
 * @param  posRi_ desired position of right wheels in steps/sec
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_FAILED otherwise
 */
EXTERNAL_ StdRtn_t RTE_Write_DrvPos(int32_t posLe_, int32_t posRi_);

/**
 * @brief RTE interface to command the desired driving control mode
 * @param  mode_ desired driving control mode (RTE_DrvMode_t)
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_VALUE for invalid requested mode
 *                     ERR_FAILED for queue overflow
 */
EXTERNAL_ StdRtn_t RTE_Write_DrvMode(DrvMode_t mode_);

/**
 * @brief RTE interface to read the current driving control mode
 * @param  *mode_ pointer to the current driving control mode (RTE_DrvMode_t)
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_DrvMode(DrvMode_t *mode_);

/**
 * @brief RTE interface to read if the sumo is driving backwards
 * @param  *isDrvgBkwd_ pointer to a flag
 *                        TRUE  - driving backward,
 *                        FALSE - driving forward
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_DrvIsDrvgBkwd(uint8_t *isDrvgBkwd_);

/**
 * @brief RTE interface to read if the sumo has stopped
 * @param  *hasStopped_ pointer to a flag
 *                        TRUE  - has stopped,
 *                        FALSE - still driving
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_DrvHasStpd(uint8_t *hasStpd_);

/**
 * @brief RTE interface to read if the sumo has just reversed
 * @param  *hasStopped_ pointer to a flag
 *                        TRUE  - has just reversed,
 *                        FALSE - has not just reversed
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_DrvHasRvsd(uint8_t *hasRvsd_);


/*================================================================================================*/


/**
 * @brief RTE interface to send a data block via RF
 * @param *payLoad_ pointer to the array where the payload is stored
 * @param payLoadSize_ size in number of bytes of the payload
 * @param msgType_ message type of packet
 * @param dstAddr_ destination address
 * @param flags_ configuration flags for the RF stack
 *                     0x00 no flag
 *                     0x01 ACK received
 *                     0x02 ACK requested
 *                     0x04 POWER_DOWN
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS for invalid addresse
 *                     ERR_OVERFLOW for too big payload size
 *                     ERR_BUSY for queue overflow
 */
EXTERNAL_ StdRtn_t RTE_Write_RFSendDataBlk(const uint8_t *payLoad_, uint8_t payLoadSize_, RTE_RF_MSG_TYPE_T msgType_,  uint8_t dstAddr_, uint8_t flags_);

/**
 * @brief RTE interface to set the pointer to the function which is called when a RF message is received
 * @param *cbFct_ pointer to the callback function of type RTE_RFRxMsgCbFct_t
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Write_RFRxMsgCbFct(const RFRxMsgCbFct_t *cbFct_);

/**
 * @brief RTE interface to get the pointer to the function which is called when a RF message is received
 * @return pointer to the callback function of type RTE_RFRxMsgCbFct_t
 */
EXTERNAL_ RFRxMsgCbFct_t *RTE_Get_RFRxMsgCbFct(void);

/**
 * @brief RTE interface to sniff the transmitted or received RF-packets
 * @param *pkt_ pointer to RF-packet descriptor
 * @param isTx_ flag if packet is transmitted or received
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_RFSniffPkt(RFPktDes_t *pkt_, uint8_t isTx_);

/**
 * @brief RTE interface to read the current network address of the source RF-node
 * @param *addr_ pointer to the 1-byte network address
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_RFSrcAddr(uint8_t *addr_);

/**
 * @brief RTE interface to write the network address of the source RF-node
 * @param addr_ the 1-byte network address
 * @return Error code, always ERR_OK
 */
EXTERNAL_ StdRtn_t RTE_Write_RFSrcAddr(uint8_t addr_);

/**
 * @brief RTE interface to read the current network address of the destination RF-node where the message should be sent
 * @param *addr_ pointer to the 1-byte network address
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_Read_RFDstAddr(uint8_t *addr_);

/**
 * @brief RTE interface to write the network address of the destination RF-node where the message should be sent
 * @param addr_ the 1-byte network address
 * @return Error code, always ERR_OK
 */
EXTERNAL_ StdRtn_t RTE_Write_RFDstAddr(const uint8_t addr_);


/*================================================================================================*/
EXTERNAL_ RTE_STREAM *RTE_stderr;
EXTERNAL_ RTE_STREAM *RTE_stdout;

/**
 * @brief RTE interface for printf-similar function, prints formatted byte output to stdout
 * @param fmt_ pointer to a null-terminated multibyte string specifying how to interpret the data.
 * @param ...  arguments specifying data to print
 * @return number of characters transmitted to the output stream
 */
EXTERNAL_ unsigned int RTE_printf(unsigned char *fmt_, ...);

/**
 * @brief RTE interface for fprintf-similar function, prints formatted byte output to an output stream
 * @param stream_  output file stream to write to
 * @param fmt_ pointer to a null-terminated multibyte string specifying how to interpret the data.
 * @param ...  arguments specifying data to print
 * @return number of characters transmitted to the output stream
 */
EXTERNAL_ unsigned int RTE_fprintf(RTE_STREAM *stream_ , unsigned char *fmt_, ...);

/**
 * @brief RTE interface for puts-similar function, writes a byte string to stdout
 * @param msg_ pointer to a null-terminated multibyte string specifying how to interpret the data.
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_puts(const uint8_t *msg_);

/**
 * @brief RTE interface for puts-similar function, writes a byte string to stderr
 * @param msg_ pointer to a null-terminated multibyte string specifying how to interpret the data.
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t RTE_putsErr(const uint8_t *msg_);



/*================================================================================================*/
/**
 * @brief RTE interface function which determines the custom ID of the sumo robot
 * @return custom ID of robot (index-like ID)
 */
EXTERNAL_ ID_Sumo_t RTE_GetSumoID(void);



/*================================================================================================*/
/**
 * @brief This function sets a flag which enables/disables hold on ENTER functionality for
 * NORMAL application state/mode
 * @param  holdOn_ TRUE/FALSE-flag for enabling or disabling holdOn ENTER functionality
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
StdRtn_t RTE_Write_HoldOnEnterNormal(const uint8_t holdOn_);

/**
 * @brief This function sets a flag which enables/disables hold on ENTER functionality for
 * IDLE application state/mode
 * @param  holdOn_ TRUE/FALSE-flag for enabling or disabling holdOn ENTER functionality
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
StdRtn_t RTE_Write_HoldOnEnterIdle(const uint8_t holdOn_);

/**
 * @brief This function allows to release hold on ENTER for NORMAL application state/mode
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
StdRtn_t RTE_Release_HoldOnEnterNormal(void);

/**
 * @brief This function allows to release hold on ENTER for IDLE application state/mode
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
StdRtn_t RTE_Release_HoldOnEnterIdle(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !RTE_H_ */
