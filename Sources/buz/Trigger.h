/*******************************************************************************
 * @brief 	Generic trigger interface.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 		09.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 *
 * ==============================================================================
 */

#ifndef TRIGGER_H_
#define TRIGGER_H_

#include "Platform.h"

#define TRG_TICKS_MS  1
  /*!< Defines the period at which TRG_IncTick gets called */

/*! @brief Triggers which can be used from the application */
typedef enum {
 /* TODO Extend the list of triggers as needed */
  TRG_BUZ_BEEP, /*!< Buzzer beep */
  TRG_KEYPRESS, /*!< key debounce */
  TRG_BUZ_TUNE, /*!< buzzer tune */
  TRG_NOF_TRIGGERS /*!< Must be last! */
} TRG_TriggerKind;

/*! @brief Type for the data pointer used by the callback */
typedef void *TRG_CallBackDataPtr;

/*! @brief Function callback prototype, with having an optional parameter pointer */
typedef void (*TRG_Callback)(TRG_CallBackDataPtr);

/*! @brief Type to hold the trigger ticks */
typedef uint16_t TRG_TriggerTime;

/*!
 * @brief Adds a new trigger
 * @param trigger Trigger to be added
 * @param ticks Trigger time in ticks. The time is relative from the current time.
 * @param callback Callback to be called when the trigger fires
 * @param data Optional pointer to data
 * @return error code, ERR_OK if everything is fine
 */
uint8_t TRG_SetTrigger(TRG_TriggerKind trigger, TRG_TriggerTime ticks, TRG_Callback callback, TRG_CallBackDataPtr data);

/*! @brief Called from interrupt service routine with a period of TRG_TICKS_MS. */
void TRG_AddTick(void);

/*! @brief De-initializes the module. */
void TRG_Deinit(void);

/*! @brief Initializes the module. */
void TRG_Init(void);


#endif /* TRIGGER_H_ */
