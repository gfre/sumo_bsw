/***********************************************************************************************//**
 * @file		buz.c
 * @ingroup		buz
 * @brief 		Implementation of a driver component for a buzzer.
 *
 * This module implements driver component for a buzzer by means of trigger provided and enhanced
 * by the firmware components @b TRG1 and @b BUZ1.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.01.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#include "Platform.h"
#include "buz.h"
#include "buz_cfg.h"
#include "buz_api.h"
#include "BUZ1.h"
#include "TRG1.h"


#define TRG_TICKS_MS  1

static BUZ_Tunes_t tune;
static volatile BUZ_TrgInfo trgInfo;
static MelodyDesc *BUZ_Melodies = NULL;
static void BUZ_Toggle() {
  if (trgInfo.buzIterationCntr==0) {
	  BUZ1_ClrVal(); /* turn buzzer off */
  } else {
	  trgInfo.buzIterationCntr--;
	  BUZ1_NegVal();
	  (void)TRG1_AddTrigger(TRG1_BUZ_BEEP, trgInfo.buzPeriodTicks, BUZ_Toggle);
  }
}

uint8_t BUZ_Beep(uint16_t freq, uint16_t durationMs) {
  if (trgInfo.buzIterationCntr==0) { /* only if buzzer is not running right now */
	  BUZ1_SetVal(); /* turn buzzer on */
	  trgInfo.buzPeriodTicks = (1000*TRG_TICKS_MS)/freq;
	  trgInfo.buzIterationCntr = durationMs/TRG_TICKS_MS/trgInfo.buzPeriodTicks;
	  TRG1_AddTrigger(TRG1_BUZ_BEEP, trgInfo.buzPeriodTicks, BUZ_Toggle);
	  return ERR_OK;
  } else {
	  return ERR_BUSY;
  }
}

static void BUZ_Play() {
  MelodyDesc *melody = &BUZ_Melodies[tune];

  if (NULL != melody)
  {
      BUZ_Beep(melody->melody[melody->idx].freq, melody->melody[melody->idx].ms);
      melody->idx++;
      if (melody->idx<melody->maxIdx) {
	TRG1_AddTrigger(TRG1_BUZ_TUNE, melody->melody[melody->idx-1].ms/TRG_TICKS_MS, BUZ_Play);
      }
  }
  return;
}

uint8_t BUZ_PlayTune(BUZ_Tunes_t tune_) {
  if (tune_>=BUZ_TUNE_NOF_TUNES) {
	  return ERR_OVERFLOW;
  }
  tune = tune_;
  BUZ_Melodies[tune].idx = 0; /* reset index */
  TRG1_AddTrigger(TRG1_BUZ_TUNE, 0, BUZ_Play);
  return ERR_OK;
}


void BUZ_Deinit(void) {
  /* nothing to do */
}

void BUZ_Init(void) {
  BUZ_Melodies = Get_BUZMelodies();
  BUZ1_SetVal(); /* turn buzzer off */
  trgInfo.buzPeriodTicks = 0;
  trgInfo.buzIterationCntr = 0;
}

