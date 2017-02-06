/*******************************************************************************
 * @brief 	Driver for the buzzer.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de,  University Kiel
 * @date 	09.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * ==============================================================================
 */

#include "Platform.h"
#include "buz.h"
#include "buz_cfg.h"
#include "BUZ1.h"
#include "TRG1.h"
#include "UTIL1.h"
#include "CLS1.h"

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



static uint8_t BUZ_PrintHelp(const CLS1_StdIOType *io) {
  CLS1_SendHelpStr((unsigned char*)"buzzer", (unsigned char*)"Group of buzzer commands\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows radio help or status\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  buz <freq> <time>", (unsigned char*)"Beep for time (ms) and frequency (kHz)\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  play tune", (unsigned char*)"Play tune\r\n", io->stdOut);
  return ERR_OK;
}

static uint8_t BUZ_PrintStatus(const CLS1_StdIOType *io) {
  (void)io; /* not used */
  return ERR_OK;
}

uint8_t BUZ_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
	const unsigned char *p;
	uint16_t freq, duration;

	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"buzzer help")==0) {
		*handled = TRUE;
		return BUZ_PrintHelp(io);
	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"buzzer status")==0) {
		*handled = TRUE;
		return BUZ_PrintStatus(io);
	} else if (UTIL1_strncmp((char*)cmd, (char*)"buzzer buz ", sizeof("buzzer buz ")-1)==0) {
		*handled = TRUE;
		p = cmd+sizeof("buzzer buz ")-1;
		if (UTIL1_ScanDecimal16uNumber(&p, &freq)==ERR_OK && UTIL1_ScanDecimal16uNumber(&p, &duration)==ERR_OK) {
			if (BUZ_Beep(freq, duration)!=ERR_OK) {
				CLS1_SendStr((unsigned char*)"Starting buzzer failed\r\n", io->stdErr);
				return ERR_FAILED;
			}
			return ERR_OK;
		}
	} else if (UTIL1_strcmp((char*)cmd, (char*)"buzzer play tune")==0) {
		*handled = TRUE;
		return BUZ_PlayTune(BUZ_TUNE_WELCOME);
	}
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

