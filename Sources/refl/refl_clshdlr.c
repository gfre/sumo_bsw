/***********************************************************************************************//**
 * @file		refl_clshdlr.c
 * @ingroup		refl
 * @brief 		Implementation of the command line shell handler for the SWC @a RNet
 *
 * This module implements the interface of the SWC @ref refl which is addressed to
 * the SWC @ref sh. It introduces application specific commands for requests
 * of status information and calibration via command line shell (@b CLS).
 *
 * @author 	Simon Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#define MASTER_refl_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "refl_clshdlr.h"
#include "refl_api.h"
/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t PrintHelp(const CLS1_StdIOType *io);
static unsigned char*REFL_GetStateString(void);
static uint8_t PrintStatus(const CLS1_StdIOType *io) ;
static unsigned char *REFL_LineKindStr(REFL_LineKind line);

/*=================================== >> GLOBAL VARIABLES << =====================================*/

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8_t PrintHelp(const CLS1_StdIOType *io) {
  CLS1_SendHelpStr((unsigned char*)"refl", (unsigned char*)"Group of Reflectance commands\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Print help or status information\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  (on|off)", (unsigned char*)"Enables or disables the reflectance measurement\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  calib (start|stop)", (unsigned char*)"Start/Stop calibrating while moving sensor over line\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  led (on|off)", (unsigned char*)"Uses LED or not\r\n", io->stdOut);
  return ERR_OK;
}


static uint8_t PrintStatus(const CLS1_StdIOType *io) {
  unsigned char buf[32];
  uint8 i = 0u;

  CLS1_SendStatusStr((unsigned char*)"reflectance", (unsigned char*)"\r\n", io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  enabled", (REFL_IsReflEnabled())?(unsigned char*)"yes\r\n":(unsigned char*)"no\r\n", io->stdOut);
#if REFL_USE_WHITE_LINE
  CLS1_SendStatusStr((unsigned char*)"  line", (unsigned char*)"white\r\n", io->stdOut);
#else
  CLS1_SendStatusStr((unsigned char*)"  line", (unsigned char*)"black\r\n", io->stdOut);
#endif
  CLS1_SendStatusStr((unsigned char*)"  state", REFL_GetStateString(), io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  CLS1_SendStatusStr((unsigned char*)"  IR led on", (REFL_IsLedOn())?(unsigned char*)"yes\r\n":(unsigned char*)"no\r\n", io->stdOut);

  UTIL1_strcpy(buf, sizeof(buf), (unsigned char*)"0x");
  UTIL1_strcatNum16Hex(buf, sizeof(buf), REFL_MIN_NOISE_VAL);
  UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
  CLS1_SendStatusStr((unsigned char*)"  min noise", buf, io->stdOut);

  UTIL1_strcpy(buf, sizeof(buf), (unsigned char*)"0x");
  UTIL1_strcatNum16Hex(buf, sizeof(buf), REFL_MIN_LINE_VAL);
  UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
  CLS1_SendStatusStr((unsigned char*)"  min line", buf, io->stdOut);

  UTIL1_Num16uToStr(buf, sizeof(buf), REFL_SENSOR_TIMEOUT_US);
  UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" us, 0x");
  UTIL1_strcatNum16Hex(buf, sizeof(buf), REFL_TIMEOUT_TICKS);
  UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" ticks\r\n");
  CLS1_SendStatusStr((unsigned char*)"  timeout", buf, io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  raw val", (unsigned char*)"", io->stdOut);
  for (i=0;i<REFL_NOF_SENSORS;i++)
  {
    if (i==0)
    {
      CLS1_SendStr((unsigned char*)"0x", io->stdOut);
    } else
      {
         CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
      }
    buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), REFL_GetRawSensorValue(i));
    CLS1_SendStr(buf, io->stdOut);
  }
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

  if (REFL_GetCalibMinMaxPtr()!=NULL) /* have calibration data */
  {
    CLS1_SendStatusStr((unsigned char*)"  min val", (unsigned char*)"", io->stdOut);

    for (i=0;i<REFL_NOF_SENSORS;i++)
    {
      if (i==0)
      {
        CLS1_SendStr((unsigned char*)"0x", io->stdOut);
      } else
      {
        CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
      }
      buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), (REFL_GetCalibMinMaxPtr())->minVal[i]);
      CLS1_SendStr(buf, io->stdOut);
    }
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  }
  if ((REFL_GetCalibMinMaxPtr())!=NULL)
  {
    CLS1_SendStatusStr((unsigned char*)"  max val", (unsigned char*)"", io->stdOut);
    for (i=0;i<REFL_NOF_SENSORS;i++)
    {
      if (i==0)
      {
        CLS1_SendStr((unsigned char*)"0x", io->stdOut);
      } else
      {
        CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
      }
      buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), (REFL_GetCalibMinMaxPtr())->maxVal[i]);
      CLS1_SendStr(buf, io->stdOut);
    }
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  }

  if ((REFL_GetCalibMinMaxPtr())!=NULL) /* have calibration data */
  {
    CLS1_SendStatusStr((unsigned char*)"  calib val", (unsigned char*)"", io->stdOut);

    for (i=0;i<REFL_NOF_SENSORS;i++)
    {
      if (i==0)
      {
        CLS1_SendStr((unsigned char*)"0x", io->stdOut);
      } else
      {
        CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
      }
      buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), REFL_GetCalibratedSensorValue(i));
      CLS1_SendStr(buf, io->stdOut);
    }
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  }

  CLS1_SendStatusStr((unsigned char*)"  line pos", (unsigned char*)"", io->stdOut);
  buf[0] = '\0'; UTIL1_strcatNum16s(buf, sizeof(buf), REFL_GetReflLineValue());
  CLS1_SendStr(buf, io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  line width", (unsigned char*)"", io->stdOut);
  buf[0] = '\0'; UTIL1_strcatNum16s(buf, sizeof(buf), REFL_GetReflLineWidth());
  CLS1_SendStr(buf, io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  line kind", REFL_LineKindStr(REFL_GetReflLineKind()), io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  return ERR_OK;
}

static unsigned char*REFL_GetStateString(void) {
  switch (REFL_GetReflState())
  {
    case REFL_STATE_INIT:                return (unsigned char*)"INIT";
    case REFL_STATE_NOT_CALIBRATED:      return (unsigned char*)"NOT CALIBRATED";
    case REFL_STATE_START_CALIBRATION:   return (unsigned char*)"START CALIBRATION";
    case REFL_STATE_CALIBRATING:         return (unsigned char*)"CALIBRATING";
    case REFL_STATE_STOP_CALIBRATION:    return (unsigned char*)"STOP CALIBRATION";
    case REFL_STATE_SAVE_CALIBRATION:    return (unsigned char*)"SAVE CALIBRATION";
    case REFL_STATE_READY:               return (unsigned char*)"READY";
    default:
      break;
  } /* switch */
  return (unsigned char*)"UNKNOWN";
}

static unsigned char *REFL_LineKindStr(REFL_LineKind line) {
  switch(line)
  {
  case REFL_LINE_NONE:
    return (unsigned char *)"NONE";
  case REFL_LINE_STRAIGHT:
    return (unsigned char *)"STRAIGHT";
  case REFL_LINE_LEFT:
    return (unsigned char *)"LEFT";
  case REFL_LINE_RIGHT:
    return (unsigned char *)"RIGHT";
  case REFL_LINE_FULL:
    return (unsigned char *)"FULL";
  case REFL_LINE_AIR:
    return (unsigned char *)"AIR";
  default:
    return (unsigned char *)"unknown";
  } /* switch */
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
byte REFL_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
  if (UTIL1_strcmp((char*)cmd, CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, "refl help")==0)
  {
    *handled = TRUE;
    return PrintHelp(io);
  } else if ((UTIL1_strcmp((char*)cmd, CLS1_CMD_STATUS)==0) || (UTIL1_strcmp((char*)cmd, "refl status")==0))
  {
    *handled = TRUE;
    return PrintStatus(io);
  } else if (UTIL1_strcmp((char*)cmd, "refl on")==0)
  {
    REFL_SetReflEnabled(TRUE);
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "refl off")==0)
  {
    REFL_SetReflEnabled(FALSE);
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "refl calib start")==0)
  {
    if (REFL_GetReflState()==REFL_STATE_NOT_CALIBRATED || REFL_GetReflState()==REFL_STATE_READY)
    {
    	REFL_CalibrateStartStop();
    } else {
      CLS1_SendStr((unsigned char*)"ERROR: cannot start calibration, must not be calibrating or be ready.\r\n", io->stdErr);
      return ERR_FAILED;
    }
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "refl calib stop")==0)
  {
    if (REFL_GetReflState()==REFL_STATE_CALIBRATING)
    {
    	REFL_CalibrateStartStop();
    } else
    {
      CLS1_SendStr((unsigned char*)"ERROR: can only stop if calibrating.\r\n", io->stdErr);
      return ERR_FAILED;
    }
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "refl led on")==0)
  {
    REFL_SetLedOn(TRUE);
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "refl led off")==0)
  {
    REFL_SetLedOn(FALSE);
    *handled = TRUE;
    return ERR_OK;
  }
  return ERR_OK;
}

#ifdef MASTER_refl_clshdlr_C_
#undef MASTER_refl_clshdlr_C_
#endif /* !MASTER_refl_clshdlr_C_ */
