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
static unsigned char*REF_GetStateString(void);
static uint8_t PrintStatus(const CLS1_StdIOType *io) ;
static unsigned char *REF_LineKindStr(REF_LineKind line);

/*=================================== >> GLOBAL VARIABLES << =====================================*/

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8_t PrintHelp(const CLS1_StdIOType *io) {
  CLS1_SendHelpStr((unsigned char*)"ref", (unsigned char*)"Group of Reflectance commands\r\n", io->stdOut);
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

  CLS1_SendStatusStr((unsigned char*)"  enabled", (REF_IsRefEnabled())?(unsigned char*)"yes\r\n":(unsigned char*)"no\r\n", io->stdOut);
#if REF_USE_WHITE_LINE
  CLS1_SendStatusStr((unsigned char*)"  line", (unsigned char*)"white\r\n", io->stdOut);
#else
  CLS1_SendStatusStr((unsigned char*)"  line", (unsigned char*)"black\r\n", io->stdOut);
#endif
  CLS1_SendStatusStr((unsigned char*)"  state", REF_GetStateString(), io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  CLS1_SendStatusStr((unsigned char*)"  IR led on", (REF_IsLedOn())?(unsigned char*)"yes\r\n":(unsigned char*)"no\r\n", io->stdOut);

  UTIL1_strcpy(buf, sizeof(buf), (unsigned char*)"0x");
  UTIL1_strcatNum16Hex(buf, sizeof(buf), REF_MIN_NOISE_VAL);
  UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
  CLS1_SendStatusStr((unsigned char*)"  min noise", buf, io->stdOut);

  UTIL1_strcpy(buf, sizeof(buf), (unsigned char*)"0x");
  UTIL1_strcatNum16Hex(buf, sizeof(buf), REF_MIN_LINE_VAL);
  UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
  CLS1_SendStatusStr((unsigned char*)"  min line", buf, io->stdOut);

  UTIL1_Num16uToStr(buf, sizeof(buf), REF_SENSOR_TIMEOUT_US);
  UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" us, 0x");
  UTIL1_strcatNum16Hex(buf, sizeof(buf), REF_TIMEOUT_TICKS);
  UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" ticks\r\n");
  CLS1_SendStatusStr((unsigned char*)"  timeout", buf, io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  raw val", (unsigned char*)"", io->stdOut);
  for (i=0;i<REF_NOF_SENSORS;i++)
  {
    if (i==0)
    {
      CLS1_SendStr((unsigned char*)"0x", io->stdOut);
    } else
      {
         CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
      }
    buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), REF_GetRawSensorValue(i));
    CLS1_SendStr(buf, io->stdOut);
  }
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

  if (REF_GetCalibMinMaxPtr()!=NULL) /* have calibration data */
  {
    CLS1_SendStatusStr((unsigned char*)"  min val", (unsigned char*)"", io->stdOut);

    for (i=0;i<REF_NOF_SENSORS;i++)
    {
      if (i==0)
      {
        CLS1_SendStr((unsigned char*)"0x", io->stdOut);
      } else
      {
        CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
      }
      buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), (REF_GetCalibMinMaxPtr())->minVal[i]);
      CLS1_SendStr(buf, io->stdOut);
    }
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  }
  if ((REF_GetCalibMinMaxPtr())!=NULL)
  {
    CLS1_SendStatusStr((unsigned char*)"  max val", (unsigned char*)"", io->stdOut);
    for (i=0;i<REF_NOF_SENSORS;i++)
    {
      if (i==0)
      {
        CLS1_SendStr((unsigned char*)"0x", io->stdOut);
      } else
      {
        CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
      }
      buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), (REF_GetCalibMinMaxPtr())->maxVal[i]);
      CLS1_SendStr(buf, io->stdOut);
    }
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  }

  if ((REF_GetCalibMinMaxPtr())!=NULL) /* have calibration data */
  {
    CLS1_SendStatusStr((unsigned char*)"  calib val", (unsigned char*)"", io->stdOut);

    for (i=0;i<REF_NOF_SENSORS;i++)
    {
      if (i==0)
      {
        CLS1_SendStr((unsigned char*)"0x", io->stdOut);
      } else
      {
        CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
      }
      buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), REF_GetCalibratedSensorValue(i));
      CLS1_SendStr(buf, io->stdOut);
    }
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  }

  CLS1_SendStatusStr((unsigned char*)"  line pos", (unsigned char*)"", io->stdOut);
  buf[0] = '\0'; UTIL1_strcatNum16s(buf, sizeof(buf), REF_GetRefLineValue());
  CLS1_SendStr(buf, io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  line width", (unsigned char*)"", io->stdOut);
  buf[0] = '\0'; UTIL1_strcatNum16s(buf, sizeof(buf), REF_GetRefLineWidth());
  CLS1_SendStr(buf, io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  line kind", REF_LineKindStr(REF_GetRefLineKind()), io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  return ERR_OK;
}

static unsigned char*REF_GetStateString(void) {
  switch (REF_GetRefState())
  {
    case REF_STATE_INIT:                return (unsigned char*)"INIT";
    case REF_STATE_NOT_CALIBRATED:      return (unsigned char*)"NOT CALIBRATED";
    case REF_STATE_START_CALIBRATION:   return (unsigned char*)"START CALIBRATION";
    case REF_STATE_CALIBRATING:         return (unsigned char*)"CALIBRATING";
    case REF_STATE_STOP_CALIBRATION:    return (unsigned char*)"STOP CALIBRATION";
    case REF_STATE_SAVE_CALIBRATION:    return (unsigned char*)"SAVE CALIBRATION";
    case REF_STATE_READY:               return (unsigned char*)"READY";
    default:
      break;
  } /* switch */
  return (unsigned char*)"UNKNOWN";
}

static unsigned char *REF_LineKindStr(REF_LineKind line) {
  switch(line)
  {
  case REF_LINE_NONE:
    return (unsigned char *)"NONE";
  case REF_LINE_STRAIGHT:
    return (unsigned char *)"STRAIGHT";
  case REF_LINE_LEFT:
    return (unsigned char *)"LEFT";
  case REF_LINE_RIGHT:
    return (unsigned char *)"RIGHT";
  case REF_LINE_FULL:
    return (unsigned char *)"FULL";
  case REF_LINE_AIR:
    return (unsigned char *)"AIR";
  default:
    return (unsigned char *)"unknown";
  } /* switch */
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
byte REF_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
  if (UTIL1_strcmp((char*)cmd, CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, "ref help")==0)
  {
    *handled = TRUE;
    return PrintHelp(io);
  } else if ((UTIL1_strcmp((char*)cmd, CLS1_CMD_STATUS)==0) || (UTIL1_strcmp((char*)cmd, "ref status")==0))
  {
    *handled = TRUE;
    return PrintStatus(io);
  } else if (UTIL1_strcmp((char*)cmd, "ref on")==0)
  {
    REF_SetRefEnabled(TRUE);
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "ref off")==0)
  {
    REF_SetRefEnabled(FALSE);
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "ref calib start")==0)
  {
    if (REF_GetRefState()==REF_STATE_NOT_CALIBRATED || REF_GetRefState()==REF_STATE_READY)
    {
    	REF_CalibrateStartStop();
    } else {
      CLS1_SendStr((unsigned char*)"ERROR: cannot start calibration, must not be calibrating or be ready.\r\n", io->stdErr);
      return ERR_FAILED;
    }
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "ref calib stop")==0)
  {
    if (REF_GetRefState()==REF_STATE_CALIBRATING)
    {
    	REF_CalibrateStartStop();
    } else
    {
      CLS1_SendStr((unsigned char*)"ERROR: can only stop if calibrating.\r\n", io->stdErr);
      return ERR_FAILED;
    }
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "ref led on")==0)
  {
    REF_SetLedOn(TRUE);
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "ref led off")==0)
  {
    REF_SetLedOn(FALSE);
    *handled = TRUE;
    return ERR_OK;
  }
  return ERR_OK;
}

#ifdef MASTER_refl_clshdlr_C_
#undef MASTER_refl_clshdlr_C_
#endif /* !MASTER_refl_clshdlr_C_ */
