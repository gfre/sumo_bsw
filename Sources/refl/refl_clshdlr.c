
#include "refl_clshdlr.h"
#include "refl_api.h"

#define MASTER_refl_clshdlr_C_

static uint8_t PrintHelp(const CLS1_StdIOType *io) {
  CLS1_SendHelpStr((unsigned char*)"ref", (unsigned char*)"Group of Reflectance commands\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Print help or status information\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  (on|off)", (unsigned char*)"Enables or disables the reflectance measurement\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  calib (start|stop)", (unsigned char*)"Start/Stop calibrating while moving sensor over line\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  led (on|off)", (unsigned char*)"Uses LED or not\r\n", io->stdOut);
  return ERR_OK;
}

static unsigned char*REF_GetStateString(void) {
  switch (refState) {
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

static uint8_t PrintStatus(const CLS1_StdIOType *io) {
  unsigned char buf[32];
  int i;

  CLS1_SendStatusStr((unsigned char*)"reflectance", (unsigned char*)"\r\n", io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  enabled", REF_IsEnabled?(unsigned char*)"yes\r\n":(unsigned char*)"no\r\n", io->stdOut);
#if REF_USE_WHITE_LINE
  CLS1_SendStatusStr((unsigned char*)"  line", (unsigned char*)"white\r\n", io->stdOut);
#else
  CLS1_SendStatusStr((unsigned char*)"  line", (unsigned char*)"black\r\n", io->stdOut);
#endif
  CLS1_SendStatusStr((unsigned char*)"  state", REF_GetStateString(), io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  CLS1_SendStatusStr((unsigned char*)"  IR led on", REF_LedOn?(unsigned char*)"yes\r\n":(unsigned char*)"no\r\n", io->stdOut);

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
#if REF_SENSOR1_IS_LEFT
  for (i=0;i<REF_NOF_SENSORS;i++) {
    if (i==0) {
#else
  for (i=REF_NOF_SENSORS-1;i>=0;i--) {
    if (i==REF_NOF_SENSORS-1) {
#endif
      CLS1_SendStr((unsigned char*)"0x", io->stdOut);
    } else {
      CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
    }
    buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), SensorRaw[i]);
    CLS1_SendStr(buf, io->stdOut);
  }
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

  if (SensorCalibMinMaxPtr!=NULL) { /* have calibration data */
    CLS1_SendStatusStr((unsigned char*)"  min val", (unsigned char*)"", io->stdOut);
  #if REF_SENSOR1_IS_LEFT
    for (i=0;i<REF_NOF_SENSORS;i++) {
      if (i==0) {
  #else
    for (i=REF_NOF_SENSORS-1;i>=0;i--) {
      if (i==REF_NOF_SENSORS-1) {
  #endif
        CLS1_SendStr((unsigned char*)"0x", io->stdOut);
      } else {
        CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
      }
      buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), SensorCalibMinMaxPtr->minVal[i]);
      CLS1_SendStr(buf, io->stdOut);
    }
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  }
  if (SensorCalibMinMaxPtr!=NULL) {
    CLS1_SendStatusStr((unsigned char*)"  max val", (unsigned char*)"", io->stdOut);
  #if REF_SENSOR1_IS_LEFT
    for (i=0;i<REF_NOF_SENSORS;i++) {
      if (i==0) {
  #else
    for (i=REF_NOF_SENSORS-1;i>=0;i--) {
      if (i==REF_NOF_SENSORS-1) {
  #endif
        CLS1_SendStr((unsigned char*)"0x", io->stdOut);
      } else {
        CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
      }
      buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), SensorCalibMinMaxPtr->maxVal[i]);
      CLS1_SendStr(buf, io->stdOut);
    }
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  }

  if (SensorCalibMinMaxPtr!=NULL) { /* have calibration data */
    CLS1_SendStatusStr((unsigned char*)"  calib val", (unsigned char*)"", io->stdOut);
  #if REF_SENSOR1_IS_LEFT
    for (i=0;i<REF_NOF_SENSORS;i++) {
      if (i==0) {
  #else
    for (i=REF_NOF_SENSORS-1;i>=0;i--) {
      if (i==REF_NOF_SENSORS-1) {
  #endif
        CLS1_SendStr((unsigned char*)"0x", io->stdOut);
      } else {
        CLS1_SendStr((unsigned char*)" 0x", io->stdOut);
      }
      buf[0] = '\0'; UTIL1_strcatNum16Hex(buf, sizeof(buf), SensorCalibrated[i]);
      CLS1_SendStr(buf, io->stdOut);
    }
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  }

  CLS1_SendStatusStr((unsigned char*)"  line pos", (unsigned char*)"", io->stdOut);
  buf[0] = '\0'; UTIL1_strcatNum16s(buf, sizeof(buf), refCenterLineVal);
  CLS1_SendStr(buf, io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  line width", (unsigned char*)"", io->stdOut);
  buf[0] = '\0'; UTIL1_strcatNum16s(buf, sizeof(buf), refLineWidth);
  CLS1_SendStr(buf, io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  line kind", REF_LineKindStr(refLineKind), io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  return ERR_OK;
}

byte REF_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
  if (UTIL1_strcmp((char*)cmd, CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, "ref help")==0) {
    *handled = TRUE;
    return PrintHelp(io);
  } else if ((UTIL1_strcmp((char*)cmd, CLS1_CMD_STATUS)==0) || (UTIL1_strcmp((char*)cmd, "ref status")==0)) {
    *handled = TRUE;
    return PrintStatus(io);
  } else if (UTIL1_strcmp((char*)cmd, "ref on")==0) {
    REF_IsEnabled = TRUE;
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "ref off")==0) {
    REF_IsEnabled = FALSE;
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "ref calib start")==0) {
    if (refState==REF_STATE_NOT_CALIBRATED || refState==REF_STATE_READY) {
      APP_StateStartCalibrate();
    } else {
      CLS1_SendStr((unsigned char*)"ERROR: cannot start calibration, must not be calibrating or be ready.\r\n", io->stdErr);
      return ERR_FAILED;
    }
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "ref calib stop")==0) {
    if (refState==REF_STATE_CALIBRATING) {
      APP_StateStopCalibrate();
    } else {
      CLS1_SendStr((unsigned char*)"ERROR: can only stop if calibrating.\r\n", io->stdErr);
      return ERR_FAILED;
    }
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "ref led on")==0) {
    REF_LedOn = TRUE;
    *handled = TRUE;
    return ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, "ref led off")==0) {
    REF_LedOn = FALSE;
    *handled = TRUE;
    return ERR_OK;
  }
  return ERR_OK;
}
