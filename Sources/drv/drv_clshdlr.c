/***********************************************************************************************//**
 * @file		drv_clshdlr.c
 * @ingroup		drv
 * @brief 		Implementation of the command line shell handler for the SWC @a Drive
 *
 * This module implements the interface of the SWC @ref drv which is addressed to the SWC @ref sh.
 * It introduces application specific commands for requesting drive related help and status
 * information via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.04.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_drv_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "drv_clshdlr.h"
#include "drv_api.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t *DRV_GetModeStr(DRV_Mode_t mode);



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8_t *DRV_GetModeStr(DRV_Mode_t mode) {
	switch(mode) {
	case DRV_MODE_NONE:   return (uint8_t*)"NONE";
	case DRV_MODE_STOP:   return (uint8_t*)"STOP";
	case DRV_MODE_SPEED:  return (uint8_t*)"SPEED";
	case DRV_MODE_POS:    return (uint8_t*)"POS";
	default: return (uint8_t*)"UNKNOWN";
	}
}

static void DRV_PrintHelp(const CLS1_StdIOType *io_) {
	CLS1_SendHelpStr((unsigned char*)"drive", (unsigned char*)"Group of drive commands\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows drive help or status\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  mode <mode>", (unsigned char*)"Set driving mode (none|stop|speed|pos)\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed <left> <right>", (unsigned char*)"Move left and right motors with given speed\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  pos <left> <right>", (unsigned char*)"Move left and right wheels to given position\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  pos reset", (unsigned char*)"Reset drive and wheel position\r\n", io_->stdOut);
}

static void DRV_PrintStatus(const CLS1_StdIOType *io_) {
	uint8_t buf[24];

	CLS1_SendStatusStr((unsigned char*)"drive", (unsigned char*)"\r\n", io_->stdOut);

	CLS1_SendStatusStr((unsigned char*)"  mode", DRV_GetModeStr(DRV_GetCurStatus()->mode), io_->stdOut);
	CLS1_SendStr((unsigned char*)"\r\n", io_->stdOut);

	UTIL1_Num32sToStr(buf, sizeof(buf), DRV_GetCurStatus()->speed.left);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" steps/sec\r\n");
	CLS1_SendStatusStr((unsigned char*)"  speed left", buf, io_->stdOut);

	UTIL1_Num32sToStr(buf, sizeof(buf), DRV_GetCurStatus()->speed.right);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" steps/sec\r\n");
	CLS1_SendStatusStr((unsigned char*)"  speed right", buf, io_->stdOut);

	UTIL1_Num32sToStr(buf, sizeof(buf), DRV_GetCurStatus()->pos.left);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" (curr: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), (int32_t)Q4CLeft_GetPos());
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)")\r\n");
	CLS1_SendStatusStr((unsigned char*)"  pos left", buf, io_->stdOut);

	UTIL1_Num32sToStr(buf, sizeof(buf), DRV_GetCurStatus()->pos.right);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" (curr: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), (int32_t)Q4CRight_GetPos());
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)")\r\n");
	CLS1_SendStatusStr((unsigned char*)"  pos right", buf, io_->stdOut);
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t DRV_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_) {
	uint8_t res = ERR_OK;
	const unsigned char *p;
	int32_t val1, val2;

	if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd_, (char*)"drive help")==0) {
		DRV_PrintHelp(io_);
		*handled_ = TRUE;
	} else if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd_, (char*)"drive status")==0) {
		DRV_PrintStatus(io_);
		*handled_ = TRUE;
	} else if (UTIL1_strncmp((char*)cmd_, (char*)"drive speed ", sizeof("drive speed ")-1)==0) {
		p = cmd_+sizeof("drive speed");
		if (UTIL1_xatoi(&p, &val1)==ERR_OK) {
			if (UTIL1_xatoi(&p, &val2)==ERR_OK) {
				if (DRV_SetSpeed(val1, val2)!=ERR_OK) {
					CLS1_SendStr((unsigned char*)"failed\r\n", io_->stdErr);
				}
				*handled_ = TRUE;
			} else {
				CLS1_SendStr((unsigned char*)"failed\r\n", io_->stdErr);
			}
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument(s)\r\n", io_->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd_, (char*)"drive pos reset", sizeof("drive pos reset")-1)==0) {
		Q4CLeft_SetPos(0);
		Q4CRight_SetPos(0);
		if (DRV_SetPos(0, 0)!=ERR_OK) {
			CLS1_SendStr((unsigned char*)"failed\r\n", io_->stdErr);
		}
		*handled_ = TRUE;
	} else if (UTIL1_strncmp((char*)cmd_, (char*)"drive pos ", sizeof("drive pos ")-1)==0) {
		p = cmd_+sizeof("drive pos");
		if (UTIL1_xatoi(&p, &val1)==ERR_OK) {
			if (UTIL1_xatoi(&p, &val2)==ERR_OK) {
				if (DRV_SetPos(val1, val2)!=ERR_OK) {
					CLS1_SendStr((unsigned char*)"failed\r\n", io_->stdErr);
				}
				*handled_ = TRUE;
			} else {
				CLS1_SendStr((unsigned char*)"failed\r\n", io_->stdErr);
			}
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument(s)\r\n", io_->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd_, (char*)"drive mode ", sizeof("drive mode ")-1)==0) {
		p = cmd_+sizeof("drive mode");
		if (UTIL1_strcmp((char*)p, (char*)"none")==0) {
			if (DRV_SetMode(DRV_MODE_NONE)!=ERR_OK) {
				res = ERR_FAILED;
			}
		} else if (UTIL1_strcmp((char*)p, (char*)"stop")==0) {
			if (DRV_SetMode(DRV_MODE_STOP)!=ERR_OK) {
				res = ERR_FAILED;
			}
		} else if (UTIL1_strcmp((char*)p, (char*)"speed")==0) {
			if (DRV_SetMode(DRV_MODE_SPEED)!=ERR_OK) {
				res = ERR_FAILED;
			}
		} else if (UTIL1_strcmp((char*)p, (char*)"pos")==0) {
			if (DRV_SetMode(DRV_MODE_POS)!=ERR_OK) {
				res = ERR_FAILED;
			}
		} else {
			res = ERR_FAILED;
		}
		if (res!=ERR_OK) {
			CLS1_SendStr((unsigned char*)"failed\r\n", io_->stdErr);
		}
		*handled_ = TRUE;
	}
	return res;
}




#ifdef MASTER_drv_clshdlr_C_
#undef MASTER_drv_clshdlr_C_
#endif /* !MASTER_drv_clshdlr_C_ */
