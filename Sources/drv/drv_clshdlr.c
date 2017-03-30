/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	30.03.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file drv_clshdlr.c
 * 
 *==================================================================================================
 */

#define MASTER_drv_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "drv_clshdlr.h"
#include "drv_Types.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"

/*======================================= >> #DEFINES << =========================================*/
/* #define TMPL_MACRO (0x01u) */



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/* typedef unit8 TmplType_t;



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t *DRV_GetModeStr(DRV_Mode_t mode);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
/* static TmplType_t tmplArray[STUD_MACRO] = {0u,TRUE,FALSE};



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

static void DRV_PrintHelp(const CLS1_StdIOType *io) {
	CLS1_SendHelpStr((unsigned char*)"drive", (unsigned char*)"Group of drive commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows drive help or status\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  mode <mode>", (unsigned char*)"Set driving mode (none|stop|speed|pos)\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed <left> <right>", (unsigned char*)"Move left and right motors with given speed\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  pos <left> <right>", (unsigned char*)"Move left and right wheels to given position\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  pos reset", (unsigned char*)"Reset drive and wheel position\r\n", io->stdOut);
}

static void DRV_PrintStatus(const CLS1_StdIOType *io) {
	uint8_t buf[24];

	CLS1_SendStatusStr((unsigned char*)"drive", (unsigned char*)"\r\n", io->stdOut);

	CLS1_SendStatusStr((unsigned char*)"  mode", DRV_GetModeStr(DRV_GetCurStatus()->mode), io->stdOut);
	CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

	UTIL1_Num32sToStr(buf, sizeof(buf), DRV_GetCurStatus()->speed.left);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" steps/sec\r\n");
	CLS1_SendStatusStr((unsigned char*)"  speed left", buf, io->stdOut);

	UTIL1_Num32sToStr(buf, sizeof(buf), DRV_GetCurStatus()->speed.right);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" steps/sec\r\n");
	CLS1_SendStatusStr((unsigned char*)"  speed right", buf, io->stdOut);

	UTIL1_Num32sToStr(buf, sizeof(buf), DRV_GetCurStatus()->pos.left);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" (curr: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), (int32_t)Q4CLeft_GetPos());
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)")\r\n");
	CLS1_SendStatusStr((unsigned char*)"  pos left", buf, io->stdOut);

	UTIL1_Num32sToStr(buf, sizeof(buf), DRV_GetCurStatus()->pos.right);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" (curr: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), (int32_t)Q4CRight_GetPos());
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)")\r\n");
	CLS1_SendStatusStr((unsigned char*)"  pos right", buf, io->stdOut);
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t DRV_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
	uint8_t res = ERR_OK;
	const unsigned char *p;
	int32_t val1, val2;

	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"drive help")==0) {
		DRV_PrintHelp(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"drive status")==0) {
		DRV_PrintStatus(io);
		*handled = TRUE;
	} else if (UTIL1_strncmp((char*)cmd, (char*)"drive speed ", sizeof("drive speed ")-1)==0) {
		p = cmd+sizeof("drive speed");
		if (UTIL1_xatoi(&p, &val1)==ERR_OK) {
			if (UTIL1_xatoi(&p, &val2)==ERR_OK) {
				if (DRV_SetSpeed(val1, val2)!=ERR_OK) {
					CLS1_SendStr((unsigned char*)"failed\r\n", io->stdErr);
				}
				*handled = TRUE;
			} else {
				CLS1_SendStr((unsigned char*)"failed\r\n", io->stdErr);
			}
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument(s)\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"drive pos reset", sizeof("drive pos reset")-1)==0) {
		Q4CLeft_SetPos(0);
		Q4CRight_SetPos(0);
		if (DRV_SetPos(0, 0)!=ERR_OK) {
			CLS1_SendStr((unsigned char*)"failed\r\n", io->stdErr);
		}
		*handled = TRUE;
	} else if (UTIL1_strncmp((char*)cmd, (char*)"drive pos ", sizeof("drive pos ")-1)==0) {
		p = cmd+sizeof("drive pos");
		if (UTIL1_xatoi(&p, &val1)==ERR_OK) {
			if (UTIL1_xatoi(&p, &val2)==ERR_OK) {
				if (DRV_SetPos(val1, val2)!=ERR_OK) {
					CLS1_SendStr((unsigned char*)"failed\r\n", io->stdErr);
				}
				*handled = TRUE;
			} else {
				CLS1_SendStr((unsigned char*)"failed\r\n", io->stdErr);
			}
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument(s)\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"drive mode ", sizeof("drive mode ")-1)==0) {
		p = cmd+sizeof("drive mode");
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
			CLS1_SendStr((unsigned char*)"failed\r\n", io->stdErr);
		}
		*handled = TRUE;
	}
	return res;
}




#ifdef MASTER_drv_clshdlr_C_
#undef MASTER_drv_clshdlr_C_
#endif /* !MASTER_drv_clshdlr_C_ */
