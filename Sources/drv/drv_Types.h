/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	30.03.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file drv_Types.h
 * 
 *==================================================================================================
 */


#ifndef DRV_TYPES_H_
#define DRV_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
/* #include "Acon_Types.h" */


#ifdef MASTER_drv_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
/* #define TMPL_MACRO (0xFFu) */



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum DRV_Mode_e {
  DRV_MODE_NONE,
  DRV_MODE_STOP,
  DRV_MODE_SPEED,
  DRV_MODE_POS,
} DRV_Mode_t;

typedef struct DRV_Status_s {
	DRV_Mode_t mode;
	struct {
		int32_t left, right;
	} speed;
	struct {
		int32_t left, right;
	} pos;
} DRV_Status_t;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ uint8_t DRV_SetMode(DRV_Mode_t mode);
EXTERNAL_ DRV_Mode_t DRV_GetMode(void);


EXTERNAL_ uint8_t DRV_SetSpeed(int32_t left, int32_t right);
EXTERNAL_ uint8_t DRV_SetPos(int32_t left, int32_t right);
EXTERNAL_ bool DRV_IsDrivingBackward(void);
EXTERNAL_ bool DRV_IsStopped(void);
EXTERNAL_ bool DRV_HasTurned(void);
EXTERNAL_ DRV_Status_t *DRV_GetCurStatus(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !DRV_TYPES_H_ */
