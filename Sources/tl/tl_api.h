/***********************************************************************************************//**
 * @file		tl.h
 * @ingroup		tl
 * @brief 		tracking loop filter
 *
 * @author 		S. Helling, stu112498@tf.uni-kiel.de
 * @date		11.08.17
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 **************************************************************************************************/

#ifndef TL_API_H_
#define TL_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"
#include "pid_api.h"


#ifdef MASTER_TL_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup TL
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/**
 * String identification of the SWC @ref tl
 */
#define TL_SWC_STRING ("tracking loop")


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum TL_ItmType_e
{
	TL_LFT_SPD_EST = 0,
	TL_RGHT_SPD_EST,
	TL_NUM_OF_ITMS,
}TL_ItmType_t;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
StdRtn_t TL_Read_CurLftPos(int32_t* result_);
StdRtn_t TL_Read_CurRghtPos(int32_t* result_);
int32_t  TL_Get_Speed(bool isLeft_);
PID_Cfg_t *Get_pTLCfg(void);
/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MAF_API_H_ */
