/***********************************************************************************************//**
 * @file		maf.h
 * @ingroup		maf
 * @brief 		moving average filter
 *
 * @author
 * @author
 * @date
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 **************************************************************************************************/

#ifndef MAF_API_H_
#define MAF_API_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_maf_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup maf
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
  *
  */
void MAF_UpdateRingBuffer(void);

/**
 *
 * @param isLeft_
 * @return
 */
int32_t MAF_Get_Speed(bool isLeft_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MAF_API_H_ */
