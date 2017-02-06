/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	06.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file rte_Types.h
 * 
 *==================================================================================================
 */


#ifndef RTE_TYPES_H_
#define RTE_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_rte_Types_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum BUZ_Tunes_e {
  BUZ_TUNE_WELCOME = 0
 ,BUZ_TUNE_BUTTON
 ,BUZ_TUNE_ACCEPT
 ,BUZ_TUNE_DECLINE
 ,BUZ_TUNE_BUTTON_LONG
 ,BUZ_TUNE_NOF_TUNES
} BUZ_Tunes_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/* EXTERNAL_ StdRtn_t TMPL_CustomFct(const TmplType_t *input_, TmplType_t *output_); */


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !RTE_TYPES_H_ */
