/**
 * \file
 * \brief This is the interface to the application entry point.
 * \author (c) 2016 Erich Styger, http://mcuoneclipse.com/
 * \note MIT License (http://opensource.org/licenses/mit-license.html)
 */

#ifndef RNET_H_
#define RNET_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_RNET_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
#define RNET_SWC_STRING ("rnet")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/** @brief Driver de-initialization */
EXTERNAL_ void RNET_Deinit(void);

/** @brief Driver initialization */
EXTERNAL_ void RNET_Init(void);

/** @brief Driver main function */
EXTERNAL_ void RNET_MainFct(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* RNET_APPL_H_ */
