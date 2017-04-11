/***********************************************************************************************//**
 * @file		buz_clshdlr.c
 * @ingroup		buz
 * @brief 		Implementation of the configuration of the SWC @a Buzzer
 *
 * This file implements the configuration of pre-defined melodies and tunes of the software
 * component SWC @a Buzzer and its internal interface.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.02.2017
 *  
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_buz_cfg_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "buz_cfg.h"


/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static const BUZ_Tune MelodyWelcome[] =
{ /* freq, ms */
    {300,500},
    {500,200},
    {300,100},
    {200,300},
    {500,400},
    {300,100},
    {200,300},
    {300,100},
    {200,300},
    {500,400},
};

static const BUZ_Tune MelodyButton[] =
{ /* freq, ms */
    {400,100},
    {600,100},
};

static const BUZ_Tune MelodyAccept[] =
{ /* freq, ms */
    {400,100},
    {900,300},
};

static const BUZ_Tune MelodyDecline[] =
{ /* freq, ms */
    {400,100},
    {200,300},
};

static const BUZ_Tune MelodyButtonLong[] =
{ /* freq, ms */
    {500,50},
    {100,100},
    {300,50},
    {150,50},
    {450,50},
    {500,50},
    {250,200},
};


static MelodyDesc BUZ_Melodies[] =
{
    {0, sizeof(MelodyWelcome)/sizeof(MelodyWelcome[0]),       {0, 0}, MelodyWelcome},    /* BUZ_TUNE_WELCOME */
    {0, sizeof(MelodyButton)/sizeof(MelodyButton[0]),         {0, 0}, MelodyButton},     /* BUZ_TUNE_BUTTON */
    {0, sizeof(MelodyAccept)/sizeof(MelodyAccept[0]),         {0, 0}, MelodyAccept},     /* BUZ_TUNE_ACCEPT */
    {0, sizeof(MelodyDecline)/sizeof(MelodyDecline[0]),       {0, 0}, MelodyDecline},    /* BUZ_TUNE_DECLINE */
    {0, sizeof(MelodyButtonLong)/sizeof(MelodyButtonLong[0]), {0, 0}, MelodyButtonLong}, /* BUZ_TUNE_BUTTON_LONG */
};



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
/* static StdRtn_t TMPL_CustomFct(const TmplType_t *input_, TmplType_t *output_)
 * {
 * 		Write your code here!
 * }
 */



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
MelodyDesc *Get_BUZMelodies(void) { return BUZ_Melodies;}



#ifdef MASTER_buz_cfg_C_
#undef MASTER_buz_cfg_C_
#endif /* !MASTER_buz_cfg_C_ */
