/***********************************************************************************************//**
 * @file		tl_cfg.c
 * @ingroup		tl
 * @brief 		Filter component to estimate system state using a PI tracking loop
 *
 *This component can estimate states from a system of the form
 *	d/dt (X1) = X2
 *with given measurements of the state X1 (see diagram below). The estimate X1_hat is given by integrating
 *the estimate X2_hat. Afterward X1_hat is fed back to a PI controller that is driven by the error
 *between X1 (measurement) and X1_hat. By regulating the error to zero, X1_hat approaches X1 and thus X2_hat
 *must approach X2.
 *The open loop is approximated in the Laplace domain by
 *	l(s) = g_PI(s) * 1/s = KP/s + KI/s^2
 *and thus
 *	t_{X1, X1_hat}(s) = X1_hat(s)/X1(s) = l(s)/(1+l(s)) = (KP/KI * s + 1) / (1/KI * s^2 + KP/KI * s + 1),
 *which is a 2nd order low pass filter and thus has no offset to neither a step nor to a ramp
 *input signal.
 *
 *   X1 						   X2_hat				  X1_hat
 * ----->(+)---->[PI-Controller]----------->[Integrator]----------
 * 		  ^(-)												     '
 * 	      '			  										     '
 *		  '------------------------------------------------------'
 *
 *Note that 'KP' and 'KI' can be evaluated using e.g. MATLABs control system toolbox. KI must be adjusted
 *by multiplying the sampling time 'Ta' on to it to work in the expected (simulated) manner.
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	17.08.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_tl_cfg_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tl.h"
#include "tl_api.h"
#include "tacho_api.h"


/*======================================= >> #DEFINES << =========================================*/
#define TL_LFT_SPD_STR		("tl L")
#define TL_RGHT_SPD_STR		("tl R")
#define TL_LFT_SAT_VAL		(0xFFFFu)
#define TL_RGHT_SAT_VAL		TL_LFT_SAT_VAL
/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/


/*=================================== >> GLOBAL VARIABLES << =====================================*/
							     /* KP   |  KI  |  KD  |  Scale  |  PI Output Limit  */
static PID_PrmCfg_t TLLftSpdCfg  = {3500u,  25u,   0u,    100u, 	TL_LFT_SAT_VAL,};  //output of PI must not be bounded so 'PI Output Limit' value...
static PID_PrmCfg_t TLRghtSpdCfg = {3500u,  25u,   0u,    100u, 	TL_RGHT_SAT_VAL,}; //...should be at least the possible output value for the tracked state!

static PID_Itm_t tlItmTbl[] =
{
		{TL_LFT_SPD_STR,  TL_LFT_SPD_EST,  &TLLftSpdCfg,  PID_NO_SAT, 0, 0, TL_Read_CurLftPos,	TACHO_Read_PosLft,  NULL, NULL, NULL},
		{TL_RGHT_SPD_STR, TL_RGHT_SPD_EST, &TLRghtSpdCfg, PID_NO_SAT, 0, 0, TL_Read_CurRghtPos, TACHO_Read_PosRght, NULL, NULL, NULL},
};

static PID_Cfg_t tlCfg =
{
		tlItmTbl,
		sizeof(tlItmTbl)/(sizeof(tlItmTbl[0])),
};

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/

PID_Cfg_t *Get_pTLCfg(void) {return &tlCfg;}


#ifdef MASTER_tl_cfg_C_
#undef MASTER_tl_cfg_C_
#endif /* !MASTER_tl_cfg_C_ */
