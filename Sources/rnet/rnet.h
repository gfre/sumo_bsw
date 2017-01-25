/**
 * \file
 * \brief This is the interface to the application entry point.
 * \author (c) 2016 Erich Styger, http://mcuoneclipse.com/
 * \note MIT License (http://opensource.org/licenses/mit-license.html)
 */

#ifndef RNET_APPL_H_
#define RNET_APPL_H_

#include "Platform.h"
#include "RNWK.h"
#include "RApp.h"

#ifdef MASTER_RNET_APPL_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

#define RNET_SWC_STRING ("rnet")

EXTERNAL_ uint8_t RNET_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);


/*!
 * \brief Return the current remote node address.
 * \return Remote node address
 */
EXTERNAL_ RNWK_ShortAddrType RNET_GetDestAddr(void);

/*! \brief Driver de-initialization */
EXTERNAL_ void RNET_Deinit(void);

/*! \brief Driver initialization */
EXTERNAL_ void RNET_Init(void);


EXTERNAL_ void RNET_MainFct(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* RNET_APPL_H_ */
