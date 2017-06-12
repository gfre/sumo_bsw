/***********************************************************************************************//**
 * @file		refl_clshdlr.h
 * @ingroup		refl
 * @brief 		Interface for the command line shell handler of the SWC @a RNet
 *
 * This header files provides the interface from the SWC @ref refl to the SWC @ref sh.
 * It introduces application specific commands for requests of status information
 * via command line shell (@b CLS)
 *
 * @author 	Simon Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#ifndef REFL_CLSHDLR_H_
#define REFL_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"
#include "refl_api.h"

#ifdef MASTER_refl_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup refl
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * \brief Shell parser routine.
 * \param cmd Pointer to command line string.
 * \param handled Pointer to status if command has been handled. Set to TRUE if command was understood.
 * \param io Pointer to stdio handle
 * \return Error code, ERR_OK if everything was ok.
 */
EXTERNAL_ byte REF_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);

EXTERNAL_ RefStateType REF_GetRefState(void);

EXTERNAL_ SensorTimeType REF_GetRawSensorValue(const uint8 i);

EXTERNAL_ SensorTimeType REF_GetCalibratedSensorValue(const uint8 i);

EXTERNAL_ int16_t REF_GetRefLineValue(void);

EXTERNAL_ NVM_ReflCalibData_t* REF_GetCalibMinMaxPtr(void);

EXTERNAL_ void REF_SetRefEnabled(bool isEnabled);

EXTERNAL_ void REF_SetLedOn(bool isOn);
/*!
 * \brief Starts or stops the calibration.
 */
EXTERNAL_ void REF_CalibrateStartStop(void);

/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !SOURCES_REFL_REFL_CLSHDLR_H_ */
