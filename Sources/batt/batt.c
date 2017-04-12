/***********************************************************************************************//**
 * @file		batt.c
 * @ingroup		batt
 * @brief 		Implementation of battery voltage measurement via ADC
 *
 * This module implements measurement functionality of a battery voltage by means of an ADC
 * provided by the firmware component @b AD1.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date	09.01.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_batt_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"
#include "batt.h"
#include "batt_api.h"
#include "AD1.h"
#include "FRTOS1.h"



/*======================================= >> #DEFINES << =========================================*/
#define SAMPLE_GROUP_SIZE 1U
#define BAT_V_DIVIDER_UP   62 /* voltage divider pull-up */
#define BAT_V_DIVIDER_DOWN 30 /* voltage divider pull-down */


/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
StdRtn_t BATT_MeasureBatteryVoltage(uint16_t *cvP)
{
  AD1_TResultData results[SAMPLE_GROUP_SIZE]={0};
  LDD_ADC_TSample SampleGroup[SAMPLE_GROUP_SIZE];
  uint32_t milliVolts;

  *cvP = 0; /* init */
  SampleGroup[0].ChannelIdx = 0U;  /* Create one-sample group */
  if (AD1_CreateSampleGroup(AD1_DeviceData, (LDD_ADC_TSample *)SampleGroup, SAMPLE_GROUP_SIZE)!=ERR_OK) {  /* Set created sample group */
    return ERR_FAILED;
  }
  if (AD1_StartSingleMeasurement(AD1_DeviceData)!=ERR_OK) {
    return ERR_FAILED;
  }
  while(!AD1_GetMeasurementCompleteStatus(AD1_DeviceData)) {
    vTaskDelay(pdMS_TO_TICKS(1)); /* wait */
  }
  if (AD1_GetMeasuredValues(AD1_DeviceData, &results[0])!=ERR_OK) {
    return ERR_FAILED;
  }
  /* reference voltage is 3.3V. Battery Voltage is using a voltage divider (R29, 62KOhm pullup to VBat, R30 30kOhm pull down to GND) */
  milliVolts = results[0]*330*(BAT_V_DIVIDER_UP+BAT_V_DIVIDER_DOWN)/BAT_V_DIVIDER_DOWN/0xffff; /* scale it to centi-volt. Do multiplication first to avoid numerical issues */
  *cvP = milliVolts;
  return ERR_OK;
}


void BATT_Init(void)
{
	/* nothing to do */
}

void BATT_Deinit(void)
{
	/* nothing to do */
}



#ifdef MASTER_batt_C_
#undef MASTER_batt_C_
#endif /* !MASTER_batt_C_ */
