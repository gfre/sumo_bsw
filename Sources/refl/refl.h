/**
 * \file
 * \brief Reflectance sensor driver interface.
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * This module implements a driver for the reflectance sensor array.
 */

#ifndef REFLECTANCE_H_
#define REFLECTANCE_H_


#define REF_NOF_SENSORS 6
#define REF_MIDDLE_LINE_VALUE  ((REF_NOF_SENSORS+1)*1000/2)
#define REF_MAX_LINE_VALUE     ((REF_NOF_SENSORS+1)*1000) /* maximum value for REF_GetLine() */

/*Welcher ist es denn nun?*/
#if PL_IS_ZUMO_ROBOT
  #define REF_SENSOR1_IS_LEFT        0   /* if sensor 1 is on the left side */
  #define REF_MIN_LINE_VAL        0x60   /* minimum value indicating a line */
  #define REF_MIN_NOISE_VAL       0x40   /* values below this are not added to the weighted sum */
  #define REF_SENSOR_TIMEOUT_US   1200   /* after this time, consider no reflection (black) */
#elif PL_IS_INTRO_ZUMO_ROBOT
  #define REF_SENSOR1_IS_LEFT    1   /* if sensor 1 is on the left side */
  #define REF_MIN_LINE_VAL    0x20   /* minimum value indicating a line */
  #define REF_MIN_NOISE_VAL   0x10   /* values below this are not added to the weighted sum */
  #define REF_SENSOR_TIMEOUT_US  3000   /* after this time, consider no reflection (black) */
#elif PL_IS_INTRO_ZUMO_ROBOT2 || PL_IS_INTRO_ZUMO_K22
  #define REF_SENSOR1_IS_LEFT       1   /* if sensor 1 is on the left side */
  #define REF_MIN_LINE_VAL      0x120   /* minimum value indicating a line */
  //#define REF_MIN_NOISE_VAL      0x40   /* values below this are not added to the weighted sum */
  #define REF_MIN_NOISE_VAL      0x80   /* values below this are not added to the weighted sum */
  #define REF_SENSOR_TIMEOUT_US  3500   /* after this time, consider no reflection (black). Must be smaller than the timeout period of the RefCnt timer! */
#else
  #error "unknown configuration!"
#endif





/*!
 * \brief Driver Deinitialization.
 */
void REF_Deinit(void);

/*!
 * \brief Driver Initialization.
 */
void REF_Init(void);


#endif /* REFLECTANCE_H_ */
