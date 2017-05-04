/**
 * @defgroup	appl Application
 * @brief		Advanced state machine which handles the application and custom application software
 *
 * This software component implements an advanced state machine which runs the application and
 * custom application software.\n
 * > It handles the following *6 states*
 * > - @b STARTUP for one-time startup procedures;
 * > - @b INIT for initialisations of the application and application software;
 * > - @b IDLE for custom application software in IDLE, and
 * > - @b NORMAL for custom application software in NORMAL mode;
 * > - @b DEBUG state for debugging the Sumo Basic Software on command line shell; and an
 * > - @b ERROR state for error handling.
 *
 * In DEBUG state the DEBUG task gets resumed which enables debugging activities and status requests
 * of the Basic Software Components via command line shell (CLS). The software component provides
 * its own status information for debugging my means of the module *application command line shell
 * handler* (@b APPL_CLSHDLR).
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup	batt Battery
 * @brief		ADC Battery Voltage Measurement
 *
 * This software component provides the battery voltage for debugging and error handling purposes.
 * The voltage is measured by means of an Analogue-Digital-Converter. Its interface is provided by
 * the firmware component @b AD1.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup	buz Buzzer
 * @brief		Buzzer for playing sound, tunes and melodies
 *
 * This software component implements a driver for the buzzer. It uses the firmware components
 * @b BUZ1 and @b TRG1 to play predefined melodies or tunes, or make certain noises.\n
 * > Pre-configured melodies and tunes are:
 * > - a Welcome Melody
 * > - sound for *Button pressed*
 * > - sound for *Button pressed long*
 * > - sound for *Accepted*
 * > - sound for *Declined*
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.01.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup	dapp Demo Application
 * @brief		Simple framework for custom demo application software.
 *
 * This module implements a simple framework for custom demo application, e.g., for demonstration,
 * testing, debugging or education purposes at Univeristy Kiel.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	03.05.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup 	drv Drive
 * @brief		Speed and position control
 *
 * This software component implements a driver for controlling the movement of the robot in
 * certain modes. The driver runs its own FreeRTOS task. Moreover it decouples the drive
 * control behaviour from the actual application using a @a queue for the communication
 * between the application and the SWC @a DRIVE .\n
 * > It implements the following driving control modes:
 * > - STOP for standstill control
 * > - SPEED for velocity control and
 * > - POSITION control which provides to drive to a certain odometer target value.
 * Speed and position control is provided independently for the left-hand and right.hand side.
 *
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.01.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup	id Identification
 * @brief		Unique identification of the robot
 *
 * This software component identifies the robot by means of the the 128bit *Universally Unique
 * Identifier* (@b UID or @b UUID) of the MCU MK22FX512VLK12. With this the component maps the
 * UUID to a custom ID according to index of the UUID in a pre-defined table. This results in
 * a custom, but more readable ID as an enumeration between 0 and MAX_ID_OF_SUMOS.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.01.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup	ind Indication
 * @brief		Indication with User-LEDs
 *
 * This software component implements status indication using the on-board User-LEDs. For this,
 * the LEDs may be set to ON or OFF state or, can be toggled. Moreover they can flash with a
 * configurable frequency. The component makes use of the efficient timer-functionality provided
 * by FreeRTOS in the so-called timer service (or daemon) task.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	24.04.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup	mot Motor
 * @brief		Software Driver for the DC motors
 *
 * This software component implements a driver for the two DC motors on the left- and right-hand
 * sided of the robot. It uses @a PWM and @a BitIO firmware components from Kinets to influence the
 * speed and direction of the motors. The driver can handle inverted polarity which makes the
 * provided interface independent of the assembly of the motors.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	25.04.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup	nvm NVM
 * @brief		Non-volatile memory - *Data Flash Memory*
 *
 * This software component provides an implementation to store and retrieve data and parameter
 * values from the on-chip memory of the micro-controller MK22FX512VLK12. The MCU provides 128KB
 * FlexNVM which is entirely used as non-volatile Data Flash Memory in this project. No EEPROM is
 * emulated in this project.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	26.04.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup	pid PID
 * @brief		PID Controller
 *
 * This software component provides PID controllers for position and speed control of the Sumo
 * robots. An Anti-Wind-Up algorithm avoids drifting of the integral part and the maximum allowed
 * control value can changed by parameter. Controller parameters are read from the [NVM software
 * component](@ref nvm) during initialisation. They may be changed or default parameter values can
 * be restored via [command line shell](@ref sh). Changed parameter values are saved back to the NVM
 * again.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	27.04.2017
 *
 * @copyright	@LGPL2_1
 */




/**
 * @defgroup	rnet RNet
 * @brief		Application entry layer for the Radio Network Stack
 *
 * This software component implements an application entry layer for the Radio Network Stack. It
 * runs a state machine where the radio is powered up and the radio network stack gets processed.
 * Furthermore the it implements callback functions for handling received message and allows to
 * set and get the destination addresses where messages are supposed to send to. The component
 * is based upon the firmware component @a RApp of the Radio Network Stack.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	27.04.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup	rte RTE
 * @brief		*Real-Time Environment* - Application Interface for Application Software Development.
 *
 * The *Real-Time Environment* (@b RTE) is the application interface for application software
 * development within the *ACon Sumo Robot Project*. The firmware and basic software of the Sumo
 * Robot is abstracted to the RTE layer, which allows hardware-independent development of
 * C99-compliant application software.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.02.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup 	sh Shell
 * @brief 		Command Line Shell
 *
 * This software component implements an application entry layer for command line shell (@a CLS). It
 * loops through the callback functions of BSW components, which have implemented a command parser
 * for the comman line shell. Furthermore it sends some nice Welcome and Goodbye message to the
 * configured ouptput streams of the CLS. Here, the standard IN and OUT, and standard ERR streams
 * of both hardware interfaces, the RTT and USB, are used for I/O streaming.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	02.05.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup 	tacho Tacho
 * @brief 		Tachometer
 *
 * This module implements a tachometer component which calculates the speed based on quadrature
 * counters for up to two speed sources. The sign of the calculated speed signal indicates the
 * direction of movement. Furthermore, it provides a moving average filter to smoothing the speed
 * signal using a ring buffer for data collection. The module uses the firmware components
 * @a Q4CLeft and @a Q4CRight for the corresponding speed signals.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	04.05.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup 	task Task
 * @brief 		Task
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	05.05.2017
 *
 * @copyright	@LGPL2_1
 */
