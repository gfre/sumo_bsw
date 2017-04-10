/**
 *
 */

/**
 * @defgroup	rte RTE
 * @brief		*Real-Time Environment* - Application Interface for Application Software Development.
 *
 * The *Real-Time Environment* (@b RTE) is the application interface for application software development
 * within the *ACon Sumo Robot Project*. The firmware and basic software of the Sumo Robot is abstracted to the
 * RTE layer, which allows hardware-independent development of C99-compliant application software.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.02.2017
 *
 * @copyright	@LGPL2_1
 */

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
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *
 * @copyright	@LGPL2_1
 */
