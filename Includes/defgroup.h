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
 * > - @b DEBUG state for debugging basic software on command line shell; and an
 * > - @b ERROR state for error handling.
 * \n\n
 * In DEBUG state the DEBUG task is awake which enables debugging activities of the basic software
 * components via command line shell (CLS).
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *
 * @copyright	@LGPL2_1
 */

/**
 * @defgroup	batt Battery
 * @brief		Software Component
 *
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *
 * @copyright	@LGPL2_1
 */
