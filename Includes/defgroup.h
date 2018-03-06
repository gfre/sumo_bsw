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
 * emulated in this project. The components stores default values of all  parameters in a (pseudo)
 * ROM as well. The ROM is emulated in the RAM, i.e., the memory is statically allocated with
 * *const* qualifier in programm flash and initialised with the default parameter values.
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
 * @defgroup	refl Reflectance
 * @brief		Reflectance Sensor Array Software Component
 *
 * This software component provides access to the reflectance sensor array.
 * It can be used to drive the robot on a black line. More description to come...
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	01.06.2017
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
 * This software component creates and handles the FreeRTOS tasks. It implements generic task
 * functions for periodically and non-periodically runtime-calls of software components applied to
 * the corresponding task. It enables to initialises the software components run by the created
 * tasks before entering the runtime loop.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	05.05.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup	incl Includes
 * @brief		Custom includes
 *
 * This group collects all included user-defined header files which provide basic configuration data
 * for the Sumo Basic Software or common data type definitions.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.05.2017
 *
 * @copyright	@LGPL2_1
 */



/**
 * @defgroup	tl Tracking loop filter
 * @brief		Tracking loop filter
 *
 * This component implements an estimation algorithm for two states X1 and X2. For this it must be
 * assumed that X1 is measured and the two states are modelled by a simple integrator according to
 * d/dt (X1) = X2. Moreover, the algorithm implements a PI controller which drives the error between
 * the measured value X1_meas and its estimation X1_est to zero. Therefore X1_est is considered as
 * the output of the plant and X1_meas as the reference signal. The PI controller calculates a control
 * value which represents an estimation of for X2.
 *
 *   X1_meas					   X2_est				  X1_est
 * ----->(+)---->[PI-Controller]----------->[Integrator]----------
 * 		  ^(-)												     '
 * 	      '			  										     '
 *		  '------------------------------------------------------'
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	05.10.2017
 *
 * @copyright	@LGPL2_1
 */

/**
 * @defgroup	mtx Matrix operations
 * @brief		Matrix operations
 *
 *	This module implements basic matrix operations such as multiplication, addition, etc.
 *	and also more advanced operations such as matrix inversion, QR-decomposition, UD-de-
 *	composition, etc.
 *	To achieve this, it makes use of the liffixmatrix library.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author 	S. Helling,       stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	05.03.2018
 *
 * @copyright	@LGPL2_1
 */

/**
 * @defgroup	kf Kalman filter
 * @brief		Kalman filter
 *
 * A Kalman filter algorithm is used that calculates the optimal state estimate x_hat(k+1|k) for
 * the timestep 'k+1' given 'k' previous values for an n-dimensional system of the form:
 *
 * x(k+1|k) = Phi*x(k|k-1) + Gamma*u(k) + w(k) ,<br>
 * y(k)		= H*x(k|k-1) + v(k) .
 *
 * The optimal state estimate x_hat(k+1|k) is then calculated according to the following
 * predictor-corrector algorithm, where x_apri denotes the a priori state estimate prior
 * to taking measurements into account, and x_apost denotes the state estimate after taking
 * measurements into account. x_apost(-) is the a posteriori estimate from the previous step,
 * x_apost(+) is the a posteriori estimate from the current step. Same goes for the error
 * covariance matrix P.
 *
 * Predictor<br>
 * x_apri = Phi*x_apost(-)  + Gamma*u(k) ,<br>
 * P_apri = Phi*P_apost(-)*Phi' + G*Q*G' ,<br>
 *
 * Corrector<br>
 * K(k)       = P_apri*H' * (H*P_apri*H' + R)^{-1} ,<br>
 * x_apost(+) = x_apri + K(k)*( y(k) - H*x_apri ) ,<br>
 * P_apost(+) = (E - K(k)*H) * P_apri.
 *
 * Initial states are set to zero and P0 = alpha*E, with alpha>>1 (see kf_cfg.h).
 *
 * Dimensions<br>
 *  x     n-by-1 state vector,<br>
 *  Phi   n-by-n system/state transition matrix,<br>
 *  P(k)  n-by-n error covariance matrix,<br>
 *  u(k)  l-by-1 input vector to the system system,<br>
 *  Gamma n-by-l input matrix (l being the number of inputs),<br>
 *  y(k)  m-by-1 measurement vector (m being the number of measured states),<br>
 *  H     m-by-n measurement matrix,<br>
 *  w(k)  n-by-1 disturbance vector to the system (normally distributed with zero mean
 *  			 and standard deviation 'sigma_w'),<br>
 *  G     n-by-n coupling matrix for process noise,<br>
 *  Q     n-by-n diagonal process noise covariance matrix containing the variances
 *  			 'sigma_w^2' of the process disturbances for each state,<br>
 *  v(k)  m-by-1 measurement noise vector (normally distributed with zero mean and
 *  			 standard deviation 'sigma_v',<br>
 *  R     m-by-m diagonal measurement noise matrix containing the variances 'sigma_v^2'
 *  			  of the measurement disturbances for each state,<br>
 *  K(k)  n-by-m Kalman gain matrix,<br>
 *  E     n-by-n identity matrix.
 *
 * These steps rely heavily on UD-factorization of the error covariance matrix which
 * ensures numerical stability. U is a unit upper diagonal matrix and D a diagonal matrix
 * such that P = UDU'. The prediction for the error covariance matrix P_apri is calculated
 * according to C. Thornton, calculating UD-factors of P_apost(-) by modified, weighted
 * Gram-Schmidt orthogonalization. The correction for the error covariance matrix P_apost(+)
 * is calculated according to G. Bierman, calculating UD-factors of P_apri. For further
 * details see "Kalman Filtering Theory and Practice Using MATLAB" by M.S. Grewal and
 * A.P. Andrews.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author 	S. Helling,       stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	05.03.2018
 *
 * @copyright	@LGPL2_1
 */
