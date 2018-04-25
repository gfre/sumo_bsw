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
 * @brief 		Entry layer for the Command Line Shell
 *
 * This software component implements an application entry layer for command line shell (@a CLS). It
 * loops through the callback functions of BSW components, which have implemented a command parser
 * for the command line shell. Furthermore it sends some nice Welcome and Goodbye message to the
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
 * direction of movement. It calculates the speed using one of several filter structures @ref maf,
 * @ref kf, and @ref tl, respectively. By default, a Kalman filter is used.
 * The module uses the firmware components
 * @a Q4CLeft and @a Q4CRight for the corresponding speed signals.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author  S. Helling,       stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
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
 * This component implements an estimation algorithm for two states \f$ x_1 \f$ and \f$ x_2 \f$ .
 * For this, it must be  assumed that \f$ x_1 \f$ is measured and the two states are modelled
 * by a simple integrator according to \f$ \frac{d}{dt}x_1 = x_2 \f$ . Moreover, the algorithm
 * implements a PI controller which drives the error \f$ e = (x_{1,meas}-\hat{x}_1)\f$ between
 * the measured value \f$ x_{1,meas} \f$ and its estimation \f$ \hat{x}_1 \f$ to zero.
 * Therefore, \f$ \hat{x}_1\f$ is considered as the output of the plant and \f$ x_{1,meas}\f$
 * as the reference signal. The PI controller calculates a control value which represents
 * an estimation \f$ \hat{x}_2\f$ of the (not measured) state \f$ x_2\f$ .
 *
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author  S. Helling,       stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
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
 *	To achieve this, it makes use of the fixmatrix library.
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
 * \f$\renewcommand{\vec}{\boldsymbol}\f$
 * \f$\newcommand{\vxh}{\hat{\vec{x}}}\f$
 * \f$\newcommand{\vx}{\vec{x}}\f$
 *
 * A Kalman filter algorithm is used that calculates the optimal state estimate \f$\vxh(k+1|k)\f$ for
 * the timestep \f$ k+1 \f$ given \f$ k\f$ previous values in fixed-point arithmetic
 * for a \f$ n\f$-dimensional (linear) system of the form:
 *
 *\f{align*}{
 * \vx(k+1|k) &= \Phi\vx(k|k-1) + \Gamma\vec{u}(k) + \vec{w}(k) \\
 * \vec{y}(k)		&= H\vx(k|k-1) + \vec{v}(k)
 * \f}
 *
 * The optimal state estimate \f$ \vxh(k+1|k)\f$ is then calculated according to the following
 * predictor-corrector algorithm, where \f$ \vxh_{apri}\f$ denotes the a priori state estimate prior
 * to taking measurements into account, and \f$ \vxh_{apost}\f$ denotes the state estimate after taking
 * measurements into account. \f$ \vxh_{apost}(-) \f$ is the a posteriori estimate from the previous step,
 * \f$ \vxh_{apost}(+) \f$ is the a posteriori estimate from the current step. Same goes for the error
 * covariance matrix \f$ P = \mathbb{E}\big\{[\vx-\vxh][\vx-\vxh]'\big\}\f$.
 * The original Kalman formulation of this algorithm can then be described as
 *
 * \f$
 *    \textbf{Predictor}
 *      \begin{cases}
 *        \vxh_{apri} = \Phi\vxh_{apost}(-)  + \Gamma\vec{u}(k)\\
 *        P_{apri}    = \Phi P_{apost}(-)\Phi' + GQG'
 *      \end{cases}
 * \f$
 *
 * \f$
 *    \textbf{Corrector}
 *    \begin{cases}
 *      K               = P_{apri}H' (HP_{apri}H' + R)^{-1}\\
 *      \vxh_{apost}(+) = \vxh_{apri} + K\big( \vec{y}(k) - H\vxh_{apri} \big)\\
 *      P_{apost}(+)    = (E - KH)P_{apri},
 *    \end{cases}
 * \f$
 *
 * with initial values set to \f$\vxh_{apost}^0(-)=\vec{0}\f$ and \f$ P_{apost}^0(-) =
 * \alpha E\f$, with \f$ \alpha>>1\f$ (see @ref kf_cfg.h).
 *
 * @b Dimensions <br>
 *  \f$ \vx\in\mathbb{R}^{n}\f$ state vector,<br>
 *  \f$ \Phi\in\mathbb{R}^{n\times n} \f$   system/state transition matrix,<br>
 *  \f$ P\in\mathbb{R}^{n\times n} \f$ error covariance matrix,<br>
 *  \f$ u(k)\in\mathbb{R}^{l} \f$ input vector to the system system,<br>
 *  \f$ \Gamma\in\mathbb{R}^{n\times l} \f$ input matrix (\f$ l\f$ being the number of inputs),<br>
 *  \f$ \vec{y}(k)\in\mathbb{R}^{m} \f$ measurement vector (\f$ m\f$ being the number of measured states),<br>
 *  \f$ H\in\mathbb{R}^{m\times n}\f$  measurement matrix,<br>
 *  \f$ \vec{w}(k)\in\mathbb{R}^n\f$ disturbance vector to the system (normally distributed with zero mean
 *  			 and standard deviation \f$ \sigma_w\f$),<br>
 *  \f$ G \in\mathbb{R}^{n\times n}\f$ coupling matrix for process noise,<br>
 *  \f$ Q\in\mathbb{R}^{n\times n}\f$ diagonal process noise covariance matrix containing the variances
 *  			 \f$ \sigma_w^2\f$ of the process disturbances for each state,<br>
 * \f$ \vec{v}(k) \in\mathbb{R}^{m}\f$ measurement noise vector (normally distributed with zero mean and
 *  			 standard deviation \f$\sigma_v\f$,<br>
 *  \f$ R \in\mathbb{R}^{m\times m}\f$ diagonal measurement noise matrix containing
 *       the variances \f$\sigma_v^2\f$ of the measurement disturbances for each state,<br>
 *  \f$ K\in\mathbb{R}^{n\times m}\f$ Kalman gain matrix,<br>
 *  \f$ E\in\mathbb{R}^{n\times n}\f$ identity matrix.
 *
 * Since the original Kalman formulation requires the inversion of a matrix it can be
 * a heavy computational burden for a small embedded system. Additionally, the positve-
 * definiteneness of the matrix \f$ P\f$ cannot be guaranteed for badly-imposed problems, e.g.,
 * if the state transition matrix is ill-condiitoned. For this reason, a numerically stable
 * so-called square-root-algorithm is implemented which does not require calculating the inverse
 * of a matrix nor does it need to perform square root calculations (even though it belongs to the
 * class of square-root-filters). This algorithm relies heavily on UD-factorization of
 * the error covariance matrix \f$ P\f$, where \f$ U\f$ is a unit upper diagonal matrix
 * and \f$ D\f$ a diagonal matrix such that \f$ P = UDU'\f$ . The prediction for the
 * error covariance matrix \f$ P_{apri} \f$ is calculated according to C. Thornton,
 * calculating UD-factors of \f$ P_{apost}(-)\f$ using modified weighted Gram-Schmidt
 * orthogonalization (MWGS). The correction for the error covariance matrix
 * \f$ P_{apost}(+)\f$ is calculated according to G. Bierman, calculating UD-factors
 * of \f$ P_{apri}\f$ . For further details see "Kalman Filtering Theory and Practice
 * Using MATLAB" by M.S. Grewal and A.P. Andrews, aswell as "Triangular Covariance
 * Factorizations for Kalman Filtering" by C. Thornton.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author 	S. Helling,       stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	05.03.2018
 *
 * @copyright	@LGPL2_1
 */
