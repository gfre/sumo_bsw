=================
API Reference
=================
This chapter guides you through code documentation of the application layer and the rest of the
code of the Sumo platform. Since the application layer is deemed to be of most importance to the user,
it is explained here, whereas for the documentation of the rest of the code you will be redirected to 
the doxygen-generated html file that describes each function, variable, etc. in further detail.

-----------------
RTE
-----------------
This section will guide you through the code documentation of the real-time environment (RTE), which
is the application interface for application software development within the ACon Sumo Robot Project.
The firmware and basic software of the Sumo Robot is abstracted to the RTE layer, which allows
hardware-independent development of C99-compliant application software.

Dependency graphs
-----------------
The first overview of the RTE is given by several dependency graphs that show which parts of the software
are used by it. This gives you the basis of possible application development.

.. figure:: ../html/rte___types_8h__dep__incl.svg
	:align: center

	Files that include the RTE_Types header file.
	
.. figure:: ../html/rte___types_8h__incl.svg
	:align: center
 
	Files that are included by the RTE_Types header file.

.. figure:: ../html/rte_8c__incl.svg
	:align: center

	Files that are included by the rte source file.

.. figure:: ../html/rte_8h__dep__incl.svg
	:align: center

	Files that include the rte header file.

.. figure:: ../html/rte_8h__incl.svg
	:align: center

	Files that are included by the rte header file.

Defines
--------
.. doxygendefine:: RTE_STREAM

.. doxygendefine:: RTE_RF_MSG_TYPE_T

.. doxygendefine:: NUM_OF_SUMOS

.. doxygendefine:: MAX_ID_OF_SUMOS

.. doxygendefine:: DRV_MODE_T

Typedefs
--------
.. doxygentypedef:: EvntCbFct_t

.. doxygentypedef:: Stream_t

.. doxygentypedef:: RFMsgType_t

.. doxygentypedef:: RFPktDes_t

.. doxygentypedef:: RFRxMsgCbFct_t

.. doxygentypedef:: BUZ_Tunes_t

.. doxygentypedef:: ID_Sumo_t

.. doxygentypedef:: DrvMode_t

Enums
-----
.. doxygenenum:: RFMsgType_e

.. doxygenenum:: BUZ_Tunes_e

.. doxygenenum:: ID_Sumo_e

.. doxygenenum:: DrvMode_e

Functions
---------
.. doxygenfunction:: RTE_Set_ReInitAppl

.. doxygenfunction:: RTE_Set_TransIdle2Normal

.. doxygenfunction:: RTE_Write_LedLeOn

.. doxygenfunction:: RTE_Write_LedLeOff

.. doxygenfunction:: RTE_Write_LedLeNeg

.. doxygenfunction:: RTE_Write_LedLeSt

.. doxygenfunction:: RTE_Write_LedLeFlshWithPerMS

.. doxygenfunction:: RTE_Read_LedLeSt

.. doxygenfunction:: RTE_Write_LedRiOn

.. doxygenfunction:: RTE_Write_LedRiOff

.. doxygenfunction:: RTE_Write_LedRiNeg

.. doxygenfunction:: RTE_Write_LedRiSt
 
.. doxygenfunction:: RTE_Write_LedRiFlshWithPerMS

.. doxygenfunction:: RTE_Read_LedRiSt 

.. doxygenfunction:: RTE_Read_BtnSt 

.. doxygenfunction:: RTE_Write_BtnOnPrsdCbFct

.. doxygenfunction:: RTE_Write_BtnOnLngPrsdCbFct

.. doxygenfunction:: RTE_Write_BtnOnRlsdCbFct

.. doxygenfunction:: RTE_Write_BtnOnLngRlsdCbFct 

.. doxygenfunction:: RTE_Get_BtnOnPrsdCbFct

.. doxygenfunction:: RTE_Get_BtnOnLngPrsdCbFct

.. doxygenfunction:: RTE_Get_BtnOnRlsdCbFct 

.. doxygenfunction:: RTE_Get_BtnOnLngRlsdCbFct

.. doxygenfunction:: RTE_Write_BuzPlayTune

.. doxygenfunction:: RTE_Write_BuzBeep

.. doxygenfunction:: RTE_Read_SpdoVelLe 

.. doxygenfunction:: RTE_Read_SpdoVelRi

.. doxygenfunction:: RTE_Write_DrvVel

.. doxygenfunction:: RTE_Write_DrvPos

.. doxygenfunction:: RTE_Write_DrvMode

.. doxygenfunction:: RTE_Read_DrvMode

.. doxygenfunction:: RTE_Read_DrvIsDrvgBkwd

.. doxygenfunction:: RTE_Read_DrvHasStpd

.. doxygenfunction:: RTE_Read_DrvHasRvsd

.. doxygenfunction:: RTE_Write_RFSendDataBlk

.. doxygenfunction:: RTE_Write_RFRxMsgCbFct

.. doxygenfunction:: RTE_Get_RFRxMsgCbFct

.. doxygenfunction:: RTE_Read_RFSniffPkt

.. doxygenfunction:: RTE_Read_RFSrcAddr

.. doxygenfunction:: RTE_Write_RFSrcAddr

.. doxygenfunction:: RTE_Read_RFDstAddr

.. doxygenfunction:: RTE_Write_RFDstAddr

.. doxygenfunction:: RTE_printf

.. doxygenfunction:: RTE_fprintf

.. doxygenfunction:: RTE_puts

.. doxygenfunction:: RTE_putsErr

.. doxygenfunction:: RTE_GetSumoID

.. doxygenfunction:: RTE_Write_HoldOnEnterNormal

.. doxygenfunction:: RTE_Write_HoldOnEnterIdle

.. doxygenfunction:: RTE_Release_HoldOnEnterNormal

.. doxygenfunction:: RTE_Release_HoldOnEnterIdle

.. doxygenfunction:: RTE_Read_DataUnitAddrInNVM

.. doxygenfunction:: RTE_Save_DataUnit2NVM

.. doxygenfunction:: RTE_Save_BytesOfDataUnit2NVM

.. doxygenfunction:: RTE_Init

------------------
BSW
------------------
This section leads you to the entire API documentation, describing every function and component of the Sumo basic software 
in detail. For this, follow `this link <../_static/index.html>`_ to the index page.