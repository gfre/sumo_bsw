#ifndef NVM_H_
#define NVM_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_NVM_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ void NVM_Init(void);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !NVM_H_ */
