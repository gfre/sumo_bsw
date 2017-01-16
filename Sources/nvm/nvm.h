#ifndef NVM_H_
#define NVM_H_

#include "Platform.h"

#ifdef MASTER_NVM_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif


EXTERNAL_ void NVM_Init(void);

EXTERNAL_ uint8_t NVM_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !NVM_H_ */