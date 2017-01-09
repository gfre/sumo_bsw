#define MASTER_ID_CFG_C_
#include "id_cfg.h"

#define NUM_OF_IDS (sizeof(idTable)/sizeof(idTable[0]))

static const KIN1_UID idTable[] =
  { /* SUMO_01 */ {{0x00,0x03,0x00,0x00,0x4E,0x45,0xB7,0x21,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
	/* SUMO_02 */
  };

static const ID_Cfg_t ID_Cfg = {
		idTable,
		NUM_OF_IDS,
};

const ID_Cfg_t *Get_ID_Cfg(void) { return &ID_Cfg; }

#ifdef MASTER_ID_CFG_C_
#undef MASTER_ID_CFG_C_
#endif
