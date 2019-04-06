#ifndef __HC05_H__
#define __HC05_H__

#include"include.h"
#include"common.h"

uint8 HC05_init(void);
void HC05_CFG_CMD(uint8 *str);
uint8 HC05_Get_Role(void);
uint8 HC05_Set_Cmd(uint8* atstr);	 



#endif