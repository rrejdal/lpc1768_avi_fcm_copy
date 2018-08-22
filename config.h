#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "structures.h"
//#include "mbed.h"

void Config_SetDefaults(T_HFC *hfc);
void Config_Read(T_Config *cfg);
void Config_Read_Compass(T_HFC *hfc);
void Config_ApplyAndInit(T_HFC *hfc);
void Config_Save(T_Config *cfg);
void Save_PIDvalues(T_HFC *hfc);
void Reset_Iterms(T_HFC *hfc);
void GenerateSpeed2AngleLUT(T_HFC *hfc);

#endif
