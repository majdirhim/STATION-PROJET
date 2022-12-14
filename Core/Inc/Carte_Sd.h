/*
 * Carte_Sd.h
 *
 *  Created on: 24 oct. 2022
 *      Author: Majdi
 */

#ifndef INC_CARTE_SD_H_
#define INC_CARTE_SD_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#include <stdarg.h>

typedef struct{
	double Total_Space;
	double Free_Space;
}SD_State;

FRESULT WR_TO_Sd(const char* file_name,const char* fmt, ...);
SD_State Sd_Space();
void Fat_Init();

#ifdef __cplusplus
 }
#endif
#endif /* INC_CARTE_SD_H_ */
