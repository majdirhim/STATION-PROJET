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
FRESULT WR_TO_Sd(const char* wtext,const char* file_name);

void Fat_Init();

#ifdef __cplusplus
 }
#endif
#endif /* INC_CARTE_SD_H_ */
