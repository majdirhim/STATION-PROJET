/*
 * Carte_Sd.c
 *
 *  Created on: 24 oct. 2022
 *      Author: Majdi
 */
#include "Carte_Sd.h"

FRESULT WR_TO_Sd(const char *wtext, const char *file_name) {

	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten; /* File write/read counts */
	uint8_t rtext[_MAX_SS];/* File read buffer */

	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK) {
		Error_Handler(); // Alerts_Action(internal_soft);
	} else {
		if (f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, rtext, sizeof(rtext)) //creates a FAT volume on the logical drive
				!= FR_OK) {
			Error_Handler(); // error handler
		} else {
			//Open file for writing (Create)
			if (f_open(&SDFile, "SWlog.TXT", FA_CREATE_ALWAYS | FA_WRITE)
					!= FR_OK) {
				Error_Handler();
			} else {
				//Write to the text file
				res = f_write(&SDFile, wtext, strlen(wtext),
						(void*) &byteswritten);
				if ((byteswritten == 0) || (res != FR_OK)) {
					Error_Handler();
				} else {

					f_close(&SDFile); // Close file object
				}
			}
		}
	}
	f_mount(&SDFatFS, (TCHAR const*) NULL, 0); //unmount file system object
return res;
}

