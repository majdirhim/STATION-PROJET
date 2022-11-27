/*
 * Carte_Sd.c
 *
 *  Created on: 24 oct. 2022
 *      Author: Majdi
 */
#include "Carte_Sd.h"
/**
 * @brief Ecriture dans le fichier
 * @param : file_name : nom du fichier
 * @param : Wtext : data à transmettre
 */
FRESULT WR_TO_Sd(const char *wtext, const char *file_name) {

	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten; /* File write/read counts */

	//Open file for writing (Create)
	if (f_open(&SDFile, file_name, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
		Error_Handler();
	}
	//Write to the text file
	res = f_write(&SDFile, wtext, strlen(wtext), (void*) &byteswritten);
	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile); // Close file object
	}
	//f_mount(&SDFatFS, (TCHAR const*) NULL, 0); //unmount file system object
	return res;
}

/**
 * @brief : Initialiser un espace de travail dans la carte Sd
 */

void Fat_Init() {

	uint8_t rtext[_MAX_SS];/* File read buffer */
	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK) {
		Error_Handler();
	} else {
		FRESULT res = f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, rtext, sizeof(rtext));
		if (res!= FR_OK){ //creates a FAT volume on the logical drive)
			Error_Handler(); // error handler
		}
	}
}

