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
FRESULT WR_TO_Sd(const char* file_name,const char* fmt, ...) {
	char wtext[100];
    va_list arg;
    va_start(arg, fmt);
    vsprintf(wtext,fmt, arg);
    va_end(arg);

	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten; /* File write/read counts */
	HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_SET);
	//Open file for writing (Create)
	if (f_open(&SDFile, file_name, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
		Error_Handler();
	}
	//Write to the text file
	res = f_write(&SDFile, wtext, strlen(wtext), (void*) &byteswritten);
	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {
		HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);
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

/**
 * @brief : retourner l'espace de stockage total et l'espace restant en gbytes
 */
SD_State Sd_Space() {
	SD_State result;
	FATFS *fs;
	DWORD fre_clust, fre_sect, tot_sect;
	/* Get volume information and free clusters of drive 1 */
	f_getfree((TCHAR const*) SDPath, &fre_clust, &fs);
	/* Get total sectors and free sectors */
	tot_sect = (fs->n_fatent - 2) * fs->csize;
	fre_sect = fre_clust * fs->csize;
	//512 bytes/sectors
	result.Total_Space=(double)(tot_sect/2)*0.000001; //Gbytes
	result.Free_Space=(double)(fre_sect/2)*0.000001; //Gbytes
	return result;
}

