/*
 * lcd_image.h
 *
 *  Created on: 10 déc. 2022
 *      Author: maxime
 */

#ifndef INC_LCD_IMAGE_H_
#define INC_LCD_IMAGE_H_

#include "stm32746g_discovery_lcd.h"

typedef struct {
    const unsigned char *data;
    uint16_t width;
    uint16_t height;
    uint8_t  dataSize;
} tImage;

typedef struct {
    long int code;
    const tImage *image;
} tChar;

typedef struct {
    int length;
    const tChar *chars;
} tFont;


void set_pixel(int x, int y, int color)
{
    if (color != 0) {;
        BSP_LCD_DrawPixel(x, y, color);
    } else {
        // reset pixel
    }
}


void draw_bitmap_mono(int x, int y, const tImage *image)
{
    uint8_t value = 0;
    int x0, y0;
    int counter = 0;
    const uint8_t *pdata = (const uint8_t *) image->data;
    // rows
    for (y0 = 0; y0 < image->height; y0++) {
        // columns
        for (x0 = 0; x0 < image->width; x0++) {
            // load new data
            if (counter == 0) {
                value = *pdata++;
                counter = image->dataSize;
            }

            counter--;

            // set pixel
            if ((value & 0x80) != 0) {
                set_pixel(x + x0, y + y0, 1);
            } else {
                set_pixel(x + x0, y + y0, 0);
            }

            value = value << 1;
        }
    }
}

void draw_bitmap_mono_rle(int x, int y, const tImage *image)
{
    uint8_t value = 0;
    int x0, y0;
    int counter = 0;
    int8_t sequence = 0;
    int8_t nonsequence = 0;
    const uint8_t *pdata = (const uint8_t *) image->data;
    // rows
    for (y0 = 0; y0 < image->height && (y0 + y) < 320; y0++) {
        // columns
        for (x0 = 0; x0 < image->width; x0++) {
            // load new data
            if (counter == 0) {
                if ((sequence == 0) && (nonsequence == 0)) {
                    sequence = *pdata++;

                    if (sequence < 0) {
                        nonsequence = -sequence;
                        sequence = 0;
                    }
                }

                if (sequence > 0) {
                    value = *pdata;
                    sequence--;

                    if (sequence == 0) {
                        pdata++;
                    }
                }

                if (nonsequence > 0) {
                    value = *pdata++;
                    nonsequence--;
                }

                counter = image->dataSize;
            }

            counter--;

            // set pixel
            if ((value & 0x80) != 0) {
                set_pixel(x + x0, y + y0, 1);
            } else {
                set_pixel(x + x0, y + y0, 0);
            }

            value = value << 1;
        }
    }
}



#endif /* INC_LCD_IMAGE_H_ */
