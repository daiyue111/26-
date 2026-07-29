#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdint.h>

void lcd_display_init(void);
void lcd_display_show_time_ms(uint32_t elapsedMs);

#endif
