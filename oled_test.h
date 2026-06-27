#ifndef __OLED_TEST_H__
#define __OLED_TEST_H__

#include <rtthread.h>

void oled_init(void);
void oled_clear(void);
void oled_show_string(rt_uint8_t x, rt_uint8_t page, char *str);

// 【修改点】将参数名字从 gx 改成 gy，增强代码可读性
void oled_show_debug_panel(short gy, int32_t fsr1, int32_t fsr2, int tick);

#endif /* __OLED_TEST_H__ */
