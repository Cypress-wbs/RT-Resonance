#ifndef __RFP602_H__
#define __RFP602_H__

#include <rtthread.h>

// 暴露出全局变量供 main 和 Python 上位机读取
extern volatile int32_t g_RFP1_raw;
extern volatile int32_t g_RFP2_raw;
extern volatile int32_t g_RFP_calib_state;

// RFP 接口声明
int RFP_init(void);
void RFP_read_raw(void);

#endif /* __RFP602_H__ */
