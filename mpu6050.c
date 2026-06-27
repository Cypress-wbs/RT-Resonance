#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

// ==================== [引脚定义] ====================
#define MPU_SCL_PIN     GET_PIN(B, 8)   // SCL 接 PB8
#define MPU_SDA_PIN     GET_PIN(B, 9)   // SDA 接 PB9

#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_GYRO_XOUT_H 0x43

// ==================== [I2C 底层驱动] ====================
#define MPU_SCL_H()     rt_pin_write(MPU_SCL_PIN, PIN_HIGH)
#define MPU_SCL_L()     rt_pin_write(MPU_SCL_PIN, PIN_LOW)
#define MPU_SDA_H()     rt_pin_write(MPU_SDA_PIN, PIN_HIGH)
#define MPU_SDA_L()     rt_pin_write(MPU_SDA_PIN, PIN_LOW)
#define MPU_SDA_READ()  rt_pin_read(MPU_SDA_PIN)

// 【时序加固】：将延时循环增大至 600，防止编译器开启 -O2/-Os 优化时 I2C 频率过高导致 MPU6050 无法响应
static void mpu_i2c_delay(void)
{
    for(volatile int i = 0; i < 600; i++);
}

static void mpu_i2c_start(void) {
    MPU_SDA_H(); MPU_SCL_H(); mpu_i2c_delay();
    MPU_SDA_L(); mpu_i2c_delay(); MPU_SCL_L(); mpu_i2c_delay();
}

static void mpu_i2c_stop(void) {
    MPU_SDA_L(); MPU_SCL_H(); mpu_i2c_delay();
    MPU_SDA_H(); mpu_i2c_delay();
}

static rt_uint8_t mpu_i2c_write_byte(rt_uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) MPU_SDA_H(); else MPU_SDA_L();
        mpu_i2c_delay(); MPU_SCL_H(); mpu_i2c_delay(); MPU_SCL_L(); mpu_i2c_delay();
        data <<= 1;
    }
    // 释放 SDA，准备读取应答
    MPU_SDA_H();
    MPU_SCL_H(); mpu_i2c_delay();
    rt_uint8_t ack = MPU_SDA_READ();
    MPU_SCL_L(); mpu_i2c_delay();
    return ack;
}

static rt_uint8_t mpu_i2c_read_byte(rt_uint8_t ack) {
    rt_uint8_t data = 0;
    // 释放 SDA 以读取从机信号
    MPU_SDA_H();
    for (int i = 0; i < 8; i++) {
        data <<= 1;
        MPU_SCL_H(); mpu_i2c_delay();
        if (MPU_SDA_READ()) data |= 0x01;
        MPU_SCL_L(); mpu_i2c_delay();
    }
    // 发送应答：0为ACK(继续读)，1为NACK(停止读)
    if (ack) MPU_SDA_L(); else MPU_SDA_H();
    mpu_i2c_delay(); MPU_SCL_H(); mpu_i2c_delay(); MPU_SCL_L(); mpu_i2c_delay();
    return data;
}

void mpu_write_reg(rt_uint8_t reg, rt_uint8_t data) {
    mpu_i2c_start();
    mpu_i2c_write_byte((MPU_ADDR << 1) | 0);
    mpu_i2c_write_byte(reg);
    mpu_i2c_write_byte(data);
    mpu_i2c_stop();
}

void mpu_read_bytes(rt_uint8_t reg, rt_uint8_t *buf, rt_uint8_t len) {
    mpu_i2c_start();
    mpu_i2c_write_byte((MPU_ADDR << 1) | 0);
    mpu_i2c_write_byte(reg);

    mpu_i2c_start();
    mpu_i2c_write_byte((MPU_ADDR << 1) | 1);
    for (rt_uint8_t i = 0; i < len; i++) {
        buf[i] = mpu_i2c_read_byte(i < (len - 1));
    }
    mpu_i2c_stop();
}

int mpu6050_init(void) {
    // 永远只需配置一次为开漏输出模式
    rt_pin_mode(MPU_SCL_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_mode(MPU_SDA_PIN, PIN_MODE_OUTPUT_OD);

    // 【死锁预防】：先释放 SDA，确保从机不会因为之前的异常传输而锁死总线
    MPU_SDA_H();
    mpu_i2c_delay();

    // [总线清道夫逻辑]：发送 10 个脉冲强制释放锁死的引脚
    for(int i = 0; i < 10; i++) {
        MPU_SCL_H(); mpu_i2c_delay();
        MPU_SCL_L(); mpu_i2c_delay();
    }

    // 手动产生一个 STOP 信号以复位从机总线状态
    MPU_SDA_L(); mpu_i2c_delay();
    MPU_SCL_H(); mpu_i2c_delay();
    MPU_SDA_H(); mpu_i2c_delay();

    rt_thread_mdelay(100);

    // 唤醒 MPU6050
    mpu_write_reg(MPU_PWR_MGMT_1, 0x00);
    rt_thread_mdelay(50);
    mpu_write_reg(MPU_GYRO_CONFIG, 0x08); // 陀螺仪量程配置
    mpu_write_reg(MPU_ACCEL_CONFIG, 0x00);

    return 0;
}

// ==================== [读取与自动唤醒] ====================
void mpu6050_get_gyro(short *gx, short *gy, short *gz) {
    rt_uint8_t buf[6];
    static int32_t consecutive_error_count = 0;

    // 1. 读取 6 个字节的数据
    mpu_read_bytes(MPU_GYRO_XOUT_H, buf, 6);

    // 2. 拼接高低位
    *gx = (short)((buf[0] << 8) | buf[1]);
    *gy = (short)((buf[2] << 8) | buf[3]);
    *gz = (short)((buf[4] << 8) | buf[5]);

    // 3. 【核心修复：自适应防误判唤醒】
    // 在静止时，gx 和 gy 抖动到 -1 属正常物理噪点。
    // 只有当 X/Y/Z 三轴同时持续为 -1 达到 30 次（即 150ms 以上持续掉线），才执行写总线唤醒
    if (*gx == -1 && *gy == -1 && *gz == -1) {
        consecutive_error_count++;
        if (consecutive_error_count >= 30) {
            // 确认为欠压引起的断电复位，一巴掌拍醒它！
            mpu_write_reg(MPU_PWR_MGMT_1, 0x00);
            consecutive_error_count = 0;
        }
    } else {
        consecutive_error_count = 0;
    }
}
