/**
 ****************************************************************************************************
 * @file        pmw3901.h
 * @brief       PMW3901 光流传感器驱动（ESP-IDF / ESP32-S3）
 * @note        传感器：PixArt PMW3901MB（逐飞科技等厂家的光流模块基本都是这颗）
 *              接口：4 线 SPI（只读为主，写用于初始化配置）+ 片选 CS
 *              寄存器约定：读地址最高位保持 0；写地址最高位置 1（addr | 0x80）
 ****************************************************************************************************
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

/* ================================ 传感器常量 ================================ */

/** 芯片 ID：读 0x00 得到 0x49（0100 1001） */
#define PMW3901_CHIP_ID         0x49
/** 芯片 ID 反码：读 0x5F 得到 0xB6（1011 0110） */
#define PMW3901_CHIP_ID_INV     0xB6

/**
 * 一帧 counts 换算成地面位移的几何系数（粗略值，正式使用请标定）：
 *   位移(mm) ≈ counts × 对地高度(mm) × PMW3901_COUNT_TO_RAD
 *   推导：PMW3901 视场约 42°、35×35 像素，每个 count 约对应 42°/35 ≈ 1.2° ≈ 0.021 rad。
 *   实际受镜头批次、地面纹理、离地高度影响，只能用来“看趋势”，不能当测距用。
 */
#define PMW3901_COUNT_TO_RAD    0.0209f

/* ================================ 默认接线 ================================ */
/**
 * 默认使用 ESP32-S3 的 SPI2（FSPI）默认引脚，正点原子 DNESP32S3 等开发板的排针上都引出了：
 *
 *   PMW3901 模块        ESP32-S3
 *   VCC        ------>  3V3（模块若标注 5V 输入则接 5V，逻辑电平仍是 3.3V）
 *   GND        ------>  GND
 *   SCLK/SCK   ------>  GPIO12
 *   MOSI/SDA   ------>  GPIO11
 *   MISO/ADDR  ------>  GPIO13
 *   CS/CSN     ------>  GPIO10
 *
 * 换板子/换引脚只改下面 4 个宏，或者直接改 pmw3901_config_t 结构体。
 */
#define PMW3901_DEFAULT_HOST        SPI2_HOST
#define PMW3901_DEFAULT_PIN_SCLK    GPIO_NUM_12
#define PMW3901_DEFAULT_PIN_MOSI    GPIO_NUM_11
#define PMW3901_DEFAULT_PIN_MISO    GPIO_NUM_13
#define PMW3901_DEFAULT_PIN_CS      GPIO_NUM_10
/** SPI 时钟：PMW3901 手册上限 2MHz，线长/杜邦线质量差时先降到 1MHz 排查 */
#define PMW3901_DEFAULT_CLOCK_HZ    2000000

/* ================================ 数据类型 ================================ */

typedef struct {
    spi_host_device_t host;         /* SPI 主机：SPI2_HOST / SPI3_HOST */
    gpio_num_t        pin_sclk;     /* 时钟 */
    gpio_num_t        pin_mosi;     /* 主出从入 */
    gpio_num_t        pin_miso;     /* 主入从出 */
    gpio_num_t        pin_cs;       /* 片选，低有效（驱动内部用 GPIO 手动控制） */
    int               clock_hz;     /* SPI 时钟频率，建议 1~2MHz */
} pmw3901_config_t;

/** 默认配置（配合 pmw3901_config_t cfg = PMW3901_DEFAULT_CONFIG(); 使用） */
#define PMW3901_DEFAULT_CONFIG()                        \
    (pmw3901_config_t) {                                \
        .host      = PMW3901_DEFAULT_HOST,              \
        .pin_sclk  = PMW3901_DEFAULT_PIN_SCLK,          \
        .pin_mosi  = PMW3901_DEFAULT_PIN_MOSI,          \
        .pin_miso  = PMW3901_DEFAULT_PIN_MISO,          \
        .pin_cs    = PMW3901_DEFAULT_PIN_CS,            \
        .clock_hz  = PMW3901_DEFAULT_CLOCK_HZ,          \
    }

/** 一次采样得到的光流数据 */
typedef struct {
    bool     motion;        /* 本帧是否检测到运动（motion burst 字节 0 的 bit7，注意不是 bit0） */
    int16_t  delta_x;       /* 本帧水平位移，单位 counts，向右为正 */
    int16_t  delta_y;       /* 本帧垂直位移，单位 counts，向下为正 */
    uint8_t  squal;         /* 地表质量 0~255，越大越好；< 40 说明纹理太差/太暗 */
    uint16_t shutter;       /* 快门值 0~0x1FFF（高字节只取低 5 位），越大说明环境越暗 */
} pmw3901_motion_t;

/* ================================ API ================================ */

/**
 * @brief  初始化 PMW3901（SPI 总线 + 片选 + 上电复位 + 写配置 + 校验 ID）
 * @param  cfg  SPI 与引脚配置，可传 NULL 使用默认配置
 * @retval ESP_OK           成功
 * @retval ESP_ERR_NOT_FOUND 读到 ID 不是 0x49/0xB6（接线/供电/时序问题）
 * @retval 其他              SPI 总线或 GPIO 初始化失败
 */
esp_err_t pmw3901_init(const pmw3901_config_t *cfg);

/**
 * @brief  读取芯片 ID 与 ID 反码，用于自检
 */
esp_err_t pmw3901_read_id(uint8_t *chip_id, uint8_t *chip_id_inv);

/**
 * @brief  读单个寄存器（地址最高位自动清零）
 */
esp_err_t pmw3901_read_reg(uint8_t reg, uint8_t *value);

/**
 * @brief  写单个寄存器（地址最高位自动置 1）
 */
esp_err_t pmw3901_write_reg(uint8_t reg, uint8_t value);

/**
 * @brief  读取一次运动数据（MOTION + Delta_X/Y + SQUAL + Shutter）
 * @note   必须先读 0x02（MOTION），再读 0x03~0x06，读完 delta 寄存器会自动清零。
 *         建议 10~100Hz 调用，太快没有新数据，太慢位移会饱和/丢帧。
 */
esp_err_t pmw3901_read_motion(pmw3901_motion_t *motion);

/**
 * @brief  软复位并重新写入配置（运行中数据异常时可调用）
 */
esp_err_t pmw3901_reset(void);

/**
 * @brief  counts 换算成地面位移（mm）的示意函数，height_mm 为镜头到地面高度
 */
static inline float pmw3901_counts_to_mm(int16_t counts, float height_mm)
{
    return (float)counts * height_mm * PMW3901_COUNT_TO_RAD;
}

#ifdef __cplusplus
}
#endif
