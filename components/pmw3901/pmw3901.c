/**
 ****************************************************************************************************
 * @file        pmw3901.c
 * @brief       PMW3901 光流传感器驱动（ESP-IDF / ESP32-S3）
 * @note        初始化寄存器序列参考 Bitcraze_PMW3901（MIT License）与 PMW3901MB 数据手册，
 *              寄存器读写时序按数据手册：读地址最高位为 0，写地址最高位为 1；
 *              读操作在发完地址后需要等待 tSRAD（≥35us）再读数据，CS 全程保持低电平。
 ****************************************************************************************************
 */

#include "pmw3901.h"

#include <stddef.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "pmw3901"

/* ================================ 寄存器地址 ================================ */
#define PMW3901_REG_PRODUCT_ID        0x00
#define PMW3901_REG_MOTION            0x02
#define PMW3901_REG_DELTA_X_L         0x03
#define PMW3901_REG_DELTA_X_H         0x04
#define PMW3901_REG_DELTA_Y_L         0x05
#define PMW3901_REG_DELTA_Y_H         0x06
#define PMW3901_REG_SQUAL             0x07
#define PMW3901_REG_SHUTTER_LOWER     0x0B   /* 快门值低字节 */
#define PMW3901_REG_SHUTTER_UPPER     0x0C   /* 快门值高字节（低 5 位有效） */
#define PMW3901_REG_MOTION_BURST      0x16   /* 运动数据 burst，一次读 12 字节 */
#define PMW3901_REG_POWER_UP_RESET    0x3A
#define PMW3901_REG_PRODUCT_ID_INV    0x5F
#define PMW3901_REG_BANK_SELECT       0x7F

/* ================================ 时序参数（us） ================================ */
#define PMW3901_T_CS_SETUP_US         50   /* CS 拉低到第一个时钟沿的建立时间 */
#define PMW3901_T_SRAD_US             50   /* 发读地址 -> 读数据，手册要求 >= 35us */
#define PMW3901_T_CS_HOLD_US          50   /* 最后一个时钟沿到 CS 拉高 */
#define PMW3901_T_SWW_US             200   /* 连续两次写之间的间隔，手册要求 >= 180us */

/* ================================ 驱动内部状态 ================================ */
static spi_device_handle_t s_dev = NULL;
static gpio_num_t          s_cs  = GPIO_NUM_NC;
static bool                s_ready = false;

static inline void cs_low(void)  { gpio_set_level(s_cs, 0); }
static inline void cs_high(void) { gpio_set_level(s_cs, 1); }

/**
 * @brief  一次 SPI 全双工传输（length 以字节计）
 */
static esp_err_t pmw3901_spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {0};

    t.length    = len * 8;
    t.rxlength  = (rx != NULL) ? (len * 8) : 0;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    return spi_device_polling_transmit(s_dev, &t);
}

/* ================================ 寄存器读写 ================================ */
esp_err_t pmw3901_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t addr  = (uint8_t)(reg & 0x7F);   /* 读：最高位保持 0 */
    uint8_t dummy = 0x00;
    esp_err_t err;

    cs_low();
    esp_rom_delay_us(PMW3901_T_CS_SETUP_US);

    err = pmw3901_spi_xfer(&addr, NULL, 1);  /* 先发地址 */
    if (err == ESP_OK) {
        esp_rom_delay_us(PMW3901_T_SRAD_US); /* 等 tSRAD 后再读数据 */
        err = pmw3901_spi_xfer(&dummy, value, 1);
    }

    esp_rom_delay_us(PMW3901_T_CS_HOLD_US);
    cs_high();
    esp_rom_delay_us(50);

    return err;
}

esp_err_t pmw3901_write_reg(uint8_t reg, uint8_t value)
{
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[2] = { (uint8_t)(reg | 0x80), value };  /* 写：最高位置 1 */
    esp_err_t err;

    cs_low();
    esp_rom_delay_us(PMW3901_T_CS_SETUP_US);

    err = pmw3901_spi_xfer(buf, NULL, sizeof(buf));

    esp_rom_delay_us(PMW3901_T_CS_HOLD_US);
    cs_high();
    esp_rom_delay_us(PMW3901_T_SWW_US);

    return err;
}

esp_err_t pmw3901_read_id(uint8_t *chip_id, uint8_t *chip_id_inv)
{
    if (chip_id == NULL || chip_id_inv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = pmw3901_read_reg(PMW3901_REG_PRODUCT_ID, chip_id);
    if (err == ESP_OK) {
        err = pmw3901_read_reg(PMW3901_REG_PRODUCT_ID_INV, chip_id_inv);
    }
    return err;
}

/* ================================ 初始化 ================================ */

/**
 * @brief  上电复位：翻转 CS 复位 SPI 接口 + 写 Power_Up_Reset 寄存器
 */
static esp_err_t pmw3901_power_up_reset(void)
{
    /* 1. CS 拉高-拉低-拉高，清掉上电瞬间 SPI 移位寄存器里的残余位 */
    cs_high();
    vTaskDelay(pdMS_TO_TICKS(1));
    cs_low();
    vTaskDelay(pdMS_TO_TICKS(1));
    cs_high();
    vTaskDelay(pdMS_TO_TICKS(1));

    /* 2. Power_Up_Reset：0x3A = 0x5A */
    esp_err_t err = pmw3901_write_reg(PMW3901_REG_POWER_UP_RESET, 0x5A);
    vTaskDelay(pdMS_TO_TICKS(5));

    return err;
}

/**
 * @brief  写入性能优化寄存器序列（芯片上电必须写一遍，否则数据不可用）
 * @note   这些寄存器分布在 0x00~0x15 多个 bank 里，0x7F 是 bank 选择寄存器。
 *         逐条照抄即可，不需要理解每一条的含义。
 */
static esp_err_t pmw3901_init_registers(void)
{
    esp_err_t err = ESP_OK;

#define PMW_WR(reg, val)                              \
    do {                                              \
        err = pmw3901_write_reg((reg), (val));        \
        if (err != ESP_OK) {                          \
            return err;                               \
        }                                             \
    } while (0)

    PMW_WR(0x7F, 0x00);
    PMW_WR(0x61, 0xAD);
    PMW_WR(0x7F, 0x03);
    PMW_WR(0x40, 0x00);
    PMW_WR(0x7F, 0x05);
    PMW_WR(0x41, 0xB3);
    PMW_WR(0x43, 0xF1);
    PMW_WR(0x45, 0x14);
    PMW_WR(0x5B, 0x32);
    PMW_WR(0x5F, 0x34);
    PMW_WR(0x7B, 0x08);
    PMW_WR(0x7F, 0x06);
    PMW_WR(0x44, 0x1B);
    PMW_WR(0x40, 0xBF);
    PMW_WR(0x4E, 0x3F);
    PMW_WR(0x7F, 0x08);
    PMW_WR(0x65, 0x20);
    PMW_WR(0x6A, 0x18);
    PMW_WR(0x7F, 0x09);
    PMW_WR(0x4F, 0xAF);
    PMW_WR(0x5F, 0x40);
    PMW_WR(0x48, 0x80);
    PMW_WR(0x49, 0x80);
    PMW_WR(0x57, 0x77);
    PMW_WR(0x60, 0x78);
    PMW_WR(0x61, 0x78);
    PMW_WR(0x62, 0x08);
    PMW_WR(0x63, 0x50);
    PMW_WR(0x7F, 0x0A);
    PMW_WR(0x45, 0x60);
    PMW_WR(0x7F, 0x00);
    PMW_WR(0x4D, 0x11);
    PMW_WR(0x55, 0x80);
    PMW_WR(0x74, 0x1F);
    PMW_WR(0x75, 0x1F);
    PMW_WR(0x4A, 0x78);
    PMW_WR(0x4B, 0x78);
    PMW_WR(0x44, 0x08);
    PMW_WR(0x45, 0x50);
    PMW_WR(0x64, 0xFF);
    PMW_WR(0x65, 0x1F);
    PMW_WR(0x7F, 0x14);
    PMW_WR(0x65, 0x60);
    PMW_WR(0x66, 0x08);
    PMW_WR(0x63, 0x78);
    PMW_WR(0x7F, 0x15);
    PMW_WR(0x48, 0x58);
    PMW_WR(0x7F, 0x07);
    PMW_WR(0x41, 0x0D);
    PMW_WR(0x43, 0x14);
    PMW_WR(0x4B, 0x0E);
    PMW_WR(0x45, 0x0F);
    PMW_WR(0x44, 0x42);
    PMW_WR(0x4C, 0x80);
    PMW_WR(0x7F, 0x10);
    PMW_WR(0x5B, 0x02);
    PMW_WR(0x7F, 0x07);
    PMW_WR(0x40, 0x41);
    PMW_WR(0x70, 0x00);

    vTaskDelay(pdMS_TO_TICKS(100));   /* 手册要求：这一小段配置之间停 100ms */

    PMW_WR(0x32, 0x44);
    PMW_WR(0x7F, 0x07);
    PMW_WR(0x40, 0x40);
    PMW_WR(0x7F, 0x06);
    PMW_WR(0x62, 0xF0);
    PMW_WR(0x63, 0x00);
    PMW_WR(0x7F, 0x0D);
    PMW_WR(0x48, 0xC0);
    PMW_WR(0x6F, 0xD5);
    PMW_WR(0x7F, 0x00);
    PMW_WR(0x5B, 0xA0);
    PMW_WR(0x4E, 0xA8);
    PMW_WR(0x5A, 0x50);
    PMW_WR(0x40, 0x80);

#undef PMW_WR

    return err;
}

/**
 * @brief  清一次运动数据寄存器（0x02~0x06 一读就清零）
 */
static esp_err_t pmw3901_clear_motion(void)
{
    uint8_t dummy;
    esp_err_t err = ESP_OK;

    for (uint8_t reg = PMW3901_REG_MOTION; reg <= PMW3901_REG_DELTA_Y_H; reg++) {
        err = pmw3901_read_reg(reg, &dummy);
        if (err != ESP_OK) {
            break;
        }
    }
    return err;
}

static esp_err_t pmw3901_bus_init(const pmw3901_config_t *cfg)
{
    esp_err_t err;

    /* ---------------- 片选 GPIO ---------------- */
    s_cs = cfg->pin_cs;
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << cfg->pin_cs),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cs gpio init failed: %s", esp_err_to_name(err));
        return err;
    }
    cs_high();                                          /* 空闲时片选拉高 */

    /* ---------------- SPI 总线 ---------------- */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = cfg->pin_mosi,
        .miso_io_num     = cfg->pin_miso,
        .sclk_io_num     = cfg->pin_sclk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 32,
    };
    err = spi_bus_initialize(cfg->host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "spi bus already initialized, reuse it");
        err = ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = cfg->clock_hz,
        .mode           = 3,        /* PMW3901 要求 SPI mode 3（CPOL=1, CPHA=1） */
        .spics_io_num   = -1,       /* 片选用 GPIO 手动控制，保证 tSRAD 等时序 */
        .queue_size     = 1,
    };
    err = spi_bus_add_device(cfg->host, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi add device failed: %s", esp_err_to_name(err));
        s_dev = NULL;
        return err;
    }

    return ESP_OK;
}

esp_err_t pmw3901_init(const pmw3901_config_t *cfg)
{
    pmw3901_config_t default_cfg = PMW3901_DEFAULT_CONFIG();
    esp_err_t err;

    if (cfg == NULL) {
        cfg = &default_cfg;
    }
    if (s_ready) {
        return ESP_OK;                                  /* 已经初始化过 */
    }

    /* 第一次调用才建总线和设备；失败后重试时不重复创建 */
    if (s_dev == NULL) {
        err = pmw3901_bus_init(cfg);
        if (err != ESP_OK) {
            return err;
        }
    }

    /* ---------------- 上电复位 + 自检 ---------------- */
    err = pmw3901_power_up_reset();
    if (err != ESP_OK) {
        return err;
    }

    uint8_t id = 0, id_inv = 0;
    bool id_ok = false;

    /* 模块刚上电时（或 SPI 上出现偶发误码时）第一次读 ID 可能失败，
     * 这里重试几次；仍失败才认为模块没接好。 */
    for (int retry = 0; retry < 5; retry++) {
        err = pmw3901_read_id(&id, &id_inv);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "read chip id failed: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "chip id: 0x%02X / 0x%02X", id, id_inv);
        if (id == PMW3901_CHIP_ID && id_inv == PMW3901_CHIP_ID_INV) {
            id_ok = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (!id_ok) {
        ESP_LOGE(TAG, "chip id mismatch, expect 0x49 / 0xB6");
        return ESP_ERR_NOT_FOUND;
    }

    /* ---------------- 写配置 ---------------- */
    err = pmw3901_clear_motion();
    if (err != ESP_OK) {
        return err;
    }
    err = pmw3901_init_registers();
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    (void)pmw3901_clear_motion();

    s_ready = true;
    ESP_LOGI(TAG, "init done");
    return ESP_OK;
}

esp_err_t pmw3901_reset(void)
{
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_ready = false;

    esp_err_t err = pmw3901_power_up_reset();
    if (err != ESP_OK) {
        return err;
    }

    uint8_t id = 0, id_inv = 0;
    err = pmw3901_read_id(&id, &id_inv);
    if (err != ESP_OK) {
        return err;
    }
    if (id != PMW3901_CHIP_ID || id_inv != PMW3901_CHIP_ID_INV) {
        return ESP_ERR_NOT_FOUND;
    }

    err = pmw3901_init_registers();
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    (void)pmw3901_clear_motion();

    s_ready = true;
    return ESP_OK;
}

/* ================================ 数据读取 ================================ */
/**
 * @brief  通过 Motion Burst（0x16）一次性读 12 字节运动数据
 * @note   burst 字节定义（PMW3901MB 手册 / LibDriver 一致）：
 *           [0]  MOTION：bit7 为运动标志
 *           [1]  Observation
 *           [2]  Delta_X_L   [3] Delta_X_H
 *           [4]  Delta_Y_L   [5] Delta_Y_H
 *           [6]  SQUAL/4      [7] RawData_Sum   [8] RawData_Max  [9] RawData_Min
 *           [10] Shutter_Upper（低 5 位有效）  [11] Shutter_Lower
 *         分开读单寄存器时，快门高低字节可能在两次读之间被刷新，所以会乱跳；
 *         用 burst 读是原子的，推荐始终走这里。
 */
esp_err_t pmw3901_read_motion(pmw3901_motion_t *motion)
{
    if (motion == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t addr = PMW3901_REG_MOTION_BURST;   /* 读地址最高位保持 0 */
    uint8_t dummy[12] = {0};
    uint8_t burst[12] = {0};
    esp_err_t err;

    cs_low();
    esp_rom_delay_us(PMW3901_T_CS_SETUP_US);

    err = pmw3901_spi_xfer(&addr, NULL, 1);              /* 发 burst 地址 */
    if (err == ESP_OK) {
        esp_rom_delay_us(PMW3901_T_SRAD_US);             /* 等 tSRAD 后连续读 12 字节 */
        err = pmw3901_spi_xfer(dummy, burst, sizeof(burst));
    }

    esp_rom_delay_us(PMW3901_T_CS_HOLD_US);
    cs_high();
    esp_rom_delay_us(50);

    if (err != ESP_OK) {
        return err;
    }

    motion->motion  = (burst[0] & 0x80) != 0;            /* bit7 = MOT */
    motion->delta_x = (int16_t)(((uint16_t)burst[3] << 8) | burst[2]);
    motion->delta_y = (int16_t)(((uint16_t)burst[5] << 8) | burst[4]);
    motion->squal   = (uint8_t)(burst[6] * 4);           /* burst 里是 SQUAL 的 1/4 */
    motion->shutter = (uint16_t)(((uint16_t)(burst[10] & 0x1F) << 8) | burst[11]);

    return ESP_OK;
}
