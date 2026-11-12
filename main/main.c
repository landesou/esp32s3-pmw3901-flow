/**
 ****************************************************************************************************
 * @file        main.c
 * @brief       PMW3901 光流传感器测试程序（ESP32-S3 / ESP-IDF v5.x）
 * @note        接线见 README.md 或 components/pmw3901/include/pmw3901.h 顶部注释。
 *              测试内容：
 *                1) SPI 通信自检（读芯片 ID 0x49 / 反码 0xB6）
 *                2) 写初始化寄存器序列
 *                3) 50Hz 轮询运动数据，串口打印 deltaX/deltaY、地表质量 SQUAL、快门 Shutter
 *                4) 累计位移并用几何系数粗估速度（mm/s），只做“有没有数据/方向对不对”的验证
 ****************************************************************************************************
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "pmw3901.h"

static const char *TAG = "FLOW";

#define SAMPLE_PERIOD_MS    20          /* 采样周期：50Hz（PMW3901 最快约 100Hz，50Hz 足够稳） */
#define PRINT_PERIOD_MS     1000        /* 人看的日志周期，数据帧每帧都发（见下） */
#define HEIGHT_MM           100.0f      /* 镜头到地面的距离（mm），仅用于粗估速度 */

/**
 * @brief  发送一帧给上位机的结构化数据（ASCII + XOR 校验）
 * @note   格式：$FLOW,<t_ms>,<dx>,<dy>,<motion>,<squal>,<shutter>*<XOR>\n
 *         XOR = '$' 之后、'*' 之前所有字符的异或（两位大写十六进制）。
 *         上位机（host/flow_host.py）按这一行解析；ESP_LOG 的日志行会被它忽略。
 */
static void flow_send_frame(int64_t t_ms, const pmw3901_motion_t *m)
{
    char body[96];
    int  len = snprintf(body, sizeof(body), "FLOW,%lld,%d,%d,%d,%u,%u",
                        (long long)t_ms, m->delta_x, m->delta_y,
                        m->motion ? 1 : 0, (unsigned)m->squal, (unsigned)m->shutter);

    uint8_t chk = 0;
    for (int i = 0; i < len; i++) {
        chk ^= (uint8_t)body[i];
    }
    printf("$%s*%02X\n", body, chk);
}

void app_main(void)
{
    pmw3901_config_t cfg = PMW3901_DEFAULT_CONFIG();
    esp_err_t err = ESP_OK;

    ESP_LOGI(TAG, "================ PMW3901 optical flow test ================");
    ESP_LOGI(TAG, "SPI: SCLK=%d MOSI=%d MISO=%d CS=%d clock=%d Hz",
             cfg.pin_sclk, cfg.pin_mosi, cfg.pin_miso, cfg.pin_cs, cfg.clock_hz);

    /* 初始化失败一般是接线/供电问题，这里每 2 秒重试一次，方便边接边看日志 */
    while ((err = pmw3901_init(&cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "check: 1) VCC/GND  2) SCLK/MOSI/MISO/CS wiring  3) chip id must be 0x49/0xB6");
        ESP_LOGE(TAG, "retry in 2 s ...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGI(TAG, "sensor ready (chip id already verified in init)");
    ESP_LOGI(TAG, "sample=%d ms, height=%.0f mm, 1 count ~= %.2f mm",
             SAMPLE_PERIOD_MS, HEIGHT_MM, HEIGHT_MM * PMW3901_COUNT_TO_RAD);
    ESP_LOGI(TAG, "move the module over a textured surface (>= 8 cm above ground)");

    int64_t  t_prev  = esp_timer_get_time();
    int64_t  t_print = t_prev;
    int64_t  t_win   = t_prev;

    int32_t  total_x = 0, total_y = 0;      /* 累计 counts */
    int32_t  win_x   = 0, win_y   = 0;      /* 本打印窗口内 counts，用来算速度 */
    uint32_t frames  = 0, motion_frames = 0;
    bool     warned  = false;

    while (1) {
        pmw3901_motion_t m = {0};

        err = pmw3901_read_motion(&m);
        int64_t now = esp_timer_get_time();

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "read motion failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* 每帧都发一包给上位机（50Hz） */
        flow_send_frame(now / 1000, &m);

        frames++;
        if (m.motion) {
            motion_frames++;
        }
        total_x += m.delta_x;
        total_y += m.delta_y;
        win_x   += m.delta_x;
        win_y   += m.delta_y;

        /* ---------------- 周期打印 ---------------- */
        if (now - t_print >= (int64_t)PRINT_PERIOD_MS * 1000) {
            float mm_per_count = HEIGHT_MM * PMW3901_COUNT_TO_RAD;
            float win_s = (float)(now - t_win) / 1e6f;
            float vx = (win_s > 0.001f) ? (win_x * mm_per_count / win_s) : 0.0f;
            float vy = (win_s > 0.001f) ? (win_y * mm_per_count / win_s) : 0.0f;

            ESP_LOGI(TAG,
                     "dx=%+4d dy=%+4d cnt | total=(%+" PRId32 ",%+" PRId32 ") | "
                     "v=(%+.1f,%+.1f) mm/s | squal=%3u shutter=%5u | "
                     "frames=%" PRIu32 " motion=%" PRIu32,
                     m.delta_x, m.delta_y,
                     total_x, total_y,
                     vx, vy,
                     m.squal, m.shutter,
                     frames, motion_frames);

            win_x = 0;
            win_y = 0;
            t_win = now;
            t_print = now;
        }

        /* ---------------- 5 秒还没有任何运动数据，提示排查方向 ---------------- */
        if (!warned && frames >= (5000 / SAMPLE_PERIOD_MS) && motion_frames == 0) {
            ESP_LOGW(TAG, "no motion detected in 5 s: lens faces down? textured surface? height >= 8 cm?");
            warned = true;
        }

        /* ---------------- 保持固定采样周期 ---------------- */
        int64_t elapsed_us = esp_timer_get_time() - now;
        int64_t wait_ms = SAMPLE_PERIOD_MS - elapsed_us / 1000;
        vTaskDelay(pdMS_TO_TICKS(wait_ms > 0 ? wait_ms : 1));
    }
}
