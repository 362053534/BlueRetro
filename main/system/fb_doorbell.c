/*
 * Copyright (c) 2026, BlueRetro
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "soc/dport_reg.h"
#include "soc/periph_defs.h"
#include "system/fb_doorbell.h"

/* FROM_CPU_2 已被 core0_stall 占用；0/1 常被 IDF 跨核 yield 占用。 */
#define FB_DOORBELL_SRC ETS_FROM_CPU_INTR3_SOURCE
#define FB_DOORBELL_REG DPORT_CPU_INTR_FROM_CPU_3_REG
#define FB_DOORBELL_BIT DPORT_CPU_INTR_FROM_CPU_3

static TaskHandle_t fb_doorbell_task;

static void IRAM_ATTR fb_doorbell_isr(void *arg) {
    BaseType_t woken = pdFALSE;

    /* 先清边沿，避免重入 */
    _DPORT_REG_WRITE(FB_DOORBELL_REG, 0);
    if (fb_doorbell_task) {
        vTaskNotifyGiveFromISR(fb_doorbell_task, &woken);
    }
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void fb_doorbell_init(TaskHandle_t fb_task) {
    fb_doorbell_task = fb_task;
    _DPORT_REG_WRITE(FB_DOORBELL_REG, 0);
    if (esp_intr_alloc(FB_DOORBELL_SRC, ESP_INTR_FLAG_IRAM, fb_doorbell_isr, NULL, NULL) != ESP_OK) {
        printf("# fb_doorbell_init fail\n");
    }
}

void IRAM_ATTR fb_doorbell_ring(void) {
    _DPORT_REG_WRITE(FB_DOORBELL_REG, FB_DOORBELL_BIT);
}