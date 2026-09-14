/*
 * Copyright (c) 2026, BlueRetro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _FB_DOORBELL_H_
#define _FB_DOORBELL_H_

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* Core0 注册门铃 ISR。必须在 fb_task 创建之后、且在 CPU0 上调用。 */
void fb_doorbell_init(TaskHandle_t fb_task);
/* APP CPU ISR 可调：只写 FROM_CPU_3，禁止任何其它 FreeRTOS API。 */
void fb_doorbell_ring(void);

#endif /* _FB_DOORBELL_H_ */