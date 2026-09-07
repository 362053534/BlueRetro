/*
 * Copyright (c) 2019-2025, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _PS3_H_
#define _PS3_H_
#include "adapter/adapter.h"

int32_t ps3_to_generic(struct bt_data *bt_data, struct wireless_ctrl *ctrl_data);
bool ps3_fb_from_generic(struct generic_fb *fb_data, struct bt_data *bt_data);
void ps3_batt_led_poll(struct bt_data *bt_data, uint32_t tick);

#endif /* _PS3_H_ */
