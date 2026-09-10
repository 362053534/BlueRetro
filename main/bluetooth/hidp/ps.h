/*
 * Copyright (c) 2019-2024, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_HIDP_PS_H_
#define _BT_HIDP_PS_H_

#include "hidp.h"

#define BT_HIDP_HID_STATUS 0x01
#define BT_HIDP_PS4_STATUS 0x11
#define BT_HIDP_PS5_STATUS 0x31
struct bt_hidp_ps4_status {
    uint8_t data[77];
} __packed;

#define BT_HIDP_PS4_SET_CONF 0x11
struct bt_hidp_ps4_set_conf {
    uint8_t conf0;
    uint8_t tbd0;
    uint8_t conf1;
    uint8_t tbd1[2];
    uint8_t hf_motor_pwr;
    uint8_t lf_motor_pwr;
    union {
        struct {
            uint8_t rgb[3];
            uint8_t led_on_delay;
        };
        uint32_t leds;
    };
    uint8_t led_off_delay;
    uint8_t tbd2[61];
    uint32_t crc;
} __packed;

#define BT_HIDP_PS5_SET_CONF 0x31
#define BT_HIDP_PS5_LED_LIGHTBAR_CONTROL 0x04
#define BT_HIDP_PS5_LED_RELEASE 0x08
#define BT_HIDP_PS5_LED_PLAYER_CONTROL 0x10
#define BT_HIDP_PS5_HAPTICS_SELECT 0x02
#define BT_HIDP_PS5_RUMBLE_IMPROVED 0x04
#define BT_HIDP_PS5_VIBRATION_ATTENUATION_ENABLE 0x40

/*
 * 现有输出缓存仍使用 USB 风格布局，发送蓝牙时需要转换为真正的蓝牙布局。
 * 蓝牙 HIDP 的 0x31 已经作为报告 ID 放在 HIDP 协议字段中，因此这里不再重复放报告 ID。
 */
struct bt_hidp_ps5_bt_set_conf {
    uint8_t seq_tag;
    uint8_t tag;
    uint8_t valid_flag0;
    uint8_t valid_flag1;
    uint8_t hf_motor_pwr;
    uint8_t lf_motor_pwr;
    uint8_t tbd0[4];
    uint8_t mic_led;
    uint8_t tbd1;
    uint8_t r2_trigger_motor_mode;
    uint8_t r2_trigger_start_resistance;
    uint8_t r2_trigger_effect_force;
    uint8_t r2_trigger_range_force;
    uint8_t r2_trigger_near_release_str;
    uint8_t r2_trigger_near_middle_str;
    uint8_t r2_trigger_pressed_str;
    uint8_t tbd2[2];
    uint8_t r2_trigger_actuation_freq;
    uint8_t tbd3;
    uint8_t l2_trigger_motor_mode;
    uint8_t l2_trigger_start_resistance;
    uint8_t l2_trigger_effect_force;
    uint8_t l2_trigger_range_force;
    uint8_t l2_trigger_near_release_str;
    uint8_t l2_trigger_near_middle_str;
    uint8_t l2_trigger_pressed_str;
    uint8_t tbd4[2];
    uint8_t l2_trigger_actuation_freq;
    /* 左触发器末尾的保留字节加上后续的六个保留字节 */
    uint8_t tbd5[7];
    uint8_t valid_flag2;
    uint8_t tbd6[2];
    uint8_t lightbar_setup;
    uint8_t led_brightness;
    uint8_t player_leds;
    uint8_t rgb[3];
    uint8_t tbd7[24];
    uint32_t crc;
} __packed;

struct bt_hidp_ps5_set_conf {
    uint8_t conf0;
    uint8_t cmd;
    uint8_t conf1;
    uint8_t hf_motor_pwr;
    uint8_t lf_motor_pwr;
    uint8_t tbd0[4];
    uint8_t mic_led;
    uint8_t tbd1; // Mic/audio mute
    uint8_t r2_trigger_motor_mode;
    uint8_t r2_trigger_start_resistance;
    uint8_t r2_trigger_effect_force;
    uint8_t r2_trigger_range_force;
    uint8_t r2_trigger_near_release_str;
    uint8_t r2_trigger_near_middle_str;
    uint8_t r2_trigger_pressed_str;
    uint8_t tbd2[2];
    uint8_t r2_trigger_actuation_freq;
    uint8_t tbd3;
    uint8_t l2_trigger_motor_mode;
    uint8_t l2_trigger_start_resistance;
    uint8_t l2_trigger_effect_force;
    uint8_t l2_trigger_range_force;
    uint8_t l2_trigger_near_release_str;
    uint8_t l2_trigger_near_middle_str;
    uint8_t l2_trigger_pressed_str;
    uint8_t tbd4[2];
    uint8_t l2_trigger_actuation_freq;
    uint8_t tbd5[5];
    uint8_t reduce_motor_power; /* common p36: bits0-2 rumble atten 0-7, bits4-6 trigger atten; needs valid_flag1 bit6 (0x40) */
    uint8_t valid_flag2; /* 第三组输出有效位，0x04为改进震动模式 */
    uint8_t use_accurate_rumble;
    uint8_t tbd7[4];
    uint8_t player_leds;
    uint32_t leds;
    uint8_t tbd8[24];
    uint32_t crc;
} __packed;

void bt_hid_cmd_ps_set_conf(struct bt_dev *device, void *report);
void bt_hid_ps5_clear_led(struct bt_dev *device);
void bt_hid_ps_init(struct bt_dev *device);
void bt_hid_ps_hdlr(struct bt_dev *device, struct bt_hci_pkt *bt_hci_acl_pkt, uint32_t len);

#endif /* _BT_HIDP_PS_H_ */
