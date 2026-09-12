/*
 * Copyright (c) 2019-2024, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_HIDP_PS_H_
#define _BT_HIDP_PS_H_

#include "hidp.h"
#include <stddef.h> /* offsetof for layout static asserts */

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
#define BT_HIDP_PS5_LIGHTBAR_SETUP_CONTROL 0x02 /* valid_flag2 bit1：允许改 lightbar_setup */
#define BT_HIDP_PS5_LIGHTBAR_SETUP_LIGHT_OUT 0x02 /* lightbar_setup bit1：灯条淡出 */
#define BT_HIDP_PS5_PLAYER_LED_INSTANT 0x20 /* player_leds bit5：立即生效，不淡出 */
#define PS5_LED_OFF_RETRY 8
#define PS5_LED_ANIM_WAIT_US 4000000 /* 首包 0x31 起 4s，略长于 SDL 10200000/3≈3.4s */
#define BT_HIDP_PS5_COMPATIBLE_VIBRATION 0x01 /* valid_flag0 bit0：v1 兼容震动 */
#define BT_HIDP_PS5_HAPTICS_SELECT 0x02
#define BT_HIDP_PS5_RUMBLE_IMPROVED 0x04
#define BT_HIDP_PS5_HAPTIC_LOW_PASS_FILTER_CONTROL 0x20 /* valid_flag1 bit5：允许改 haptic LPF */
#define BT_HIDP_PS5_HAPTICS_FLAG_LOW_PASS_FILTER 0x01   /* common p39 bit0：1=开 LPF */
#define BT_HIDP_PS5_VIBRATION_ATTENUATION_ENABLE 0x40
/* 最大衰减档数：8=7..0，6=5..0，0或1=仅 0。HID 低 3 位最多 7，超过 8 按 8。 */
#define PS5_LF_ATTEN_GEARS 0
#if PS5_LF_ATTEN_GEARS <= 1
#define PS5_LF_ATTEN_MAX 0
#elif PS5_LF_ATTEN_GEARS >= 8
#define PS5_LF_ATTEN_MAX 7
#else
#define PS5_LF_ATTEN_MAX (PS5_LF_ATTEN_GEARS - 1)
#endif
/* 大电机衰减步长：最高档→0 从死区外第一点起算，作用区间 = 步长 * 档数。0=全程档 0。 */
#define PS5_LF_ATTEN_STEP 32
/* DS5 大电机死区：0-255，lf<=此值固定死区档再叠系数，外侧不含。可与步长对齐。0=仅 lf=0。 */
#define PS5_LF_MOTOR_DEADZONE 31
/* 死区固定衰减档：0-7，与 PS5_LF_ATTEN_GEARS 无关。 */
#define PS5_LF_DEADZONE_GEAR 7
/* 死区额外衰减系数：0=无震；>0 为除数，越大越弱，商<1 则发 1。 */
#define PS5_LF_DEADZONE_COEFF 31
/* 1=允许小电机；0=临时关掉，只测大电机。 */
#define PS5_HF_MOTOR_ENABLE 0

/* 1=v2 改进震动（惯性、软起停）；0=v1 经典（尖锐）。两者互斥，不要同时开。 */
#define BT_HIDP_PS5_USE_RUMBLE_V2 0
#if BT_HIDP_PS5_USE_RUMBLE_V2
#define BT_HIDP_PS5_VALID_FLAG0_RUMBLE BT_HIDP_PS5_HAPTICS_SELECT
#define BT_HIDP_PS5_VALID_FLAG2_RUMBLE BT_HIDP_PS5_RUMBLE_IMPROVED
#else
#define BT_HIDP_PS5_VALID_FLAG0_RUMBLE (BT_HIDP_PS5_HAPTICS_SELECT | BT_HIDP_PS5_COMPATIBLE_VIBRATION)
#define BT_HIDP_PS5_VALID_FLAG2_RUMBLE 0x00
#endif

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
    uint8_t tbd0[4];                 /* headphone/speaker/mic volume + audio control (common p4-p7) */
    uint8_t mic_led;                 /* mute button LED (common p8) */
    uint8_t tbd1;                    /* power save control (common p9) */
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
    uint8_t l2_tbd3;                 /* left trigger param[9] (common p31), mirrors right-side tbd3 */
    uint8_t reserved2[4];            /* common p32-p35 */
    uint8_t reduce_motor_power;      /* common p36 */
    uint8_t audio_control2;          /* common p37 */
    uint8_t valid_flag2;             /* common p38 */
    uint8_t haptics_flags;           /* common p39：bit0 haptic LPF，1=开 */
    uint8_t reserved3;               /* common p40 */
    uint8_t lightbar_setup;          /* common p41 */
    uint8_t led_brightness;          /* common p42 */
    uint8_t player_leds;             /* common p43 */
    uint8_t rgb[3];                  /* common p44-p46 */
    uint8_t tbd7[24];
    uint32_t crc;
} __packed;

struct bt_hidp_ps5_set_conf {
    uint8_t conf0;
    uint8_t valid_flag0;
    uint8_t valid_flag1;
    uint8_t hf_motor_pwr;
    uint8_t lf_motor_pwr;
    uint8_t tbd0[4];
    uint8_t mic_led;
    uint8_t tbd1;                    /* mic/audio mute (common p9) */
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
    uint8_t l2_tbd3;                 /* left trigger param[9] (common p31) */
    uint8_t reserved2[4];            /* common p32-p35 */
    uint8_t reduce_motor_power;      /* common p36: bits0-2 rumble atten, bits4-6 trigger atten; needs valid_flag1 bit6 */
    uint8_t audio_control2;          /* common p37 */
    uint8_t valid_flag2;             /* common p38, 0x04 = improved rumble */
    uint8_t haptics_flags;           /* common p39：bit0 haptic LPF，1=开 */
    uint8_t reserved3;               /* common p40 */
    uint8_t lightbar_setup;          /* common p41 */
    uint8_t led_brightness;          /* common p42 */
    uint8_t player_leds;             /* common p43 */
    uint32_t leds;                   /* packed RGB, mapped to bt rgb[3] (common p44-p46) */
    uint8_t tbd8[24];
    uint32_t crc;
} __packed;

void bt_hid_cmd_ps_set_conf(struct bt_dev *device, void *report);
void bt_hid_ps5_clear_led(struct bt_dev *device);
int bt_hid_ps5_led_ready(struct bt_data *bt_data);
void bt_hid_ps_init(struct bt_dev *device);
void bt_hid_ps_hdlr(struct bt_dev *device, struct bt_hci_pkt *bt_hci_acl_pkt, uint32_t len);


/* DS5 output report layout locks: struct offset = common index + 2 (bt) / + 1 (usb buffer) */
_Static_assert(sizeof(struct bt_hidp_ps5_bt_set_conf) == 77, "ps5 bt output must be 77 bytes");
_Static_assert(offsetof(struct bt_hidp_ps5_bt_set_conf, valid_flag0) == 2, "");
_Static_assert(offsetof(struct bt_hidp_ps5_bt_set_conf, lf_motor_pwr) == 5, "");
_Static_assert(offsetof(struct bt_hidp_ps5_bt_set_conf, l2_tbd3) == 33, "");
_Static_assert(offsetof(struct bt_hidp_ps5_bt_set_conf, reduce_motor_power) == 38, "");
_Static_assert(offsetof(struct bt_hidp_ps5_bt_set_conf, audio_control2) == 39, "");
_Static_assert(offsetof(struct bt_hidp_ps5_bt_set_conf, valid_flag2) == 40, "");
_Static_assert(offsetof(struct bt_hidp_ps5_bt_set_conf, haptics_flags) == 41, "");
_Static_assert(offsetof(struct bt_hidp_ps5_bt_set_conf, lightbar_setup) == 43, "");
_Static_assert(offsetof(struct bt_hidp_ps5_bt_set_conf, player_leds) == 45, "");
_Static_assert(offsetof(struct bt_hidp_ps5_bt_set_conf, rgb) == 46, "");
_Static_assert(offsetof(struct bt_hidp_ps5_bt_set_conf, crc) == 73, "");
_Static_assert(sizeof(struct bt_hidp_ps5_set_conf) == 77, "ps5 usb buffer must be 77 bytes");
_Static_assert(offsetof(struct bt_hidp_ps5_set_conf, valid_flag0) == 1, "");
_Static_assert(offsetof(struct bt_hidp_ps5_set_conf, l2_tbd3) == 32, "");
_Static_assert(offsetof(struct bt_hidp_ps5_set_conf, reduce_motor_power) == 37, "");
_Static_assert(offsetof(struct bt_hidp_ps5_set_conf, audio_control2) == 38, "");
_Static_assert(offsetof(struct bt_hidp_ps5_set_conf, valid_flag2) == 39, "");
_Static_assert(offsetof(struct bt_hidp_ps5_set_conf, haptics_flags) == 40, "");
_Static_assert(offsetof(struct bt_hidp_ps5_set_conf, lightbar_setup) == 42, "");
_Static_assert(offsetof(struct bt_hidp_ps5_set_conf, player_leds) == 44, "");
_Static_assert(offsetof(struct bt_hidp_ps5_set_conf, leds) == 45, "");
_Static_assert(offsetof(struct bt_hidp_ps5_set_conf, crc) == 73, "");
_Static_assert((offsetof(struct bt_hidp_ps5_bt_set_conf, reserved2) - offsetof(struct bt_hidp_ps5_bt_set_conf, r2_trigger_motor_mode)) == (offsetof(struct bt_hidp_ps5_set_conf, reserved2) - offsetof(struct bt_hidp_ps5_set_conf, r2_trigger_motor_mode)), "trigger region length mismatch");

#endif /* _BT_HIDP_PS_H_ */
