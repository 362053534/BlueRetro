/*
 * Copyright (c) 2019-2024, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "zephyr/types.h"
#include "tools/util.h"
#include "ps3.h"
#include "bluetooth/hidp/ps3.h"

enum {
    PS3_SELECT = 8,
    PS3_L3,
    PS3_R3,
    PS3_START,
    PS3_D_UP,
    PS3_D_RIGHT,
    PS3_D_DOWN,
    PS3_D_LEFT,
    PS3_L2,
    PS3_R2,
    PS3_L1,
    PS3_R1,
    PS3_T,
    PS3_C,
    PS3_X,
    PS3_S,
    PS3_PS,
};

static const uint8_t ps3_axes_idx[ADAPTER_PS2_MAX_AXES] =
{
/*  AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY, TRIG_L, TRIG_R  */
    0,       1,       2,       3,       12,     13,
/*  TRIG_LS, TRIG_RS, DPAD_L,  DPAD_R,  DPAD_D, DPAD_U  */
    14,      15,      11,      9,       10,     8,
/*  BTN_L,   BTN_R,   BTN_D,   BTN_U  */
    19,      17,      18,      16
};

static const struct ctrl_meta ps3_axes_meta[ADAPTER_PS2_MAX_AXES] =
{
    {.neutral = 0x80, .abs_max = 0x7F, .abs_min = 0x80},
    {.neutral = 0x80, .abs_max = 0x7F, .abs_min = 0x80, .polarity = 1},
    {.neutral = 0x80, .abs_max = 0x7F, .abs_min = 0x80},
    {.neutral = 0x80, .abs_max = 0x7F, .abs_min = 0x80, .polarity = 1},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
};

struct ps3_map {
    uint32_t buttons;
    uint8_t reserved;
    uint8_t axes[20];
} __packed;

static const uint32_t ps3_mask[4] = {0xBB7F0FFF, 0x00000000, 0x00000000, 0x00000000};
static const uint32_t ps3_desc[4] = {0x110000FF, 0x00000000, 0x00000000, 0x00000000};
static const uint32_t ps3_pressure_desc[4] = {0x330F0FFF, 0x00000000, 0x00000000, 0x00000000};
static const uint32_t ps3_btns_mask[32] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    BIT(PS3_D_LEFT), BIT(PS3_D_RIGHT), BIT(PS3_D_DOWN), BIT(PS3_D_UP),
    0, 0, 0, 0,
    BIT(PS3_S), BIT(PS3_C), BIT(PS3_X), BIT(PS3_T),
    BIT(PS3_START), BIT(PS3_SELECT), BIT(PS3_PS), 0,
    0, BIT(PS3_L1), 0, BIT(PS3_L3),
    0, BIT(PS3_R1), 0, BIT(PS3_R3),
};

/* hidp_data[0]=0x00，Power 在完整报告偏移 30，对应 input[29] */
#define PS3_BATT_OFF 29
#define PS3_BATT_SAMPLE_MASK 0xFF
#define PS3_BATT_CHARGING 0xEE /* 充电中；已满会变成 0xEF，不会命中 */
#define PS3_BATT_DYING 0x01

static void ps3_batt_sample(struct bt_data *bt_data) {
    uint8_t power;

    if ((bt_data->base.report_cnt & PS3_BATT_SAMPLE_MASK) ||
            bt_data->base.input_len <= PS3_BATT_OFF) {
        return;
    }

    power = bt_data->base.input[PS3_BATT_OFF];
    bt_data->base.batt_level = power;
    /* 只认 0xEE。已满是 0xEF，不会当充电灯 */
    bt_data->base.batt_charging = (power == PS3_BATT_CHARGING);
    bt_data->base.batt_valid = 1;
}

enum {
    PS3_BATT_LED_OFF = 0,
    PS3_BATT_LED_LOW,
    PS3_BATT_LED_CHARGE,
};

static void ps3_set_led_blink_pattern(struct bt_hidp_ps3_set_conf *set_conf, uint8_t leds) {
    uint32_t i;

    set_conf->leds = leds;
    for (i = 0; i < 4; i++) {
        uint8_t *pat = &set_conf->tbd2[i * 5];
        uint8_t bit = 0x10 >> i;

        if (leds & bit) {
            pat[0] = 0xFF;
            pat[1] = 0x27;
            pat[2] = 0x10;
            pat[3] = 0x32;
            pat[4] = 0x32;
        }
    }
}

static void ps3_set_batt_led(struct bt_data *bt_data, uint8_t mode) {
    struct bt_hidp_ps3_set_conf *set_conf =
        (struct bt_hidp_ps3_set_conf *)bt_data->base.output;
    uint32_t idx = bt_data->base.pids->out_idx;

    if (idx >= 7) {
        idx = 0;
    }

    if (mode == PS3_BATT_LED_CHARGE) {
        /* 充电且未满：当前玩家灯慢闪，不做低电四灯闪 */
        ps3_set_led_blink_pattern(set_conf, bt_hid_led_dev_id_map[idx] << 1);
    }
    else if (mode == PS3_BATT_LED_LOW) {
        ps3_set_led_blink_pattern(set_conf, 0x1E);
    }
    else {
        set_conf->leds = 0x00;
    }
}

void ps3_batt_led_poll(struct bt_data *bt_data, uint32_t tick) {
    uint8_t want_low;

    if (!bt_data->base.batt_valid || (tick % 500) != 0) {
        return;
    }

    if (bt_data->base.batt_charging) {
        bt_data->base.batt_low = 0;
        bt_data->base.batt_low_pending = 0;
        ps3_set_batt_led(bt_data, PS3_BATT_LED_CHARGE);
        return;
    }

    want_low = bt_data->base.batt_level == PS3_BATT_DYING;

    if (want_low) {
        if (bt_data->base.batt_low_pending) {
            bt_data->base.batt_low = 1;
        }
        else {
            bt_data->base.batt_low_pending = 1;
        }
    }
    else {
        bt_data->base.batt_low_pending = 0;
        bt_data->base.batt_low = 0;
        ps3_set_batt_led(bt_data, PS3_BATT_LED_OFF);
    }

    if (bt_data->base.batt_low) {
        ps3_set_batt_led(bt_data, PS3_BATT_LED_LOW);
    }
}

int32_t ps3_to_generic(struct bt_data *bt_data, struct wireless_ctrl *ctrl_data) {
    uint32_t i = 8;
    uint32_t axes_cnt = ADAPTER_MAX_AXES;
    struct ps3_map *map = (struct ps3_map *)bt_data->base.input;
    struct ctrl_meta *meta = bt_data->raw_src_mappings[PAD].meta;

#ifdef CONFIG_BLUERETRO_RAW_INPUT
    printf("{\"log_type\": \"wireless_input\", \"report_id\": %ld, \"axes\": [%u, %u, %u, %u, %u, %u], \"btns\": %lu}\n",
        bt_data->base.report_id, map->axes[ps3_axes_idx[0]], map->axes[ps3_axes_idx[1]], map->axes[ps3_axes_idx[2]],
        map->axes[ps3_axes_idx[3]], map->axes[ps3_axes_idx[4]], map->axes[ps3_axes_idx[5]], map->buttons);
#endif

    memset((void *)ctrl_data, 0, sizeof(*ctrl_data));

    ctrl_data->mask = (uint32_t *)ps3_mask;
    ctrl_data->desc = (uint32_t *)ps3_desc;

    if (wired_adapter.system_id == PSX || wired_adapter.system_id == PS2) {
        ctrl_data->desc = (uint32_t *)ps3_pressure_desc;
        axes_cnt = ADAPTER_PS2_MAX_AXES;
        i = 20;
    }

    for (; i < ARRAY_SIZE(generic_btns_mask); i++) {
        if (map->buttons & ps3_btns_mask[i]) {
            ctrl_data->btns[0].value |= generic_btns_mask[i];
        }
    }

    if (!atomic_test_bit(&bt_data->base.flags[PAD], BT_INIT)) {
        memcpy(meta, ps3_axes_meta, sizeof(ps3_axes_meta));
        for (uint32_t i = 0; i < ADAPTER_PS2_MAX_AXES; i++) {
            meta[i].abs_max *= MAX_PULL_BACK;
            meta[i].abs_min *= MAX_PULL_BACK;
            bt_data->base.axes_cal[i] = -(map->axes[ps3_axes_idx[i]] - ps3_axes_meta[i].neutral);
        }
        atomic_set_bit(&bt_data->base.flags[PAD], BT_INIT);
    }

    for (i = 0; i < axes_cnt; i++) {
        ctrl_data->axes[i].meta = &meta[i];
        ctrl_data->axes[i].value = map->axes[ps3_axes_idx[i]] - ps3_axes_meta[i].neutral + bt_data->base.axes_cal[i];
    }

    ps3_batt_sample(bt_data);

    return 0;
}

bool ps3_fb_from_generic(struct generic_fb *fb_data, struct bt_data *bt_data) {
    struct bt_hidp_ps3_set_conf *set_conf = (struct bt_hidp_ps3_set_conf *)bt_data->base.output;
    bool ret = true;

    switch (fb_data->type) {
        case FB_TYPE_RUMBLE:
            if (fb_data->state) {
                /* 对齐 PADEMU：左右 duration 用 0xFE；offset 2 写右电机力度 */
                set_conf->hf_motor_len = 0xFE;
                set_conf->hf_motor_pwr = fb_data->hf_pwr ? 0x01 : 0x00;
                set_conf->lf_motor_len = 0xFE;
                set_conf->lf_motor_pwr = fb_data->lf_pwr;
            }
            else {
                set_conf->hf_motor_len = 0x00;
                set_conf->hf_motor_pwr = 0x00;
                set_conf->lf_motor_len = 0x00;
                set_conf->lf_motor_pwr = 0x00;
            }
            break;
        case FB_TYPE_PLAYER_LED:
            /* 充电未满玩家灯闪 / 低电四灯闪优先；满电或正常保持熄灭 */
            if (!bt_data->base.batt_low && !bt_data->base.batt_charging) {
                ps3_set_batt_led(bt_data, PS3_BATT_LED_OFF);
            }
            break;
    }
    return ret;
}
