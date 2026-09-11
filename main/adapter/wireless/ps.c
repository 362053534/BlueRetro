/*
 * Copyright (c) 2019-2024, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "zephyr/types.h"
#include "tools/util.h"
#include "bluetooth/hidp/ps.h"
#include "adapter/config.h"
#include "ps.h"

/* DS5 对 PS2 二值小电机恢复满幅输出，用于单独测试小电机震感。 */
#define PS5_BINARY_HF_MOTOR_PWR 0xFF

enum {
    PS4_S = 4,
    PS4_X,
    PS4_C,
    PS4_T,
    PS4_L1,
    PS4_R1,
    PS4_L2,
    PS4_R2,
    PS4_SHARE,
    PS4_OPTIONS,
    PS4_L3,
    PS4_R3,
    PS4_PS,
    PS4_TP,
    PS5_MUTE,
};

static const uint8_t ps4_axes_idx[ADAPTER_MAX_AXES] =
{
/*  AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY, TRIG_L, TRIG_R  */
    0,       1,       2,       3,       7,      8
};

static const uint8_t ps5_axes_idx[ADAPTER_MAX_AXES] =
{
/*  AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY, TRIG_L, TRIG_R  */
    0,       1,       2,       3,       4,      5
};

static const struct ctrl_meta ps4_axes_meta[ADAPTER_MAX_AXES] =
{
    {.neutral = 0x80, .abs_max = 0x7F, .abs_min = 0x80},
    {.neutral = 0x80, .abs_max = 0x7F, .abs_min = 0x80, .polarity = 1},
    {.neutral = 0x80, .abs_max = 0x7F, .abs_min = 0x80},
    {.neutral = 0x80, .abs_max = 0x7F, .abs_min = 0x80, .polarity = 1},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
    {.neutral = 0x00, .abs_max = 0xFF, .abs_min = 0x00},
};

struct hid_map {
    union {
        struct {
            uint8_t reserved2[4];
            union {
                uint8_t hat;
                uint32_t buttons;
            };
        };
        uint8_t axes[9];
    };
} __packed;

struct ps4_map {
    uint8_t reserved[2];
    union {
        struct {
            uint8_t reserved2[4];
            union {
                uint8_t hat;
                uint32_t buttons;
            };
        };
        uint8_t axes[9];
    };
} __packed;

struct ps5_map {
    uint8_t reserved;
    uint8_t axes[6];
    uint8_t reserved2;
    union {
        uint8_t hat;
        uint32_t buttons;
    };
} __packed;

static const uint32_t ps4_mask[4] = {0xBBFF0FFF, 0x00000000, 0x00000000, 0x00000000};
static const uint32_t ps4_desc[4] = {0x110000FF, 0x00000000, 0x00000000, 0x00000000};
static const uint32_t ps4_btns_mask[32] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    BIT(PS4_S), BIT(PS4_C), BIT(PS4_X), BIT(PS4_T),
    BIT(PS4_OPTIONS), BIT(PS4_SHARE), BIT(PS4_PS), BIT(PS4_TP),
    0, BIT(PS4_L1), 0, BIT(PS4_L3),
    0, BIT(PS4_R1), 0, BIT(PS4_R3),
};

/* BT 0x11：common.status[0] 在 hidp_data[31] */
#define PS4_BT_BATT_OFF 31
/* BT 0x31：common.status[0] 在 hidp_data[53] */
#define PS5_BT_BATT_OFF 53
#define PS_BATT_SAMPLE_MASK 0xFF
#define PS4_BATT_CABLE BIT(4)
#define PS5_BATT_CHARGE_SHIFT 4
#define PS5_BATT_CHARGING 0x1
#define PS_BATT_LEVEL_FULL 10 /* 0-10 对应 0-100% */

static void ps_batt_sample(struct bt_data *bt_data, uint32_t off) {
    uint8_t status, level, charging;

    if ((bt_data->base.report_cnt & PS_BATT_SAMPLE_MASK) ||
            bt_data->base.input_len <= off) {
        return;
    }

    status = bt_data->base.input[off];
    level = status & 0x0F;
    if (bt_data->base.report_id == 0x31) {
        /* 高半字节 1=充电中，2=已充满。只认 1，满电自然不会亮绿灯 */
        charging = (status >> PS5_BATT_CHARGE_SHIFT) == PS5_BATT_CHARGING;
    }
    else {
        /* 插电且容量 nibble < 10 才算充电中；10=100%，11+=插电满电 */
        charging = (status & PS4_BATT_CABLE) && (level < PS_BATT_LEVEL_FULL);
    }

    bt_data->base.batt_level = level;
    bt_data->base.batt_charging = charging;
    bt_data->base.batt_valid = 1;
}

enum {
    PS_BATT_LED_OFF = 0,
    PS_BATT_LED_LOW,
    PS_BATT_LED_CHARGE,
};

static void ps4_set_batt_led(struct bt_hidp_ps4_set_conf *set_conf, uint8_t mode) {
    if (mode == PS_BATT_LED_CHARGE) {
        /* 充电：绿灯常亮，不开硬件闪 */
        set_conf->conf1 = 0x03;
        set_conf->rgb[0] = 0x00;
        set_conf->rgb[1] = 0xFF;
        set_conf->rgb[2] = 0x00;
        set_conf->led_on_delay = 0;
        set_conf->led_off_delay = 0;
    }
    else if (mode == PS_BATT_LED_LOW) {
        /* 低电：红灯硬件慢闪 */
        set_conf->conf1 = 0x07;
        set_conf->rgb[0] = 0xFF;
        set_conf->rgb[1] = 0x00;
        set_conf->rgb[2] = 0x00;
        set_conf->led_on_delay = 0x80;
        set_conf->led_off_delay = 0x80;
    }
    else {
        set_conf->conf1 = 0x03;
        set_conf->leds = 0;
        set_conf->led_off_delay = 0;
    }
}

static void ps5_set_batt_led(struct bt_hidp_ps5_set_conf *set_conf, uint8_t mode) {
    if (mode == PS_BATT_LED_CHARGE) {
        set_conf->leds = 0x0000FF00; /* G */
    }
    else if (mode == PS_BATT_LED_LOW) {
        set_conf->leds = 0x000000FF; /* R */
    }
    else {
        set_conf->leds = 0;
    }
}

static void ps_apply_batt_led(struct bt_data *bt_data, uint8_t mode) {
    if (bt_data->base.pids->subtype == BT_PS5_DS) {
        ps5_set_batt_led((struct bt_hidp_ps5_set_conf *)bt_data->base.output, mode);
    }
    else {
        ps4_set_batt_led((struct bt_hidp_ps4_set_conf *)bt_data->base.output, mode);
    }
}

static uint8_t ps_batt_is_low(struct bt_data *bt_data) {
    return bt_data->base.batt_valid &&
        !bt_data->base.batt_charging &&
        bt_data->base.batt_level == 0;
}

void ps_batt_led_poll(struct bt_data *bt_data, uint32_t tick) {
    uint8_t want_low;

    if (!bt_data->base.batt_valid) {
        return;
    }

    /* 充电且未满：绿灯常亮。插电已满则走熄灯，不做低电红闪 */
    if (bt_data->base.batt_charging) {
        if ((tick % 500) == 0) {
            bt_data->base.batt_low = 0;
            bt_data->base.batt_low_pending = 0;
            bt_data->base.batt_ds5_on = 0;
            ps_apply_batt_led(bt_data, PS_BATT_LED_CHARGE);
        }
        return;
    }

    want_low = ps_batt_is_low(bt_data);

    if ((tick % 500) == 0) {
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
            bt_data->base.batt_ds5_on = 0;
            ps_apply_batt_led(bt_data, PS_BATT_LED_OFF);
        }

        if (bt_data->base.batt_low) {
            bt_data->base.batt_ds5_on = 1;
            ps_apply_batt_led(bt_data, PS_BATT_LED_LOW);
        }
    }
    else if ((tick % 100) == 0 &&
            bt_data->base.batt_low &&
            bt_data->base.pids->subtype == BT_PS5_DS) {
        bt_data->base.batt_ds5_on ^= 1;
        ps5_set_batt_led((struct bt_hidp_ps5_set_conf *)bt_data->base.output,
            bt_data->base.batt_ds5_on ? PS_BATT_LED_LOW : PS_BATT_LED_OFF);
    }
}

static void ps4_to_generic(struct bt_data *bt_data, struct wireless_ctrl *ctrl_data) {
    struct ps4_map *map = (struct ps4_map *)bt_data->base.input;
    struct ctrl_meta *meta = bt_data->raw_src_mappings[PAD].meta;

#ifdef CONFIG_BLUERETRO_RAW_INPUT
    printf("{\"log_type\": \"wireless_input\", \"report_id\": %ld, \"axes\": [%u, %u, %u, %u, %u, %u], \"btns\": %lu, \"hat\": %u}\n",
        bt_data->base.report_id, map->axes[ps4_axes_idx[0]], map->axes[ps4_axes_idx[1]], map->axes[ps4_axes_idx[2]],
        map->axes[ps4_axes_idx[3]], map->axes[ps4_axes_idx[4]], map->axes[ps4_axes_idx[5]], map->buttons, map->hat & 0xF);
#endif

    memset((void *)ctrl_data, 0, sizeof(*ctrl_data));

    ctrl_data->mask = (uint32_t *)ps4_mask;
    ctrl_data->desc = (uint32_t *)ps4_desc;

    for (uint32_t i = 0; i < ARRAY_SIZE(generic_btns_mask); i++) {
        if (map->buttons & ps4_btns_mask[i]) {
            ctrl_data->btns[0].value |= generic_btns_mask[i];
        }
    }

    /* Convert hat to regular btns */
    ctrl_data->btns[0].value |= hat_to_ld_btns[map->hat & 0xF];

    if (!atomic_test_bit(&bt_data->base.flags[PAD], BT_INIT)) {
        memcpy(meta, ps4_axes_meta, sizeof(ps4_axes_meta));
        for (uint32_t i = 0; i < ADAPTER_MAX_AXES; i++) {
            meta[i].abs_max *= MAX_PULL_BACK;
            meta[i].abs_min *= MAX_PULL_BACK;
            bt_data->base.axes_cal[i] = -(map->axes[ps4_axes_idx[i]] - ps4_axes_meta[i].neutral);
        }
        atomic_set_bit(&bt_data->base.flags[PAD], BT_INIT);
    }

    for (uint32_t i = 0; i < ADAPTER_MAX_AXES; i++) {
        ctrl_data->axes[i].meta = &meta[i];
        ctrl_data->axes[i].value = map->axes[ps4_axes_idx[i]] - ps4_axes_meta[i].neutral + bt_data->base.axes_cal[i];
    }

    ps_batt_sample(bt_data, PS4_BT_BATT_OFF);
}

static void ps5_to_generic(struct bt_data *bt_data, struct wireless_ctrl *ctrl_data) {
    struct ps5_map *map = (struct ps5_map *)bt_data->base.input;
    struct ctrl_meta *meta = bt_data->raw_src_mappings[PAD].meta;

#ifdef CONFIG_BLUERETRO_RAW_INPUT
    printf("{\"log_type\": \"wireless_input\", \"report_id\": %ld, \"axes\": [%u, %u, %u, %u, %u, %u], \"btns\": %lu, \"hat\": %u}\n",
        bt_data->base.report_id, map->axes[ps5_axes_idx[0]], map->axes[ps5_axes_idx[1]], map->axes[ps5_axes_idx[2]],
        map->axes[ps5_axes_idx[3]], map->axes[ps5_axes_idx[4]], map->axes[ps5_axes_idx[5]], map->buttons, map->hat & 0xF);
#endif

    memset((void *)ctrl_data, 0, sizeof(*ctrl_data));

    ctrl_data->mask = (uint32_t *)ps4_mask;
    ctrl_data->desc = (uint32_t *)ps4_desc;

    for (uint32_t i = 0; i < ARRAY_SIZE(generic_btns_mask); i++) {
        if (map->buttons & ps4_btns_mask[i]) {
            ctrl_data->btns[0].value |= generic_btns_mask[i];
        }
    }

    /* Convert hat to regular btns */
    ctrl_data->btns[0].value |= hat_to_ld_btns[map->hat & 0xF];

    if (!atomic_test_bit(&bt_data->base.flags[PAD], BT_INIT)) {
        memcpy(meta, ps4_axes_meta, sizeof(ps4_axes_meta));
        for (uint32_t i = 0; i < ADAPTER_MAX_AXES; i++) {
            meta[i].abs_max *= MAX_PULL_BACK;
            meta[i].abs_min *= MAX_PULL_BACK;
            bt_data->base.axes_cal[i] = -(map->axes[ps5_axes_idx[i]] - ps4_axes_meta[i].neutral);
        }
        atomic_set_bit(&bt_data->base.flags[PAD], BT_INIT);
    }

    for (uint32_t i = 0; i < ADAPTER_MAX_AXES; i++) {
        ctrl_data->axes[i].meta = &meta[i];
        ctrl_data->axes[i].value = map->axes[ps5_axes_idx[i]] - ps4_axes_meta[i].neutral + bt_data->base.axes_cal[i];
    }

    ps_batt_sample(bt_data, PS5_BT_BATT_OFF);
}

static void hid_to_generic(struct bt_data *bt_data, struct wireless_ctrl *ctrl_data) {
    struct hid_map *map = (struct hid_map *)bt_data->base.input;
    struct ctrl_meta *meta = bt_data->raw_src_mappings[PAD].meta;

#ifdef CONFIG_BLUERETRO_RAW_INPUT
    printf("{\"log_type\": \"wireless_input\", \"report_id\": %ld, \"axes\": [%u, %u, %u, %u, %u, %u], \"btns\": %lu, \"hat\": %u}\n",
        bt_data->base.report_id, map->axes[ps4_axes_idx[0]], map->axes[ps4_axes_idx[1]], map->axes[ps4_axes_idx[2]],
        map->axes[ps4_axes_idx[3]], map->axes[ps4_axes_idx[4]], map->axes[ps4_axes_idx[5]], map->buttons, map->hat & 0xF);
#endif

    memset((void *)ctrl_data, 0, sizeof(*ctrl_data));

    ctrl_data->mask = (uint32_t *)ps4_mask;
    ctrl_data->desc = (uint32_t *)ps4_desc;

    for (uint32_t i = 0; i < ARRAY_SIZE(generic_btns_mask); i++) {
        if (map->buttons & ps4_btns_mask[i]) {
            ctrl_data->btns[0].value |= generic_btns_mask[i];
        }
    }

    /* Convert hat to regular btns */
    ctrl_data->btns[0].value |= hat_to_ld_btns[map->hat & 0xF];

    if (!atomic_test_bit(&bt_data->base.flags[PAD], BT_INIT)) {
        memcpy(meta, ps4_axes_meta, sizeof(ps4_axes_meta));
        for (uint32_t i = 0; i < ADAPTER_MAX_AXES; i++) {
            meta[i].abs_max *= MAX_PULL_BACK;
            meta[i].abs_min *= MAX_PULL_BACK;
            bt_data->base.axes_cal[i] = -(map->axes[ps4_axes_idx[i]] - ps4_axes_meta[i].neutral);
        }
        atomic_set_bit(&bt_data->base.flags[PAD], BT_INIT);
    }

    for (uint32_t i = 0; i < ADAPTER_MAX_AXES; i++) {
        ctrl_data->axes[i].meta = &meta[i];
        ctrl_data->axes[i].value = map->axes[ps4_axes_idx[i]] - ps4_axes_meta[i].neutral + bt_data->base.axes_cal[i];
    }
}

static void ps4_fb_from_generic(struct generic_fb *fb_data, struct bt_data *bt_data) {
    struct bt_hidp_ps4_set_conf *set_conf = (struct bt_hidp_ps4_set_conf *)bt_data->base.output;

    switch (fb_data->type) {
        case FB_TYPE_RUMBLE:
            if (fb_data->state) {
                set_conf->hf_motor_pwr = fb_data->hf_pwr;
                set_conf->lf_motor_pwr = fb_data->lf_pwr;
            }
            else {
                set_conf->hf_motor_pwr = 0x00;
                set_conf->lf_motor_pwr = 0x00;
            }
            break;
        case FB_TYPE_PLAYER_LED:
            /* 充电未满绿灯 / 低电红闪优先；满电或正常保持灯条熄灭 */
            if (!bt_data->base.batt_low && !bt_data->base.batt_charging) {
                ps4_set_batt_led(set_conf, PS_BATT_LED_OFF);
            }
            break;
    }
}

/* Large (low-freq) motor magnitude -> attenuation step, one step every 8 counts:
 * weak -> strongest (7); reaches 0 at lf=56 and is clamped at 0 across 56..255 (no unsigned underflow). */
static inline uint8_t ps5_lf_atten(uint32_t lf) {
    uint32_t step = lf >> 3;
    return step >= 7 ? 0 : (uint8_t)(7 - step);
}

static void ps5_fb_from_generic(struct generic_fb *fb_data, struct bt_data *bt_data) {
    struct bt_hidp_ps5_set_conf *set_conf = (struct bt_hidp_ps5_set_conf *)bt_data->base.output;

    switch (fb_data->type) {
        case FB_TYPE_RUMBLE:
            if (fb_data->state) {
                /* Enable DS5 vibration attenuation: reduce_motor_power low nibble = main-motor atten 0..7 (7 = weakest). */
                /* Large (low-freq) motor keeps the raw PS2 magnitude. */
                /* 开/停都显式关 haptic LPF，防止固件默认滤波吃掉短脉冲。 */
                set_conf->valid_flag1 |= BT_HIDP_PS5_VIBRATION_ATTENUATION_ENABLE
                    | BT_HIDP_PS5_HAPTIC_LOW_PASS_FILTER_CONTROL;
                set_conf->haptics_flags = 0;
                /* Small (hf) motor active -> force atten 0 so it is never attenuated; only when the large
                 * motor alone rumbles do we apply the 8-step dynamic atten. Trigger (high) nibble stays 0. */
                set_conf->reduce_motor_power = (fb_data->hf_pwr != 0) ? 0 : ps5_lf_atten(fb_data->lf_pwr);
                set_conf->hf_motor_pwr = (fb_data->hf_pwr == 0xFF) ?
                    PS5_BINARY_HF_MOTOR_PWR : fb_data->hf_pwr;
                set_conf->lf_motor_pwr = 0x00; /* TEST ONLY: large motor OFF to isolate small (hf) motor; restore: fb_data->lf_pwr */
            }
            else {
                /* Keep attenuation enabled on stop as well, so no controller default leaks through. */
                set_conf->valid_flag1 |= BT_HIDP_PS5_VIBRATION_ATTENUATION_ENABLE
                    | BT_HIDP_PS5_HAPTIC_LOW_PASS_FILTER_CONTROL;
                set_conf->haptics_flags = 0;
                /* low nibble = large-motor atten step from ps5_lf_atten(); trigger (high) nibble stays 0 */
                set_conf->reduce_motor_power = ps5_lf_atten(fb_data->lf_pwr);
                set_conf->hf_motor_pwr = 0x00;
                set_conf->lf_motor_pwr = 0x00;
            }
            break;
        case FB_TYPE_PLAYER_LED:
            /* 充电未满绿灯 / 低电红闪优先；满电或正常保持灯条熄灭 */
            set_conf->valid_flag1 |= BT_HIDP_PS5_LED_PLAYER_CONTROL;
            set_conf->player_leds = 0;
            if (!bt_data->base.batt_low && !bt_data->base.batt_charging) {
                ps5_set_batt_led(set_conf, PS_BATT_LED_OFF);
            }
            break;
    }
}

int32_t ps_to_generic(struct bt_data *bt_data, struct wireless_ctrl *ctrl_data) {
    switch (bt_data->base.report_id) {
        case 0x01:
            hid_to_generic(bt_data, ctrl_data);
            break;
        case 0x11:
            ps4_to_generic(bt_data, ctrl_data);
            break;
        case 0x31:
            ps5_to_generic(bt_data, ctrl_data);
            break;
        default:
            printf("# Unknown report type: %02lX\n", bt_data->base.report_type);
            return -1;
    }

    return 0;
}

bool ps_fb_from_generic(struct generic_fb *fb_data, struct bt_data *bt_data) {
    bool ret = true;
    switch (bt_data->base.pids->subtype) {
        case BT_PS5_DS:
            ps5_fb_from_generic(fb_data, bt_data);
            break;
        default:
            ps4_fb_from_generic(fb_data, bt_data);
            break;
    }
    return ret;
}
