/*
 * Copyright (c) 2019-2024, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stddef.h>
#include <esp32/rom/crc.h>
#include <esp_timer.h>
#include "adapter/adapter.h"
#include "adapter/config.h"
#include "bluetooth/host.h"
#include "ps.h"

#define PS5_BT_OUTPUT_TAG 0x10

static uint8_t ps5_bt_output_seq[BT_MAX_DEV];

static void bt_hid_cmd_ps5_set_conf(struct bt_dev *device, void *report);

static void bt_hid_cmd_ps4_set_conf(struct bt_dev *device, void *report) {
    struct bt_hidp_ps4_set_conf *set_conf = (struct bt_hidp_ps4_set_conf *)bt_hci_pkt_tmp.hidp_data;

    bt_hci_pkt_tmp.hidp_hdr.hdr = BT_HIDP_DATA_OUT;
    bt_hci_pkt_tmp.hidp_hdr.protocol = BT_HIDP_PS4_SET_CONF;

    memcpy((void *)set_conf, report, sizeof(*set_conf));

    set_conf->crc = crc32_le((uint32_t)~0xFFFFFFFF, (void *)&bt_hci_pkt_tmp.hidp_hdr,
        sizeof(bt_hci_pkt_tmp.hidp_hdr) + sizeof(*set_conf) - sizeof(set_conf->crc));

    bt_hid_cmd(device->acl_handle, device->intr_chan.dcid, BT_HIDP_DATA_OUT, BT_HIDP_PS4_SET_CONF, sizeof(*set_conf));
}

static void bt_hid_cmd_ps5_trigger_init(struct bt_dev *device) {
    int32_t perc_threshold_l = -1;
    int32_t perc_threshold_r = -1;
    int32_t dev = device->ids.id;
    uint32_t map_cnt_l = 0;
    uint32_t map_cnt_r = 0;

    if (wired_adapter.system_id == WIRED_AUTO) {
        /* Can't configure feature if target system is unknown */
        return;
    }

    printf("# %s\n", __FUNCTION__);

    /* Make sure meta desc is init */
    adapter_meta_init();

    /* Go through the list of mappings, looking for PAD_RM and PAD_LM */
    for (uint32_t i = 0; i < config.in_cfg[dev].map_size; i++) {
        if (config.in_cfg[dev].map_cfg[i].dst_btn < BR_COMBO_BASE_1) {
            uint8_t is_axis = btn_is_axis(config.in_cfg[dev].map_cfg[i].dst_id, config.in_cfg[dev].map_cfg[i].dst_btn);
            if (config.in_cfg[dev].map_cfg[i].src_btn == PAD_RM) {
                map_cnt_r++;
                if (is_axis) {
                    continue;
                }
                if (config.in_cfg[dev].map_cfg[i].perc_threshold > perc_threshold_r) {
                    perc_threshold_r = config.in_cfg[dev].map_cfg[i].perc_threshold;
                }
            }
            else if (config.in_cfg[dev].map_cfg[i].src_btn == PAD_LM) {
                map_cnt_l++;
                if (is_axis) {
                    continue;
                }
                if (config.in_cfg[dev].map_cfg[i].perc_threshold > perc_threshold_l) {
                    perc_threshold_l = config.in_cfg[dev].map_cfg[i].perc_threshold;
                }
            }
        }
    }
    /* If only one mapping exist do not set resistance */
    if (map_cnt_r < 2) {
        perc_threshold_r = -1;
    }
    if (map_cnt_l < 2) {
        perc_threshold_l = -1;
    }

    uint8_t r2_start_resistance_value = (perc_threshold_r * 255) / 100;
    uint8_t l2_start_resistance_value = (perc_threshold_l * 255) / 100;

    uint8_t r2_trigger_start_resistance = (uint8_t)(0x94 * (r2_start_resistance_value / 255.0));
    uint8_t r2_trigger_effect_force =
        (uint8_t)((0xb4 - r2_trigger_start_resistance) * (r2_start_resistance_value / 255.0) + r2_trigger_start_resistance);

    uint8_t l2_trigger_start_resistance = (uint8_t)(0x94 * (l2_start_resistance_value / 255.0));
    uint8_t l2_trigger_effect_force =
        (uint8_t)((0xb4 - l2_trigger_start_resistance) * (l2_start_resistance_value / 255.0) + l2_trigger_start_resistance);

    struct bt_hidp_ps5_set_conf ps5_set_conf = {
        .conf0 = 0x02,
        .valid_flag0 = 0x0c,
        .r2_trigger_motor_mode = perc_threshold_r > -1 ? 0x02 : 0x00,
        .r2_trigger_start_resistance = r2_trigger_start_resistance,
        .r2_trigger_effect_force = r2_trigger_effect_force,
        .r2_trigger_range_force = 0xff,
        .r2_trigger_near_release_str = 0x00,
        .r2_trigger_near_middle_str = 0x00,
        .r2_trigger_pressed_str = 0x00,
        .r2_trigger_actuation_freq = 0x00,
        .l2_trigger_motor_mode = perc_threshold_l > -1 ? 0x02 : 0x00,
        .l2_trigger_start_resistance = l2_trigger_start_resistance,
        .l2_trigger_effect_force = l2_trigger_effect_force,
        .l2_trigger_range_force = 0xff,
        .l2_trigger_near_release_str = 0x00,
        .l2_trigger_near_middle_str = 0x00,
        .l2_trigger_pressed_str = 0x00,
        .l2_trigger_actuation_freq = 0x00,
    };

    bt_hid_cmd_ps5_set_conf(device, (void *)&ps5_set_conf);
}

static void bt_hid_ps5_init_callback(void *arg) {
    struct bt_dev *device = (struct bt_dev *)arg;

    if (device->ids.report_type != BT_HIDP_PS4_STATUS) {
        struct bt_data *bt_data = &bt_adapter.data[device->ids.id];
        struct bt_hidp_ps5_set_conf *set_conf = (struct bt_hidp_ps5_set_conf *)bt_data->base.output;

        /* Init output data for Rumble/LED feedback */
        memset(set_conf, 0x00, sizeof(*set_conf));
        set_conf->conf0 = 0x02;
        /* 震动模式由 BT_HIDP_PS5_USE_RUMBLE_V2 选择，v1/v2 互斥。 */
        set_conf->valid_flag0 = BT_HIDP_PS5_VALID_FLAG0_RUMBLE;
        set_conf->valid_flag2 = BT_HIDP_PS5_VALID_FLAG2_RUMBLE;
        set_conf->valid_flag1 = BT_HIDP_PS5_LED_LIGHTBAR_CONTROL;
        set_conf->leds = 0; /* 默认熄灭灯条 */
        ps5_bt_output_seq[device->ids.id] = 0;
        bt_data->base.led_off_retry = PS5_LED_OFF_RETRY;

        printf("# %s\n", __FUNCTION__);

        /* 不在 init 扫射 RELEASE。等 0x31 后再按时间戳接管。 */
        bt_hid_cmd_ps5_set_conf(device, (void *)set_conf);

        /* Set trigger "click" haptic effect when rumble is on */
        if (config.out_cfg[device->ids.out_idx].acc_mode == ACC_RUMBLE
                || config.out_cfg[device->ids.out_idx].acc_mode == ACC_BOTH) {
            bt_hid_cmd_ps5_trigger_init(device);
        }
    }

    esp_timer_delete(device->timer_hdl);
    device->timer_hdl = NULL;

    atomic_set_bit(&device->flags, BT_DEV_HID_INIT_DONE);
    printf("# PS init done\n");
}

static void bt_hid_cmd_ps5_set_conf(struct bt_dev *device, void *report) {
    struct bt_hidp_ps5_set_conf *src = (struct bt_hidp_ps5_set_conf *)report;
    struct bt_hidp_ps5_bt_set_conf *set_conf =
        (struct bt_hidp_ps5_bt_set_conf *)bt_hci_pkt_tmp.hidp_data;

    bt_hci_pkt_tmp.hidp_hdr.hdr = BT_HIDP_DATA_OUT;
    bt_hci_pkt_tmp.hidp_hdr.protocol = BT_HIDP_PS5_SET_CONF;

    /* USB/common working buffer -> Bluetooth 0x31 layout, field by field (no blind memcpy). */
    memset(set_conf, 0x00, sizeof(*set_conf));
    set_conf->seq_tag = (ps5_bt_output_seq[device->ids.id]++ & 0x0F) << 4;
    set_conf->tag = PS5_BT_OUTPUT_TAG;
    set_conf->valid_flag0 = src->valid_flag0;
    set_conf->valid_flag1 = src->valid_flag1;
    set_conf->hf_motor_pwr = src->hf_motor_pwr;
    set_conf->lf_motor_pwr = src->lf_motor_pwr;
    memcpy(set_conf->tbd0, src->tbd0, sizeof(set_conf->tbd0));
    set_conf->mic_led = src->mic_led;
    set_conf->tbd1 = src->tbd1;
    /* adaptive triggers: identical contiguous named region in both structs */
    memcpy(&set_conf->r2_trigger_motor_mode, &src->r2_trigger_motor_mode,
        offsetof(struct bt_hidp_ps5_bt_set_conf, reserved2)
        - offsetof(struct bt_hidp_ps5_bt_set_conf, r2_trigger_motor_mode));
    memcpy(set_conf->reserved2, src->reserved2, sizeof(set_conf->reserved2));
    set_conf->reduce_motor_power = src->reduce_motor_power;
    set_conf->audio_control2 = src->audio_control2;
    set_conf->valid_flag2 = src->valid_flag2;
    set_conf->haptics_flags = src->haptics_flags;
    set_conf->reserved3 = src->reserved3;
    set_conf->lightbar_setup = src->lightbar_setup;
    set_conf->led_brightness = src->led_brightness;
    set_conf->player_leds = src->player_leds;
    memcpy(set_conf->rgb, (uint8_t *)&src->leds, sizeof(set_conf->rgb));
    memcpy(set_conf->tbd7, src->tbd8, sizeof(set_conf->tbd7));

    set_conf->crc = crc32_le((uint32_t)~0xFFFFFFFF, (void *)&bt_hci_pkt_tmp.hidp_hdr,
        sizeof(bt_hci_pkt_tmp.hidp_hdr) + sizeof(*set_conf) - sizeof(set_conf->crc));

    bt_hid_cmd(device->acl_handle, device->intr_chan.dcid, BT_HIDP_DATA_OUT, BT_HIDP_PS5_SET_CONF, sizeof(*set_conf));
}

int bt_hid_ps5_led_ready(struct bt_data *bt_data) {
    uint32_t start = bt_data->base.led_anim_start_us;

    /* 未见 0x31，或距首包不足 4s。 */
    if (!start) {
        return 0;
    }
    return ((uint32_t)esp_timer_get_time() - start) >= PS5_LED_ANIM_WAIT_US;
}

void bt_hid_ps5_clear_led(struct bt_dev *device) {
    struct bt_data *bt_data = &bt_adapter.data[device->ids.id];
    struct bt_hidp_ps5_set_conf *out =
        (struct bt_hidp_ps5_set_conf *)bt_data->base.output;
    struct bt_hidp_ps5_set_conf clear = *out;

    /* pair 动画未结束时 RELEASE 无效，甚至会卡在快闪。 */
    if (!bt_hid_ps5_led_ready(bt_data)) {
        return;
    }

    clear.conf0 = 0x02;
    clear.valid_flag0 = BT_HIDP_PS5_VALID_FLAG0_RUMBLE;
    clear.leds = 0;

    /* 先 RELEASE，并按 Linux 做一次 LIGHT_OUT，从无线固件收回灯控。 */
    clear.valid_flag1 = BT_HIDP_PS5_LED_RELEASE;
    clear.valid_flag2 = BT_HIDP_PS5_VALID_FLAG2_RUMBLE
        | BT_HIDP_PS5_LIGHTBAR_SETUP_CONTROL;
    clear.lightbar_setup = BT_HIDP_PS5_LIGHTBAR_SETUP_LIGHT_OUT;
    bt_hid_cmd_ps5_set_conf(device, &clear);

    /* 再明确接管并立即关闭玩家灯和灯条。 */
    clear.valid_flag1 = BT_HIDP_PS5_LED_LIGHTBAR_CONTROL | BT_HIDP_PS5_LED_PLAYER_CONTROL;
    clear.valid_flag2 = BT_HIDP_PS5_VALID_FLAG2_RUMBLE;
    clear.lightbar_setup = 0;
    clear.player_leds = BT_HIDP_PS5_PLAYER_LED_INSTANT;
    bt_hid_cmd_ps5_set_conf(device, &clear);
}

void bt_hid_cmd_ps_set_conf(struct bt_dev *device, void *report) {
    switch (device->ids.subtype) {
        case BT_PS5_DS:
            bt_hid_cmd_ps5_set_conf(device, report);
            break;
        default:
            bt_hid_cmd_ps4_set_conf(device, report);
            break;
    }
}

void bt_hid_ps_init(struct bt_dev *device) {
#ifndef CONFIG_BLUERETRO_TEST_FALLBACK_REPORT
    struct bt_data *bt_data = &bt_adapter.data[device->ids.id];
    struct bt_hidp_ps4_set_conf *set_conf = (struct bt_hidp_ps4_set_conf *)bt_data->base.output;

    /* Init output data for Rumble/LED feedback */
    set_conf->conf0 = 0xc4;
    set_conf->conf1 = 0x03;
    /* 连上后灯条默认熄灭，最低电量再红闪 */
    set_conf->leds = 0;
    bt_data->base.batt_valid = 0;
    bt_data->base.batt_low = 0;
    bt_data->base.batt_low_pending = 0;
    bt_data->base.batt_ds5_on = 0;

    switch (device->ids.subtype) {
        case BT_PS5_DS:
            bt_hid_ps5_init_callback((void *)device);
            break;
        default:
        {
            const esp_timer_create_args_t ps5_timer_args = {
                .callback = &bt_hid_ps5_init_callback,
                .arg = (void *)device,
                .name = "ps5_init_timer"
            };
            struct bt_hidp_ps4_set_conf ps4_set_conf = {
                .conf0 = 0xc0,
                .conf1 = 0x03,
            };
            ps4_set_conf.leds = 0; /* 默认熄灭灯条 */

            printf("# %s\n", __FUNCTION__);

            esp_timer_create(&ps5_timer_args, (esp_timer_handle_t *)&device->timer_hdl);
            esp_timer_start_once(device->timer_hdl, 1000000);
            bt_hid_cmd_ps4_set_conf(device, (void *)&ps4_set_conf);
            break;
        }
    }
#endif
}

void bt_hid_ps_hdlr(struct bt_dev *device, struct bt_hci_pkt *bt_hci_acl_pkt, uint32_t len) {
    uint32_t hidp_data_len = len - (BT_HCI_H4_HDR_SIZE + BT_HCI_ACL_HDR_SIZE
                                    + sizeof(struct bt_l2cap_hdr) + sizeof(struct bt_hidp_hdr));

    switch (bt_hci_acl_pkt->sig_hdr.code) {
        case BT_HIDP_DATA_IN:
            switch (bt_hci_acl_pkt->hidp_hdr.protocol) {
                case BT_HIDP_HID_STATUS:
                    if (device->ids.report_type != BT_HIDP_HID_STATUS) {
                        bt_type_update(device->ids.id, BT_PS, BT_SUBTYPE_DEFAULT);
                        device->ids.report_type = BT_HIDP_HID_STATUS;
                    }
                    bt_host_bridge(device, bt_hci_acl_pkt->hidp_hdr.protocol, bt_hci_acl_pkt->hidp_data, hidp_data_len);
                    break;
                case BT_HIDP_PS4_STATUS:
                    if (device->ids.report_type != BT_HIDP_PS4_STATUS) {
                        bt_type_update(device->ids.id, BT_PS, BT_SUBTYPE_DEFAULT);
                        device->ids.report_type = BT_HIDP_PS4_STATUS;
                    }
                    bt_host_bridge(device, bt_hci_acl_pkt->hidp_hdr.protocol, bt_hci_acl_pkt->hidp_data, hidp_data_len);
                    break;
                case BT_HIDP_PS5_STATUS:
                    if (device->ids.report_type != BT_HIDP_PS5_STATUS) {
                        struct bt_data_base *base = &bt_adapter.data[device->ids.id].base;

                        bt_type_update(device->ids.id, BT_PS, BT_PS5_DS);
                        device->ids.report_type = BT_HIDP_PS5_STATUS;
                        /* 首包 0x31：记下时刻，4s 后再 RELEASE。 */
                        base->led_off_retry = PS5_LED_OFF_RETRY;
                        if (!base->led_anim_start_us) {
                            uint32_t now = (uint32_t)esp_timer_get_time();

                            base->led_anim_start_us = now ? now : 1;
                        }
                    }
                    bt_host_bridge(device, bt_hci_acl_pkt->hidp_hdr.protocol, bt_hci_acl_pkt->hidp_data, hidp_data_len);
                    break;
            }
            break;
    }
}
