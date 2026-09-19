/*
 * Copyright (c) 2019-2024, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "zephyr/types.h"
#include "tools/util.h"
#include "adapter/config.h"
#include "adapter/kb_monitor.h"
#include "adapter/wired/wired.h"
#include "ps.h"
#include <esp_timer.h>
#include <esp_attr.h>

#define PS_JOYSTICK_AXES_CNT 4

enum {
    PS_SELECT = 0,
    PS_L3,
    PS_R3,
    PS_START,
    PS_D_UP,
    PS_D_RIGHT,
    PS_D_DOWN,
    PS_D_LEFT,
    PS_L2,
    PS_R2,
    PS_L1,
    PS_R1,
    PS_T,
    PS_C,
    PS_X,
    PS_S,
};

static DRAM_ATTR const uint8_t ps_axes_idx[ADAPTER_PS2_MAX_AXES] =
{
/*  AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY, TRIG_L, TRIG_R  */
    2,       3,       0,       1,       14,     15,
/*  TRIG_LS, TRIG_RS, DPAD_L,  DPAD_R,  DPAD_D, DPAD_U  */
    12,      13,      5,       4,       7,      6,
/*  BTN_L,   BTN_R,   BTN_D,   BTN_U  */
    11,      9,       10,      8
};

static DRAM_ATTR const uint8_t ps_mouse_axes_idx[ADAPTER_MAX_AXES] =
{
/*  AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY, TRIG_L, TRIG_R  */
    0,       1,       0,       1,       0,     1
};

static DRAM_ATTR const struct ctrl_meta ps_axes_meta[ADAPTER_PS2_MAX_AXES] =
{
    {.size_min = -128, .size_max = 127, .neutral = 0x80, .abs_max = 0xB0, .abs_min = 0xB0},
    {.size_min = -128, .size_max = 127, .neutral = 0x80, .abs_max = 0xB0, .abs_min = 0xB0, .polarity = 1},
    {.size_min = -128, .size_max = 127, .neutral = 0x80, .abs_max = 0xB0, .abs_min = 0xB0},
    {.size_min = -128, .size_max = 127, .neutral = 0x80, .abs_max = 0xB0, .abs_min = 0xB0, .polarity = 1},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0x100, .abs_min = 0x00},
};

static DRAM_ATTR const struct ctrl_meta ps_mouse_axes_meta[ADAPTER_MAX_AXES] =
{
    {.size_min = -128, .size_max = 127, .neutral = 0x00, .abs_max = 127, .abs_min = 128},
    {.size_min = -128, .size_max = 127, .neutral = 0x00, .abs_max = 127, .abs_min = 128, .polarity = 1},
    {.size_min = -128, .size_max = 127, .neutral = 0x00, .abs_max = 127, .abs_min = 128},
    {.size_min = -128, .size_max = 127, .neutral = 0x00, .abs_max = 127, .abs_min = 128, .polarity = 1},
    {.size_min = -128, .size_max = 127, .neutral = 0x00, .abs_max = 127, .abs_min = 128},
    {.size_min = -128, .size_max = 127, .neutral = 0x00, .abs_max = 127, .abs_min = 128},
};

struct ps_map {
    uint16_t buttons;
    union {
        uint8_t axes[16];
        struct {
            uint32_t sticks;
            uint8_t pressure[12];
        };
    };
    uint8_t analog_btn;
} __packed;

struct ps_mouse_map {
    uint16_t buttons;
    uint8_t relative[2];
    int32_t raw_axes[2];
} __packed;

static const uint32_t ps_mask[4] = {0xBB7F0FFF, 0x00000000, 0x00000000, BR_COMBO_MASK};
static const uint32_t ps_desc[4] = {0x330F0FFF, 0x00000000, 0x00000000, 0x00000000};
static DRAM_ATTR const uint32_t ps_btns_mask[32] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    BIT(PS_D_LEFT), BIT(PS_D_RIGHT), BIT(PS_D_DOWN), BIT(PS_D_UP),
    0, 0, 0, 0,
    BIT(PS_S), BIT(PS_C), BIT(PS_X), BIT(PS_T),
    BIT(PS_START), BIT(PS_SELECT), 0, 0,
    BIT(PS_L2), BIT(PS_L1), 0, BIT(PS_L3),
    BIT(PS_R2), BIT(PS_R1), 0, BIT(PS_R3),
};
static DRAM_ATTR const uint8_t ps_btns_idx[32] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    5, 4, 7, 6,
    0, 0, 0, 0,
    11, 9, 10, 8,
    0, 0, 0, 0,
    14, 12, 0, 0,
    15, 13, 0, 0,
};


#define PS_DPAD_MASK (BIT(PS_D_UP) | BIT(PS_D_RIGHT) | BIT(PS_D_DOWN) | BIT(PS_D_LEFT))
#define PS_FACE_MASK (BIT(PS_L2) | BIT(PS_R2) | BIT(PS_L1) | BIT(PS_R1) | BIT(PS_T) | BIT(PS_C) | BIT(PS_X) | BIT(PS_S))
#define PS_QUEUE_MASK (PS_DPAD_MASK | PS_FACE_MASK)
#define INPUT_BTN_QUEUE_SLOTS (INPUT_BTN_QUEUE_DEPTH + 1)
#define INPUT_Q_MEMW() __asm__ __volatile__("memw" ::: "memory")
/* 空格来源：两边都空时，方向空可并脸键，脸键空不并脸键 */
#define PS_Q_KIND_NORMAL 0
#define PS_Q_KIND_DIR_EMPTY 1
#define PS_Q_KIND_FACE_EMPTY 2
#define PS_Q_KIND_BOTH_EMPTY 3

static uint16_t btn_q_slots[WIRED_MAX_DEV][INPUT_BTN_QUEUE_SLOTS];
static uint16_t btn_q_new[WIRED_MAX_DEV][INPUT_BTN_QUEUE_SLOTS];
static uint8_t btn_q_kind[WIRED_MAX_DEV][INPUT_BTN_QUEUE_SLOTS];
static volatile uint8_t btn_q_head[WIRED_MAX_DEV];
static volatile uint8_t btn_q_tail[WIRED_MAX_DEV];
static uint16_t btn_q_last_ah[WIRED_MAX_DEV];
static int64_t btn_q_last_us[WIRED_MAX_DEV];
#if INPUT_BTN_QUEUE_HOLD_MS > 0
static int64_t btn_q_hold_start_us[WIRED_MAX_DEV];
#endif

static void ps_btn_queue_reset(uint8_t wired_id) {
    if (wired_id >= WIRED_MAX_DEV) {
        return;
    }
    btn_q_head[wired_id] = 0;
    btn_q_tail[wired_id] = 0;
    btn_q_last_ah[wired_id] = 0;
    btn_q_last_us[wired_id] = 0;
#if INPUT_BTN_QUEUE_HOLD_MS > 0
    btn_q_hold_start_us[wired_id] = 0;
#endif
}

static uint8_t ps_btn_queue_kind(int dir_changed, int face_release, uint16_t dir, uint16_t face) {
    if (dir || face) {
        return PS_Q_KIND_NORMAL;
    }
    if (dir_changed && face_release) {
        return PS_Q_KIND_BOTH_EMPTY;
    }
    if (dir_changed) {
        return PS_Q_KIND_DIR_EMPTY;
    }
    if (face_release) {
        return PS_Q_KIND_FACE_EMPTY;
    }
    return PS_Q_KIND_NORMAL;
}

static void ps_btn_queue_push(uint8_t wired_id, uint16_t buttons_al) {
    uint16_t ah, dir, face, nhead, idx, merged, newly, released, slot_new;
    uint16_t slot_dir, slot_face, prev_new_dir, prev_new_face, new_dir, new_face;
    int64_t now;
    uint8_t head, tail, slot_kind, kind;
    int in_win, no_merge, dir_changed, face_release, dir_roll;

    if (wired_id >= WIRED_MAX_DEV) {
        return;
    }
    ah = (uint16_t)((~buttons_al) & PS_QUEUE_MASK);
    dir = ah & PS_DPAD_MASK;
    face = ah & PS_FACE_MASK;

    /* 按住不变不入队；按下和回中都要入队 */
    if (ah == btn_q_last_ah[wired_id]) {
        return;
    }

    newly = (uint16_t)(ah & (uint16_t)~btn_q_last_ah[wired_id]);
    released = (uint16_t)(btn_q_last_ah[wired_id] & (uint16_t)~ah);
    new_dir = newly & PS_DPAD_MASK;
    new_face = newly & PS_FACE_MASK;
    dir_changed = (dir != (btn_q_last_ah[wired_id] & PS_DPAD_MASK));
    face_release = (released & PS_FACE_MASK) != 0 && new_face == 0;
    /* 只松方向且仍按着方向：剩余方向当作这一拍新增 */
    dir_roll = (released & PS_DPAD_MASK) && dir
            && !(released & PS_FACE_MASK) && !new_face;
    if (dir_roll) {
        newly = dir;
        new_dir = dir;
    }

    now = esp_timer_get_time();
    head = btn_q_head[wired_id];
    tail = btn_q_tail[wired_id];
    in_win = 0;
    if (INPUT_CHORD_WINDOW_MS > 0 && head != tail &&
            (now - btn_q_last_us[wired_id]) <= ((int64_t)INPUT_CHORD_WINDOW_MS * 1000)) {
        in_win = 1;
    }

    no_merge = 1;
    merged = 0;
    idx = 0;
    slot_new = 0;
    if (in_win) {
        idx = (uint16_t)((head + INPUT_BTN_QUEUE_SLOTS - 1) % INPUT_BTN_QUEUE_SLOTS);
        merged = btn_q_slots[wired_id][idx];
        slot_new = btn_q_new[wired_id][idx];
        slot_dir = (uint16_t)(merged & PS_DPAD_MASK);
        slot_face = (uint16_t)(merged & PS_FACE_MASK);
        prev_new_dir = (uint16_t)(slot_new & PS_DPAD_MASK);
        prev_new_face = (uint16_t)(slot_new & PS_FACE_MASK);
        slot_kind = btn_q_kind[wired_id][idx];
        no_merge = 0;
        if (released && !dir_roll) {
            /* 松脸键、松光方向、同一帧又松又按：开新格 */
            no_merge = 1;
        } else if (newly) {
            /* 只拿新增键判定：还按着的不参与 */
            if (new_face && prev_new_dir && !prev_new_face) {
                no_merge = 1; /* 方向后不并脸键按下 */
            }
            if (new_dir && prev_new_dir && !prev_new_face) {
                no_merge = 1; /* 纯方向不并 */
            }
            if (new_face && !slot_face && slot_dir) {
                no_merge = 1; /* 还按着方向的松脸键格，不并脸键 */
            }
            if (new_face && !slot_face && !slot_dir &&
                    (slot_kind == PS_Q_KIND_FACE_EMPTY || slot_kind == PS_Q_KIND_BOTH_EMPTY)) {
                no_merge = 1; /* 脸键空格不并脸键 */
            }
            if (new_dir && !slot_dir && !slot_face &&
                    (slot_kind == PS_Q_KIND_DIR_EMPTY || slot_kind == PS_Q_KIND_BOTH_EMPTY)) {
                no_merge = 1;
            }
        }
    }

    if (!no_merge) {
        /* 写成当前手势，松下时下会从上一格清掉；新增掩码覆盖为这一拍新增 */
        merged = (uint16_t)(dir | face);
        slot_new = newly;
        btn_q_slots[wired_id][idx] = merged;
        btn_q_new[wired_id][idx] = slot_new;
        btn_q_kind[wired_id][idx] = ps_btn_queue_kind(dir_changed, face_release,
            (uint16_t)(merged & PS_DPAD_MASK), (uint16_t)(merged & PS_FACE_MASK));
        INPUT_Q_MEMW();
        btn_q_last_ah[wired_id] = ah;
        btn_q_last_us[wired_id] = now;
        return;
    }

    nhead = (uint16_t)((head + 1) % INPUT_BTN_QUEUE_SLOTS);
    if (nhead == tail) {
        btn_q_tail[wired_id] = (uint8_t)((tail + 1) % INPUT_BTN_QUEUE_SLOTS);
#if INPUT_BTN_QUEUE_HOLD_MS > 0
        btn_q_hold_start_us[wired_id] = 0;
#endif
    }
    kind = ps_btn_queue_kind(dir_changed, face_release, dir, face);
    btn_q_slots[wired_id][head] = (uint16_t)(dir | face);
    /* 仅方向松开且仍按着方向时，把剩余方向记为上一笔，防止正方向被斜向吞掉 */
    if ((released & PS_DPAD_MASK) && dir) {
        btn_q_new[wired_id][head] = dir;
    } else {
        btn_q_new[wired_id][head] = newly;
    }
    btn_q_kind[wired_id][head] = kind;
    INPUT_Q_MEMW();
    btn_q_head[wired_id] = (uint8_t)nhead;
    btn_q_last_ah[wired_id] = ah;
    btn_q_last_us[wired_id] = now;
}

uint16_t IRAM_ATTR ps_btn_queue_overlay(uint8_t wired_id, uint16_t live_buttons) {
    uint8_t head, tail;
    uint16_t ah;
#if INPUT_BTN_QUEUE_HOLD_MS > 0
    int64_t now;
#endif

    if (wired_id >= WIRED_MAX_DEV) {
        return live_buttons;
    }
    head = btn_q_head[wired_id];
    tail = btn_q_tail[wired_id];
    if (head == tail) {
        return live_buttons;
    }
    INPUT_Q_MEMW();
    ah = btn_q_slots[wired_id][tail];
#if INPUT_BTN_QUEUE_HOLD_MS > 0
    now = esp_timer_get_time();
    if (btn_q_hold_start_us[wired_id] == 0) {
        btn_q_hold_start_us[wired_id] = now;
    }
    /* 这一格还没展示满，主机继续读到同一格 */
    if ((now - btn_q_hold_start_us[wired_id]) < ((int64_t)INPUT_BTN_QUEUE_HOLD_MS * 1000)) {
        live_buttons |= PS_QUEUE_MASK;
        live_buttons &= (uint16_t)(~ah);
        return live_buttons;
    }
    /* 保持时间到，出队；还有下一格就立刻开始计时 */
    btn_q_tail[wired_id] = (uint8_t)((tail + 1) % INPUT_BTN_QUEUE_SLOTS);
    INPUT_Q_MEMW();
    tail = btn_q_tail[wired_id];
    if (head == tail) {
        btn_q_hold_start_us[wired_id] = 0;
        return live_buttons;
    }
    INPUT_Q_MEMW();
    ah = btn_q_slots[wired_id][tail];
    btn_q_hold_start_us[wired_id] = now;
#else
    btn_q_tail[wired_id] = (uint8_t)((tail + 1) % INPUT_BTN_QUEUE_SLOTS);
    INPUT_Q_MEMW();
#endif
    live_buttons |= PS_QUEUE_MASK;
    live_buttons &= (uint16_t)(~ah);
    return live_buttons;
}

static const uint32_t ps_mouse_mask[4] = {0x110000F0, 0x00000000, 0x00000000, BR_COMBO_MASK};
static const uint32_t ps_mouse_desc[4] = {0x000000F0, 0x00000000, 0x00000000, 0x00000000};
static const uint32_t ps_mouse_btns_mask[32] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    BIT(PS_L1), 0, 0, 0,
    BIT(PS_R1), 0, 0, 0,
};

static const uint32_t ps_kb_mask[4] = {0xE6FF0F0F, 0xFFFFFFFF, 0xFFFFFFFF, 0x0007FFFF | BR_COMBO_MASK};
static const uint32_t ps_kb_desc[4] = {0x00000000, 0x00000000, 0x00000000, 0x00000000};
static const uint8_t ps_kb_scancode[KBM_MAX] = {
 /* KB_A, KB_D, KB_S, KB_W, MOUSE_X_LEFT, MOUSE_X_RIGHT, MOUSE_Y_DOWN MOUSE_Y_UP */
    0x1C, 0x23, 0x1B, 0x1D, 0x00, 0x00, 0x00, 0x00,
 /* KB_LEFT, KB_RIGHT, KB_DOWN, KB_UP, MOUSE_WX_LEFT, MOUSE_WX_RIGHT, MOUSE_WY_DOWN, MOUSE_WY_UP */
    0x86, 0x8D, 0x8A, 0x89, 0x00, 0x00, 0x00, 0x00,
 /* KB_Q, KB_R, KB_E, KB_F, KB_ESC, KB_ENTER, KB_LWIN, KB_HASH */
    0x15, 0x2D, 0x24, 0x2B, 0x76, 0x5A, 0x1F, 0x0E,
 /* MOUSE_RIGHT, KB_Z, KB_LCTRL, MOUSE_MIDDLE, MOUSE_LEFT, KB_X, KB_LSHIFT, KB_SPACE */
    0x00, 0x1A, 0x14, 0x00, 0x00, 0x22, 0x12, 0x29,

 /* KB_B, KB_C, KB_G, KB_H, KB_I, KB_J, KB_K, KB_L */
    0x32, 0x21, 0x34, 0x33, 0x43, 0x3B, 0x42, 0x4B,
 /* KB_M, KB_N, KB_O, KB_P, KB_T, KB_U, KB_V, KB_Y */
    0x3A, 0x31, 0x44, 0x4D, 0x2C, 0x3C, 0x2A, 0x35,
 /* KB_1, KB_2, KB_3, KB_4, KB_5, KB_6, KB_7, KB_8 */
    0x16, 0x1E, 0x26, 0x25, 0x2E, 0x36, 0x3D, 0x3E,
 /* KB_9, KB_0, KB_BACKSPACE, KB_TAB, KB_MINUS, KB_EQUAL, KB_LEFTBRACE, KB_RIGHTBRACE */
    0x46, 0x45, 0x66, 0x0D, 0x4E, 0x55, 0x54, 0x5B,

 /* KB_BACKSLASH, KB_SEMICOLON, KB_APOSTROPHE, KB_GRAVE, KB_COMMA, KB_DOT, KB_SLASH, KB_CAPSLOCK */
    0x5D, 0x4C, 0x51, 0x00, 0x41, 0x49, 0x4A, 0x58,
 /* KB_F1, KB_F2, KB_F3, KB_F4, KB_F5, KB_F6, KB_F7, KB_F8 */
    0x05, 0x06, 0x04, 0x0C, 0x03, 0x0B, 0x83, 0x0A,
 /* KB_F9, KB_F10, KB_F11, KB_F12, KB_PSCREEN, KB_SCROLL, KB_PAUSE, KB_INSERT */
    0x01, 0x09, 0x78, 0x07, 0x84, 0x7E, 0x82, 0x81,
 /* KB_HOME, KB_PAGEUP, KB_DEL, KB_END, KB_PAGEDOWN, KB_NUMLOCK, KB_KP_DIV, KB_KP_MULTI */
    0x87, 0x8B, 0x85, 0x88, 0x8C, 0x77, 0x80, 0x7C,

 /* KB_KP_MINUS, KB_KP_PLUS, KB_KP_ENTER, KB_KP_1, KB_KP_2, KB_KP_3, KB_KP_4, KB_KP_5 */
    0x7B, 0x79, 0x19, 0x69, 0x72, 0x7A, 0x6B, 0x73,
 /* KB_KP_6, KB_KP_7, KB_KP_8, KB_KP_9, KB_KP_0, KB_KP_DOT, KB_LALT, KB_RCTRL */
    0x74, 0x6C, 0x75, 0x7D, 0x70, 0x71, 0x11, 0x18,
 /* KB_RSHIFT, KB_RALT, KB_RWIN */
    0x59, 0x17, 0x00,
};

void IRAM_ATTR ps_init_buffer(int32_t dev_mode, struct wired_data *wired_data) {
    switch (dev_mode) {
        case DEV_KB:
        {
            memset(wired_data->output, 0x00, 12);
            break;
        }
        case DEV_MOUSE:
        {
            struct ps_mouse_map *map = (struct ps_mouse_map *)wired_data->output;

            map->buttons = 0xFCFF;
            for (uint32_t i = 0; i < 2; i++) {
                map->raw_axes[i] = 0;
                map->relative[i] = 1;
            }
            break;
        }
        default:
        {
            struct ps_map *map = (struct ps_map *)wired_data->output;
            struct ps_map *map_mask = (struct ps_map *)wired_data->output_mask;

            ps_btn_queue_reset((uint8_t)(wired_data - wired_adapter.data));
            memset((void *)map, 0, sizeof(*map));
            map->buttons = 0xFFFF;
            map->analog_btn = 0x00;
            for (uint32_t i = 0; i < PS_JOYSTICK_AXES_CNT; i++) {
                map->axes[ps_axes_idx[i]] = ps_axes_meta[i].neutral;
            }
            memset(map->pressure, 0x00, sizeof(map->pressure));

            map_mask->buttons = 0x0000;
            map_mask->sticks = 0x00000000;
            memset(map_mask->pressure, 0xFF, sizeof(map_mask->pressure));
            break;
        }
    }
}

void ps_meta_init(struct wired_ctrl *ctrl_data) {
    memset((void *)ctrl_data, 0, sizeof(*ctrl_data)*WIRED_MAX_DEV);

    for (uint32_t i = 0; i < WIRED_MAX_DEV; i++) {
        for (uint32_t j = 0; j < ADAPTER_PS2_MAX_AXES; j++) {
            switch (config.out_cfg[i].dev_mode) {
                case DEV_KB:
                    ctrl_data[i].mask = ps_kb_mask;
                    ctrl_data[i].desc = ps_kb_desc;
                    goto exit_axes_loop;
                case DEV_MOUSE:
                    if (j >= PS_JOYSTICK_AXES_CNT) {
                        goto exit_axes_loop;
                    }
                    ctrl_data[i].mask = ps_mouse_mask;
                    ctrl_data[i].desc = ps_mouse_desc;
                    ctrl_data[i].axes[j].meta = &ps_mouse_axes_meta[j];
                    break;
                default:
                    ctrl_data[i].mask = ps_mask;
                    ctrl_data[i].desc = ps_desc;
                    ctrl_data[i].axes[j].meta = &ps_axes_meta[j];
                    break;
            }
        }
exit_axes_loop:
        ;
    }
}

static void ps_ctrl_from_generic(struct wired_ctrl *ctrl_data, struct wired_data *wired_data) {
    struct ps_map map_tmp;

    memcpy((void *)&map_tmp, wired_data->output, sizeof(map_tmp));

    for (uint32_t i = 0; i < ARRAY_SIZE(generic_btns_mask); i++) {
        if (ctrl_data->map_mask[0] & BIT(i)) {
            if (ctrl_data->btns[0].value & generic_btns_mask[i]) {
                map_tmp.buttons &= ~ps_btns_mask[i];
                wired_data->cnt_mask[i] = ctrl_data->btns[0].cnt_mask[i];
            }
            else {
                map_tmp.buttons |= ps_btns_mask[i];
                wired_data->cnt_mask[i] = 0;
            }
        }
    }

    if (ctrl_data->map_mask[0] & generic_btns_mask[PAD_MT]) {
        if (ctrl_data->btns[0].value & generic_btns_mask[PAD_MT]) {
            map_tmp.analog_btn |= 0x01;
        }
        else {
            map_tmp.analog_btn &= ~0x01;
        }
    }

    for (uint32_t i = 0; i < ADAPTER_PS2_MAX_AXES; i++) {
        uint32_t btn_id = axis_to_btn_id(i);
        uint32_t btn_mask = axis_to_btn_mask(i);
        if (ctrl_data->map_mask[0] & (btn_mask & ps_desc[0])) {
            if (ctrl_data->axes[i].value > ctrl_data->axes[i].meta->size_max) {
                map_tmp.axes[ps_axes_idx[i]] = 255;
            }
            else if (ctrl_data->axes[i].value < ctrl_data->axes[i].meta->size_min) {
                map_tmp.axes[ps_axes_idx[i]] = 0;
            }
            else {
                map_tmp.axes[ps_axes_idx[i]] = (uint8_t)(ctrl_data->axes[i].value + ctrl_data->axes[i].meta->neutral);
            }

            if (i >= PS_JOYSTICK_AXES_CNT) {
                if (map_tmp.axes[ps_axes_idx[i]]) {
                    map_tmp.buttons &= ~ps_btns_mask[btn_id];;
                    wired_data->cnt_mask[btn_id] = ctrl_data->btns[0].cnt_mask[btn_id];
                }
                else {
                    map_tmp.buttons |= ps_btns_mask[btn_id];
                    wired_data->cnt_mask[btn_id] = 0;
                }
            }
        }
        wired_data->cnt_mask[btn_id] = ctrl_data->axes[i].cnt_mask;
    }

    memcpy(wired_data->output, (void *)&map_tmp, sizeof(map_tmp));
    ps_btn_queue_push(ctrl_data->index, map_tmp.buttons);

#ifdef CONFIG_BLUERETRO_RAW_OUTPUT
    printf("{\"log_type\": \"wired_output\", \"axes\": [%d, %d, %d, %d], \"btns\": [%d, %d]}\n",
        map_tmp.axes[ps_axes_idx[0]], map_tmp.axes[ps_axes_idx[1]],
        map_tmp.axes[ps_axes_idx[2]], map_tmp.axes[ps_axes_idx[3]],
        map_tmp.buttons, map_tmp.analog_btn);
#endif
}

static void ps_mouse_from_generic(struct wired_ctrl *ctrl_data, struct wired_data *wired_data) {
    struct ps_mouse_map map_tmp;
    int32_t *raw_axes = (int32_t *)(wired_data->output + 4);

    memcpy((void *)&map_tmp, wired_data->output, sizeof(map_tmp));

    for (uint32_t i = 0; i < ARRAY_SIZE(generic_btns_mask); i++) {
        if (ctrl_data->map_mask[0] & BIT(i)) {
            if (ctrl_data->btns[0].value & generic_btns_mask[i]) {
                map_tmp.buttons &= ~ps_mouse_btns_mask[i];
            }
            else {
                map_tmp.buttons |= ps_mouse_btns_mask[i];
            }
        }
    }

    for (uint32_t i = 2; i < PS_JOYSTICK_AXES_CNT; i++) {
        if (ctrl_data->map_mask[0] & (axis_to_btn_mask(i) & ps_mouse_desc[0])) {
            if (ctrl_data->axes[i].relative) {
                map_tmp.relative[ps_mouse_axes_idx[i]] = 1;
                atomic_add(&raw_axes[ps_mouse_axes_idx[i]], ctrl_data->axes[i].value);
            }
            else {
                map_tmp.relative[ps_mouse_axes_idx[i]] = 0;
                raw_axes[ps_mouse_axes_idx[i]] = ctrl_data->axes[i].value;
            }
        }
    }

    memcpy(wired_data->output, (void *)&map_tmp, sizeof(map_tmp) - 8);
}

static void ps_kb_from_generic(struct wired_ctrl *ctrl_data, struct wired_data *wired_data) {
    if (!atomic_test_bit(&wired_data->flags, WIRED_KBMON_INIT)) {
        kbmon_init(ctrl_data->index, ps_kb_id_to_scancode);
        kbmon_set_typematic(ctrl_data->index, 1, 500000, 90000);
        atomic_set_bit(&wired_data->flags, WIRED_KBMON_INIT);
    }
    kbmon_update(ctrl_data->index, ctrl_data);
}

void ps_kb_id_to_scancode(uint32_t dev_id, uint8_t type, uint8_t id) {
    uint8_t kb_buf[12] = {0};
    uint8_t scancode = ps_kb_scancode[id];
    uint32_t idx = 0;

    if (scancode) {
        switch (type) {
            case KBMON_TYPE_MAKE:
                kb_buf[idx++] = 0x01;
                break;
            case KBMON_TYPE_BREAK:
                kb_buf[idx++] = 0x02;
                kb_buf[idx++] = 0xF0;
                break;
        }
        if (id < KBM_MAX) {
            kb_buf[idx++] = scancode;
        }
        kbmon_set_code(dev_id, kb_buf, 12);
    }
}

void ps_from_generic(int32_t dev_mode, struct wired_ctrl *ctrl_data, struct wired_data *wired_data) {
    switch (dev_mode) {
        case DEV_KB:
            ps_kb_from_generic(ctrl_data, wired_data);
            break;
        case DEV_MOUSE:
            ps_mouse_from_generic(ctrl_data, wired_data);
            break;
        default:
            ps_ctrl_from_generic(ctrl_data, wired_data);
            break;
    }
}

void ps_fb_to_generic(int32_t dev_mode, struct raw_fb *raw_fb_data, struct generic_fb *fb_data) {
    fb_data->wired_id = raw_fb_data->header.wired_id;
    fb_data->type = raw_fb_data->header.type;

    switch (fb_data->type) {
        case FB_TYPE_RUMBLE:
            /* 真 DS2 小电机只看最低位：奇数开、偶数关。
             * 0x01/0xFF 都会开；RE4 常见偶数残值（0x40/0x80/0xFE）仍关掉。
             * 不要改成任意非 0，那会撤掉 RE4 修复。 */
            fb_data->lf_pwr = raw_fb_data->data[1];
            fb_data->hf_pwr = (raw_fb_data->data[0] & 0x01) ? 0xFF : 0x00;
            fb_data->state = (fb_data->hf_pwr || fb_data->lf_pwr) ? 1 : 0;
            break;
        case FB_TYPE_STATUS_LED:
            fb_data->led = raw_fb_data->data[0];
            break;
    }
}

void IRAM_ATTR ps_gen_turbo_mask(struct wired_data *wired_data) {
    struct ps_map *map_mask = (struct ps_map *)wired_data->output_mask;

    map_mask->buttons = 0x0000;
    map_mask->sticks = 0x00000000;
    memset(map_mask->pressure, 0xFF, sizeof(map_mask->pressure));

    for (uint32_t i = 0; i < ARRAY_SIZE(ps_btns_mask); i++) {
        uint8_t mask = wired_data->cnt_mask[i] >> 1;

        if (ps_btns_mask[i] && mask) {
            if (wired_data->cnt_mask[i] & 1) {
                if (!(mask & wired_data->frame_cnt)) {
                    map_mask->buttons |= ps_btns_mask[i];
                    if (ps_btns_idx[i]) {
                        map_mask->axes[ps_btns_idx[i]] = 0x00;
                    }
                }
            }
            else {
                if (!((mask & wired_data->frame_cnt) == mask)) {
                    map_mask->buttons |= ps_btns_mask[i];
                    if (ps_btns_idx[i]) {
                        map_mask->axes[ps_btns_idx[i]] = 0x00;
                    }
                }
            }
        }
    }

    wired_gen_turbo_mask_axes8(wired_data, map_mask->axes, 4, ps_axes_idx, ps_axes_meta);
}
