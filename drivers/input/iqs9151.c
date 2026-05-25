/*
 * File:   IQS9151.c
 * Author: ShiniNet
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "iqs9151_init.h"
#include "iqs9151_regs.h"
#include "iqs9151_test.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

LOG_MODULE_REGISTER(iqs9151, CONFIG_INPUT_IQS9151_LOG_LEVEL);

#define DT_DRV_COMPAT azoteq_iqs9151

#define IQS9151_I2C_CHUNK_SIZE 30
#define IQS9151_RSTD_DELAY_MS 100
#define IQS9151_ATI_TIMEOUT_MS 1000
#define IQS9151_ATI_POLL_INTERVAL_MS 10
#define INERTIA_FP_SHIFT 8
#define EMA_FP_SHIFT INERTIA_FP_SHIFT
#define EMA_ALPHA_DEN (1 << EMA_FP_SHIFT)
#define IQS9151_FRAME_READ_SIZE 28
#define IQS9151_INERTIA_MOTION_HISTORY_SIZE 12

#define SCROLL_INERTIA_INTERVAL_MS 10
#define SCROLL_INERTIA_MAX_DURATION_MS 3000
#define SCROLL_INERTIA_DECAY_NUM CONFIG_INPUT_IQS9151_SCROLL_INERTIA_DECAY
#define SCROLL_INERTIA_DECAY_DEN 1000
#define SCROLL_INERTIA_START_THRESHOLD 1
#define SCROLL_INERTIA_MIN_VELOCITY 1
#define SCROLL_EMA_ALPHA 10
#define SCROLL_INERTIA_RECENT_WINDOW_MS CONFIG_INPUT_IQS9151_SCROLL_INERTIA_RECENT_WINDOW_MS
#define SCROLL_INERTIA_STALE_GAP_MS CONFIG_INPUT_IQS9151_SCROLL_INERTIA_STALE_GAP_MS
#define SCROLL_INERTIA_MIN_SAMPLES CONFIG_INPUT_IQS9151_SCROLL_INERTIA_MIN_SAMPLES
#define SCROLL_INERTIA_MIN_AVG_SPEED CONFIG_INPUT_IQS9151_SCROLL_INERTIA_MIN_AVG_SPEED

#define CURSOR_INERTIA_INTERVAL_MS 10
#define CURSOR_INERTIA_MAX_DURATION_MS 3000
#define CURSOR_INERTIA_DECAY_NUM CONFIG_INPUT_IQS9151_CURSOR_INERTIA_DECAY
#define CURSOR_INERTIA_DECAY_DEN 1000
#define CURSOR_INERTIA_START_THRESHOLD 2
#define CURSOR_INERTIA_MIN_VELOCITY 2
#define CURSOR_EMA_ALPHA 30
#define CURSOR_INERTIA_RECENT_WINDOW_MS CONFIG_INPUT_IQS9151_CURSOR_INERTIA_RECENT_WINDOW_MS
#define CURSOR_INERTIA_STALE_GAP_MS CONFIG_INPUT_IQS9151_CURSOR_INERTIA_STALE_GAP_MS
#define CURSOR_INERTIA_MIN_SAMPLES CONFIG_INPUT_IQS9151_CURSOR_INERTIA_MIN_SAMPLES
#define CURSOR_INERTIA_MIN_AVG_SPEED CONFIG_INPUT_IQS9151_CURSOR_INERTIA_MIN_AVG_SPEED
#define ONE_FINGER_TAP_MAX_MS CONFIG_INPUT_IQS9151_1F_TAP_MAX_MS
#define TWO_FINGER_TAP_MAX_MS CONFIG_INPUT_IQS9151_2F_TAP_MAX_MS
#define IQS9151_TAP_REENTRY_WINDOW_MS 30
#define ONE_FINGER_TAPDRAG_GAP_MAX_MS CONFIG_INPUT_IQS9151_1F_TAPDRAG_GAP_MAX_MS
#define ONE_FINGER_CLICK_HOLD_MAX_MS ONE_FINGER_TAPDRAG_GAP_MAX_MS
#define TWO_FINGER_TAPDRAG_GAP_MAX_MS CONFIG_INPUT_IQS9151_2F_TAPDRAG_GAP_MAX_MS
#define TWO_FINGER_CLICK_HOLD_MAX_MS TWO_FINGER_TAPDRAG_GAP_MAX_MS
#define THREE_FINGER_TAPDRAG_GAP_MAX_MS CONFIG_INPUT_IQS9151_3F_TAPDRAG_GAP_MAX_MS
#define THREE_FINGER_CLICK_HOLD_MAX_MS THREE_FINGER_TAPDRAG_GAP_MAX_MS
#define TWO_FINGER_RELEASE_PENDING_MAX_MS 150
#define THREE_FINGER_RELEASE_PENDING_MAX_MS 150
#define TWO_FINGER_ONE_LEAD_MAX_MS 120
#define THREE_FINGER_ONE_LEAD_MAX_MS 120
#define THREE_FINGER_TWO_LEAD_MAX_MS 120
#define IQS9151_FINGER_HISTORY_SIZE 5
#define THREE_FINGER_TAP_MAX_MS CONFIG_INPUT_IQS9151_3F_TAP_MAX_MS
#define THREE_FINGER_TAP_MOVE CONFIG_INPUT_IQS9151_3F_TAP_MOVE
#define ONE_FINGER_TAP_MOVE CONFIG_INPUT_IQS9151_1F_TAP_MOVE
#define TWO_FINGER_TAP_MOVE CONFIG_INPUT_IQS9151_2F_TAP_MOVE
#define TWO_FINGER_SCROLL_START_MOVE CONFIG_INPUT_IQS9151_2F_SCROLL_START_MOVE
#define TWO_FINGER_PINCH_START_DISTANCE CONFIG_INPUT_IQS9151_2F_PINCH_START_DISTANCE
#define TWO_FINGER_PINCH_WHEEL_DIV 12
#define TWO_FINGER_PINCH_WHEEL_GAIN_X10 CONFIG_INPUT_IQS9151_2F_PINCH_WHEEL_GAIN_X10
#define TWO_FINGER_PINCH_WHEEL_GAIN_DEN 10

struct iqs9151_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec irq_gpio;
};
struct iqs9151_frame {
    int16_t rel_x;
    int16_t rel_y;
    uint16_t info_flags;
    uint16_t trackpad_flags;
    uint8_t finger_count;
    uint16_t finger1_x;
    uint16_t finger1_y;
    uint16_t finger2_x;
    uint16_t finger2_y;
};
enum iqs9151_two_finger_mode {
    IQS9151_2F_MODE_NONE = 0,
    IQS9151_2F_MODE_SCROLL,
    IQS9151_2F_MODE_PINCH,
};
struct iqs9151_one_finger_state {
    bool active;
    bool hold_sent;
    bool tap_candidate;
    bool hold_candidate;
    bool tapdrag_second_touch;
    int64_t down_ms;
    int32_t dx;
    int32_t dy;
    uint16_t last_x;
    uint16_t last_y;
};
struct iqs9151_two_finger_state {
    bool active;
    bool hold_sent;
    bool tap_candidate;
    bool hold_candidate;
    bool tapdrag_second_touch;
    bool release_pending;
    int64_t down_ms;
    int64_t release_pending_ms;
    int32_t centroid_dx;
    int32_t centroid_dy;
    int32_t distance_delta;
    int32_t centroid_last_x;
    int32_t centroid_last_y;
    int32_t distance_last;
    int32_t pinch_wheel_remainder;
    enum iqs9151_two_finger_mode mode;
};
struct iqs9151_two_finger_result {
    bool scroll_active;
    bool scroll_started;
    bool scroll_ended;
    bool pinch_active;
    bool pinch_started;
    bool pinch_ended;
    int16_t scroll_x;
    int16_t scroll_y;
    int16_t pinch_wheel;
};
struct iqs9151_inertia_params {
    uint16_t interval_ms;
    uint16_t max_duration_ms;
    uint16_t decay_num;
    uint16_t decay_den;
    uint8_t fp_shift;
    int16_t start_threshold;
    int16_t min_velocity;
    uint16_t ema_alpha;
};
struct iqs9151_inertia_gate_params {
    uint16_t recent_window_ms;
    uint16_t stale_gap_ms;
    uint8_t min_samples;
    int16_t min_avg_speed;
};
struct iqs9151_inertia_state {
    bool active;
    int32_t vx_fp;
    int32_t vy_fp;
    int32_t accum_x_fp;
    int32_t accum_y_fp;
    int64_t last_ms;
    uint32_t elapsed_ms;
};

struct iqs9151_finger_history_entry {
    int64_t ms;
    uint8_t finger_count;
};
struct iqs9151_motion_sample {
    int64_t ms;
    int16_t x;
    int16_t y;
};
struct iqs9151_motion_history {
    struct iqs9151_motion_sample samples[IQS9151_INERTIA_MOTION_HISTORY_SIZE];
    uint8_t head;
    uint8_t count;
};

struct iqs9151_data {
    const struct device *dev;
    struct gpio_callback gpio_cb;
    struct k_work work;
    struct k_work_delayable one_finger_click_work;
    struct k_work_delayable two_finger_click_work;
    struct k_work_delayable three_finger_click_work;
    struct k_work_delayable inertia_scroll_work;
    struct k_work_delayable inertia_cursor_work;
    struct iqs9151_inertia_state inertia_scroll;
    struct iqs9151_inertia_state inertia_cursor;
    int32_t scroll_ema_x_fp;
    int32_t scroll_ema_y_fp;
    int32_t cursor_ema_x_fp;
    int32_t cursor_ema_y_fp;
    struct iqs9151_motion_history scroll_motion_history;
    struct iqs9151_motion_history cursor_motion_history;
    struct iqs9151_one_finger_state one_finger;
    struct iqs9151_two_finger_state two_finger;
    bool one_finger_click_pending;
    int64_t one_finger_click_pending_ms;
    bool two_finger_click_pending;
    int64_t two_finger_click_pending_ms;
    bool three_finger_click_pending;
    int64_t three_finger_click_pending_ms;
    bool two_finger_one_lead_valid;
    bool two_finger_tail_suppresses_cursor;
    bool three_finger_one_lead_valid;
    bool three_finger_two_lead_valid;
    struct iqs9151_frame prev_frame;
    bool three_active;
    bool three_hold_sent;
    bool three_swipe_sent;
    bool three_tap_candidate;
    bool three_hold_candidate;
    bool three_tapdrag_second_touch;
    bool three_release_pending;
    bool three_have_last;
    int64_t three_down_ms;
    int64_t three_release_pending_ms;
    int32_t three_dx;
    int32_t three_dy;
    uint16_t three_last_x;
    uint16_t three_last_y;
    uint16_t hold_button;
    struct iqs9151_finger_history_entry finger_history[IQS9151_FINGER_HISTORY_SIZE];
    uint8_t finger_history_head;
    uint8_t finger_history_count;
};

#ifdef CONFIG_INPUT_IQS9151_TEST
static struct {
    iqs9151_test_event_hook_t hook;
    void *user_data;
} iqs9151_test_hook;
#endif

static int iqs9151_report_key_event(const struct device *dev, uint16_t code,
                                    int32_t value, bool sync, k_timeout_t timeout) {
#ifdef CONFIG_INPUT_IQS9151_TEST
    if (iqs9151_test_hook.hook != NULL) {
        const struct iqs9151_test_event event = {
            .type = IQS9151_TEST_EVENT_KEY,
            .dev = dev,
            .code = code,
            .value = !!value,
            .sync = sync,
            .timeout = timeout,
        };
        iqs9151_test_hook.hook(&event, iqs9151_test_hook.user_data);
        return 0;
    }
#endif
    return input_report_key(dev, code, value, sync, timeout);
}

static int iqs9151_report_rel_event(const struct device *dev, uint16_t code,
                                    int32_t value, bool sync, k_timeout_t timeout) {
#ifdef CONFIG_INPUT_IQS9151_TEST
    if (iqs9151_test_hook.hook != NULL) {
        const struct iqs9151_test_event event = {
            .type = IQS9151_TEST_EVENT_REL,
            .dev = dev,
            .code = code,
            .value = value,
            .sync = sync,
            .timeout = timeout,
        };
        iqs9151_test_hook.hook(&event, iqs9151_test_hook.user_data);
        return 0;
    }
#endif
    return input_report_rel(dev, code, value, sync, timeout);
}

static const uint8_t iqs9151_alp_compensation[] = {
    ALP_COMPENSATION_RX0_0,  ALP_COMPENSATION_RX0_1,  ALP_COMPENSATION_RX1_0,
    ALP_COMPENSATION_RX1_1,  ALP_COMPENSATION_RX2_0,  ALP_COMPENSATION_RX2_1,
    ALP_COMPENSATION_RX3_0,  ALP_COMPENSATION_RX3_1,  ALP_COMPENSATION_RX4_0,
    ALP_COMPENSATION_RX4_1,  ALP_COMPENSATION_RX5_0,  ALP_COMPENSATION_RX5_1,
    ALP_COMPENSATION_RX6_0,  ALP_COMPENSATION_RX6_1,  ALP_COMPENSATION_RX7_0,
    ALP_COMPENSATION_RX7_1,  ALP_COMPENSATION_RX8_0,  ALP_COMPENSATION_RX8_1,
    ALP_COMPENSATION_RX9_0,  ALP_COMPENSATION_RX9_1,  ALP_COMPENSATION_RX10_0,
    ALP_COMPENSATION_RX10_1, ALP_COMPENSATION_RX11_0, ALP_COMPENSATION_RX11_1,
    ALP_COMPENSATION_RX12_0, ALP_COMPENSATION_RX12_1,
};
static const uint8_t iqs9151_main_config[] = {
    MINOR_VERSION,
    MAJOR_VERSION,
    TP_ATI_MULTDIV_L,
    TP_ATI_MULTDIV_H,
    ALP_ATI_COARSE_RX0_L,
    ALP_ATI_COARSE_RX0_H,
    ALP_ATI_COARSE_RX1_L,
    ALP_ATI_COARSE_RX1_H,
    ALP_ATI_COARSE_RX2_L,
    ALP_ATI_COARSE_RX2_H,
    ALP_ATI_COARSE_RX3_L,
    ALP_ATI_COARSE_RX3_H,
    ALP_ATI_COARSE_RX4_L,
    ALP_ATI_COARSE_RX4_H,
    ALP_ATI_COARSE_RX5_L,
    ALP_ATI_COARSE_RX5_H,
    ALP_ATI_COARSE_RX6_L,
    ALP_ATI_COARSE_RX6_H,
    ALP_ATI_COARSE_RX7_L,
    ALP_ATI_COARSE_RX7_H,
    ALP_ATI_COARSE_RX8_L,
    ALP_ATI_COARSE_RX8_H,
    ALP_ATI_COARSE_RX9_L,
    ALP_ATI_COARSE_RX9_H,
    ALP_ATI_COARSE_RX10_L,
    ALP_ATI_COARSE_RX10_H,
    ALP_ATI_COARSE_RX11_L,
    ALP_ATI_COARSE_RX11_H,
    ALP_ATI_COARSE_RX12_L,
    ALP_ATI_COARSE_RX12_H,
    TP_ATI_TARGET_0,
    TP_ATI_TARGET_1,
    ALP_ATI_TARGET_0,
    ALP_ATI_TARGET_1,
    ALP_BASE_TARGET_0,
    ALP_BASE_TARGET_1,
    TP_NEG_DELTA_REATI_0,
    TP_NEG_DELTA_REATI_1,
    TP_POS_DELTA_REATI_0,
    TP_POS_DELTA_REATI_1,
    TP_REF_DRIFT_LIMIT,
    ALP_LTA_DRIFT_LIMIT,
    ACTIVE_MODE_SAMPLING_PERIOD_0,
    ACTIVE_MODE_SAMPLING_PERIOD_1,
    IDLE_TOUCH_MODE_SAMPLING_PERIOD_0,
    IDLE_TOUCH_MODE_SAMPLING_PERIOD_1,
    IDLE_MODE_SAMPLING_PERIOD_0,
    IDLE_MODE_SAMPLING_PERIOD_1,
    LP1_MODE_SAMPLING_PERIOD_0,
    LP1_MODE_SAMPLING_PERIOD_1,
    LP2_MODE_SAMPLING_PERIOD_0,
    LP2_MODE_SAMPLING_PERIOD_1,
    STATIONARY_TOUCH_TIMEOUT_0,
    STATIONARY_TOUCH_TIMEOUT_1,
    IDLE_TOUCH_MODE_TIMEOUT_0,
    IDLE_TOUCH_MODE_TIMEOUT_1,
    IDLE_MODE_TIMEOUT_0,
    IDLE_MODE_TIMEOUT_1,
    LP1_MODE_TIMEOUT_0,
    LP1_MODE_TIMEOUT_1,
    ACTIVE_MODE_TIMEOUT_0,
    ACTIVE_MODE_TIMEOUT_1,
    REATI_RETRY_TIME,
    REF_UPDATE_TIME,
    I2C_TIMEOUT_0,
    I2C_TIMEOUT_1,
    SNAP_TIMEOUT,
    OPEN_TIMING,
    SYSTEM_CONTROL_0,
    SYSTEM_CONTROL_1,
    CONFIG_SETTINGS_0,
    CONFIG_SETTINGS_1,
    OTHER_SETTINGS_0,
    OTHER_SETTINGS_1,
    ALP_SETUP_0,
    ALP_SETUP_1,
    ALP_SETUP_2,
    ALP_SETUP_3,
    ALP_TX_ENABLE_0,
    ALP_TX_ENABLE_1,
    ALP_TX_ENABLE_2,
    ALP_TX_ENABLE_3,
    ALP_TX_ENABLE_4,
    ALP_TX_ENABLE_5,
    TRACKPAD_TOUCH_SET_THRESHOLD,
    TRACKPAD_TOUCH_CLEAR_THRESHOLD,
    ALP_THRESHOLD,
    ALP_AUTOPROX_THRESHOLD,
    ALP_SET_DEBOUNCE,
    ALP_CLEAR_DEBOUNCE,
    SNAP_SET_THRESHOLD,
    SNAP_CLEAR_THRESHOLD,
    ALP_COUNT_BETA_LP1,
    ALP_LTA_BETA_LP1,
    ALP_COUNT_BETA_LP2,
    ALP_LTA_BETA_LP2,
    TP_FRAC,
    TP_PERIOD1,
    TP_PERIOD2,
    ALP_FRAC,
    ALP_PERIOD1,
    ALP_PERIOD2,
    TRACKPAD_HARDWARE_SETTINGS_0,
    TRACKPAD_HARDWARE_SETTINGS_1,
    ALP_HARDWARE_SETTINGS_0,
    ALP_HARDWARE_SETTINGS_1,
    TRACKPAD_SETTINGS_0_0,
    TRACKPAD_SETTINGS_0_1,
    TRACKPAD_SETTINGS_1_0,
    TRACKPAD_SETTINGS_1_1,
    X_RESOLUTION_0,
    X_RESOLUTION_1,
    Y_RESOLUTION_0,
    Y_RESOLUTION_1,
    XY_DYNAMIC_FILTER_BOTTOM_SPEED_0,
    XY_DYNAMIC_FILTER_BOTTOM_SPEED_1,
    XY_DYNAMIC_FILTER_TOP_SPEED_0,
    XY_DYNAMIC_FILTER_TOP_SPEED_1,
    XY_DYNAMIC_FILTER_BOTTOM_BETA,
    XY_DYNAMIC_FILTER_STATIC_FILTER_BETA,
    STATIONARY_TOUCH_MOV_THRESHOLD,
    FINGER_SPLIT_FACTOR,
    X_TRIM_VALUE,
    Y_TRIM_VALUE,
    JITTER_FILTER_DELTA,
    FINGER_CONFIDENCE_THRESHOLD,
};
static const uint8_t iqs9151_rxtx_map[] = {
    RX_TX_MAP_0,  RX_TX_MAP_1,  RX_TX_MAP_2,  RX_TX_MAP_3,  RX_TX_MAP_4,
    RX_TX_MAP_5,  RX_TX_MAP_6,  RX_TX_MAP_7,  RX_TX_MAP_8,  RX_TX_MAP_9,
    RX_TX_MAP_10, RX_TX_MAP_11, RX_TX_MAP_12, RX_TX_MAP_13, RX_TX_MAP_14,
    RX_TX_MAP_15, RX_TX_MAP_16, RX_TX_MAP_17, RX_TX_MAP_18, RX_TX_MAP_19,
    RX_TX_MAP_20, RX_TX_MAP_21, RX_TX_MAP_22, RX_TX_MAP_23, RX_TX_MAP_24,
    RX_TX_MAP_25, RX_TX_MAP_26, RX_TX_MAP_27, RX_TX_MAP_28, RX_TX_MAP_29,
    RX_TX_MAP_30, RX_TX_MAP_31, RX_TX_MAP_32, RX_TX_MAP_33, RX_TX_MAP_34,
    RX_TX_MAP_35, RX_TX_MAP_36, RX_TX_MAP_37, RX_TX_MAP_38, RX_TX_MAP_39,
    RX_TX_MAP_40, RX_TX_MAP_41, RX_TX_MAP_42, RX_TX_MAP_43, RX_TX_MAP_44,
    RX_TX_OPEN,
};
static const uint8_t iqs9151_channel_disable[] = {
    TPCHANNELDISABLE_0,  TPCHANNELDISABLE_1,  TPCHANNELDISABLE_2,
    TPCHANNELDISABLE_3,  TPCHANNELDISABLE_4,  TPCHANNELDISABLE_5,
    TPCHANNELDISABLE_6,  TPCHANNELDISABLE_7,  TPCHANNELDISABLE_8,
    TPCHANNELDISABLE_9,  TPCHANNELDISABLE_10, TPCHANNELDISABLE_11,
    TPCHANNELDISABLE_12, TPCHANNELDISABLE_13, TPCHANNELDISABLE_14,
    TPCHANNELDISABLE_15, TPCHANNELDISABLE_16, TPCHANNELDISABLE_17,
    TPCHANNELDISABLE_18, TPCHANNELDISABLE_19, TPCHANNELDISABLE_20,
    TPCHANNELDISABLE_21, TPCHANNELDISABLE_22, TPCHANNELDISABLE_23,
    TPCHANNELDISABLE_24, TPCHANNELDISABLE_25, TPCHANNELDISABLE_26,
    TPCHANNELDISABLE_27, TPCHANNELDISABLE_28, TPCHANNELDISABLE_29,
    TPCHANNELDISABLE_30, TPCHANNELDISABLE_31, TPCHANNELDISABLE_32,
    TPCHANNELDISABLE_33, TPCHANNELDISABLE_34, TPCHANNELDISABLE_35,
    TPCHANNELDISABLE_36, TPCHANNELDISABLE_37, TPCHANNELDISABLE_38,
    TPCHANNELDISABLE_39, TPCHANNELDISABLE_40, TPCHANNELDISABLE_41,
    TPCHANNELDISABLE_42, TPCHANNELDISABLE_43, TPCHANNELDISABLE_44,
    TPCHANNELDISABLE_45, TPCHANNELDISABLE_46, TPCHANNELDISABLE_47,
    TPCHANNELDISABLE_48, TPCHANNELDISABLE_49, TPCHANNELDISABLE_50,
    TPCHANNELDISABLE_51, TPCHANNELDISABLE_52, TPCHANNELDISABLE_53,
    TPCHANNELDISABLE_54, TPCHANNELDISABLE_55, TPCHANNELDISABLE_56,
    TPCHANNELDISABLE_57, TPCHANNELDISABLE_58, TPCHANNELDISABLE_59,
    TPCHANNELDISABLE_60, TPCHANNELDISABLE_61, TPCHANNELDISABLE_62,
    TPCHANNELDISABLE_63, TPCHANNELDISABLE_64, TPCHANNELDISABLE_65,
    TPCHANNELDISABLE_66, TPCHANNELDISABLE_67, TPCHANNELDISABLE_68,
    TPCHANNELDISABLE_69, TPCHANNELDISABLE_70, TPCHANNELDISABLE_71,
    TPCHANNELDISABLE_72, TPCHANNELDISABLE_73, TPCHANNELDISABLE_74,
    TPCHANNELDISABLE_75, TPCHANNELDISABLE_76, TPCHANNELDISABLE_77,
    TPCHANNELDISABLE_78, TPCHANNELDISABLE_79, TPCHANNELDISABLE_80,
    TPCHANNELDISABLE_81, TPCHANNELDISABLE_82, TPCHANNELDISABLE_83,
    TPCHANNELDISABLE_84, TPCHANNELDISABLE_85, TPCHANNELDISABLE_86,
    TPCHANNELDISABLE_87,
};
static const uint8_t iqs9151_snap_enable[] = {
    SNAPCHANNELENABLE_0,  SNAPCHANNELENABLE_1,  SNAPCHANNELENABLE_2,
    SNAPCHANNELENABLE_3,  SNAPCHANNELENABLE_4,  SNAPCHANNELENABLE_5,
    SNAPCHANNELENABLE_6,  SNAPCHANNELENABLE_7,  SNAPCHANNELENABLE_8,
    SNAPCHANNELENABLE_9,  SNAPCHANNELENABLE_10, SNAPCHANNELENABLE_11,
    SNAPCHANNELENABLE_12, SNAPCHANNELENABLE_13, SNAPCHANNELENABLE_14,
    SNAPCHANNELENABLE_15, SNAPCHANNELENABLE_16, SNAPCHANNELENABLE_17,
    SNAPCHANNELENABLE_18, SNAPCHANNELENABLE_19, SNAPCHANNELENABLE_20,
    SNAPCHANNELENABLE_21, SNAPCHANNELENABLE_22, SNAPCHANNELENABLE_23,
    SNAPCHANNELENABLE_24, SNAPCHANNELENABLE_25, SNAPCHANNELENABLE_26,
    SNAPCHANNELENABLE_27, SNAPCHANNELENABLE_28, SNAPCHANNELENABLE_29,
    SNAPCHANNELENABLE_30, SNAPCHANNELENABLE_31, SNAPCHANNELENABLE_32,
    SNAPCHANNELENABLE_33, SNAPCHANNELENABLE_34, SNAPCHANNELENABLE_35,
    SNAPCHANNELENABLE_36, SNAPCHANNELENABLE_37, SNAPCHANNELENABLE_38,
    SNAPCHANNELENABLE_39, SNAPCHANNELENABLE_40, SNAPCHANNELENABLE_41,
    SNAPCHANNELENABLE_42, SNAPCHANNELENABLE_43, SNAPCHANNELENABLE_44,
    SNAPCHANNELENABLE_45, SNAPCHANNELENABLE_46, SNAPCHANNELENABLE_47,
    SNAPCHANNELENABLE_48, SNAPCHANNELENABLE_49, SNAPCHANNELENABLE_50,
    SNAPCHANNELENABLE_51, SNAPCHANNELENABLE_52, SNAPCHANNELENABLE_53,
    SNAPCHANNELENABLE_54, SNAPCHANNELENABLE_55, SNAPCHANNELENABLE_56,
    SNAPCHANNELENABLE_57, SNAPCHANNELENABLE_58, SNAPCHANNELENABLE_59,
    SNAPCHANNELENABLE_60, SNAPCHANNELENABLE_61, SNAPCHANNELENABLE_62,
    SNAPCHANNELENABLE_63, SNAPCHANNELENABLE_64, SNAPCHANNELENABLE_65,
    SNAPCHANNELENABLE_66, SNAPCHANNELENABLE_67, SNAPCHANNELENABLE_68,
    SNAPCHANNELENABLE_69, SNAPCHANNELENABLE_70, SNAPCHANNELENABLE_71,
    SNAPCHANNELENABLE_72, SNAPCHANNELENABLE_73, SNAPCHANNELENABLE_74,
    SNAPCHANNELENABLE_75, SNAPCHANNELENABLE_76, SNAPCHANNELENABLE_77,
    SNAPCHANNELENABLE_78, SNAPCHANNELENABLE_79, SNAPCHANNELENABLE_80,
    SNAPCHANNELENABLE_81, SNAPCHANNELENABLE_82, SNAPCHANNELENABLE_83,
    SNAPCHANNELENABLE_84, SNAPCHANNELENABLE_85, SNAPCHANNELENABLE_86,
    SNAPCHANNELENABLE_87,
};
static const struct iqs9151_inertia_params iqs9151_scroll_params = {
    .interval_ms = SCROLL_INERTIA_INTERVAL_MS,
    .max_duration_ms = SCROLL_INERTIA_MAX_DURATION_MS,
    .decay_num = SCROLL_INERTIA_DECAY_NUM,
    .decay_den = SCROLL_INERTIA_DECAY_DEN,
    .fp_shift = INERTIA_FP_SHIFT,
    .start_threshold = SCROLL_INERTIA_START_THRESHOLD,
    .min_velocity = SCROLL_INERTIA_MIN_VELOCITY,
    .ema_alpha = SCROLL_EMA_ALPHA,
};
static const struct iqs9151_inertia_gate_params iqs9151_scroll_gate_params = {
    .recent_window_ms = SCROLL_INERTIA_RECENT_WINDOW_MS,
    .stale_gap_ms = SCROLL_INERTIA_STALE_GAP_MS,
    .min_samples = SCROLL_INERTIA_MIN_SAMPLES,
    .min_avg_speed = SCROLL_INERTIA_MIN_AVG_SPEED,
};
static const struct iqs9151_inertia_params iqs9151_cursor_params = {
    .interval_ms = CURSOR_INERTIA_INTERVAL_MS,
    .max_duration_ms = CURSOR_INERTIA_MAX_DURATION_MS,
    .decay_num = CURSOR_INERTIA_DECAY_NUM,
    .decay_den = CURSOR_INERTIA_DECAY_DEN,
    .fp_shift = INERTIA_FP_SHIFT,
    .start_threshold = CURSOR_INERTIA_START_THRESHOLD,
    .min_velocity = CURSOR_INERTIA_MIN_VELOCITY,
    .ema_alpha = CURSOR_EMA_ALPHA,
};
static const struct iqs9151_inertia_gate_params iqs9151_cursor_gate_params = {
    .recent_window_ms = CURSOR_INERTIA_RECENT_WINDOW_MS,
    .stale_gap_ms = CURSOR_INERTIA_STALE_GAP_MS,
    .min_samples = CURSOR_INERTIA_MIN_SAMPLES,
    .min_avg_speed = CURSOR_INERTIA_MIN_AVG_SPEED,
};

static int iqs9151_i2c_write(const struct iqs9151_config *cfg, uint16_t reg, const uint8_t *buf, size_t len) {
    uint8_t tx[2 + IQS9151_I2C_CHUNK_SIZE];

    if (len > (sizeof(tx) - 2)) {
        return -EINVAL;
    }

    sys_put_le16(reg, tx);
    memcpy(&tx[2], buf, len);
    return i2c_write_dt(&cfg->i2c, tx, len + 2);
}

static int iqs9151_i2c_read(const struct iqs9151_config *cfg, uint16_t reg, uint8_t *buf, size_t len) {
    uint8_t addr_buf[2];

    sys_put_le16(reg, addr_buf);
    return i2c_write_read_dt(&cfg->i2c, addr_buf, sizeof(addr_buf), buf, len);
}

static int iqs9151_write_u16(const struct iqs9151_config *cfg, uint16_t reg, uint16_t value) {
    uint8_t buf[2];

    sys_put_le16(value, buf);
    return iqs9151_i2c_write(cfg, reg, buf, sizeof(buf));
}

static int iqs9151_read_u16(const struct iqs9151_config *cfg, uint16_t reg, uint16_t *value) {
    uint8_t buf[2];
    int ret = iqs9151_i2c_read(cfg, reg, buf, sizeof(buf));

    if (ret != 0) {
        return ret;
    }

    *value = sys_get_le16(buf);
    return 0;
}

static int iqs9151_update_bits_u16(const struct iqs9151_config *cfg, uint16_t reg,
                                   uint16_t mask, uint16_t value) {
    uint16_t current;
    int ret = iqs9151_read_u16(cfg, reg, &current);

    if (ret != 0) {
        return ret;
    }

    current = (uint16_t)((current & ~mask) | (value & mask));
    return iqs9151_write_u16(cfg, reg, current);
}

static void iqs9151_wait_for_ready(const struct device *dev, uint16_t timeout_ms) {
    const struct iqs9151_config *cfg = dev->config;
    uint16_t elapsed = 0;

    while (!gpio_pin_get_dt(&cfg->irq_gpio) && elapsed < timeout_ms) {
        k_sleep(K_MSEC(1));
        elapsed++;
    }

    if (elapsed >= timeout_ms) {
        LOG_WRN("RDY timeout after %dms", timeout_ms);
    }
    LOG_DBG("IRQGPIO=%d,TIME=%dms", gpio_pin_get_dt(&cfg->irq_gpio), elapsed);
}

static int iqs9151_write_chunks(const struct device *dev, const struct iqs9151_config *cfg
                                    , uint16_t start_reg, const uint8_t *buf, size_t len) {
    size_t offset = 0U;

    while (offset < len) {
        const size_t chunk_len = MIN(IQS9151_I2C_CHUNK_SIZE, len - offset);
        
        iqs9151_wait_for_ready(dev, 200);

        const int ret = iqs9151_i2c_write(cfg, start_reg + offset, buf + offset, chunk_len);
        if (ret != 0) {
            return ret;
        }
        offset += chunk_len;
    }
    return 0;
}

static int iqs9151_check_product_number(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    uint8_t product[2];
    int ret;
    
    ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_PRODUCT_NUMBER, product, sizeof(product));
    if (ret != 0) {
        return ret;
    }

    uint16_t product_num = sys_get_le16(product);
    if (ret == 0 && product_num != IQS9151_PRODUCT_NUMBER) {
        LOG_ERR("unexpected product number 0x%04x", product_num);
        return -ENODEV;
    }

    LOG_DBG("product number 0x%04x", product_num);
    return ret;
}

static void iqs9151_parse_frame(const uint8_t *raw, struct iqs9151_frame *frame) {
    frame->rel_x = (int16_t)sys_get_le16(&raw[0]);
    frame->rel_y = (int16_t)sys_get_le16(&raw[2]);
    frame->info_flags = sys_get_le16(&raw[12]);
    frame->trackpad_flags = sys_get_le16(&raw[14]);
    frame->finger_count =
        (uint8_t)(frame->trackpad_flags & IQS9151_TP_FINGER_COUNT_MASK);
    frame->finger1_x = sys_get_le16(&raw[16]);
    frame->finger1_y = sys_get_le16(&raw[18]);
    frame->finger2_x = sys_get_le16(&raw[24]);
    frame->finger2_y = sys_get_le16(&raw[26]);
}

static bool iqs9151_finger1_valid(const struct iqs9151_frame *frame) {
    const bool finger1_confident =
        (frame->trackpad_flags & IQS9151_TP_FINGER1_CONFIDENCE) != 0U;
    const bool finger1_coord_valid =
        (frame->finger1_x != UINT16_MAX) && (frame->finger1_y != UINT16_MAX);

    return finger1_confident && finger1_coord_valid;
}

static bool iqs9151_finger2_valid(const struct iqs9151_frame *frame) {
    const bool finger2_confident =
        (frame->trackpad_flags & IQS9151_TP_FINGER2_CONFIDENCE) != 0U;
    const bool finger2_coord_valid =
        (frame->finger2_x != UINT16_MAX) && (frame->finger2_y != UINT16_MAX);

    return finger2_confident && finger2_coord_valid;
}

static void iqs9151_update_prev_frame(struct iqs9151_data *data,
                                      const struct iqs9151_frame *frame,
                                      const struct iqs9151_frame *prev_frame) {
    data->prev_frame = *frame;

    if (frame->finger_count == 0U) {
        data->prev_frame.finger1_x = 0;
        data->prev_frame.finger1_y = 0;
        data->prev_frame.finger2_x = 0;
        data->prev_frame.finger2_y = 0;
        return;
    }

    if (!iqs9151_finger1_valid(frame)) {
        data->prev_frame.finger1_x = prev_frame->finger1_x;
        data->prev_frame.finger1_y = prev_frame->finger1_y;
    }

    if (frame->finger_count < 2U) {
        data->prev_frame.finger2_x = 0;
        data->prev_frame.finger2_y = 0;
        return;
    }

    if (!iqs9151_finger2_valid(frame)) {
        data->prev_frame.finger2_x = prev_frame->finger2_x;
        data->prev_frame.finger2_y = prev_frame->finger2_y;
    }
}

static int32_t iqs9151_abs32(int32_t value) {
    return (value < 0) ? -value : value;
}

static void iqs9151_ema_reset(int32_t *ema_x_fp, int32_t *ema_y_fp) {
    *ema_x_fp = 0;
    *ema_y_fp = 0;
}

static void iqs9151_ema_update(int32_t *ema_x_fp, int32_t *ema_y_fp,
                               int16_t sample_x, int16_t sample_y, uint16_t alpha) {
    const int32_t sample_x_fp = ((int32_t)sample_x) << EMA_FP_SHIFT;
    const int32_t sample_y_fp = ((int32_t)sample_y) << EMA_FP_SHIFT;
    const int32_t inv_alpha = (int32_t)(EMA_ALPHA_DEN - alpha);

    *ema_x_fp =
        ((*ema_x_fp * (int32_t)alpha) + (sample_x_fp * inv_alpha)) >> EMA_FP_SHIFT;
    *ema_y_fp =
        ((*ema_y_fp * (int32_t)alpha) + (sample_y_fp * inv_alpha)) >> EMA_FP_SHIFT;
}

static void iqs9151_motion_history_reset(struct iqs9151_motion_history *history) {
    memset(history, 0, sizeof(*history));
}

static void iqs9151_motion_history_push(struct iqs9151_motion_history *history,
                                        int16_t sample_x, int16_t sample_y,
                                        int64_t now_ms) {
    struct iqs9151_motion_sample *entry;

    if (sample_x == 0 && sample_y == 0) {
        return;
    }

    entry = &history->samples[history->head];
    entry->ms = now_ms;
    entry->x = sample_x;
    entry->y = sample_y;

    history->head =
        (uint8_t)((history->head + 1U) % IQS9151_INERTIA_MOTION_HISTORY_SIZE);
    if (history->count < IQS9151_INERTIA_MOTION_HISTORY_SIZE) {
        history->count++;
    }
}

static bool iqs9151_inertia_seed_from_history(
    const struct iqs9151_motion_history *history,
    const struct iqs9151_inertia_params *params,
    const struct iqs9151_inertia_gate_params *gate,
    int64_t now_ms,
    int32_t *seed_vx_fp,
    int32_t *seed_vy_fp) {
    struct iqs9151_motion_sample recent[IQS9151_INERTIA_MOTION_HISTORY_SIZE];
    uint8_t recent_count = 0U;
    int32_t total_x = 0;
    int32_t total_y = 0;
    int64_t latest_ms;
    int64_t earliest_ms;
    int64_t span_ms;
    int32_t avg_speed;
    int32_t dominant_total;
    uint8_t consistent_count = 0U;

    for (uint8_t i = 0U; i < history->count; i++) {
        const uint8_t idx =
            (uint8_t)((history->head + IQS9151_INERTIA_MOTION_HISTORY_SIZE - 1U - i) %
                      IQS9151_INERTIA_MOTION_HISTORY_SIZE);
        const struct iqs9151_motion_sample *entry = &history->samples[idx];
        const int64_t elapsed_ms = now_ms - entry->ms;

        if (elapsed_ms > gate->recent_window_ms) {
            break;
        }

        recent[recent_count++] = *entry;
    }

    if (recent_count < gate->min_samples) {
        return false;
    }

    latest_ms = recent[0].ms;
    if ((now_ms - latest_ms) > gate->stale_gap_ms) {
        return false;
    }

    earliest_ms = recent[recent_count - 1U].ms;
    for (uint8_t i = 0U; i < recent_count; i++) {
        total_x += recent[i].x;
        total_y += recent[i].y;
    }

    if (total_x == 0 && total_y == 0) {
        return false;
    }

    if (iqs9151_abs32(total_x) >= iqs9151_abs32(total_y)) {
        dominant_total = total_x;
        for (uint8_t i = 0U; i < recent_count; i++) {
            if ((recent[i].x > 0 && dominant_total > 0) ||
                (recent[i].x < 0 && dominant_total < 0)) {
                consistent_count++;
            }
        }
    } else {
        dominant_total = total_y;
        for (uint8_t i = 0U; i < recent_count; i++) {
            if ((recent[i].y > 0 && dominant_total > 0) ||
                (recent[i].y < 0 && dominant_total < 0)) {
                consistent_count++;
            }
        }
    }

    if (consistent_count < gate->min_samples) {
        return false;
    }

    span_ms = latest_ms - earliest_ms;
    if (span_ms < params->interval_ms) {
        span_ms = params->interval_ms;
    }

    avg_speed = (int32_t)(((int64_t)(iqs9151_abs32(total_x) + iqs9151_abs32(total_y)) *
                           params->interval_ms) /
                          span_ms);
    if (avg_speed < gate->min_avg_speed) {
        return false;
    }

    *seed_vx_fp = (int32_t)((((int64_t)total_x * params->interval_ms) << params->fp_shift) /
                            span_ms);
    *seed_vy_fp = (int32_t)((((int64_t)total_y * params->interval_ms) << params->fp_shift) /
                            span_ms);
    return true;
}

static void iqs9151_inertia_state_reset(struct iqs9151_inertia_state *state) {
    state->active = false;
    state->vx_fp = 0;
    state->vy_fp = 0;
    state->accum_x_fp = 0;
    state->accum_y_fp = 0;
    state->last_ms = 0;
    state->elapsed_ms = 0U;
}

static void iqs9151_inertia_cancel(struct iqs9151_inertia_state *state,
                                   struct k_work_delayable *work) {
    iqs9151_inertia_state_reset(state);
    k_work_cancel_delayable(work);
}

static void iqs9151_release_hold(struct iqs9151_data *data, const struct device *dev) {
    if (data->hold_button == 0U) {
        return;
    }

    iqs9151_report_key_event(dev, data->hold_button, false, true, K_NO_WAIT);
    data->hold_button = 0U;
}

static void iqs9151_clear_one_finger_click_pending(struct iqs9151_data *data) {
    data->one_finger_click_pending = false;
    data->one_finger_click_pending_ms = 0;
}

static void iqs9151_clear_two_finger_click_pending(struct iqs9151_data *data) {
    data->two_finger_click_pending = false;
    data->two_finger_click_pending_ms = 0;
}

static void iqs9151_clear_three_finger_click_pending(struct iqs9151_data *data) {
    data->three_finger_click_pending = false;
    data->three_finger_click_pending_ms = 0;
}

static void iqs9151_reset_finger_history(struct iqs9151_data *data) {
    memset(data->finger_history, 0, sizeof(data->finger_history));
    data->finger_history_head = 0U;
    data->finger_history_count = 0U;
}

static void iqs9151_push_finger_history(struct iqs9151_data *data,
                                        uint8_t finger_count,
                                        int64_t now_ms) {
    struct iqs9151_finger_history_entry *entry =
        &data->finger_history[data->finger_history_head];

    entry->ms = now_ms;
    entry->finger_count = finger_count;

    data->finger_history_head =
        (uint8_t)((data->finger_history_head + 1U) % IQS9151_FINGER_HISTORY_SIZE);
    if (data->finger_history_count < IQS9151_FINGER_HISTORY_SIZE) {
        data->finger_history_count++;
    }
}

static bool iqs9151_has_recent_finger_count(const struct iqs9151_data *data,
                                            uint8_t finger_count,
                                            int64_t now_ms,
                                            int32_t window_ms) {
    for (uint8_t i = 0U; i < data->finger_history_count; i++) {
        const uint8_t idx =
            (uint8_t)((data->finger_history_head + IQS9151_FINGER_HISTORY_SIZE - 1U - i) %
                      IQS9151_FINGER_HISTORY_SIZE);
        const struct iqs9151_finger_history_entry *entry = &data->finger_history[idx];
        const int64_t elapsed_ms = now_ms - entry->ms;

        if (elapsed_ms > window_ms) {
            break;
        }
        if (entry->finger_count == finger_count) {
            return true;
        }
    }

    return false;
}

static bool iqs9151_try_tap_hold_emit(struct iqs9151_data *data,
                                      const struct device *dev) {
    if (data->hold_button == 0U) {
        return true;
    }

    /*
     * Hold is latched until another Tap/Hold condition is recognized.
     * In that case, release the current hold and suppress the new event.
     */
    iqs9151_release_hold(data, dev);
    return false;
}

static bool iqs9151_emit_click(struct iqs9151_data *data,
                               const struct device *dev,
                               uint16_t button) {
    if (!iqs9151_try_tap_hold_emit(data, dev)) {
        return false;
    }

    iqs9151_report_key_event(dev, button, true, true, K_FOREVER);
    iqs9151_report_key_event(dev, button, false, true, K_FOREVER);
    return true;
}

static bool iqs9151_emit_hold_press(struct iqs9151_data *data,
                                    const struct device *dev,
                                    uint16_t button) {
    if (!iqs9151_try_tap_hold_emit(data, dev)) {
        return false;
    }

    iqs9151_report_key_event(dev, button, true, true, K_FOREVER);
    data->hold_button = button;
    return true;
}

static bool iqs9151_get_finger1_xy(const struct iqs9151_frame *frame,
                                   const struct iqs9151_frame *prev_frame,
                                   uint16_t *x, uint16_t *y) {
    if (iqs9151_finger1_valid(frame)) {
        *x = frame->finger1_x;
        *y = frame->finger1_y;
        return true;
    }

    if (iqs9151_finger1_valid(prev_frame)) {
        *x = prev_frame->finger1_x;
        *y = prev_frame->finger1_y;
        return true;
    }

    return false;
}

static bool iqs9151_get_finger2_xy(const struct iqs9151_frame *frame,
                                   const struct iqs9151_frame *prev_frame,
                                   uint16_t *x, uint16_t *y) {
    if (iqs9151_finger2_valid(frame)) {
        *x = frame->finger2_x;
        *y = frame->finger2_y;
        return true;
    }

    if (iqs9151_finger2_valid(prev_frame)) {
        *x = prev_frame->finger2_x;
        *y = prev_frame->finger2_y;
        return true;
    }

    return false;
}

static int32_t iqs9151_two_finger_distance(uint16_t x1, uint16_t y1,
                                           uint16_t x2, uint16_t y2) {
    const int32_t dx = (int32_t)x1 - (int32_t)x2;
    const int32_t dy = (int32_t)y1 - (int32_t)y2;

    return iqs9151_abs32(dx) + iqs9151_abs32(dy);
}

static void iqs9151_one_finger_reset(struct iqs9151_one_finger_state *state) {
    state->active = false;
    state->hold_sent = false;
    state->tap_candidate = false;
    state->hold_candidate = false;
    state->tapdrag_second_touch = false;
    state->down_ms = 0;
    state->dx = 0;
    state->dy = 0;
    state->last_x = 0;
    state->last_y = 0;
}

static void iqs9151_two_finger_reset(struct iqs9151_two_finger_state *state) {
    state->active = false;
    state->hold_sent = false;
    state->tap_candidate = false;
    state->hold_candidate = false;
    state->tapdrag_second_touch = false;
    state->release_pending = false;
    state->down_ms = 0;
    state->release_pending_ms = 0;
    state->centroid_dx = 0;
    state->centroid_dy = 0;
    state->distance_delta = 0;
    state->centroid_last_x = 0;
    state->centroid_last_y = 0;
    state->distance_last = 0;
    state->pinch_wheel_remainder = 0;
    state->mode = IQS9151_2F_MODE_NONE;
}

static void iqs9151_two_finger_result_reset(struct iqs9151_two_finger_result *result) {
    memset(result, 0, sizeof(*result));
}

static bool iqs9151_one_finger_update(struct iqs9151_data *data,
                                      const struct iqs9151_frame *frame,
                                      const struct iqs9151_frame *prev_frame,
                                      const struct device *dev) {
    struct iqs9151_one_finger_state *state = &data->one_finger;
    const bool one_now = frame->finger_count == 1U;
    const int64_t now_ms = k_uptime_get();
    uint16_t x = 0U;
    uint16_t y = 0U;
    const bool have_xy = one_now && iqs9151_get_finger1_xy(frame, prev_frame, &x, &y);
    bool released_from_hold = false;
    bool tap_detected = false;
    bool tap_emitted = false;

    if (!state->active && one_now) {
        bool tapdrag_second_touch = false;

        if (!have_xy) {
            return false;
        }
        if (data->one_finger_click_pending) {
            const int64_t armed_elapsed_ms =
                now_ms - data->one_finger_click_pending_ms;

            tapdrag_second_touch = (armed_elapsed_ms >= 0) &&
                                   (armed_elapsed_ms <= ONE_FINGER_CLICK_HOLD_MAX_MS);
            if (!tapdrag_second_touch && data->hold_button == INPUT_BTN_0) {
                iqs9151_release_hold(data, dev);
            }
            iqs9151_clear_one_finger_click_pending(data);
            (void)k_work_cancel_delayable(&data->one_finger_click_work);
        }
        state->active = true;
        state->hold_sent = tapdrag_second_touch;
        state->tap_candidate = !tapdrag_second_touch &&
            ((prev_frame->finger_count == 0U) ||
             iqs9151_has_recent_finger_count(data, 0U, now_ms, IQS9151_TAP_REENTRY_WINDOW_MS));
        state->hold_candidate = tapdrag_second_touch;
        state->tapdrag_second_touch = tapdrag_second_touch;
        state->down_ms = now_ms;
        state->dx = 0;
        state->dy = 0;
        state->last_x = x;
        state->last_y = y;
    }

    if (!state->active) {
        return false;
    }

    if (one_now) {
        const int64_t elapsed_ms = now_ms - state->down_ms;

        if (have_xy) {
            const int32_t step_x = (int32_t)x - (int32_t)state->last_x;
            const int32_t step_y = (int32_t)y - (int32_t)state->last_y;
            state->dx += step_x;
            state->dy += step_y;
            state->last_x = x;
            state->last_y = y;
        }

        if (state->tap_candidate &&
            (elapsed_ms > ONE_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(state->dx) > ONE_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->dy) > ONE_FINGER_TAP_MOVE)) {
            state->tap_candidate = false;
        }
        if (state->tapdrag_second_touch && state->hold_candidate &&
            (elapsed_ms > ONE_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(state->dx) > ONE_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->dy) > ONE_FINGER_TAP_MOVE)) {
            state->hold_candidate = false;
        }
        return false;
    }

    if (state->tapdrag_second_touch) {
        const int64_t elapsed_ms = now_ms - state->down_ms;
        const bool second_tap_detected =
            (frame->finger_count == 0U) &&
            state->hold_candidate &&
            elapsed_ms <= ONE_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(state->dx) <= ONE_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->dy) <= ONE_FINGER_TAP_MOVE;

        released_from_hold = state->hold_sent;
        if (state->hold_sent) {
            iqs9151_release_hold(data, dev);
        }
        if (second_tap_detected &&
            IS_ENABLED(CONFIG_INPUT_IQS9151_1F_TAP_ENABLE)) {
            (void)iqs9151_emit_click(data, dev, INPUT_BTN_0);
        }
        iqs9151_one_finger_reset(state);
        return released_from_hold;
    }

    if (frame->finger_count == 0U && state->tap_candidate) {
        const int64_t elapsed_ms = now_ms - state->down_ms;

        if (elapsed_ms <= ONE_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(state->dx) <= ONE_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->dy) <= ONE_FINGER_TAP_MOVE) {
            tap_detected = true;
            if (IS_ENABLED(CONFIG_INPUT_IQS9151_1F_PRESSHOLD_ENABLE)) {
                tap_emitted = iqs9151_emit_hold_press(data, dev, INPUT_BTN_0);
            } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_1F_TAP_ENABLE)) {
                tap_emitted = iqs9151_emit_click(data, dev, INPUT_BTN_0);
            } else {
                tap_emitted = true;
            }
        }
    }

    if (tap_detected &&
        tap_emitted &&
        IS_ENABLED(CONFIG_INPUT_IQS9151_1F_PRESSHOLD_ENABLE)) {
        data->one_finger_click_pending = true;
        data->one_finger_click_pending_ms = now_ms;
        k_work_reschedule(&data->one_finger_click_work,
                          K_MSEC(ONE_FINGER_CLICK_HOLD_MAX_MS));
    } else if (frame->finger_count != 0U) {
        iqs9151_clear_one_finger_click_pending(data);
        (void)k_work_cancel_delayable(&data->one_finger_click_work);
    }

    iqs9151_one_finger_reset(state);
    return released_from_hold;
}

static void iqs9151_two_finger_update(struct iqs9151_data *data,
                                      const struct iqs9151_frame *frame,
                                      const struct iqs9151_frame *prev_frame,
                                      const struct device *dev,
                                      struct iqs9151_two_finger_result *result) {
    struct iqs9151_two_finger_state *state = &data->two_finger;
    const bool two_now = frame->finger_count == 2U;
    const bool one_lead_tap_candidate = data->two_finger_one_lead_valid;
    const int64_t now_ms = k_uptime_get();
    uint16_t f1x = 0U;
    uint16_t f1y = 0U;
    uint16_t f2x = 0U;
    uint16_t f2y = 0U;
    const bool have_xy = two_now &&
        iqs9151_get_finger1_xy(frame, prev_frame, &f1x, &f1y) &&
        iqs9151_get_finger2_xy(frame, prev_frame, &f2x, &f2y);
    bool tap_detected = false;
    bool tap_emitted = false;

    iqs9151_two_finger_result_reset(result);

    if (!state->active && two_now) {
        bool tapdrag_second_touch = false;

        if (!have_xy) {
            return;
        }
        if (data->two_finger_click_pending) {
            const int64_t armed_elapsed_ms =
                now_ms - data->two_finger_click_pending_ms;

            tapdrag_second_touch = (armed_elapsed_ms >= 0) &&
                                   (armed_elapsed_ms <= TWO_FINGER_CLICK_HOLD_MAX_MS);
            if (!tapdrag_second_touch && data->hold_button == INPUT_BTN_1) {
                iqs9151_release_hold(data, dev);
            }
            iqs9151_clear_two_finger_click_pending(data);
            (void)k_work_cancel_delayable(&data->two_finger_click_work);
        }
        state->active = true;
        state->hold_sent = tapdrag_second_touch;
        state->tap_candidate = !tapdrag_second_touch &&
            ((prev_frame->finger_count == 0U) ||
             iqs9151_has_recent_finger_count(data, 0U, now_ms, IQS9151_TAP_REENTRY_WINDOW_MS) ||
             one_lead_tap_candidate);
        state->hold_candidate = tapdrag_second_touch;
        state->tapdrag_second_touch = tapdrag_second_touch;
        state->release_pending = false;
        state->down_ms = now_ms;
        state->release_pending_ms = 0;
        state->centroid_dx = 0;
        state->centroid_dy = 0;
        state->distance_delta = 0;
        state->pinch_wheel_remainder = 0;
        state->mode = IQS9151_2F_MODE_NONE;
        if (have_xy) {
            state->centroid_last_x = ((int32_t)f1x + (int32_t)f2x) / 2;
            state->centroid_last_y = ((int32_t)f1y + (int32_t)f2y) / 2;
            state->distance_last = iqs9151_two_finger_distance(f1x, f1y, f2x, f2y);
        }
    }

    if (!state->active) {
        data->two_finger_one_lead_valid = false;
        return;
    }

    data->two_finger_one_lead_valid = false;

    if (two_now) {
        const int64_t elapsed_ms = now_ms - state->down_ms;
        int32_t step_x = 0;
        int32_t step_y = 0;
        int32_t step_dist = 0;

        if (state->release_pending) {
            state->release_pending = false;
            state->release_pending_ms = 0;
        }

        if (have_xy) {
            const int32_t center_x = ((int32_t)f1x + (int32_t)f2x) / 2;
            const int32_t center_y = ((int32_t)f1y + (int32_t)f2y) / 2;
            const int32_t distance = iqs9151_two_finger_distance(f1x, f1y, f2x, f2y);

            step_x = center_x - state->centroid_last_x;
            step_y = center_y - state->centroid_last_y;
            step_dist = distance - state->distance_last;

            state->centroid_last_x = center_x;
            state->centroid_last_y = center_y;
            state->distance_last = distance;
            state->centroid_dx += step_x;
            state->centroid_dy += step_y;
            state->distance_delta += step_dist;
        }

        if (state->tap_candidate &&
            (elapsed_ms > TWO_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(state->centroid_dx) > TWO_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->centroid_dy) > TWO_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->distance_delta) > TWO_FINGER_TAP_MOVE)) {
            state->tap_candidate = false;
        }
        if (state->tapdrag_second_touch && state->hold_candidate &&
            (elapsed_ms > TWO_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(state->centroid_dx) > TWO_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->centroid_dy) > TWO_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->distance_delta) > TWO_FINGER_TAP_MOVE)) {
            state->hold_candidate = false;
        }
        if (state->tapdrag_second_touch) {
            return;
        }

        if (state->mode == IQS9151_2F_MODE_NONE) {
            const int32_t abs_center =
                MAX(iqs9151_abs32(state->centroid_dx), iqs9151_abs32(state->centroid_dy));
            const int32_t abs_dist = iqs9151_abs32(state->distance_delta);
            const bool scroll_enabled = IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_X_ENABLE) ||
                                        IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_Y_ENABLE);

            if (IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PINCH_ENABLE) &&
                abs_dist >= TWO_FINGER_PINCH_START_DISTANCE &&
                abs_dist > abs_center) {
                state->mode = IQS9151_2F_MODE_PINCH;
                result->pinch_started = true;
                state->tap_candidate = false;
            } else if (scroll_enabled && abs_center >= TWO_FINGER_SCROLL_START_MOVE) {
                state->mode = IQS9151_2F_MODE_SCROLL;
                result->scroll_started = true;
                state->tap_candidate = false;
            }
        }

        if (state->mode == IQS9151_2F_MODE_SCROLL) {
            result->scroll_active = true;
            if (IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_X_ENABLE)) {
                result->scroll_x = (int16_t)CLAMP(step_x, INT16_MIN, INT16_MAX);
            }
            if (IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_Y_ENABLE)) {
                result->scroll_y = (int16_t)CLAMP(step_y, INT16_MIN, INT16_MAX);
            }
        } else if (state->mode == IQS9151_2F_MODE_PINCH) {
            const int32_t wheel_div =
                TWO_FINGER_PINCH_WHEEL_DIV * TWO_FINGER_PINCH_WHEEL_GAIN_DEN;
            const int32_t wheel_acc =
                state->pinch_wheel_remainder +
                (step_dist * TWO_FINGER_PINCH_WHEEL_GAIN_X10);
            const int32_t wheel = wheel_acc / wheel_div;

            state->pinch_wheel_remainder =
                wheel_acc - (wheel * wheel_div);
            result->pinch_active = true;
            result->pinch_wheel = (int16_t)CLAMP(wheel, INT16_MIN, INT16_MAX);
        }
        return;
    }

    if (state->mode == IQS9151_2F_MODE_SCROLL) {
        result->scroll_ended = true;
    } else if (state->mode == IQS9151_2F_MODE_PINCH) {
        result->pinch_ended = true;
    }

    if (state->tapdrag_second_touch) {
        const int64_t elapsed_ms = now_ms - state->down_ms;
        const bool second_tap_detected =
            (frame->finger_count == 0U) &&
            state->hold_candidate &&
            elapsed_ms <= TWO_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(state->centroid_dx) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->centroid_dy) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->distance_delta) <= TWO_FINGER_TAP_MOVE;

        if (frame->finger_count > 0U) {
            state->hold_candidate = false;
            return;
        }

        if (state->hold_sent) {
            iqs9151_release_hold(data, dev);
        }
        if (second_tap_detected &&
            IS_ENABLED(CONFIG_INPUT_IQS9151_2F_TAP_ENABLE)) {
            (void)iqs9151_emit_click(data, dev, INPUT_BTN_1);
        }

        iqs9151_two_finger_reset(state);
        return;
    }

    if (state->release_pending) {
        const int64_t pending_ms = now_ms - state->release_pending_ms;

        if (frame->finger_count == 1U &&
            pending_ms <= TWO_FINGER_RELEASE_PENDING_MAX_MS) {
            return;
        }

        if (frame->finger_count == 0U &&
            pending_ms <= TWO_FINGER_RELEASE_PENDING_MAX_MS &&
            !state->hold_sent &&
            state->mode == IQS9151_2F_MODE_NONE &&
            state->tap_candidate) {
            tap_detected = true;
        }

        if (tap_detected) {
            if (IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PRESSHOLD_ENABLE)) {
                tap_emitted = iqs9151_emit_hold_press(data, dev, INPUT_BTN_1);
            } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_2F_TAP_ENABLE)) {
                tap_emitted = iqs9151_emit_click(data, dev, INPUT_BTN_1);
            } else {
                tap_emitted = true;
            }
        }
        if (tap_detected &&
            tap_emitted &&
            IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PRESSHOLD_ENABLE)) {
            data->two_finger_click_pending = true;
            data->two_finger_click_pending_ms = now_ms;
            k_work_reschedule(&data->two_finger_click_work,
                              K_MSEC(TWO_FINGER_CLICK_HOLD_MAX_MS));
        }

        iqs9151_two_finger_reset(state);
        return;
    }

    if (!state->hold_sent &&
        state->mode == IQS9151_2F_MODE_NONE && state->tap_candidate &&
        IS_ENABLED(CONFIG_INPUT_IQS9151_2F_TAP_ENABLE)) {
        const int64_t elapsed_ms = now_ms - state->down_ms;

        if (frame->finger_count == 1U &&
            elapsed_ms <= TWO_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(state->centroid_dx) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->centroid_dy) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->distance_delta) <= TWO_FINGER_TAP_MOVE) {
            state->release_pending = true;
            state->release_pending_ms = now_ms;
            return;
        }

        if (frame->finger_count == 0U &&
            elapsed_ms <= TWO_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(state->centroid_dx) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->centroid_dy) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->distance_delta) <= TWO_FINGER_TAP_MOVE) {
            tap_detected = true;
        }
    }

    if (tap_detected) {
        if (IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PRESSHOLD_ENABLE)) {
            tap_emitted = iqs9151_emit_hold_press(data, dev, INPUT_BTN_1);
        } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_2F_TAP_ENABLE)) {
            tap_emitted = iqs9151_emit_click(data, dev, INPUT_BTN_1);
        } else {
            tap_emitted = true;
        }
    }
    if (tap_detected &&
        tap_emitted &&
        IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PRESSHOLD_ENABLE)) {
        data->two_finger_click_pending = true;
        data->two_finger_click_pending_ms = now_ms;
        k_work_reschedule(&data->two_finger_click_work,
                          K_MSEC(TWO_FINGER_CLICK_HOLD_MAX_MS));
    }

    iqs9151_two_finger_reset(state);
}

static void iqs9151_three_finger_reset(struct iqs9151_data *data) {
    data->three_active = false;
    data->three_hold_sent = false;
    data->three_swipe_sent = false;
    data->three_tap_candidate = false;
    data->three_hold_candidate = false;
    data->three_tapdrag_second_touch = false;
    data->three_release_pending = false;
    data->three_have_last = false;
    data->three_down_ms = 0;
    data->three_release_pending_ms = 0;
    data->three_dx = 0;
    data->three_dy = 0;
    data->three_last_x = 0;
    data->three_last_y = 0;
}

static bool iqs9151_three_finger_update(struct iqs9151_data *data,
                                        const struct iqs9151_frame *frame,
                                        const struct iqs9151_frame *prev_frame,
                                        const struct device *dev) {
    const bool finger1_valid = iqs9151_finger1_valid(frame);
    const bool one_lead_tap_candidate = data->three_finger_one_lead_valid;
    const bool two_lead_tap_candidate = data->three_finger_two_lead_valid;
    const int64_t now_ms = k_uptime_get();
    bool tap_detected = false;
    bool tap_emitted = false;

    if (!data->three_active && frame->finger_count == 3U) {
        bool tapdrag_second_touch = false;

        if (data->three_finger_click_pending) {
            const int64_t armed_elapsed_ms =
                now_ms - data->three_finger_click_pending_ms;

            tapdrag_second_touch = (armed_elapsed_ms >= 0) &&
                                   (armed_elapsed_ms <= THREE_FINGER_CLICK_HOLD_MAX_MS);
            if (!tapdrag_second_touch && data->hold_button == INPUT_BTN_2) {
                iqs9151_release_hold(data, dev);
            }
            iqs9151_clear_three_finger_click_pending(data);
            (void)k_work_cancel_delayable(&data->three_finger_click_work);
        }
        data->three_active = true;
        data->three_hold_sent = tapdrag_second_touch;
        data->three_swipe_sent = false;
        data->three_tap_candidate = !tapdrag_second_touch &&
            ((prev_frame->finger_count == 0U) ||
             iqs9151_has_recent_finger_count(data, 0U, now_ms,
                                             IQS9151_TAP_REENTRY_WINDOW_MS) ||
             one_lead_tap_candidate ||
             two_lead_tap_candidate);
        data->three_hold_candidate = tapdrag_second_touch;
        data->three_tapdrag_second_touch = tapdrag_second_touch;
        data->three_release_pending = false;
        data->three_have_last = false;
        data->three_down_ms = now_ms;
        data->three_release_pending_ms = 0;
        data->three_dx = 0;
        data->three_dy = 0;
        if (finger1_valid) {
            data->three_last_x = frame->finger1_x;
            data->three_last_y = frame->finger1_y;
            data->three_have_last = true;
        } else if (iqs9151_finger1_valid(prev_frame)) {
            data->three_last_x = prev_frame->finger1_x;
            data->three_last_y = prev_frame->finger1_y;
            data->three_have_last = true;
        }
    }

    if (!data->three_active) {
        data->three_finger_one_lead_valid = false;
        data->three_finger_two_lead_valid = false;
        return false;
    }

    data->three_finger_one_lead_valid = false;
    data->three_finger_two_lead_valid = false;

    if (frame->finger_count == 3U) {
        const int64_t elapsed = now_ms - data->three_down_ms;

        if (data->three_release_pending) {
            data->three_release_pending = false;
            data->three_release_pending_ms = 0;
        }

        if (finger1_valid) {
            if (data->three_have_last) {
                const int32_t dx = (int32_t)frame->finger1_x - (int32_t)data->three_last_x;
                const int32_t dy = (int32_t)frame->finger1_y - (int32_t)data->three_last_y;
                data->three_dx += dx;
                data->three_dy += dy;
            }
            data->three_last_x = frame->finger1_x;
            data->three_last_y = frame->finger1_y;
            data->three_have_last = true;
        }

        if (data->three_tap_candidate &&
            (elapsed > THREE_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(data->three_dx) > THREE_FINGER_TAP_MOVE ||
             iqs9151_abs32(data->three_dy) > THREE_FINGER_TAP_MOVE)) {
            data->three_tap_candidate = false;
        }
        if (data->three_tapdrag_second_touch && data->three_hold_candidate &&
            (elapsed > THREE_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(data->three_dx) > THREE_FINGER_TAP_MOVE ||
             iqs9151_abs32(data->three_dy) > THREE_FINGER_TAP_MOVE)) {
            data->three_hold_candidate = false;
        }
        if (data->three_tapdrag_second_touch) {
            return true;
        }

        if (!data->three_swipe_sent && !data->three_hold_sent) {
            if (iqs9151_abs32(data->three_dx) >= CONFIG_INPUT_IQS9151_3F_SWIPE_THRESHOLD &&
                iqs9151_abs32(data->three_dx) >= iqs9151_abs32(data->three_dy)) {
                const uint16_t key = (data->three_dx < 0) ? INPUT_BTN_4 : INPUT_BTN_3;
                iqs9151_report_key_event(dev, key, true, true, K_FOREVER);
                iqs9151_report_key_event(dev, key, false, true, K_FOREVER);
                data->three_swipe_sent = true;
                return true;
            } else if (iqs9151_abs32(data->three_dy) >= CONFIG_INPUT_IQS9151_3F_SWIPE_THRESHOLD &&
                       iqs9151_abs32(data->three_dy) > iqs9151_abs32(data->three_dx)) {
                const uint16_t key = (data->three_dy < 0) ? INPUT_BTN_5 : INPUT_BTN_6;
                iqs9151_report_key_event(dev, key, true, true, K_FOREVER);
                iqs9151_report_key_event(dev, key, false, true, K_FOREVER);
                data->three_swipe_sent = true;
                return true;
            }
        }
        return true;
    }

    if (data->three_tapdrag_second_touch) {
        const int64_t elapsed = now_ms - data->three_down_ms;
        const bool second_tap_detected =
            (frame->finger_count == 0U) &&
            data->three_hold_candidate &&
            elapsed <= THREE_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(data->three_dx) <= THREE_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->three_dy) <= THREE_FINGER_TAP_MOVE;

        if (frame->finger_count > 0U) {
            data->three_hold_candidate = false;
            return true;
        }

        if (data->three_hold_sent) {
            iqs9151_release_hold(data, dev);
        }
        if (second_tap_detected &&
            IS_ENABLED(CONFIG_INPUT_IQS9151_3F_TAP_ENABLE)) {
            (void)iqs9151_emit_click(data, dev, INPUT_BTN_2);
        }

        iqs9151_three_finger_reset(data);
        return true;
    }

    if (data->three_release_pending) {
        const int64_t pending_ms = now_ms - data->three_release_pending_ms;

        if (frame->finger_count > 0U && frame->finger_count < 3U &&
            pending_ms <= THREE_FINGER_RELEASE_PENDING_MAX_MS) {
            return true;
        }

        if (frame->finger_count == 0U &&
            pending_ms <= THREE_FINGER_RELEASE_PENDING_MAX_MS &&
            !data->three_hold_sent && !data->three_swipe_sent &&
            data->three_tap_candidate) {
            const int64_t elapsed = now_ms - data->three_down_ms;
            if (elapsed <= THREE_FINGER_TAP_MAX_MS &&
                iqs9151_abs32(data->three_dx) <= THREE_FINGER_TAP_MOVE &&
                iqs9151_abs32(data->three_dy) <= THREE_FINGER_TAP_MOVE) {
                tap_detected = true;
            }
        }

        if (tap_detected) {
            if (IS_ENABLED(CONFIG_INPUT_IQS9151_3F_PRESSHOLD_ENABLE)) {
                tap_emitted = iqs9151_emit_hold_press(data, dev, INPUT_BTN_2);
            } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_3F_TAP_ENABLE)) {
                tap_emitted = iqs9151_emit_click(data, dev, INPUT_BTN_2);
            } else {
                tap_emitted = true;
            }
        }
        if (tap_detected &&
            tap_emitted &&
            IS_ENABLED(CONFIG_INPUT_IQS9151_3F_PRESSHOLD_ENABLE)) {
            data->three_finger_click_pending = true;
            data->three_finger_click_pending_ms = now_ms;
            k_work_reschedule(&data->three_finger_click_work,
                              K_MSEC(THREE_FINGER_CLICK_HOLD_MAX_MS));
        }

        iqs9151_three_finger_reset(data);
        return true;
    }

    if (IS_ENABLED(CONFIG_INPUT_IQS9151_3F_TAP_ENABLE) &&
        !data->three_hold_sent && !data->three_swipe_sent &&
        data->three_tap_candidate) {
        const int64_t elapsed = now_ms - data->three_down_ms;

        if (frame->finger_count > 0U && frame->finger_count < 3U &&
            elapsed <= THREE_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(data->three_dx) <= THREE_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->three_dy) <= THREE_FINGER_TAP_MOVE) {
            data->three_release_pending = true;
            data->three_release_pending_ms = now_ms;
            return true;
        }

        if (frame->finger_count == 0U &&
            elapsed <= THREE_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(data->three_dx) <= THREE_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->three_dy) <= THREE_FINGER_TAP_MOVE) {
            tap_detected = true;
        }
    }

    if (tap_detected) {
        if (IS_ENABLED(CONFIG_INPUT_IQS9151_3F_PRESSHOLD_ENABLE)) {
            tap_emitted = iqs9151_emit_hold_press(data, dev, INPUT_BTN_2);
        } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_3F_TAP_ENABLE)) {
            tap_emitted = iqs9151_emit_click(data, dev, INPUT_BTN_2);
        } else {
            tap_emitted = true;
        }
    }
    if (tap_detected &&
        tap_emitted &&
        IS_ENABLED(CONFIG_INPUT_IQS9151_3F_PRESSHOLD_ENABLE)) {
        data->three_finger_click_pending = true;
        data->three_finger_click_pending_ms = now_ms;
        k_work_reschedule(&data->three_finger_click_work,
                          K_MSEC(THREE_FINGER_CLICK_HOLD_MAX_MS));
    }

    iqs9151_three_finger_reset(data);
    return true;
}

static void iqs9151_reset_gesture_states(struct iqs9151_data *data,
                                         const struct device *dev,
                                         bool release_hold) {
    if (data->two_finger.active && data->two_finger.mode == IQS9151_2F_MODE_PINCH) {
        iqs9151_report_key_event(dev, INPUT_BTN_7, false, true, K_FOREVER);
    }
    if (release_hold) {
        iqs9151_release_hold(data, dev);
    }

    iqs9151_one_finger_reset(&data->one_finger);
    iqs9151_two_finger_reset(&data->two_finger);
    iqs9151_clear_one_finger_click_pending(data);
    iqs9151_clear_two_finger_click_pending(data);
    iqs9151_clear_three_finger_click_pending(data);
    (void)k_work_cancel_delayable(&data->one_finger_click_work);
    (void)k_work_cancel_delayable(&data->two_finger_click_work);
    (void)k_work_cancel_delayable(&data->three_finger_click_work);
    data->two_finger_one_lead_valid = false;
    data->two_finger_tail_suppresses_cursor = false;
    data->three_finger_one_lead_valid = false;
    data->three_finger_two_lead_valid = false;
    iqs9151_three_finger_reset(data);
    iqs9151_reset_finger_history(data);
}

static void iqs9151_inertia_start(struct iqs9151_inertia_state *state,
                                  struct k_work_delayable *work,
                                  const struct iqs9151_inertia_params *params,
                                  int32_t ema_vx_fp, int32_t ema_vy_fp) {
    const int32_t threshold_fp =
        (int32_t)params->start_threshold << params->fp_shift;
    const int32_t start_vx_fp =
        (iqs9151_abs32(ema_vx_fp) >= threshold_fp) ? ema_vx_fp : 0;
    const int32_t start_vy_fp =
        (iqs9151_abs32(ema_vy_fp) >= threshold_fp) ? ema_vy_fp : 0;

    if (start_vx_fp == 0 && start_vy_fp == 0) {
        return;
    }

    state->vx_fp = start_vx_fp;
    state->vy_fp = start_vy_fp;
    state->accum_x_fp = 0;
    state->accum_y_fp = 0;
    state->elapsed_ms = 0U;
    state->last_ms = k_uptime_get();
    state->active = true;
    k_work_schedule(work, K_MSEC(params->interval_ms));
}

static bool iqs9151_inertia_step(struct iqs9151_inertia_state *state,
                                 const struct iqs9151_inertia_params *params,
                                 int32_t *out_x, int32_t *out_y) {
    int64_t now;
    int64_t dt_ms;
    uint32_t steps;

    if (!state->active) {
        *out_x = 0;
        *out_y = 0;
        return false;
    }

    now = k_uptime_get();
    dt_ms = now - state->last_ms;
    if (dt_ms <= 0) {
        dt_ms = params->interval_ms;
    }
    steps = (uint32_t)((dt_ms + params->interval_ms - 1) / params->interval_ms);
    if (steps == 0U) {
        steps = 1U;
    }

    for (uint32_t i = 0U; i < steps; i++) {
        state->accum_x_fp += state->vx_fp;
        state->accum_y_fp += state->vy_fp;
        state->vx_fp = (state->vx_fp * params->decay_num) / params->decay_den;
        state->vy_fp = (state->vy_fp * params->decay_num) / params->decay_den;
    }

    state->last_ms = now;
    state->elapsed_ms += steps * params->interval_ms;

    *out_x = state->accum_x_fp >> params->fp_shift;
    *out_y = state->accum_y_fp >> params->fp_shift;
    state->accum_x_fp -= (*out_x) << params->fp_shift;
    state->accum_y_fp -= (*out_y) << params->fp_shift;

    const int32_t min_v_fp = (int32_t)(params->min_velocity << params->fp_shift);
    if (state->elapsed_ms >= params->max_duration_ms ||
        (iqs9151_abs32(state->vx_fp) < min_v_fp &&
         iqs9151_abs32(state->vy_fp) < min_v_fp)) {
        state->active = false;
        return false;
    }

    return true;
}

static void iqs9151_inertia_scroll_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs9151_data *data =
        CONTAINER_OF(dwork, struct iqs9151_data, inertia_scroll_work);
    const struct device *dev = data->dev;
    int32_t out_x;
    int32_t out_y;

    const bool active =
        iqs9151_inertia_step(&data->inertia_scroll, &iqs9151_scroll_params, &out_x, &out_y);

    if (out_x > INT16_MAX) {
        out_x = INT16_MAX;
    } else if (out_x < INT16_MIN) {
        out_x = INT16_MIN;
    }
    if (out_y > INT16_MAX) {
        out_y = INT16_MAX;
    } else if (out_y < INT16_MIN) {
        out_y = INT16_MIN;
    }

    const bool have_x = out_x != 0;
    const bool have_y = out_y != 0;
    if (have_x) {
        iqs9151_report_rel_event(dev, INPUT_REL_HWHEEL, (int16_t)(-out_x), !have_y, K_NO_WAIT);
    }
    if (have_y) {
        iqs9151_report_rel_event(dev, INPUT_REL_WHEEL, (int16_t)out_y, true, K_NO_WAIT);
    }

    if (active) {
        k_work_schedule(&data->inertia_scroll_work,
                        K_MSEC(iqs9151_scroll_params.interval_ms));
    }
}

static bool iqs9151_should_suppress_cursor_for_two_finger_tail(
    struct iqs9151_data *data,
    const struct iqs9151_frame *frame,
    const struct iqs9151_frame *prev_frame,
    const struct iqs9151_two_finger_result *two_result) {
    bool suppress = data->two_finger_tail_suppresses_cursor;

    if (prev_frame->finger_count == 2U && frame->finger_count == 1U &&
        (two_result->scroll_ended || two_result->pinch_ended)) {
        suppress = true;
        data->two_finger_tail_suppresses_cursor = true;
    }

    if (frame->finger_count == 0U || frame->finger_count >= 2U) {
        data->two_finger_tail_suppresses_cursor = false;
    }

    return suppress;
}

static void iqs9151_inertia_cursor_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs9151_data *data =
        CONTAINER_OF(dwork, struct iqs9151_data, inertia_cursor_work);
    const struct device *dev = data->dev;
    int32_t out_x;
    int32_t out_y;

    const bool active =
        iqs9151_inertia_step(&data->inertia_cursor, &iqs9151_cursor_params, &out_x, &out_y);

    if (out_x > INT16_MAX) {
        out_x = INT16_MAX;
    } else if (out_x < INT16_MIN) {
        out_x = INT16_MIN;
    }
    if (out_y > INT16_MAX) {
        out_y = INT16_MAX;
    } else if (out_y < INT16_MIN) {
        out_y = INT16_MIN;
    }

    const bool have_x = out_x != 0;
    const bool have_y = out_y != 0;
    if (have_x) {
        iqs9151_report_rel_event(dev, INPUT_REL_X, (int16_t)out_x, !have_y, K_NO_WAIT);
    }
    if (have_y) {
        iqs9151_report_rel_event(dev, INPUT_REL_Y, (int16_t)out_y, true, K_NO_WAIT);
    }

    if (active) {
        k_work_schedule(&data->inertia_cursor_work,
                        K_MSEC(iqs9151_cursor_params.interval_ms));
    }
}

static void iqs9151_one_finger_click_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs9151_data *data =
        CONTAINER_OF(dwork, struct iqs9151_data, one_finger_click_work);

    if (!data->one_finger_click_pending) {
        return;
    }

    iqs9151_clear_one_finger_click_pending(data);
    if (data->hold_button == INPUT_BTN_0) {
        iqs9151_release_hold(data, data->dev);
    }
}

static void iqs9151_two_finger_click_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs9151_data *data =
        CONTAINER_OF(dwork, struct iqs9151_data, two_finger_click_work);

    if (!data->two_finger_click_pending) {
        return;
    }

    iqs9151_clear_two_finger_click_pending(data);
    if (data->hold_button == INPUT_BTN_1) {
        iqs9151_release_hold(data, data->dev);
    }
}

static void iqs9151_three_finger_click_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs9151_data *data =
        CONTAINER_OF(dwork, struct iqs9151_data, three_finger_click_work);

    if (!data->three_finger_click_pending) {
        return;
    }

    iqs9151_clear_three_finger_click_pending(data);
    if (data->hold_button == INPUT_BTN_2) {
        iqs9151_release_hold(data, data->dev);
    }
}

static int iqs9151_read_frame(const struct iqs9151_config *cfg,
                              struct iqs9151_frame *frame) {
    uint8_t raw_frame[IQS9151_FRAME_READ_SIZE];
    int ret;

    /* Read RelativeX(0x1014) .. Finger2Y(0x102E) in one transaction. */
    ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_RELATIVE_X, raw_frame, sizeof(raw_frame));
    if (ret != 0) {
        return ret;
    }

    iqs9151_parse_frame(raw_frame, frame);
    return 0;
}

static bool iqs9151_handle_show_reset(struct iqs9151_data *data,
                                      const struct iqs9151_frame *frame) {
    const struct device *dev = data->dev;

    if ((frame->info_flags & IQS9151_INFO_SHOW_RESET) == 0U) {
        return false;
    }

    LOG_WRN("SHOW_RESET detected: info=0x%04x", frame->info_flags);
    iqs9151_reset_gesture_states(data, dev, true);
    iqs9151_inertia_cancel(&data->inertia_scroll, &data->inertia_scroll_work);
    iqs9151_inertia_cancel(&data->inertia_cursor, &data->inertia_cursor_work);
    iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
    iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
    iqs9151_motion_history_reset(&data->scroll_motion_history);
    iqs9151_motion_history_reset(&data->cursor_motion_history);
    memset(&data->prev_frame, 0, sizeof(data->prev_frame));
    return true;
}

static bool iqs9151_update_gesture_sessions(struct iqs9151_data *data,
                                            const struct iqs9151_frame *frame,
                                            const struct iqs9151_frame *prev_frame,
                                            struct iqs9151_two_finger_result *two_result) {
    const struct device *dev = data->dev;
    bool released_from_hold = false;

    if (frame->finger_count > 1U && data->one_finger_click_pending) {
        if (data->hold_button == INPUT_BTN_0) {
            iqs9151_release_hold(data, dev);
            released_from_hold = true;
        }
        iqs9151_clear_one_finger_click_pending(data);
        (void)k_work_cancel_delayable(&data->one_finger_click_work);
    }
    if (frame->finger_count > 2U && data->two_finger_click_pending) {
        if (data->hold_button == INPUT_BTN_1) {
            iqs9151_release_hold(data, dev);
            released_from_hold = true;
        }
        iqs9151_clear_two_finger_click_pending(data);
        (void)k_work_cancel_delayable(&data->two_finger_click_work);
    }
    if (frame->finger_count != 0U &&
        frame->finger_count != 3U &&
        data->three_finger_click_pending) {
        const int64_t armed_elapsed_ms =
            k_uptime_get() - data->three_finger_click_pending_ms;

        if (armed_elapsed_ms > THREE_FINGER_CLICK_HOLD_MAX_MS) {
            if (data->hold_button == INPUT_BTN_2) {
                iqs9151_release_hold(data, dev);
                released_from_hold = true;
            }
            iqs9151_clear_three_finger_click_pending(data);
            (void)k_work_cancel_delayable(&data->three_finger_click_work);
        } else {
            /* Keep 3F deferred-click armed during staged 0->1->2->3 re-entry. */
            return released_from_hold;
        }
    }
    if (frame->finger_count == 3U && data->one_finger.active) {
        const int64_t elapsed_ms = k_uptime_get() - data->one_finger.down_ms;

        data->three_finger_one_lead_valid =
            !data->one_finger.hold_sent &&
            data->one_finger.tap_candidate &&
            elapsed_ms <= THREE_FINGER_ONE_LEAD_MAX_MS &&
            iqs9151_abs32(data->one_finger.dx) <= ONE_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->one_finger.dy) <= ONE_FINGER_TAP_MOVE;
        if (data->one_finger.hold_sent) {
            iqs9151_release_hold(data, dev);
            released_from_hold = true;
        }
        iqs9151_one_finger_reset(&data->one_finger);
    } else {
        data->three_finger_one_lead_valid = false;
    }

    if (frame->finger_count == 3U && data->two_finger.active) {
        const int64_t elapsed_ms = k_uptime_get() - data->two_finger.down_ms;

        data->three_finger_two_lead_valid =
            !data->two_finger.hold_sent &&
            data->two_finger.mode == IQS9151_2F_MODE_NONE &&
            data->two_finger.tap_candidate &&
            elapsed_ms <= THREE_FINGER_TWO_LEAD_MAX_MS &&
            iqs9151_abs32(data->two_finger.centroid_dx) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->two_finger.centroid_dy) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->two_finger.distance_delta) <= TWO_FINGER_TAP_MOVE;

        if (data->two_finger.mode == IQS9151_2F_MODE_SCROLL) {
            two_result->scroll_ended = true;
        } else if (data->two_finger.mode == IQS9151_2F_MODE_PINCH) {
            two_result->pinch_ended = true;
        }
        if (data->two_finger.hold_sent) {
            iqs9151_release_hold(data, dev);
            released_from_hold = true;
        }

        iqs9151_two_finger_reset(&data->two_finger);
        data->two_finger_one_lead_valid = false;
    } else {
        data->three_finger_two_lead_valid = false;
    }

    if (frame->finger_count == 2U && data->one_finger.active) {
        const int64_t elapsed_ms = k_uptime_get() - data->one_finger.down_ms;

        data->two_finger_one_lead_valid =
            !data->one_finger.hold_sent &&
            data->one_finger.tap_candidate &&
            elapsed_ms <= TWO_FINGER_ONE_LEAD_MAX_MS &&
            iqs9151_abs32(data->one_finger.dx) <= ONE_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->one_finger.dy) <= ONE_FINGER_TAP_MOVE;
        if (data->one_finger.hold_sent) {
            iqs9151_release_hold(data, dev);
            released_from_hold = true;
        }
        iqs9151_one_finger_reset(&data->one_finger);
    } else {
        data->two_finger_one_lead_valid = false;
    }

    if (frame->finger_count != 1U && data->one_finger.active) {
        released_from_hold = iqs9151_one_finger_update(data, frame, prev_frame, dev);
    }
    if (frame->finger_count != 2U && data->two_finger.active) {
        iqs9151_two_finger_update(data, frame, prev_frame, dev, two_result);
    }
    if (frame->finger_count != 3U && data->three_active) {
        (void)iqs9151_three_finger_update(data, frame, prev_frame, dev);
    }

    switch (frame->finger_count) {
    case 1U:
        if (!(data->two_finger.active && data->two_finger.release_pending)) {
            if (!(data->three_active && data->three_release_pending)) {
                released_from_hold = iqs9151_one_finger_update(data, frame, prev_frame, dev);
            }
        }
        break;
    case 2U:
        if (!(data->three_active && data->three_release_pending)) {
            iqs9151_two_finger_update(data, frame, prev_frame, dev, two_result);
        }
        break;
    case 3U:
        (void)iqs9151_three_finger_update(data, frame, prev_frame, dev);
        break;
    default:
        break;
    }

    return released_from_hold;
}

static void iqs9151_update_inertia_ema(struct iqs9151_data *data,
                                       const struct iqs9151_frame *frame,
                                       const struct iqs9151_frame *prev_frame,
                                       const struct iqs9151_two_finger_result *two_result,
                                       int64_t now_ms,
                                       bool released_from_hold,
                                       bool cursor_moving,
                                       bool suppress_cursor_tail) {
    const bool finger1_started =
        (prev_frame->finger_count == 0U) && (frame->finger_count == 1U);
    const bool cursor_released =
        (prev_frame->finger_count == 1U) && (frame->finger_count == 0U);
    int32_t seed_vx_fp;
    int32_t seed_vy_fp;

    /* Cancel Inertial */
    if (two_result->scroll_started || frame->finger_count == 2U || finger1_started) {
        iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
        iqs9151_inertia_cancel(&data->inertia_scroll, &data->inertia_scroll_work);
    }
    if (two_result->scroll_active) {
        iqs9151_ema_update(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp,
                           two_result->scroll_x, two_result->scroll_y,
                           iqs9151_scroll_params.ema_alpha);
        iqs9151_motion_history_push(&data->scroll_motion_history, two_result->scroll_x,
                                    two_result->scroll_y, now_ms);
    }

    if (suppress_cursor_tail) {
        iqs9151_inertia_cancel(&data->inertia_cursor, &data->inertia_cursor_work);
        iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
        iqs9151_motion_history_reset(&data->cursor_motion_history);
    } else {
        if (finger1_started) {
            iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
            iqs9151_motion_history_reset(&data->cursor_motion_history);
        }
        if (frame->finger_count == 1U && cursor_moving) {
            iqs9151_inertia_cancel(&data->inertia_cursor, &data->inertia_cursor_work);
            iqs9151_ema_update(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp,
                               frame->rel_x, frame->rel_y, iqs9151_cursor_params.ema_alpha);
            iqs9151_motion_history_push(&data->cursor_motion_history, frame->rel_x,
                                        frame->rel_y, now_ms);
        }
    }

    /* Inertial Cursolling */
    if (cursor_released && !released_from_hold && !suppress_cursor_tail) {
        if (IS_ENABLED(CONFIG_INPUT_IQS9151_CURSOR_INERTIA_ENABLE) &&
            iqs9151_inertia_seed_from_history(&data->cursor_motion_history,
                                              &iqs9151_cursor_params,
                                              &iqs9151_cursor_gate_params, now_ms,
                                              &seed_vx_fp, &seed_vy_fp)) {
            iqs9151_inertia_start(&data->inertia_cursor, &data->inertia_cursor_work,
                                  &iqs9151_cursor_params, seed_vx_fp, seed_vy_fp);
        }
        iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
        iqs9151_motion_history_reset(&data->cursor_motion_history);
    }

    /* Inertial Scrolling */
    if (two_result->scroll_ended) {
        if (IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_INERTIA_ENABLE) &&
            iqs9151_inertia_seed_from_history(&data->scroll_motion_history,
                                              &iqs9151_scroll_params,
                                              &iqs9151_scroll_gate_params, now_ms,
                                              &seed_vx_fp, &seed_vy_fp)) {
            iqs9151_inertia_start(&data->inertia_scroll, &data->inertia_scroll_work,
                                  &iqs9151_scroll_params, seed_vx_fp, seed_vy_fp);
        }
        iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
        iqs9151_motion_history_reset(&data->scroll_motion_history);
    }
    if (two_result->pinch_active) {
        iqs9151_inertia_cancel(&data->inertia_scroll, &data->inertia_scroll_work);
        iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
        iqs9151_motion_history_reset(&data->scroll_motion_history);
    }
}

static void iqs9151_report_frame_events(const struct device *dev,
                                        const struct iqs9151_frame *frame,
                                        const struct iqs9151_two_finger_result *two_result,
                                        bool cursor_moving,
                                        bool suppress_cursor_tail) {
    if (two_result->pinch_started) {
        iqs9151_report_key_event(dev, INPUT_BTN_7, true, true, K_FOREVER);
    }
    if (two_result->pinch_ended) {
        iqs9151_report_key_event(dev, INPUT_BTN_7, false, true, K_FOREVER);
    }

    if (two_result->pinch_active) {
        if (two_result->pinch_wheel != 0) {
            iqs9151_report_rel_event(dev, INPUT_REL_WHEEL, two_result->pinch_wheel, true, K_NO_WAIT);
        }
    } else if (two_result->scroll_active) {
        const bool have_x = two_result->scroll_x != 0;
        const bool have_y = two_result->scroll_y != 0;
        if (have_x) {
            iqs9151_report_rel_event(dev, INPUT_REL_HWHEEL, (int16_t)(-two_result->scroll_x),
                                     !have_y, K_NO_WAIT);
        }
        if (have_y) {
            iqs9151_report_rel_event(dev, INPUT_REL_WHEEL, two_result->scroll_y, true, K_NO_WAIT);
        }
    } else if (frame->finger_count == 1U && cursor_moving && !suppress_cursor_tail) {
        iqs9151_report_rel_event(dev, INPUT_REL_X, frame->rel_x, false, K_NO_WAIT);
        iqs9151_report_rel_event(dev, INPUT_REL_Y, frame->rel_y, true, K_NO_WAIT);
    }
}

static void iqs9151_process_frame(struct iqs9151_data *data,
                                  const struct iqs9151_frame *frame,
                                  int64_t now_ms) {
    const struct device *dev = data->dev;
    const struct iqs9151_frame prev_frame = data->prev_frame;
    struct iqs9151_two_finger_result two_result;
    const bool cursor_moving =
        (frame->trackpad_flags & IQS9151_TP_MOVEMENT_DETECTED) != 0U;
    bool released_from_hold;
    bool suppress_cursor_tail;

    iqs9151_two_finger_result_reset(&two_result);

    if (iqs9151_handle_show_reset(data, frame)) {
        return;
    }

    released_from_hold =
        iqs9151_update_gesture_sessions(data, frame, &prev_frame, &two_result);
    suppress_cursor_tail =
        iqs9151_should_suppress_cursor_for_two_finger_tail(data, frame, &prev_frame,
                                                           &two_result);

    if (frame->finger_count == 3U || data->three_active) {
        iqs9151_inertia_cancel(&data->inertia_scroll, &data->inertia_scroll_work);
        iqs9151_inertia_cancel(&data->inertia_cursor, &data->inertia_cursor_work);
        iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
        iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
        iqs9151_motion_history_reset(&data->scroll_motion_history);
        iqs9151_motion_history_reset(&data->cursor_motion_history);
    }

    if (data->one_finger.active && data->one_finger.hold_sent) {
        iqs9151_inertia_cancel(&data->inertia_cursor, &data->inertia_cursor_work);
        iqs9151_motion_history_reset(&data->cursor_motion_history);
    }

    iqs9151_report_frame_events(dev, frame, &two_result, cursor_moving,
                                suppress_cursor_tail);

    LOG_DBG("rel x=%d y=%d info=0x%04x tp=0x%04x finger=%d f1x=%u f1y=%u f2x=%u f2y=%u",
            frame->rel_x, frame->rel_y, frame->info_flags, frame->trackpad_flags,
            frame->finger_count, frame->finger1_x, frame->finger1_y,
            frame->finger2_x, frame->finger2_y);
    LOG_DBG("gesture_state: hold_button=0x%04x 2f_mode=%d",
            data->hold_button,
            data->two_finger.mode);

    iqs9151_update_inertia_ema(data, frame, &prev_frame, &two_result, now_ms,
                               released_from_hold, cursor_moving,
                               suppress_cursor_tail);
    iqs9151_update_prev_frame(data, frame, &prev_frame);
    iqs9151_push_finger_history(data, frame->finger_count, now_ms);
}

static void iqs9151_work_cb(struct k_work *work) {
    struct iqs9151_data *data = CONTAINER_OF(work, struct iqs9151_data, work);
    const struct device *dev = data->dev;
    const struct iqs9151_config *cfg = dev->config;
    struct iqs9151_frame frame;
    int ret;
    const int64_t now_ms = k_uptime_get();

    ret = iqs9151_read_frame(cfg, &frame);
    if (ret != 0) {
        LOG_ERR("frame read failed (%d)", ret);
        return;
    }

    iqs9151_process_frame(data, &frame, now_ms);
}

static void iqs9151_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct iqs9151_data *data = CONTAINER_OF(cb, struct iqs9151_data, gpio_cb);
    k_work_submit(&data->work);
}

static int iqs9151_set_interrupt(const struct device *dev, const bool en) {
    const struct iqs9151_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt( &config->irq_gpio, en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

static int iqs9151_run_ati(const struct iqs9151_config *config) {
    uint8_t ctrl[2] = {
        SYSTEM_CONTROL_0,
        SYSTEM_CONTROL_1 | IQS9151_SYS_CTRL_ALP_RE_ATI | IQS9151_SYS_CTRL_TP_RE_ATI,
    };
    return iqs9151_i2c_write(config, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
}

static int iqs9151_wait_for_ati(const struct device *dev, uint16_t timeout_ms) {
    const struct iqs9151_config *cfg = dev->config;
    int64_t start_ms = k_uptime_get();

    while ((k_uptime_get() - start_ms) < timeout_ms) {
        uint8_t ctrl[2];
        int ret;

        iqs9151_wait_for_ready(dev, 100);
        ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
        if (ret != 0) {
            return ret;
        }

        if ((sys_get_le16(ctrl) &
             (IQS9151_SYS_CTRL_ALP_RE_ATI | IQS9151_SYS_CTRL_TP_RE_ATI)) == 0U) {
            return 0;
        }

        k_sleep(K_MSEC(IQS9151_ATI_POLL_INTERVAL_MS));
    }

    LOG_ERR("ATI timeout after %dms", timeout_ms);
    return -EIO;
}

static int iqs9151_ack_reset(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    uint8_t ctrl[2];
    int ret;

    ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
    if (ret != 0) {
        LOG_ERR("Read SYSTEM CONTROL(ACK_RESET) failed (%d)", ret);
        return ret;
    }

    uint16_t config = sys_get_le16(ctrl);
    config |= IQS9151_SYS_CTRL_ACK_RESET;
    sys_put_le16(config, ctrl);

    iqs9151_wait_for_ready(dev, 500);

    ret = iqs9151_i2c_write(cfg, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
    if (ret != 0) {
        LOG_ERR("Wrte SYSTEM CONTROL(ACK_RESET) failed (%d)", ret);
        return ret;
    }

    k_msleep(IQS9151_RSTD_DELAY_MS);
    return ret;
}

static int iqs9151_wait_for_show_reset(const struct device *dev, uint16_t timeout_ms) {
    const struct iqs9151_config *cfg = dev->config;
    int64_t start_ms = k_uptime_get();

    while ((k_uptime_get() - start_ms) < timeout_ms) {
        uint8_t info[2];
        int ret;

        iqs9151_wait_for_ready(dev, 100);
        ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_INFO_FLAGS, info, sizeof(info));
        if (ret != 0) {
            return ret;
        }

        if ((sys_get_le16(info) & IQS9151_INFO_SHOW_RESET) != 0U) {
            return 0;
        }

        k_sleep(K_MSEC(IQS9151_ATI_POLL_INTERVAL_MS));
    }

    LOG_ERR("Show Reset timeout after %dms", timeout_ms);
    return -EIO;
}

static int iqs9151_sw_reset(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    uint8_t ctrl[2];
    int ret;

    ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
    if (ret != 0) {
        LOG_ERR("Read SYSTEM CONTROL(SW_RESET) failed (%d)", ret);
        return ret;
    }

    uint16_t config = sys_get_le16(ctrl);
    config |= IQS9151_SYS_CTRL_SW_RESET;
    sys_put_le16(config, ctrl);

    iqs9151_wait_for_ready(dev, 500);

    ret = iqs9151_i2c_write(cfg, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
    if (ret != 0) {
        LOG_ERR("Wrte SYSTEM CONTROL(SW_RESET) failed (%d)", ret);
        return ret;
    }

    ret = iqs9151_wait_for_show_reset(dev, 3000);
    if (ret != 0) {
        return ret;
    }

    return ret;
}

static int iqs9151_set_event_mode(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    uint8_t config_settings[2];

    int ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_CONFIG_SETTINGS, config_settings, sizeof(config_settings));
    if (ret != 0) {
        return ret;
    }

    uint16_t settings = sys_get_le16(config_settings);
    settings |= IQS9151_CFG_EVENT_MODE;
    sys_put_le16(settings, config_settings);

    iqs9151_wait_for_ready(dev, 500);

    return iqs9151_i2c_write(cfg, IQS9151_ADDR_CONFIG_SETTINGS, config_settings, sizeof(config_settings));
}

static int iqs9151_configure(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    int ret;

    iqs9151_wait_for_ready(dev, 500);

    ret = iqs9151_write_chunks(dev, cfg, IQS9151_ADDR_ALP_COMPENSATION,
                                    iqs9151_alp_compensation,
                                    ARRAY_SIZE(iqs9151_alp_compensation));
    if (ret) {
        return ret;
    }
    ret = iqs9151_write_chunks(dev, cfg, IQS9151_ADDR_SETTINGS_MINOR,
                                    iqs9151_main_config,
                                    ARRAY_SIZE(iqs9151_main_config));
    if (ret) {
        return ret;
    }
    ret = iqs9151_write_chunks(dev, cfg, IQS9151_ADDR_RX_TX_MAPPING,
                                    iqs9151_rxtx_map,
                                    ARRAY_SIZE(iqs9151_rxtx_map));
    if (ret) {
        return ret;
    }
    ret = iqs9151_write_chunks(dev, cfg, IQS9151_ADDR_CHANNEL_DISABLE,
                                    iqs9151_channel_disable,
                                    ARRAY_SIZE(iqs9151_channel_disable));
    if (ret) {
        return ret;
    }
    ret = iqs9151_write_chunks(dev, cfg, IQS9151_ADDR_SNAP_ENABLE,
                                    iqs9151_snap_enable,
                                    ARRAY_SIZE(iqs9151_snap_enable));
    if (ret) {
        return ret;
    }
    return ret;
}

static int iqs9151_apply_kconfig_overrides(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    uint16_t rotate_bits = 0U;
    int ret;

    /* 90/270 are counterclockwise. */
    if (IS_ENABLED(CONFIG_INPUT_IQS9151_ROTATE_90)) {
        rotate_bits = IQS9151_TRACKPAD_SETTING_SWITCH_XY |
                      IQS9151_TRACKPAD_SETTING_FLIP_Y;
    } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_ROTATE_180)) {
        rotate_bits = IQS9151_TRACKPAD_SETTING_FLIP_X |
                      IQS9151_TRACKPAD_SETTING_FLIP_Y;
    } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_ROTATE_270)) {
        rotate_bits = IQS9151_TRACKPAD_SETTING_SWITCH_XY |
                      IQS9151_TRACKPAD_SETTING_FLIP_X;
    }

    ret = iqs9151_update_bits_u16(cfg, IQS9151_ADDR_TRACKPAD_SETTINGS,
                                  IQS9151_TRACKPAD_SETTING_FLIP_X |
                                      IQS9151_TRACKPAD_SETTING_FLIP_Y |
                                      IQS9151_TRACKPAD_SETTING_SWITCH_XY,
                                  rotate_bits);
    if (ret != 0) {
        LOG_ERR("Failed to apply rotate settings (%d)", ret);
        return ret;
    }

    ret = iqs9151_write_u16(cfg, IQS9151_ADDR_X_RESOLUTION,
                            (uint16_t)CONFIG_INPUT_IQS9151_RESOLUTION_X);
    if (ret != 0) {
        LOG_ERR("Failed to apply X resolution (%d)", ret);
        return ret;
    }

    ret = iqs9151_write_u16(cfg, IQS9151_ADDR_Y_RESOLUTION,
                            (uint16_t)CONFIG_INPUT_IQS9151_RESOLUTION_Y);
    if (ret != 0) {
        LOG_ERR("Failed to apply Y resolution (%d)", ret);
        return ret;
    }

    ret = iqs9151_write_u16(cfg, IQS9151_ADDR_TRACKPAD_ATI_TARGET,
                            (uint16_t)CONFIG_INPUT_IQS9151_ATI_TARGETCOUNT);
    if (ret != 0) {
        LOG_ERR("Failed to apply ATI target (%d)", ret);
        return ret;
    }

    ret = iqs9151_write_u16(cfg, IQS9151_ADDR_XY_DYNAMIC_FILTER_BOTTOM_SPEED,
                            (uint16_t)CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_BOTTOM_SPEED);
    if (ret != 0) {
        LOG_ERR("Failed to apply dynamic filter bottom speed (%d)", ret);
        return ret;
    }

    ret = iqs9151_write_u16(cfg, IQS9151_ADDR_XY_DYNAMIC_FILTER_TOP_SPEED,
                            (uint16_t)CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_TOP_SPEED);
    if (ret != 0) {
        LOG_ERR("Failed to apply dynamic filter top speed (%d)", ret);
        return ret;
    }

    ret = iqs9151_i2c_write(
        cfg, IQS9151_ADDR_XY_DYNAMIC_FILTER_BOTTOM_BETA,
        (const uint8_t[]){(uint8_t)CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_BOTTOM_BETA}, 1);
    if (ret != 0) {
        LOG_ERR("Failed to apply dynamic filter bottom beta (%d)", ret);
        return ret;
    }

    return 0;
}

static int iqs9151_init(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    struct iqs9151_data *data = dev->data;
    int ret;
    data->dev = dev;

    LOG_DBG("Initialization Start");

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    if (!cfg->irq_gpio.port) {
        LOG_ERR("IRQ GPIO not defined");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
    if (ret) {
        return ret;
    }

    iqs9151_wait_for_ready(dev, 500);
    
    // Check Product Number
    ret = iqs9151_check_product_number(dev);
    if (ret != 0) {
        return ret;
    }

    iqs9151_wait_for_ready(dev, 500);

    // SW Reset (Show Reset wait + ACK)
    ret = iqs9151_sw_reset(dev);
    if (ret) {
        LOG_ERR("SW Reset failed (%d)", ret);
        return ret;
    }
    LOG_DBG("SW Reset complete");

    iqs9151_wait_for_ready(dev, 500);

    // ACK Reset
    ret = iqs9151_ack_reset(dev);
    if (ret) {
        LOG_ERR("Reset flag clear failed (%d)", ret);
        return ret;
    }
    LOG_DBG("ACK Reset complete");

    iqs9151_wait_for_ready(dev, 500);

    // Setup Initial Config
    ret = iqs9151_configure(dev);
    if (ret != 0) {
        LOG_ERR("Device configuration failed: %d", ret);
        return ret;
    }
    LOG_DBG("Setup Initial Config complete");

    iqs9151_wait_for_ready(dev, 100);

    ret = iqs9151_apply_kconfig_overrides(dev);
    if (ret != 0) {
        LOG_ERR("Kconfig override apply failed: %d", ret);
        return ret;
    }
    LOG_DBG("Kconfig overrides applied");

    iqs9151_wait_for_ready(dev, 100);

    // ATI
    ret = iqs9151_run_ati(cfg);
    if (ret) {
        LOG_ERR("ATI request failed (%d)", ret);
        return ret;
    }
    LOG_DBG("ATI requested");

    ret = iqs9151_wait_for_ati(dev, IQS9151_ATI_TIMEOUT_MS);
    if (ret != 0) {
        LOG_ERR("ATI failed (%d)", ret);
        return ret;
    }
    LOG_DBG("ATI complete");

    // Setup IRQ Call Back
    k_work_init(&data->work, iqs9151_work_cb);
    k_work_init_delayable(&data->one_finger_click_work, iqs9151_one_finger_click_work_cb);
    k_work_init_delayable(&data->two_finger_click_work, iqs9151_two_finger_click_work_cb);
    k_work_init_delayable(&data->three_finger_click_work, iqs9151_three_finger_click_work_cb);
    k_work_init_delayable(&data->inertia_scroll_work, iqs9151_inertia_scroll_work_cb);
    k_work_init_delayable(&data->inertia_cursor_work, iqs9151_inertia_cursor_work_cb);
    iqs9151_inertia_state_reset(&data->inertia_scroll);
    iqs9151_inertia_state_reset(&data->inertia_cursor);
    iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
    iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
    iqs9151_motion_history_reset(&data->scroll_motion_history);
    iqs9151_motion_history_reset(&data->cursor_motion_history);
    iqs9151_one_finger_reset(&data->one_finger);
    iqs9151_two_finger_reset(&data->two_finger);
    iqs9151_clear_one_finger_click_pending(data);
    iqs9151_clear_two_finger_click_pending(data);
    iqs9151_clear_three_finger_click_pending(data);
    data->two_finger_one_lead_valid = false;
    data->two_finger_tail_suppresses_cursor = false;
    data->three_finger_one_lead_valid = false;
    data->three_finger_two_lead_valid = false;
    iqs9151_three_finger_reset(data);
    data->hold_button = 0U;
    iqs9151_reset_finger_history(data);
    gpio_init_callback(&data->gpio_cb, iqs9151_gpio_cb,
                        BIT(cfg->irq_gpio.pin));
    ret = gpio_add_callback(cfg->irq_gpio.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    iqs9151_wait_for_ready(dev, 100);

    // Set Event Mode
    ret = iqs9151_set_event_mode(dev);
    if (ret) {
        LOG_ERR("Set Event Mode failed (%d)", ret);
        return ret;
    }
    LOG_DBG("Set Event Mode complete complete");

    // start IRQ
    iqs9151_set_interrupt(dev, true);
    LOG_DBG("Initialization complete");
    return 0;
}

#ifdef CONFIG_INPUT_IQS9151_TEST
size_t iqs9151_test_context_size(void) {
    return sizeof(struct iqs9151_data);
}

void iqs9151_test_context_init(void *ctx, const struct device *dev) {
    struct iqs9151_data *data = (struct iqs9151_data *)ctx;

    memset(data, 0, sizeof(*data));
    data->dev = dev;
    k_work_init(&data->work, iqs9151_work_cb);
    k_work_init_delayable(&data->one_finger_click_work, iqs9151_one_finger_click_work_cb);
    k_work_init_delayable(&data->two_finger_click_work, iqs9151_two_finger_click_work_cb);
    k_work_init_delayable(&data->three_finger_click_work, iqs9151_three_finger_click_work_cb);
    k_work_init_delayable(&data->inertia_scroll_work, iqs9151_inertia_scroll_work_cb);
    k_work_init_delayable(&data->inertia_cursor_work, iqs9151_inertia_cursor_work_cb);
    iqs9151_inertia_state_reset(&data->inertia_scroll);
    iqs9151_inertia_state_reset(&data->inertia_cursor);
    iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
    iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
    iqs9151_motion_history_reset(&data->scroll_motion_history);
    iqs9151_motion_history_reset(&data->cursor_motion_history);
    iqs9151_one_finger_reset(&data->one_finger);
    iqs9151_two_finger_reset(&data->two_finger);
    iqs9151_clear_one_finger_click_pending(data);
    iqs9151_clear_two_finger_click_pending(data);
    iqs9151_clear_three_finger_click_pending(data);
    data->two_finger_one_lead_valid = false;
    data->two_finger_tail_suppresses_cursor = false;
    data->three_finger_one_lead_valid = false;
    data->three_finger_two_lead_valid = false;
    iqs9151_three_finger_reset(data);
    data->hold_button = 0U;
    iqs9151_reset_finger_history(data);
}

void iqs9151_test_cancel_pending_work(void *ctx) {
    struct iqs9151_data *data = (struct iqs9151_data *)ctx;

    (void)k_work_cancel_delayable(&data->one_finger_click_work);
    (void)k_work_cancel_delayable(&data->two_finger_click_work);
    (void)k_work_cancel_delayable(&data->three_finger_click_work);
    (void)k_work_cancel_delayable(&data->inertia_scroll_work);
    (void)k_work_cancel_delayable(&data->inertia_cursor_work);
    (void)k_work_cancel(&data->work);
}

void iqs9151_test_process_frame(void *ctx,
                                const struct iqs9151_test_frame *frame,
                                int64_t now_ms) {
    struct iqs9151_data *data = (struct iqs9151_data *)ctx;
    const struct iqs9151_frame internal_frame = {
        .rel_x = frame->rel_x,
        .rel_y = frame->rel_y,
        .info_flags = frame->info_flags,
        .trackpad_flags = frame->trackpad_flags,
        .finger_count = frame->finger_count,
        .finger1_x = frame->finger1_x,
        .finger1_y = frame->finger1_y,
        .finger2_x = frame->finger2_x,
        .finger2_y = frame->finger2_y,
    };

    iqs9151_process_frame(data, &internal_frame, now_ms);
}

void iqs9151_test_set_event_hook(iqs9151_test_event_hook_t hook, void *user_data) {
    iqs9151_test_hook.hook = hook;
    iqs9151_test_hook.user_data = user_data;
}

uint16_t iqs9151_test_hold_button(const void *ctx) {
    const struct iqs9151_data *data = (const struct iqs9151_data *)ctx;

    return data->hold_button;
}

void iqs9151_test_force_hold_button(void *ctx, uint16_t button) {
    struct iqs9151_data *data = (struct iqs9151_data *)ctx;

    data->hold_button = button;
}

uint8_t iqs9151_test_prev_finger_count(const void *ctx) {
    const struct iqs9151_data *data = (const struct iqs9151_data *)ctx;

    return data->prev_frame.finger_count;
}

bool iqs9151_test_cursor_inertia_active(const void *ctx) {
    const struct iqs9151_data *data = (const struct iqs9151_data *)ctx;

    return data->inertia_cursor.active;
}

bool iqs9151_test_scroll_inertia_active(const void *ctx) {
    const struct iqs9151_data *data = (const struct iqs9151_data *)ctx;

    return data->inertia_scroll.active;
}

void iqs9151_test_force_pinch_session(void *ctx, bool active) {
    struct iqs9151_data *data = (struct iqs9151_data *)ctx;

    data->two_finger.active = active;
    data->two_finger.mode = active ? IQS9151_2F_MODE_PINCH : IQS9151_2F_MODE_NONE;
}
#endif

#define IQS9151_INIT(inst)                                                \
    static const struct iqs9151_config iqs9151_config_##inst = {    \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                      \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                     \
  };                                                                          \
  static struct iqs9151_data iqs9151_data_##inst;                 \
  DEVICE_DT_INST_DEFINE(inst, iqs9151_init, NULL,                       \
                        &iqs9151_data_##inst,                           \
                        &iqs9151_config_##inst, POST_KERNEL,            \
                        CONFIG_INPUT_IQS9151_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS9151_INIT);
