#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/battery.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>

#if defined(CONFIG_BDKB_BATTERY_STATUS_LED_MODE_SINGLE)
#define BDKB_SINGLE_LED_MODE 1
#else
#define BDKB_SINGLE_LED_MODE 0
#endif

BUILD_ASSERT(DT_NODE_EXISTS(DT_PATH(leds)), "BDKB_BATTERY_STATUS_LEDS requires a /leds node");
BUILD_ASSERT(DT_NODE_EXISTS(DT_NODELABEL(red_led)),
             "BDKB_BATTERY_STATUS_LEDS requires red_led node label");
#if !BDKB_SINGLE_LED_MODE
BUILD_ASSERT(DT_NODE_EXISTS(DT_NODELABEL(blue_led)),
             "BDKB_BATTERY_STATUS_LEDS requires blue_led node label");
BUILD_ASSERT(DT_NODE_EXISTS(DT_NODELABEL(green_led)),
             "BDKB_BATTERY_STATUS_LEDS requires green_led node label");
#endif

static const struct device *const led_dev = DEVICE_DT_GET(DT_PATH(leds));

enum {
    RED_LED_IDX = DT_NODE_CHILD_IDX(DT_NODELABEL(red_led)),
#if !BDKB_SINGLE_LED_MODE
    BLUE_LED_IDX = DT_NODE_CHILD_IDX(DT_NODELABEL(blue_led)),
    GREEN_LED_IDX = DT_NODE_CHILD_IDX(DT_NODELABEL(green_led)),
#endif
};

enum battery_led_mode {
    BAT_LED_OFF,
    BAT_LED_LOW,
    BAT_LED_CHARGING,
    BAT_LED_FULL,
};

static enum battery_led_mode mode = BAT_LED_OFF;
static uint8_t last_soc;
static bool usb_powered;
#if !BDKB_SINGLE_LED_MODE
static bool low_batt_active;
static bool full_batt_active;
#endif
static bool red_blink_on;
#if BDKB_SINGLE_LED_MODE
static uint8_t breath_phase;
static uint8_t breath_pwm_ctr;
#define SINGLE_BREATH_PWM_MAX 7U
#define SINGLE_BREATH_CYCLE_STEPS (SINGLE_BREATH_PWM_MAX * 2U)
#define SINGLE_BREATH_TICK_MS 10
#endif

static const char *charging_state_str(void) {
    switch (mode) {
    case BAT_LED_FULL:
        return "full";
    case BAT_LED_CHARGING:
        return "charging";
    default:
        return "no charging";
    }
}

static void status_log_timer_handler(struct k_timer *timer) {
    ARG_UNUSED(timer);
    LOG_INF("Battery: %u%%, status: %s", last_soc, charging_state_str());
}

K_TIMER_DEFINE(status_log_timer, status_log_timer_handler, NULL);

static void set_led(int idx, bool on) {
    int rc = on ? led_on(led_dev, idx) : led_off(led_dev, idx);
    if (rc != 0) {
        LOG_WRN("Failed to set LED %d: %d", idx, rc);
    }
}

static void apply_leds(void) {
    switch (mode) {
    case BAT_LED_LOW:
#if BDKB_SINGLE_LED_MODE
        set_led(RED_LED_IDX, true);
#else
        set_led(RED_LED_IDX, red_blink_on);
#endif
#if !BDKB_SINGLE_LED_MODE
        set_led(BLUE_LED_IDX, false);
        set_led(GREEN_LED_IDX, false);
#endif
        break;
    case BAT_LED_CHARGING:
#if BDKB_SINGLE_LED_MODE
        set_led(RED_LED_IDX, false);
#else
        set_led(RED_LED_IDX, false);
        set_led(BLUE_LED_IDX, true);
        set_led(GREEN_LED_IDX, false);
#endif
        break;
    case BAT_LED_FULL:
#if !BDKB_SINGLE_LED_MODE
        set_led(RED_LED_IDX, false);
        set_led(BLUE_LED_IDX, false);
        set_led(GREEN_LED_IDX, true);
#endif
        break;
    default:
        set_led(RED_LED_IDX, false);
#if !BDKB_SINGLE_LED_MODE
        set_led(BLUE_LED_IDX, false);
        set_led(GREEN_LED_IDX, false);
#endif
        break;
    }
}

static void recompute_mode(void) {
#if BDKB_SINGLE_LED_MODE
    if (usb_powered) {
        mode = BAT_LED_CHARGING;
        return;
    }

    mode = (last_soc < CONFIG_BDKB_BATTERY_STATUS_LED_LOW_ON_PCT) ? BAT_LED_LOW : BAT_LED_OFF;
#else
    if (usb_powered) {
        low_batt_active = false;

        if (full_batt_active) {
            if (last_soc <= CONFIG_BDKB_BATTERY_STATUS_LED_FULL_OFF_PCT) {
                full_batt_active = false;
            }
        } else if (last_soc >= CONFIG_BDKB_BATTERY_STATUS_LED_FULL_ON_PCT) {
            full_batt_active = true;
        }

        mode = full_batt_active ? BAT_LED_FULL : BAT_LED_CHARGING;
        return;
    }

    full_batt_active = false;

    if (low_batt_active) {
        if (last_soc >= CONFIG_BDKB_BATTERY_STATUS_LED_LOW_OFF_PCT) {
            low_batt_active = false;
        }
    } else if (last_soc <= CONFIG_BDKB_BATTERY_STATUS_LED_LOW_ON_PCT) {
        low_batt_active = true;
    }

    mode = low_batt_active ? BAT_LED_LOW : BAT_LED_OFF;
#endif
}

static void blink_timer_handler(struct k_timer *timer) {
    ARG_UNUSED(timer);

    if (mode != BAT_LED_LOW) {
        return;
    }

    red_blink_on = !red_blink_on;
    apply_leds();
}

K_TIMER_DEFINE(low_batt_blink_timer, blink_timer_handler, NULL);

#if BDKB_SINGLE_LED_MODE
static uint8_t breath_level_from_phase(uint8_t phase) {
    if (phase <= SINGLE_BREATH_PWM_MAX) {
        return phase;
    }

    return (uint8_t)(SINGLE_BREATH_CYCLE_STEPS - phase);
}

static void charging_breath_timer_handler(struct k_timer *timer) {
    ARG_UNUSED(timer);

    if (mode != BAT_LED_CHARGING) {
        return;
    }

    breath_pwm_ctr = (uint8_t)((breath_pwm_ctr + 1U) % (SINGLE_BREATH_PWM_MAX + 1U));
    if (breath_pwm_ctr == 0U) {
        breath_phase = (uint8_t)((breath_phase + 1U) % (SINGLE_BREATH_CYCLE_STEPS + 1U));
    }

    uint8_t brightness = breath_level_from_phase(breath_phase);
    set_led(RED_LED_IDX, breath_pwm_ctr <= brightness);
}

K_TIMER_DEFINE(charging_breath_timer, charging_breath_timer_handler, NULL);
#endif

static void update_led_state(void) {
    enum battery_led_mode prev_mode = mode;

    recompute_mode();

#if BDKB_SINGLE_LED_MODE
    if (mode == BAT_LED_CHARGING) {
        if (prev_mode != BAT_LED_CHARGING) {
            breath_phase = 0;
            breath_pwm_ctr = 0;
            k_timer_start(&charging_breath_timer, K_NO_WAIT, K_MSEC(SINGLE_BREATH_TICK_MS));
        }
    } else if (prev_mode == BAT_LED_CHARGING) {
        k_timer_stop(&charging_breath_timer);
    }
#else
    if (mode == BAT_LED_LOW) {
        if (prev_mode != BAT_LED_LOW) {
            red_blink_on = true;
            k_timer_start(&low_batt_blink_timer, K_MSEC(CONFIG_BDKB_BATTERY_STATUS_LED_BLINK_MS),
                          K_MSEC(CONFIG_BDKB_BATTERY_STATUS_LED_BLINK_MS));
        }
    } else if (prev_mode == BAT_LED_LOW) {
        k_timer_stop(&low_batt_blink_timer);
        red_blink_on = false;
    }
#endif

    apply_leds();
}

static int battery_status_leds_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *battery_ev = as_zmk_battery_state_changed(eh);
    if (battery_ev != NULL) {
        last_soc = battery_ev->state_of_charge;
        update_led_state();
        return 0;
    }

    if (as_zmk_usb_conn_state_changed(eh) != NULL) {
        usb_powered = zmk_usb_is_powered();
        update_led_state();
        return 0;
    }

    return -ENOTSUP;
}

ZMK_LISTENER(battery_status_leds, battery_status_leds_listener);
ZMK_SUBSCRIPTION(battery_status_leds, zmk_battery_state_changed);
ZMK_SUBSCRIPTION(battery_status_leds, zmk_usb_conn_state_changed);

static int zmk_battery_status_leds_init(void) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("LED device \"%s\" is not ready", led_dev->name);
        return -ENODEV;
    }

    last_soc = zmk_battery_state_of_charge();
    usb_powered = zmk_usb_is_powered();
    update_led_state();
    k_timer_start(&status_log_timer, K_SECONDS(10), K_SECONDS(10));

    return 0;
}

SYS_INIT(zmk_battery_status_leds_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
