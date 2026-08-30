#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/endpoints.h>
#include <zmk/pm.h>

#define SP3T_KSCAN_NODE DT_NODELABEL(kscan_toggle)

BUILD_ASSERT(DT_NODE_EXISTS(SP3T_KSCAN_NODE),
             "BDKB_SP3T_MODE_SWITCH requires kscan_toggle node label");
BUILD_ASSERT(DT_PROP_LEN(SP3T_KSCAN_NODE, input_gpios) == 3,
             "BDKB_SP3T_MODE_SWITCH expects exactly 3 input-gpios for SP3T");

enum sp3t_mode {
    SP3T_MODE_USB = 0,
    SP3T_MODE_BLE,
    SP3T_MODE_SOFT_OFF,
};

static const struct device *const sp3t_kscan_dev = DEVICE_DT_GET(SP3T_KSCAN_NODE);
static const struct gpio_dt_spec sp3t_inputs[] = {
    GPIO_DT_SPEC_GET_BY_IDX(SP3T_KSCAN_NODE, input_gpios, 0),
    GPIO_DT_SPEC_GET_BY_IDX(SP3T_KSCAN_NODE, input_gpios, 1),
    GPIO_DT_SPEC_GET_BY_IDX(SP3T_KSCAN_NODE, input_gpios, 2),
};
static struct k_work_delayable sp3t_apply_work;

static bool pending_mode_valid;
static enum sp3t_mode pending_mode;

static bool active_mode_valid;
static enum sp3t_mode active_mode;

static const char *sp3t_mode_to_str(enum sp3t_mode mode) {
    switch (mode) {
    case SP3T_MODE_USB:
        return "USB_ONLY";
    case SP3T_MODE_BLE:
        return "BLE_ON";
    case SP3T_MODE_SOFT_OFF:
        return "SOFT_OFF";
    default:
        return "UNKNOWN";
    }
}

static bool sp3t_mode_from_column(uint32_t column, enum sp3t_mode *mode) {
    switch (column) {
    case 0:
        *mode = SP3T_MODE_USB;
        return true;
    case 1:
        *mode = SP3T_MODE_BLE;
        return true;
    case 2:
        *mode = SP3T_MODE_SOFT_OFF;
        return true;
    default:
        return false;
    }
}

static int sp3t_apply_mode(enum sp3t_mode mode) {
    switch (mode) {
    case SP3T_MODE_USB:
        return zmk_endpoints_select_transport(ZMK_TRANSPORT_USB);
    case SP3T_MODE_BLE:
        return zmk_endpoints_select_transport(ZMK_TRANSPORT_BLE);
    case SP3T_MODE_SOFT_OFF:
        LOG_INF("SP3T applying SOFT_OFF");
        return zmk_pm_soft_off();
    default:
        return -EINVAL;
    }
}

static void sp3t_log_boot_levels(void) {
    for (size_t i = 0; i < ARRAY_SIZE(sp3t_inputs); i++) {
        const struct gpio_dt_spec *pin = &sp3t_inputs[i];

        if (!device_is_ready(pin->port)) {
            LOG_WRN("SP3T boot pin[%u] port not ready", (unsigned int)i);
            continue;
        }

        int rc = gpio_pin_configure_dt(pin, GPIO_INPUT);
        if (rc != 0) {
            LOG_WRN("SP3T boot pin[%u] configure failed: %d", (unsigned int)i, rc);
            continue;
        }

        int raw = gpio_pin_get_raw(pin->port, pin->pin);
        if (raw < 0) {
            LOG_WRN("SP3T boot pin[%u] raw read failed: %d", (unsigned int)i, raw);
            continue;
        }

        int logical = gpio_pin_get_dt(pin);
        if (logical < 0) {
            LOG_WRN("SP3T boot pin[%u] logical read failed: %d", (unsigned int)i, logical);
            continue;
        }

        LOG_INF("SP3T boot pin[%u] %s/%u raw=%d logical=%d flags=0x%x", (unsigned int)i,
                pin->port->name, pin->pin, raw, logical, (unsigned int)pin->dt_flags);
    }

}

static void sp3t_apply_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (!pending_mode_valid) {
        return;
    }

    enum sp3t_mode mode = pending_mode;
    pending_mode_valid = false;

    if (active_mode_valid && active_mode == mode) {
        LOG_DBG("SP3T unchanged: %s", sp3t_mode_to_str(mode));
        return;
    }

    LOG_DBG("SP3T applying mode: %s", sp3t_mode_to_str(mode));

    int rc = sp3t_apply_mode(mode);
    if (rc != 0) {
        LOG_WRN("SP3T failed to apply mode %s: %d", sp3t_mode_to_str(mode), rc);
        return;
    }

    active_mode = mode;
    active_mode_valid = true;
    LOG_INF("SP3T active mode: %s", sp3t_mode_to_str(mode));
}

static void sp3t_kscan_callback(const struct device *dev, uint32_t row, uint32_t column,
                                bool pressed) {
    ARG_UNUSED(dev);

    LOG_INF("SP3T event row=%u col=%u state=%s", row, column, pressed ? "on" : "off");

    if (!pressed) {
        LOG_INF("SP3T ignore release event at col=%u", column);
        return;
    }

    if (row != 0) {
        LOG_WRN("SP3T unexpected row=%u (expected 0)", row);
        return;
    }

    enum sp3t_mode mode;
    if (!sp3t_mode_from_column(column, &mode)) {
        LOG_WRN("SP3T ignoring unknown column: %u", column);
        return;
    }

    pending_mode = mode;
    pending_mode_valid = true;
    k_work_reschedule(&sp3t_apply_work, K_MSEC(CONFIG_BDKB_SP3T_MODE_SWITCH_DEBOUNCE_MS));
}

static int bdkb_sp3t_mode_switch_init(void) {
    if (!device_is_ready(sp3t_kscan_dev)) {
        LOG_ERR("SP3T kscan device \"%s\" is not ready", sp3t_kscan_dev->name);
        return -ENODEV;
    }

#if IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
    if (pm_device_runtime_is_enabled(sp3t_kscan_dev)) {
        int pm_rc = pm_device_runtime_get(sp3t_kscan_dev);
        if (pm_rc != 0) {
            LOG_WRN("SP3T pm_device_runtime_get failed: %d", pm_rc);
        }
    }
#elif IS_ENABLED(CONFIG_PM_DEVICE)
    if (pm_device_wakeup_is_capable(sp3t_kscan_dev)) {
        pm_device_wakeup_enable(sp3t_kscan_dev, true);
    }

    int pm_rc = pm_device_action_run(sp3t_kscan_dev, PM_DEVICE_ACTION_RESUME);
    if (pm_rc != 0 && pm_rc != -EALREADY) {
        LOG_WRN("SP3T PM resume failed: %d", pm_rc);
    }
#endif

    sp3t_log_boot_levels();

    k_work_init_delayable(&sp3t_apply_work, sp3t_apply_work_handler);

    int rc = kscan_config(sp3t_kscan_dev, sp3t_kscan_callback);
    if (rc != 0) {
        LOG_ERR("SP3T kscan_config failed: %d", rc);
        return rc;
    }

    rc = kscan_enable_callback(sp3t_kscan_dev);
    if (rc != 0) {
        LOG_ERR("SP3T kscan_enable_callback failed: %d", rc);
        return rc;
    }

    LOG_INF("SP3T mode switch initialized (col0=USB, col1=BLE, col2=SOFT_OFF)");
    return 0;
}

SYS_INIT(bdkb_sp3t_mode_switch_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
