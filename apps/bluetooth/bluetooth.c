/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

#include "action.h"
#include "bt_hci.h"
#include "button.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "file.h"
#include "kernel.h"
#include "list.h"
#include "misc.h"
#include "mutex.h"
#include "pcm-internal.h"
#include "pcm.h"
#include "pcm_sink.h"
#include "queue.h"
#include "rbpaths.h"
#include "semaphore.h"
#include "settings.h"
#include "sound.h"
#include "pcm_sw_volume.h"
#include "system.h"
#include "thread.h"
#include "tick.h"
#include "tlsf.h"
#include "vuprintf.h"
#include "bluetooth.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Paired devices in most recently used order, one "aa:bb:cc:dd:ee:ff name"
 * per line. Bluedroid keeps the link keys itself, in bt_config.conf. */
#define BT_DEVICES_FILE ROCKBOX_DIR "/bt_devices.txt"

/* How long to wait for the A2DP link to drop when powering down */
#define BT_DISCONNECT_TIMEOUT (3*HZ)

/***************************************************************************
 * Debug log (BT_DEBUG only, see the target config)
 ***************************************************************************/

static int gen_log_fd = -1;
static struct mutex gen_log_mutex;

/* Syncing every line throttled streaming at higher bitrates, as the stack
 * logs every L2CAP congestion. Sync at most every 100 ms instead, which
 * still keeps almost everything before a crash. */
static void bt_log_sync(void)
{
    static long next_sync;
    if (TIME_AFTER(current_tick, next_sync)) {
        fsync(gen_log_fd);
        next_sync = current_tick + HZ/10;
    }
}

static void bt_log(const char *fmt, ...)
{
    if (gen_log_fd == -1)
        return;

    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    mutex_lock(&gen_log_mutex);
    write(gen_log_fd, buf, strlen(buf));
    bt_log_sync();
    mutex_unlock(&gen_log_mutex);
}

/***************************************************************************
 * State
 *
 * Written by the manager thread and by Bluedroid's callbacks (which run on
 * its BTC thread), read by the UI. Rockbox threads are cooperative, so
 * updates that don't block are effectively atomic.
 ***************************************************************************/

enum bt_power {
    BT_POWER_OFF,
    BT_POWER_STARTING,
    BT_POWER_ON,
    BT_POWER_STOPPING,
};

enum bt_conn {
    BT_CONN_NONE,
    BT_CONN_CONNECTING,
    BT_CONN_CONNECTED,
};

static volatile enum bt_power bt_power = BT_POWER_OFF;
static volatile enum bt_conn bt_conn = BT_CONN_NONE;
static bool bt_start_failed = false;
static bool bt_connect_failed = false;
/* Failures of automatic reconnects aren't reported, the device may be off */
static bool bt_user_connect = false;
/* Reconnecting needs both the device list loaded (bt_power is ON) and the
 * A2DP profile up, which Bluedroid reports asynchronously */
static bool bt_a2dp_ready = false;
static bool bt_autoconnect_pending = false;
static bool bt_scanning = false;

/* Device being connected or connected to, valid unless bt_conn is NONE */
static esp_bd_addr_t bt_active_addr;
/* Device to connect to once the current link is down */
static esp_bd_addr_t bt_pending_addr;
static bool bt_have_pending = false;
/* Device the user asked to connect to, which may pair with us. Anything
 * else must already be paired, since we accept incoming connections. */
static esp_bd_addr_t bt_pairing_addr;
static bool bt_pairing_allowed = false;
/* Disconnect once the link being set up is up: it can't be cancelled */
static bool bt_disconnect_requested = false;
/* SYS_BLUETOOTH_CONNECTED was sent, so DISCONNECTED is due */
static bool bt_announced = false;

/* Paired devices first, most recently used first; then scan results */
static struct bt_device bt_devices[BT_MAX_DEVICES];
static int bt_num_devices = 0;

/* Ask whichever screen is showing Bluetooth state to redraw. A full button
 * queue overwrites its oldest event, which may be a key press or one of our
 * SYS events, so don't pile up redraws (search results come in bursts). */
static void bt_notify(void)
{
    if (button_queue_count() < 4)
        button_queue_post(BUTTON_REDRAW, 0);
}

/* Moves playback to or from Bluetooth, powering the local output stage
 * down while it isn't used */
static void bt_route_audio(bool bluetooth)
{
    if (bluetooth) {
        pcm_switch_sink(PCM_SINK_BLUETOOTH);
        bt_target_local_output(false);
    } else {
        bt_target_local_output(true);
        pcm_switch_sink(PCM_SINK_BUILTIN);
    }
}

/***************************************************************************
 * Device list
 ***************************************************************************/

static int bt_find_device(const uint8_t *addr)
{
    for (int i = 0; i < bt_num_devices; i++) {
        if (memcmp(bt_devices[i].addr, addr, BT_ADDR_LEN) == 0)
            return i;
    }
    return -1;
}

/* Finds a device, or appends it as unpaired. Returns -1 if the list is full
 * of paired devices. */
static int bt_add_device(const uint8_t *addr)
{
    int i = bt_find_device(addr);
    if (i >= 0)
        return i;

    if (bt_num_devices == BT_MAX_DEVICES) {
        /* Make room by dropping the last scan result */
        if (bt_devices[BT_MAX_DEVICES - 1].paired)
            return -1;
        bt_num_devices--;
    }

    i = bt_num_devices++;
    memset(&bt_devices[i], 0, sizeof(bt_devices[i]));
    memcpy(bt_devices[i].addr, addr, BT_ADDR_LEN);
    return i;
}

/* Marks a device paired and moves it to the front of the list */
static void bt_promote_device(int i)
{
    struct bt_device dev = bt_devices[i];
    dev.paired = true;
    memmove(&bt_devices[1], &bt_devices[0], i * sizeof(dev));
    bt_devices[0] = dev;
}

static void bt_remove_device(int i)
{
    bt_num_devices--;
    memmove(&bt_devices[i], &bt_devices[i + 1],
            (bt_num_devices - i) * sizeof(bt_devices[0]));
}

/* Names come from the remote device, and may not be NUL terminated (EIR
 * data) or may hold control characters, which would break the devices
 * file: it has one per line */
static void bt_set_device_name(int i, const uint8_t *name, size_t len)
{
    if (name == NULL || len == 0 || name[0] == '\0')
        return;

    char *out = bt_devices[i].name;
    size_t n = 0;
    while (n < len && n < BT_NAME_LEN - 1 && name[n] != '\0') {
        out[n] = name[n] < 0x20 ? ' ' : name[n];
        n++;
    }
    out[n] = '\0';
}

static void bt_drop_scan_results(void)
{
    int i = 0;
    while (i < bt_num_devices && bt_devices[i].paired)
        i++;
    bt_num_devices = i;
}

static bool bt_parse_addr(const char *s, uint8_t *addr)
{
    for (int i = 0; i < BT_ADDR_LEN; i++) {
        int val = 0;
        for (int j = 0; j < 2; j++) {
            char c = *s++;
            val <<= 4;
            if (c >= '0' && c <= '9')
                val |= c - '0';
            else if (c >= 'a' && c <= 'f')
                val |= c - 'a' + 10;
            else if (c >= 'A' && c <= 'F')
                val |= c - 'A' + 10;
            else
                return false;
        }
        addr[i] = val;
        if (*s++ != (i < BT_ADDR_LEN - 1 ? ':' : ' '))
            return false;
    }
    return true;
}

/* Rebuilds the list from Bluedroid's bonds, in the order and with the names
 * we saved. Called on the manager thread after the stack is up. */
static void bt_load_devices(void)
{
    esp_bd_addr_t bonded[BT_MAX_DEVICES];
    int num_bonded = BT_MAX_DEVICES;
    if (esp_bt_gap_get_bond_device_list(&num_bonded, bonded) != ESP_OK)
        num_bonded = 0;

    bt_num_devices = 0;

    int fd = open(BT_DEVICES_FILE, O_RDONLY);
    if (fd >= 0) {
        char line[18 + BT_NAME_LEN];
        while (read_line(fd, line, sizeof(line)) > 0) {
            uint8_t addr[BT_ADDR_LEN];
            if (!bt_parse_addr(line, addr))
                continue;

            /* Skip devices whose bond has gone missing */
            bool bonded_dev = false;
            for (int i = 0; i < num_bonded && !bonded_dev; i++)
                bonded_dev = memcmp(bonded[i], addr, BT_ADDR_LEN) == 0;
            if (!bonded_dev || bt_find_device(addr) >= 0)
                continue;

            int i = bt_add_device(addr);
            if (i < 0)
                break;
            bt_devices[i].paired = true;
            bt_set_device_name(i, (const uint8_t *)line + 18,
                               sizeof(line) - 18);
        }
        close(fd);
    }

    /* Bonded devices we never saved, e.g. paired by an older build */
    for (int b = 0; b < num_bonded; b++) {
        int i = bt_add_device(bonded[b]);
        if (i >= 0)
            bt_devices[i].paired = true;
    }
}

/* Manager thread only: file I/O yields, so work on a snapshot */
static void bt_save_devices(void)
{
    struct bt_device devs[BT_MAX_DEVICES];
    int num = bluetooth_get_devices(devs, BT_MAX_DEVICES);

    int fd = open(BT_DEVICES_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0)
        return;
    for (int i = 0; i < num && devs[i].paired; i++) {
        fdprintf(fd, ESP_BD_ADDR_STR " %s\n",
                 ESP_BD_ADDR_HEX(devs[i].addr), devs[i].name);
    }
    close(fd);
}

/***************************************************************************
 * Manager thread
 ***************************************************************************/

enum {
    BTM_ENABLE = 1,
    BTM_DISABLE,
    BTM_SAVE_DEVICES,
    BTM_SEND_VOLUME,
};

static struct event_queue bt_queue;
static bool bt_queue_ready = false;
static long bt_stack[0x2000 / sizeof(long)];
static const char bt_thread_name[] = "bluetooth";

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event,
                          esp_bt_gap_cb_param_t *param);
static void bt_app_a2d_cb(esp_a2d_cb_event_t event,
                          esp_a2d_cb_param_t *param);
static void bt_app_avrc_ct_cb(esp_avrc_ct_cb_event_t event,
                              esp_avrc_ct_cb_param_t *param);
static void bt_app_avrc_tg_cb(esp_avrc_tg_cb_event_t event,
                              esp_avrc_tg_cb_param_t *param);
static int32_t bt_app_a2d_data_cb(uint8_t *buf, int32_t len);
static void bt_reset_abs_volume(void);
static void bt_send_pending_volume(void);
static void bt_check_reconfigure_timeout(void);

/* Tolerates a partially started stack */
static void bt_stack_teardown(void)
{
    esp_bt_gap_cancel_discovery();

    /* Profile deinit completes asynchronously on the BTC thread. Shutting
     * Bluedroid down first leaves A2DP's global state behind, and then
     * esp_a2d_source_init() fails until reboot. */
    if (esp_a2d_source_deinit() == ESP_OK) {
        /* It waits for the profile to be idle, e.g. for a connection
         * attempt to time out (5.12 s for paging) */
        long deadline = current_tick + 6*HZ;
        while (bt_a2dp_ready && TIME_BEFORE(current_tick, deadline))
            sleep(HZ/20);
        if (bt_a2dp_ready)
            bt_log("a2dp deinit timed out\n");
    }
    esp_avrc_ct_deinit();
    esp_avrc_tg_deinit();
    sleep(HZ/10);

    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    bt_hci_disable();

    int old_fd = gen_log_fd;
    gen_log_fd = -1;
    if (old_fd != -1)
        close(old_fd);
}

#define BT_CHECK(call) \
    do { \
        esp_err_t ret_ = (call); \
        if (ret_ != ESP_OK) { \
            bt_log("%s failed: %d\n", #call, ret_); \
            goto fail; \
        } \
    } while (0)

static bool bt_stack_start(void)
{
#ifdef BT_DEBUG
    /* Keep the previous log, e.g. from before a crash */
    remove("/bt_log.old.txt");
    rename("/bt_log.txt", "/bt_log.old.txt");
    gen_log_fd = open("/bt_log.txt", O_WRONLY | O_CREAT | O_TRUNC, 0666);
#endif

    if (!bt_hci_enable()) {
        bt_log("controller bring-up failed\n");
        goto fail;
    }

    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    BT_CHECK(esp_bluedroid_init_with_cfg(&cfg));
    BT_CHECK(esp_bluedroid_enable());

    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    BT_CHECK(esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap,
                                           sizeof(uint8_t)));
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, pin_code);
    BT_CHECK(esp_bt_gap_register_callback(bt_app_gap_cb));
    BT_CHECK(esp_bt_gap_set_device_name("Rockbox"));

    BT_CHECK(esp_avrc_ct_init());
    BT_CHECK(esp_avrc_ct_register_callback(bt_app_avrc_ct_cb));
    BT_CHECK(esp_avrc_tg_init());
    BT_CHECK(esp_avrc_tg_register_callback(bt_app_avrc_tg_cb));

    /* Passthrough commands we handle, see bt_app_avrc_tg_cb() */
    static const uint8_t psth_cmds[] = {
        ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_PAUSE, ESP_AVRC_PT_CMD_STOP,
        ESP_AVRC_PT_CMD_FORWARD, ESP_AVRC_PT_CMD_BACKWARD,
        ESP_AVRC_PT_CMD_FAST_FORWARD, ESP_AVRC_PT_CMD_REWIND,
    };
    esp_avrc_psth_bit_mask_t psth = {0};
    for (size_t i = 0; i < ARRAYLEN(psth_cmds); i++) {
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth,
                                         psth_cmds[i]);
    }
    BT_CHECK(esp_avrc_tg_set_psth_cmd_filter(
        ESP_AVRC_PSTH_FILTER_SUPPORTED_CMD, &psth));

    BT_CHECK(esp_a2d_source_init());
    BT_CHECK(esp_a2d_register_callback(bt_app_a2d_cb));
    BT_CHECK(esp_a2d_source_register_data_callback(bt_app_a2d_data_cb));

    /* Connectable, so paired headphones can reconnect on their own */
    BT_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                      ESP_BT_NON_DISCOVERABLE));
    return true;

fail:
    bt_stack_teardown();
    return false;
}

static void bt_start_connect(const uint8_t *addr, bool allow_pairing)
{
    esp_bt_gap_cancel_discovery();

    memcpy(bt_active_addr, addr, BT_ADDR_LEN);
    if (allow_pairing) {
        memcpy(bt_pairing_addr, addr, BT_ADDR_LEN);
        bt_pairing_allowed = true;
    }

    bt_conn = BT_CONN_CONNECTING;
    bt_user_connect = allow_pairing;
    bt_connect_failed = false;
    bt_disconnect_requested = false;
    esp_err_t ret = esp_a2d_source_connect(bt_active_addr);
    if (ret != ESP_OK) {
        bt_log("connect failed: %d\n", ret);
        bt_conn = BT_CONN_NONE;
        bt_connect_failed = bt_user_connect;
        bt_pairing_allowed = false;
    }
    bt_notify();
}

/* Takes the current link down, or the one being set up once it's up */
static void bt_drop_link(void)
{
    if (bt_conn == BT_CONN_CONNECTING)
        bt_disconnect_requested = true;
    esp_a2d_source_disconnect(bt_active_addr);
}

/* Reconnects to the most recently used device, unless it beat us to it */
static void bt_try_autoconnect(void)
{
    if (!bt_autoconnect_pending || !bt_a2dp_ready || bt_power != BT_POWER_ON)
        return;

    bt_autoconnect_pending = false;
    if (bt_conn == BT_CONN_NONE && bt_num_devices > 0 && bt_devices[0].paired)
        bt_start_connect(bt_devices[0].addr, false);
}

static void bt_do_enable(void)
{
    if (bt_power != BT_POWER_OFF)
        return;

    bt_power = BT_POWER_STARTING;
    bt_start_failed = false;
    bt_a2dp_ready = false;
    bt_notify();

    if (!bt_stack_start()) {
        bt_power = BT_POWER_OFF;
        bt_start_failed = true;
        bt_notify();
        return;
    }

    bt_load_devices();
    bt_power = BT_POWER_ON;
    bt_autoconnect_pending = true;
    bt_try_autoconnect();
    bt_notify();
}

static void bt_do_disable(void)
{
    if (bt_power != BT_POWER_ON)
        return;

    bt_power = BT_POWER_STOPPING;
    bt_have_pending = false;
    bt_autoconnect_pending = false;
    bt_notify();

    if (bt_conn != BT_CONN_NONE) {
        esp_a2d_source_disconnect(bt_active_addr);
        long deadline = current_tick + BT_DISCONNECT_TIMEOUT;
        while (bt_conn != BT_CONN_NONE && TIME_BEFORE(current_tick, deadline))
            sleep(HZ/10);
    }

    bt_route_audio(false);
    bt_stack_teardown();
    bt_reset_abs_volume();

    /* In case the link didn't go down in time */
    if (bt_announced) {
        bt_announced = false;
        button_queue_post_remove_head(SYS_BLUETOOTH_DISCONNECTED, 0);
    }

    bt_conn = BT_CONN_NONE;
    bt_scanning = false;
    bt_pairing_allowed = false;
    bt_disconnect_requested = false;
    bt_power = BT_POWER_OFF;
    bt_notify();
}

static void bt_thread(void)
{
    struct queue_event ev;

    while (true) {
        queue_wait_w_tmo(&bt_queue, &ev, HZ);
        switch (ev.id) {
            case SYS_TIMEOUT:
                bt_check_reconfigure_timeout();
                break;
            case BTM_ENABLE:
                bt_do_enable();
                break;
            case BTM_DISABLE:
                bt_do_disable();
                break;
            case BTM_SAVE_DEVICES:
                bt_save_devices();
                break;
            case BTM_SEND_VOLUME:
                bt_send_pending_volume();
                break;
        }
    }
}

/***************************************************************************
 * Public API
 ***************************************************************************/

void bluetooth_init(void)
{
    mutex_init(&gen_log_mutex);
    /* Not registered for broadcasts: we'd have to acknowledge USB */
    queue_init(&bt_queue, false);
    create_thread(bt_thread, bt_stack, sizeof(bt_stack), 0, bt_thread_name
                  IF_PRIO(, PRIORITY_BLUETOOTH) IF_COP(, CPU));
    bt_queue_ready = true;

    if (global_settings.bt_enabled)
        bluetooth_set_enabled(true);
}

/* Also the setting's callback, which runs when config.cfg is loaded at
 * boot, before bluetooth_init() */
void bluetooth_set_enabled(bool enable)
{
    if (bt_queue_ready)
        queue_post(&bt_queue, enable ? BTM_ENABLE : BTM_DISABLE, 0);
}

enum bt_state bluetooth_get_state(void)
{
    switch (bt_power) {
        case BT_POWER_OFF:
            return BT_STATE_OFF;
        case BT_POWER_STARTING:
            return BT_STATE_STARTING;
        case BT_POWER_STOPPING:
            return BT_STATE_STOPPING;
        default:
            break;
    }

    switch (bt_conn) {
        case BT_CONN_CONNECTING:
            return BT_STATE_CONNECTING;
        case BT_CONN_CONNECTED:
            return BT_STATE_CONNECTED;
        default:
            return BT_STATE_IDLE;
    }
}

bool bluetooth_failed(void)
{
    return bt_start_failed;
}

bool bluetooth_connect_failed(void)
{
    return bt_connect_failed;
}

bool bluetooth_is_scanning(void)
{
    return bt_scanning;
}

void bluetooth_scan(void)
{
    if (bt_power != BT_POWER_ON || bt_scanning ||
        bt_conn == BT_CONN_CONNECTING)
        return;

    bt_drop_scan_results();
    /* 5 * 1.28 s */
    esp_err_t ret =
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 5, 0);
    if (ret == ESP_OK)
        bt_scanning = true;
    else
        bt_log("start discovery failed: %d\n", ret);
    bt_notify();
}

void bluetooth_connect(const uint8_t *addr)
{
    if (bt_power != BT_POWER_ON)
        return;

    if (bt_conn != BT_CONN_NONE) {
        if (memcmp(addr, bt_active_addr, BT_ADDR_LEN) == 0)
            return;
        /* Only one link at a time: switch once this one is down */
        memcpy(bt_pending_addr, addr, BT_ADDR_LEN);
        bt_have_pending = true;
        bt_drop_link();
        return;
    }

    bt_start_connect(addr, true);
}

void bluetooth_disconnect(void)
{
    if (bt_power != BT_POWER_ON || bt_conn == BT_CONN_NONE)
        return;

    bt_have_pending = false;
    bt_drop_link();
}

void bluetooth_forget(const uint8_t *addr)
{
    if (bt_power != BT_POWER_ON)
        return;

    esp_bd_addr_t bda;
    memcpy(bda, addr, BT_ADDR_LEN);

    if (bt_conn != BT_CONN_NONE &&
        memcmp(bda, bt_active_addr, BT_ADDR_LEN) == 0) {
        bt_have_pending = false;
        bt_drop_link();
    }
    esp_bt_gap_remove_bond_device(bda);

    int i = bt_find_device(bda);
    if (i >= 0)
        bt_remove_device(i);
    queue_post(&bt_queue, BTM_SAVE_DEVICES, 0);
    bt_notify();
}

int bluetooth_get_devices(struct bt_device *devs, int max)
{
    int num = MIN(max, bt_num_devices);
    memcpy(devs, bt_devices, num * sizeof(*devs));
    return num;
}

bool bluetooth_get_active(uint8_t *addr)
{
    if (bt_power != BT_POWER_ON || bt_conn == BT_CONN_NONE)
        return false;
    memcpy(addr, bt_active_addr, BT_ADDR_LEN);
    return true;
}

/***************************************************************************
 * GAP: discovery, pairing, names
 ***************************************************************************/

static bool bt_may_pair(const uint8_t *addr)
{
    if (bt_pairing_allowed && memcmp(addr, bt_pairing_addr, BT_ADDR_LEN) == 0)
        return true;

    /* Known devices may redo pairing, e.g. after a reset on their side */
    int i = bt_find_device(addr);
    return i >= 0 && bt_devices[i].paired;
}

static void process_inquiry_scan_result(esp_bt_gap_cb_param_t *param)
{
    uint32_t cod = 0;     /* class of device */
    uint8_t *eir = NULL;
    esp_bt_gap_dev_prop_t *p;

    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR:
            eir = (uint8_t *)(p->val);
            break;
        default:
            break;
        }
    }

    if (!esp_bt_gap_is_valid_cod(cod) || eir == NULL) {
        return;
    }

    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    /* get complete or short local name from eir data */
    rmt_bdname = esp_bt_gap_resolve_eir_data(eir,
        ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir,
            ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }
    if (!rmt_bdname) {
        return; /* Don't show nameless devices */
    }

    int i = bt_add_device(param->disc_res.bda);
    if (i < 0)
        return;

    bt_set_device_name(i, rmt_bdname, rmt_bdname_len);
    bt_notify();
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event,
                          esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT:
            process_inquiry_scan_result(param);
            break;

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            bt_log("gap discovery %d\n", param->disc_st_chg.state);
            bt_scanning =
                param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED;
            bt_notify();
            break;

        case ESP_BT_GAP_CFM_REQ_EVT: {
            bool accept = bt_may_pair(param->cfm_req.bda);
            bt_log("gap confirm " ESP_BD_ADDR_STR " %d\n",
                   ESP_BD_ADDR_HEX(param->cfm_req.bda), accept);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, accept);
            break;
        }

        case ESP_BT_GAP_PIN_REQ_EVT: {
            /* Legacy pairing: headphones almost always use 0000 */
            esp_bt_pin_code_t pin_code = {0};
            if (!bt_may_pair(param->pin_req.bda)) {
                esp_bt_gap_pin_reply(param->pin_req.bda, false, 0, pin_code);
            } else if (param->pin_req.min_16_digit) {
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {
                memset(pin_code, '0', 4);
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;
        }

        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            bt_log("gap auth " ESP_BD_ADDR_STR " stat %d\n",
                   ESP_BD_ADDR_HEX(param->auth_cmpl.bda),
                   param->auth_cmpl.stat);
            if (param->auth_cmpl.stat != ESP_BT_STATUS_SUCCESS)
                break;
            int i = bt_add_device(param->auth_cmpl.bda);
            if (i < 0)
                break;
            bt_set_device_name(i, param->auth_cmpl.device_name,
                               sizeof(param->auth_cmpl.device_name));
            bt_promote_device(i);
            queue_post(&bt_queue, BTM_SAVE_DEVICES, 0);
            bt_notify();
            break;
        }

        case ESP_BT_GAP_READ_REMOTE_NAME_EVT: {
            if (param->read_rmt_name.stat != ESP_BT_STATUS_SUCCESS)
                break;
            int i = bt_find_device(param->read_rmt_name.bda);
            if (i < 0)
                break;
            bt_set_device_name(i, param->read_rmt_name.rmt_name,
                               sizeof(param->read_rmt_name.rmt_name));
            if (bt_devices[i].paired)
                queue_post(&bt_queue, BTM_SAVE_DEVICES, 0);
            bt_notify();
            break;
        }

        default:
            bt_log("gap %d\n", event);
            break;
    }
}

/***************************************************************************
 * A2DP and AVRCP
 ***************************************************************************/

static esp_a2d_conn_hdl_t conn_hdl;
/* Channel modes the connected sink supports */
static uint8_t bt_sink_ch_modes = ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO;
static bool sampr_switch_ongoing = false;

/* Channel mode last asked for */
static bool bt_pref_dual = false;

/* Dual channel at the highest bitpool the sink allows ("SBC XQ") has about
 * twice the bitrate of joint stereo */
static bool bt_want_dual(void)
{
    return global_settings.bt_high_quality &&
           (bt_sink_ch_modes & ESP_A2D_SBC_CIE_CH_MODE_DUAL_CHANNEL);
}

/* Bluedroid picks the bitpool within min_bitpool..max_bitpool and the
 * sink's limits */
static void bt_fill_pref_mcc(esp_a2d_mcc_t *mcc, uint8_t samp_freq)
{
    bt_pref_dual = bt_want_dual();

    mcc->type = ESP_A2D_MCT_SBC;
    mcc->cie.sbc_info.samp_freq    = samp_freq;
    mcc->cie.sbc_info.ch_mode      = bt_pref_dual ?
                                     ESP_A2D_SBC_CIE_CH_MODE_DUAL_CHANNEL :
                                     ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO;
    mcc->cie.sbc_info.block_len    = ESP_A2D_SBC_CIE_BLOCK_LEN_16;
    mcc->cie.sbc_info.num_subbands = ESP_A2D_SBC_CIE_NUM_SUBBANDS_8;
    mcc->cie.sbc_info.alloc_mthd   = ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS;
    mcc->cie.sbc_info.min_bitpool  = 3;
    mcc->cie.sbc_info.max_bitpool  = 53;
}

static struct semaphore sink_control_sem;
static struct event_queue sink_control_queue;
static esp_a2d_mcc_t new_pref_mcc;

static int sink_suspended = 1;

/* A reconfiguration that hasn't finished by then is given up, so that a
 * lost event can't leave playback suspended for good */
#define BT_RECONFIGURE_TIMEOUT (3*HZ)
static long bt_reconfigure_deadline;

static bool bt_sink_resume(void);

static void bt_end_reconfigure(void)
{
    sampr_switch_ongoing = false;
    bt_sink_resume();
}

static void bt_set_new_pref_mcc(void)
{
    esp_err_t ret = esp_a2d_source_set_pref_mcc(conn_hdl, &new_pref_mcc);
    if (ret != ESP_OK) {
        bt_log("set pref mcc failed: %d\n", ret);
        bt_end_reconfigure();
    }
}

/* Called periodically by the manager thread */
static void bt_check_reconfigure_timeout(void)
{
    if (sampr_switch_ongoing &&
        TIME_AFTER(current_tick, bt_reconfigure_deadline)) {
        bt_log("codec reconfiguration timed out\n");
        bt_end_reconfigure();
    }
}

/* Media control commands go out one at a time: the semaphore is free while
 * none is in flight, and the A2DP callback sends queued ones on its ack */
static bool bt_sink_suspend(void)
{
    if (++sink_suspended == 1) {
        bt_log("suspending playback: sink locked\n");
        if (semaphore_wait(&sink_control_sem, TIMEOUT_NOBLOCK) ==
            OBJ_WAIT_SUCCEEDED) {
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
        } else {
            queue_post(&sink_control_queue, 1, ESP_A2D_MEDIA_CTRL_SUSPEND);
        }
        return true;
    }
    return false;
}

static bool bt_sink_resume(void)
{
    if (--sink_suspended == 0) {
        bt_log("starting playback: sink unlocked\n");
        if (semaphore_wait(&sink_control_sem, TIMEOUT_NOBLOCK) ==
            OBJ_WAIT_SUCCEEDED) {
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
        } else {
            queue_post(&sink_control_queue, 1, ESP_A2D_MEDIA_CTRL_START);
        }
        return true;
    }
    return false;
}

static void bt_handle_a2d_connection(esp_a2d_cb_param_t *param)
{
    const uint8_t *addr = param->conn_stat.remote_bda;

    /* Only one link at a time: anything else is about another device, e.g.
     * a connection attempt we turned down */
    if (param->conn_stat.state != ESP_A2D_CONNECTION_STATE_CONNECTED &&
        bt_conn != BT_CONN_NONE &&
        memcmp(addr, bt_active_addr, BT_ADDR_LEN) != 0)
        return;

    switch (param->conn_stat.state) {
        case ESP_A2D_CONNECTION_STATE_CONNECTING:
            memcpy(bt_active_addr, addr, BT_ADDR_LEN);
            bt_conn = BT_CONN_CONNECTING;
            break;

        case ESP_A2D_CONNECTION_STATE_CONNECTED: {
            memcpy(bt_active_addr, addr, BT_ADDR_LEN);
            bt_conn = BT_CONN_CONNECTED;
            bt_pairing_allowed = false;

            int i = bt_add_device(addr);
            if (i >= 0) {
                if (bt_devices[i].name[0] == '\0')
                    esp_bt_gap_read_remote_name(param->conn_stat.remote_bda);
                bt_promote_device(i);
                queue_post(&bt_queue, BTM_SAVE_DEVICES, 0);
            }

            if (bt_disconnect_requested) {
                bt_disconnect_requested = false;
                esp_a2d_source_disconnect(bt_active_addr);
                break;
            }

            semaphore_init(&sink_control_sem, 1, 0);
            queue_init(&sink_control_queue, false);
            sink_suspended = 1;
            sampr_switch_ongoing = false;
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);

            /* Lets misc.c resume playback, like plugging in headphones */
            bt_announced = true;
            button_queue_post_remove_head(SYS_BLUETOOTH_CONNECTED, 0);
            break;
        }

        case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
            bt_route_audio(false);
            break;

        case ESP_A2D_CONNECTION_STATE_DISCONNECTED: {
            bt_route_audio(false);
            bt_reset_abs_volume();

            if (bt_conn == BT_CONN_CONNECTING) {
                /* Not a failure if it was called off */
                bt_connect_failed = bt_user_connect &&
                                    !bt_disconnect_requested;
                bt_pairing_allowed = false;
            }
            bt_conn = BT_CONN_NONE;
            bt_disconnect_requested = false;
            if (bt_announced) {
                /* Lets misc.c pause playback, like unplugging headphones */
                bt_announced = false;
                button_queue_post_remove_head(SYS_BLUETOOTH_DISCONNECTED, 0);
            }

            if (bt_have_pending && bt_power == BT_POWER_ON) {
                bt_have_pending = false;
                bt_start_connect(bt_pending_addr, true);
            }
            break;
        }
    }
    bt_notify();
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event,
                          esp_a2d_cb_param_t *param)
{
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            bt_log("a2d connection %d mtu %d rsn %d\n",
                   param->conn_stat.state, param->conn_stat.audio_mtu,
                   param->conn_stat.disc_rsn);
            bt_handle_a2d_connection(param);
            return;
        case ESP_A2D_PROF_STATE_EVT:
            bt_log("a2d profile %d\n", param->a2d_prof_stat.init_state);
            bt_a2dp_ready =
                param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS;
            bt_try_autoconnect();
            return;
        case ESP_A2D_MEDIA_CTRL_ACK_EVT: {
            esp_a2d_media_ctrl_t cmd = param->media_ctrl_stat.cmd;
            bool ok = param->media_ctrl_stat.status ==
                      ESP_A2D_MEDIA_CTRL_ACK_SUCCESS;
            bt_log("a2d media ctrl %d %d\n", cmd,
                   param->media_ctrl_stat.status);
            if (cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY) {
                /* The link may have gone down meanwhile */
                if (ok && bt_conn == BT_CONN_CONNECTED &&
                    bt_power == BT_POWER_ON)
                    bt_route_audio(true);
            } else if (sampr_switch_ongoing &&
                       cmd == ESP_A2D_MEDIA_CTRL_SUSPEND) {
                bt_set_new_pref_mcc();
            }
            struct queue_event ev;
            queue_wait_w_tmo(&sink_control_queue, &ev, TIMEOUT_NOBLOCK);
            if (ev.id == 1) {
                esp_a2d_media_ctrl(ev.data);
            } else if (ev.id == SYS_TIMEOUT) {
                semaphore_release(&sink_control_sem);
            }
            return;
        }
        case ESP_A2D_REPORT_SNK_CODEC_CAPS_EVT: {
            esp_a2d_mcc_t *sink_mcc =
                &param->a2d_report_snk_codec_caps_stat.mcc;
            bt_log("a2d sink codec %d\n", sink_mcc->type);
            if (sink_mcc->type == ESP_A2D_MCT_SBC) {
                bt_sink_ch_modes = sink_mcc->cie.sbc_info.ch_mode;
                bt_log("a2d sink sbc channel modes 0x%x\n", bt_sink_ch_modes);

                esp_a2d_mcc_t pref_mcc;
                bt_fill_pref_mcc(&pref_mcc, ESP_A2D_SBC_CIE_SF_44K);

                conn_hdl = param->a2d_report_snk_codec_caps_stat.conn_hdl;
                esp_err_t ret = esp_a2d_source_set_pref_mcc(conn_hdl,
                                                            &pref_mcc);
                if (ret != ESP_OK) {
                    bt_log("a2dp source set pref mcc failed: %d\n", ret);
                }
            }
            break;
        }
        case ESP_A2D_SRC_SET_PREF_MCC_EVT:
            bt_log("a2d set pref mcc %d\n",
                   param->a2d_set_pref_mcc_stat.set_status);
            if (sampr_switch_ongoing) {
                bt_log("starting playback: sampr switch done\n");
                bt_end_reconfigure();
            }
            return;
        default:
            bt_log("a2d %d\n", event);
            return;
    }
}

/***************************************************************************
 * Volume
 *
 * Headphones supporting AVRCP absolute volume apply the volume themselves
 * and get a full scale signal, like phones do it; their own volume controls
 * then change Rockbox's volume. Anything else is scaled in software.
 ***************************************************************************/

/* AVRCP transaction labels: the volume notification stays registered, so
 * keep it apart from the commands */
#define BT_TL_GET_CAPS      3
#define BT_TL_VOLUME_NOTIFY 4
#define BT_TL_SET_VOLUME    5

#define BT_AVRC_VOLUME_MAX  127

/* Latest volume from audiohw_set_volume(), in centibels or PCM_MUTE_LEVEL */
static int bt_vol_l = 0;
static int bt_vol_r = 0;
/* The headphones apply the volume */
static bool bt_abs_volume = false;
/* We sent our volume, and switch over once the headphones accept it.
 * Until then both scale, which errs on the quiet side. */
static bool bt_abs_volume_probe = false;
/* Last absolute volume sent to or reported by the headphones, or -1 */
static int bt_remote_volume = -1;
/* A BTM_SEND_VOLUME is queued */
static bool bt_volume_send_queued = false;
/* Latest volume the headphones reported, and a SYS_BLUETOOTH_VOLUME for it
 * is queued */
static int bt_remote_request = 0;
static bool bt_remote_request_queued = false;
static long bt_remote_request_tick;
/* The volume is being set to follow the headphones: don't send it back */
static bool bt_following_remote = false;

/* Volume setting value to centibels, as sound.c does it */
static int bt_volume_to_cb(int value)
{
    int shift = 1 - sound_numdecimals(SOUND_VOLUME);
    if (shift < 0) do { value /= 10; } while (++shift);
    if (shift > 0) do { value *= 10; } while (--shift);
    return value;
}

/* Lossy codecs overshoot near full scale, so with the headphones applying
 * the volume, a full scale signal clipped in their decoder. Keep some
 * headroom; in centibels. */
#define BT_ABS_VOLUME_HEADROOM (-30)

/* Centibels to a volume setting value, the inverse of bt_volume_to_cb() */
static int bt_cb_to_volume(int cb)
{
    int shift = 1 - sound_numdecimals(SOUND_VOLUME);
    if (shift < 0) do { cb *= 10; } while (++shift);
    if (shift > 0) do { cb = (cb + (cb < 0 ? -5 : 5)) / 10; } while (--shift);
    return cb;
}

/* Headphones treat absolute volume as a fader with even steps, which is
 * what Rockbox's volume bar is too, so follow the bar: 127 times its
 * position. Curves of the amplitude, from linear in dB to 10^(dB/35),
 * either played too loud at low volumes or crowded them at the bottom. */
static int bt_cb_to_avrc(int cb)
{
    int min = sound_min(SOUND_VOLUME);
    int max = sound_max(SOUND_VOLUME);

    if (cb == PCM_MUTE_LEVEL || cb <= bt_volume_to_cb(min))
        return 0;
    if (cb >= bt_volume_to_cb(max))
        return BT_AVRC_VOLUME_MAX;

    /* In tenths, to round */
    long pos = to_normalized_volume(bt_cb_to_volume(cb), min, max,
                                    BT_AVRC_VOLUME_MAX * 10);
    int avrc = (pos + 5) / 10;
    /* Only our minimum (mute) maps to 0 */
    return MAX(avrc, 1);
}

/* Returns the volume setting step that maps closest to avrc, given that
 * the target gets offset (in centibels) on top of the setting */
static int bt_avrc_to_volume(int avrc, int offset)
{
    int min = sound_min(SOUND_VOLUME);
    int max = sound_max(SOUND_VOLUME);
    int step = sound_steps(SOUND_VOLUME);

    int best = min, best_diff = INT_MAX;
    for (int vol = min; vol <= max; vol += step) {
        int cb = bt_volume_to_cb(vol) + offset;
        int diff = abs(bt_cb_to_avrc(cb) - avrc);
        if (diff < best_diff) {
            best = vol;
            best_diff = diff;
        }
    }
    return best;
}

/* Thread context only: sending allocates and may wait for the BTC thread */
static void bt_send_remote_volume(int avrc)
{
    bt_remote_volume = avrc;
    bt_log("avrc-ct set volume %d\n", avrc);
    esp_err_t err =
        esp_avrc_ct_send_set_absolute_volume_cmd(BT_TL_SET_VOLUME, avrc);
    if (err != ESP_OK)
        bt_log("set absolute volume failed: %d\n", err);
}

/* Manager thread: sends the latest volume, however often it changed */
static void bt_send_pending_volume(void)
{
    bt_volume_send_queued = false;
    if (bt_abs_volume && bt_remote_volume >= 0)
        bt_send_remote_volume(bt_remote_volume);
}

/* The output level relative to a channel's volume: unity, or the headroom
 * when the headphones apply the volume */
static int bt_channel_level(int vol, int ref)
{
    if (vol == PCM_MUTE_LEVEL)
        return PCM_MUTE_LEVEL;
    return vol - ref + BT_ABS_VOLUME_HEADROOM;
}

/* May run in interrupt context, see bt_sink_set_volume() */
static void bt_apply_volume(void)
{
    if (!bt_abs_volume) {
        pcm_set_master_volume(bt_vol_l, bt_vol_r);
        return;
    }

    /* The headphones get the louder channel's volume, balance stays here */
    int ref = MAX(bt_vol_l, bt_vol_r);
    if (ref == PCM_MUTE_LEVEL) {
        pcm_set_master_volume(PCM_MUTE_LEVEL, PCM_MUTE_LEVEL);
    } else {
        pcm_set_master_volume(bt_channel_level(bt_vol_l, ref),
                              bt_channel_level(bt_vol_r, ref));
    }

    int avrc = bt_cb_to_avrc(ref);
    if (avrc != bt_remote_volume) {
        bt_remote_volume = avrc;
        if (!bt_following_remote && !bt_volume_send_queued &&
            bt_queue_ready) {
            bt_volume_send_queued = true;
            queue_post(&bt_queue, BTM_SEND_VOLUME, 0);
        }
    }
}

/* Called by audiohw_set_volume(), which also runs from the headphone
 * detection in the button tick */
void bt_sink_set_volume(int vol_l, int vol_r)
{
    bt_vol_l = vol_l;
    bt_vol_r = vol_r;
    bt_apply_volume();
}

void bluetooth_remote_volume_changed(void)
{
    bt_remote_request_queued = false;
    if (!bt_abs_volume)
        return;

    /* The target doesn't necessarily get the volume setting: sound.c
     * raises it by the tone controls' or EQ's boost, cutting the signal by
     * as much. Balance only lowers the other channel. */
    int ref = MAX(bt_vol_l, bt_vol_r);
    int offset = 0;
    if (ref != PCM_MUTE_LEVEL)
        offset = ref - bt_volume_to_cb(global_status.volume);

    int volume = bt_avrc_to_volume(bt_remote_request, offset);
    bt_log("remote volume %d -> setting %d, offset %d\n",
           bt_remote_request, volume, offset);
    if (volume == global_status.volume)
        return;

    /* Our steps are coarser, so this may round to a neighbouring value:
     * leave the headphones where they are */
    bt_following_remote = true;
    global_status.volume = volume;
    setvol();
    bt_following_remote = false;
}

static void bt_reset_abs_volume(void)
{
    bt_abs_volume_probe = false;
    bt_remote_volume = -1;
    if (bt_abs_volume) {
        bt_abs_volume = false;
        bt_apply_volume();
    }
}

static bool bt_avrc_connected = false;

/* Absolute volume needs volume change notifications: ask if they're there */
static void bt_probe_abs_volume(void)
{
    esp_err_t err = esp_avrc_ct_send_get_rn_capabilities_cmd(BT_TL_GET_CAPS);
    if (err != ESP_OK)
        bt_log("get notification capabilities failed: %d\n", err);
}

void bluetooth_volume_sync_changed(bool sync)
{
    if (!sync)
        bt_reset_abs_volume();
    else if (bt_avrc_connected && !bt_abs_volume && !bt_abs_volume_probe)
        bt_probe_abs_volume();
}

static void bt_app_avrc_ct_cb(esp_avrc_ct_cb_event_t event,
                              esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
        case ESP_AVRC_CT_CONNECTION_STATE_EVT:
            bt_log("avrc-ct connected %d\n", param->conn_stat.connected);
            bt_reset_abs_volume();
            bt_avrc_connected = param->conn_stat.connected;
            if (bt_avrc_connected && global_settings.bt_volume_sync)
                bt_probe_abs_volume();
            break;

        case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
            bool volume = esp_avrc_rn_evt_bit_mask_operation(
                ESP_AVRC_BIT_MASK_OP_TEST, &param->get_rn_caps_rsp.evt_set,
                ESP_AVRC_RN_VOLUME_CHANGE);
            bt_log("avrc-ct notification caps 0x%x\n",
                   param->get_rn_caps_rsp.evt_set.bits);
            if (!volume || !global_settings.bt_volume_sync)
                break;
            esp_err_t err = esp_avrc_ct_send_register_notification_cmd(
                BT_TL_VOLUME_NOTIFY, ESP_AVRC_RN_VOLUME_CHANGE, 0);
            if (err != ESP_OK) {
                bt_log("error registering for volume change: %d\n", err);
                break;
            }
            bt_abs_volume_probe = true;
            bt_send_remote_volume(bt_cb_to_avrc(MAX(bt_vol_l, bt_vol_r)));
            break;
        }

        case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT:
            bt_log("avrc-ct volume set to %d\n", param->set_volume_rsp.volume);
            bt_remote_volume = param->set_volume_rsp.volume;
            if (bt_abs_volume_probe) {
                bt_abs_volume_probe = false;
                bt_abs_volume = true;
                bt_apply_volume();
            }
            break;

        case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
            if (param->change_ntf.event_id != ESP_AVRC_RN_VOLUME_CHANGE)
                break;

            int avrc = param->change_ntf.event_parameter.volume;
            bt_log("avrc-ct new vol %d\n", avrc);
            if (!global_settings.bt_volume_sync)
                break;
            /* Notifications are one-shot */
            esp_avrc_ct_send_register_notification_cmd(
                BT_TL_VOLUME_NOTIFY, ESP_AVRC_RN_VOLUME_CHANGE, 0);
            if (!bt_abs_volume)
                break;

            /* Follow the headphones, see bluetooth_remote_volume_changed().
             * Turning their volume knob sends a burst of these. */
            bt_remote_request = avrc;
            /* Post again if it went unhandled for a while, e.g. lost */
            if (!bt_remote_request_queued ||
                TIME_AFTER(current_tick, bt_remote_request_tick + HZ)) {
                bt_remote_request_queued = true;
                bt_remote_request_tick = current_tick;
                button_queue_post(SYS_BLUETOOTH_VOLUME, 0);
            }
            break;
        }

        default:
            bt_log("avrc-ct %d\n", event);
            return;
    }
}

static void bt_app_avrc_tg_cb(esp_avrc_tg_cb_event_t event,
                              esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
        case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
            bt_log("avrc-tg passthrough %d %d\n", param->psth_cmd.key_code,
                   param->psth_cmd.key_state);
            if (param->psth_cmd.key_state == 1) {
                switch (param->psth_cmd.key_code) {
                    case ESP_AVRC_PT_CMD_PLAY:
                    case ESP_AVRC_PT_CMD_PAUSE:
                        button_queue_post(BUTTON_MULTIMEDIA_PLAYPAUSE, 0);
                        break;
                    case ESP_AVRC_PT_CMD_STOP:
                        button_queue_post(BUTTON_MULTIMEDIA_STOP, 0);
                        break;
                    case ESP_AVRC_PT_CMD_FORWARD:
                        button_queue_post(BUTTON_MULTIMEDIA_NEXT, 0);
                        break;
                    case ESP_AVRC_PT_CMD_BACKWARD:
                        button_queue_post(BUTTON_MULTIMEDIA_PREV, 0);
                        break;
                    case ESP_AVRC_PT_CMD_REWIND:
                        button_queue_post(BUTTON_MULTIMEDIA_REW, 0);
                        break;
                    case ESP_AVRC_PT_CMD_FAST_FORWARD:
                        button_queue_post(BUTTON_MULTIMEDIA_FFWD, 0);
                        break;
                }
            }
            break;
        default:
            bt_log("avrc-tg %d\n", event);
            return;
    }
}

/***************************************************************************
 * Bluedroid logging glue
 ***************************************************************************/

#ifndef SIMULATOR
#define FPR_WRBUF_CKSZ 32   /* write buffer chunk size */

struct for_fprintf {
    int fd;  /* where to store it */
    int rem; /* amount remaining */
    int idx; /* index of next buffer write */
    unsigned char wrbuf[FPR_WRBUF_CKSZ]; /* write buffer */
};

static int fpr_buffer_flush(struct for_fprintf *fpr)
{
    /* set idx to actual but negative unflushed count to signal error */
    ssize_t done = write(fpr->fd, fpr->wrbuf, fpr->idx);
    fpr->idx = MAX(done, 0) - fpr->idx;
    return fpr->idx;
}

static int fprfunc(void *pr, int letter)
{
    struct for_fprintf *fpr  = (struct for_fprintf *)pr;

    if (fpr->idx >= FPR_WRBUF_CKSZ && fpr_buffer_flush(fpr)) {
        return -1; /* don't count this one */
    }

    fpr->wrbuf[fpr->idx++] = letter;
    return --fpr->rem;
}
#endif /* !SIMULATOR */

/* TODO(skyevg): clean these log.h impls up and move to bluedroid */

void esp_log_write(esp_log_level_t level, const char *tag,
                   const char *format, ...)
{
    (void)level;
#ifdef SIMULATOR
    va_list ap;
    printf("[%s] ", tag);
    va_start(ap, format);
    vprintf(format, ap);
    va_end(ap);
#else
    if (gen_log_fd != -1) {
        mutex_lock(&gen_log_mutex);
        fdprintf(gen_log_fd, "[%s] ", tag);

        struct for_fprintf fpr;
        va_list ap;

        fpr.fd  = gen_log_fd;
        fpr.rem = INT_MAX;
        fpr.idx = 0;

        va_start(ap, format);
        vuprintf(fprfunc, &fpr, format, ap);
        va_end(ap);

        /* flush any tail bytes */
        if (fpr.idx >= 0) {
            fpr_buffer_flush(&fpr);
        }
        bt_log_sync();
        mutex_unlock(&gen_log_mutex);
    }
#endif
}

uint32_t esp_log_timestamp(void)
{
    return current_tick;
}

/***************************************************************************
 * Debug screen
 ***************************************************************************/

extern void* btbuf;

static int bt_heap_info_action_callback(int action, struct gui_synclist *lists)
{
    (void)lists;
    switch (action) {
        case ACTION_NONE:
            return ACTION_REDRAW;
        default:
            return action;
    }
}

static const char *bt_heap_info_getname(int selected_item, void *data,
                                        char *buffer, size_t buffer_len)
{
    (void)data;
    switch (selected_item) {
        case 0:
#ifndef SIMULATOR
            snprintf(buffer, buffer_len, "heap usage: %d/%d",
                     (int)get_used_size(btbuf), (int)get_max_size(btbuf));
            return buffer;
#else
            return "this is sim build you dummy";
#endif
        case 1: {
            const uint8_t *addr = esp_bt_dev_get_address();
            if (addr) {
                snprintf(buffer, buffer_len, "MAC addr: " ESP_BD_ADDR_STR,
                         ESP_BD_ADDR_HEX(addr));
                return buffer;
            } else {
                return "MAC addr: unknown";
            }
        }
        case 2:
            return bt_hci_sleep_clock() ? "32k sleep clock: on"
                                        : "32k sleep clock: off";
        default:
            return "Unknown item!!";
    }
}

bool bt_heap_info(void)
{
    struct simplelist_info info;

    simplelist_info_init(&info, "Bluetooth debug info:", 3, NULL);
    info.scroll_all = false;
    info.action_callback = bt_heap_info_action_callback;
    info.get_name = bt_heap_info_getname;
    return simplelist_show_list(&info);
}

/***************************************************************************
 * PCM sink
 ***************************************************************************/

/* Only 44.1 kHz for now: reconfiguring a running stream to 48 kHz via
 * esp_a2d_source_set_pref_mcc() gets ESP_BT_STATUS_UNSUPPORTED, leaving the
 * stream at 44.1 kHz while we feed 48 kHz audio. Rockbox resamples instead. */
static const unsigned long bt_freq_sampr[] =
{
    SAMPR_44,
};

static const uint8_t bt_freq_sampr_cie[] =
{
    ESP_A2D_SBC_CIE_SF_44K,
};

/* Index into bt_freq_sampr of the rate the stream is configured for; A2DP
 * connections start out at 44.1 kHz */
static uint16_t bt_cur_freq = 0;

static const void *pcm_data_start = NULL;
static size_t  pcm_data_size = 0;
static int     audio_locked = 0;

static void bt_sink_init(void)
{
}

static void bt_sink_postinit(void)
{
}

static void bt_sink_lock(void)
{
    audio_locked++;
}

static void bt_sink_unlock(void)
{
    audio_locked--;
}

/* Applies a new codec configuration: suspends the stream if needed, sets the
 * preference and resumes once it's done (see the A2DP callback) */
static void bt_reconfigure_codec(void)
{
    bt_fill_pref_mcc(&new_pref_mcc, bt_freq_sampr_cie[bt_cur_freq]);
    bt_reconfigure_deadline = current_tick + BT_RECONFIGURE_TIMEOUT;

    /* One already running picks up the new preference, unless it already
     * sent its own; then this one is lost, but it was a corner case */
    if (sampr_switch_ongoing)
        return;
    sampr_switch_ongoing = true;

    if (!bt_sink_suspend()) {
        bt_set_new_pref_mcc();
    }
}

void bluetooth_quality_changed(bool high_quality)
{
    (void)high_quality;
    /* Also runs when a .cfg file is loaded */
    if (bt_power == BT_POWER_ON && bt_conn == BT_CONN_CONNECTED &&
        bt_want_dual() != bt_pref_dual) {
        bt_log("reconfiguring for quality setting %d\n", high_quality);
        bt_reconfigure_codec();
    }
}

static void bt_sink_set_freq(uint16_t freq)
{
    if (freq == bt_cur_freq)
        return;
    bt_cur_freq = freq;

    bt_log("suspending playback: sampr switch start, to %d\n", freq);
    bt_reconfigure_codec();
}

static void bt_sink_play(const void *addr, size_t size)
{
    pcm_data_start = addr;
    pcm_data_size = size;
    bt_log("starting playback: starting to play\n");
    bt_sink_resume();
    pcm_play_dma_status_callback(PCM_DMAST_STARTED);
}

static void bt_sink_stop(void)
{
    pcm_data_start = NULL;
    pcm_data_size = 0;
    bt_log("suspending playback: stopped playing\n");
    bt_sink_suspend();
}

static bool get_new_buf_maybe(void)
{
    bool ret = pcm_play_dma_complete_callback(PCM_DMAST_OK, &pcm_data_start,
                                              &pcm_data_size);
    if (ret) {
        pcm_play_dma_status_callback(PCM_DMAST_STARTED);
    }
    return ret;
}

static int32_t bt_app_a2d_data_cb(uint8_t *buf, int32_t len)
{
    if (len <= 0 || buf == NULL) {
        return 0;
    }
    size_t written = 0;
    if (pcm_data_start != NULL && audio_locked == 0) {
        while (len > 0 && (pcm_data_size > 0 || get_new_buf_maybe())) {
#if (PCM_NATIVE_BITDEPTH > 16)
            size_t sent = MIN(pcm_data_size / 2, (size_t)len);
            int32_t *in = (int32_t *) pcm_data_start;
            int16_t *out = (int16_t *) buf;
            for (size_t i = 0; i < (sent / 2); i++) {
                out[i] = in[i] >> (PCM_NATIVE_BITDEPTH - 16);
            }
            pcm_data_start += sent * 2;
            pcm_data_size -= sent * 2;
            buf += sent;
            len -= sent;
            written += sent;
#else
            size_t sent = MIN(pcm_data_size, (size_t)len);
            memcpy(buf, pcm_data_start, sent);
            pcm_data_start += sent;
            pcm_data_size -= sent;
            buf += sent;
            len -= sent;
            written += sent;
#endif
        }
    } else {
        /* Transmit digital silence */
        written = len;
        memset(buf, 0, len);
    }
    return written;
}

struct pcm_sink bt_pcm_sink = {
    .caps = {
        .samprs       = bt_freq_sampr,
        .num_samprs   = ARRAYLEN(bt_freq_sampr),
        .default_freq = 0,
        /* Scaled in software, at unity when the headphones apply the
         * volume (see bt_apply_volume); this also makes the sample format
         * match bt_app_a2d_data_cb's expectations */
        .volume_type  = PCM_SINK_SWVOL,
    },
    .ops = {
        .init     = bt_sink_init,
        .postinit = bt_sink_postinit,
        .set_freq = bt_sink_set_freq,
        .lock     = bt_sink_lock,
        .unlock   = bt_sink_unlock,
        .play     = bt_sink_play,
        .stop     = bt_sink_stop,
    },
};
