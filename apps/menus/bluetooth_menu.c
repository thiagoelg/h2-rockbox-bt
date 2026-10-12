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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "action.h"
#include "lang.h"
#include "list.h"
#include "menu.h"
#include "settings.h"
#include "yesno.h"
#include "exported_menus.h"
#include "bluetooth/bluetooth.h"

/* Snapshot of the device list, refreshed on every redraw */
static struct bt_device devices[BT_MAX_DEVICES];
static int num_devices;
/* Search for devices once Bluetooth has finished starting up */
static bool scan_pending;

static bool is_active(const struct bt_device *dev)
{
    uint8_t addr[BT_ADDR_LEN];
    return bluetooth_get_active(addr) &&
           memcmp(addr, dev->addr, BT_ADDR_LEN) == 0;
}

static void format_device_name(const struct bt_device *dev,
                               char *buf, size_t len)
{
    if (dev->name[0] != '\0') {
        snprintf(buf, len, "%s", dev->name);
    } else {
        const uint8_t *a = dev->addr;
        snprintf(buf, len, "%02x:%02x:%02x:%02x:%02x:%02x",
                 a[0], a[1], a[2], a[3], a[4], a[5]);
    }
}

static const char *status_text(char *buf, size_t len)
{
    enum bt_state state = bluetooth_get_state();
    switch (state) {
        case BT_STATE_OFF:
            return str(bluetooth_failed() ? LANG_BLUETOOTH_START_FAILED
                                          : LANG_BLUETOOTH_OFF_HINT);
        case BT_STATE_STARTING:
            return str(LANG_BLUETOOTH_STARTING);
        case BT_STATE_STOPPING:
            return str(LANG_BLUETOOTH_STOPPING);
        case BT_STATE_CONNECTING:
        case BT_STATE_CONNECTED: {
            int lang = state == BT_STATE_CONNECTED ?
                LANG_BLUETOOTH_CONNECTED : LANG_BLUETOOTH_CONNECTING;
            char name[BT_NAME_LEN];
            struct bt_device dev;
            memset(&dev, 0, sizeof(dev));
            if (!bluetooth_get_active(dev.addr))
                return str(lang); /* the state just changed */
            for (int i = 0; i < num_devices; i++) {
                if (memcmp(devices[i].addr, dev.addr, BT_ADDR_LEN) == 0)
                    dev = devices[i];
            }
            format_device_name(&dev, name, sizeof(name));
            snprintf(buf, len, "%s: %s", str(lang), name);
            return buf;
        }
        default:
            if (bluetooth_is_scanning())
                return str(LANG_BLUETOOTH_SEARCHING);
            if (bluetooth_connect_failed())
                return str(LANG_BLUETOOTH_CONNECT_FAILED);
            return str(LANG_BLUETOOTH_NOT_CONNECTED);
    }
}

/* Item 0 is the status line, the devices follow */
static const char *bt_get_name(int item, void *data, char *buf, size_t len)
{
    (void)data;
    if (item == 0)
        return status_text(buf, len);
    if (item > num_devices)
        return "";

    const struct bt_device *dev = &devices[item - 1];
    format_device_name(dev, buf, len);

    int lang = -1;
    if (is_active(dev))
        lang = bluetooth_get_state() == BT_STATE_CONNECTED ?
            LANG_BLUETOOTH_CONNECTED : LANG_BLUETOOTH_CONNECTING;
    else if (dev->paired)
        lang = LANG_BLUETOOTH_PAIRED;

    if (lang >= 0) {
        size_t n = strlen(buf);
        snprintf(buf + n, len - n, " (%s)", str(lang));
    }
    return buf;
}

static void enable_bluetooth(void)
{
    if (!global_settings.bt_enabled) {
        global_settings.bt_enabled = true;
        settings_save();
    }
    bluetooth_set_enabled(true);
    scan_pending = true;
}

static int bt_action_callback(int action, struct gui_synclist *lists)
{
    int item = gui_synclist_get_sel_pos(lists);

    switch (action) {
        case ACTION_REDRAW:
            if (scan_pending && bluetooth_get_state() == BT_STATE_IDLE) {
                scan_pending = false;
                bluetooth_scan();
            }
            num_devices = bluetooth_get_devices(devices, BT_MAX_DEVICES);
            gui_synclist_set_nb_items(lists, num_devices + 1);
            /* The list may have shrunk, e.g. after forgetting a device */
            if (item > num_devices)
                gui_synclist_select_item(lists, num_devices);
            return ACTION_REDRAW;

        case ACTION_STD_OK:
            if (item == 0) {
                /* Status line: power up, or search again */
                if (bluetooth_get_state() == BT_STATE_OFF)
                    enable_bluetooth();
                else
                    bluetooth_scan();
            } else if (item <= num_devices) {
                const struct bt_device *dev = &devices[item - 1];
                if (is_active(dev))
                    bluetooth_disconnect();
                else
                    bluetooth_connect(dev->addr);
            }
            /* Don't let the list exit */
            return ACTION_REDRAW;

        case ACTION_STD_CONTEXT:
            if (item > 0 && item <= num_devices && devices[item - 1].paired &&
                yesno_pop(str(LANG_BLUETOOTH_FORGET)))
                bluetooth_forget(devices[item - 1].addr);
            return ACTION_REDRAW;

        default:
            return action;
    }
}

static int bt_devices_screen(void)
{
    struct simplelist_info info;

    switch (bluetooth_get_state()) {
        case BT_STATE_OFF:
            /* Selecting the status line turns it on */
            scan_pending = false;
            break;
        case BT_STATE_IDLE:
            scan_pending = true;
            break;
        default:
            /* Searching while connected would interrupt the audio */
            scan_pending = false;
            break;
    }

    num_devices = bluetooth_get_devices(devices, BT_MAX_DEVICES);
    simplelist_info_init(&info, str(LANG_BLUETOOTH_DEVICES),
                         num_devices + 1, NULL);
    info.scroll_all = false;
    info.action_callback = bt_action_callback;
    info.get_name = bt_get_name;
    simplelist_show_list(&info);
    return 0;
}

MENUITEM_SETTING(bt_enabled_item, &global_settings.bt_enabled, NULL);
MENUITEM_FUNCTION(bt_devices_item, 0, ID2P(LANG_BLUETOOTH_DEVICES),
                  bt_devices_screen, NULL, Icon_NOICON);
MENUITEM_SETTING(bt_high_quality_item, &global_settings.bt_high_quality, NULL);
MENUITEM_SETTING(bt_volume_sync_item, &global_settings.bt_volume_sync, NULL);

MAKE_MENU(bluetooth_menu, ID2P(LANG_BLUETOOTH), NULL, Icon_Audio,
          &bt_enabled_item, &bt_devices_item, &bt_high_quality_item,
          &bt_volume_sync_item);
