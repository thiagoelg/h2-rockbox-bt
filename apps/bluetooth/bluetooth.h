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

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdbool.h>
#include <stdint.h>

#define BT_ADDR_LEN     6
#define BT_NAME_LEN     64
#define BT_MAX_DEVICES  32

enum bt_state {
    BT_STATE_OFF = 0,
    BT_STATE_STARTING,      /* loading controller firmware, starting stack */
    BT_STATE_IDLE,          /* on, no audio device connected */
    BT_STATE_CONNECTING,
    BT_STATE_CONNECTED,
    BT_STATE_STOPPING,
};

struct bt_device {
    uint8_t addr[BT_ADDR_LEN];
    char    name[BT_NAME_LEN];  /* empty if unknown */
    bool    paired;
};

/* Starts the Bluetooth manager thread and, if enabled in the settings,
 * powers up the controller and reconnects to the last device. */
void bluetooth_init(void);

/* Powering up or down happens in the background; poll bluetooth_get_state()
 * or wait for a BUTTON_REDRAW to see progress. */
void bluetooth_set_enabled(bool enable);
/* Callback of the high quality setting: renegotiates a connected stream */
void bluetooth_quality_changed(bool high_quality);
/* Callback of the volume sync setting */
void bluetooth_volume_sync_changed(bool sync);
/* For SYS_BLUETOOTH_VOLUME: follows a volume change on the headphones */
void bluetooth_remote_volume_changed(void);
enum bt_state bluetooth_get_state(void);
bool bluetooth_failed(void);    /* last attempt to power up failed */
bool bluetooth_connect_failed(void); /* last connection attempt failed */

void bluetooth_scan(void);
bool bluetooth_is_scanning(void);
void bluetooth_connect(const uint8_t *addr);
void bluetooth_disconnect(void);
void bluetooth_forget(const uint8_t *addr);

/* Copies the known devices (paired ones first) into devs, returns how many */
int bluetooth_get_devices(struct bt_device *devs, int max);
/* Address of the device being connected or connected to, if any */
bool bluetooth_get_active(uint8_t *addr);

bool bt_heap_info(void);

#endif /* BLUETOOTH_H */
