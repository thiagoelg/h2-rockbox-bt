/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2026 Skye Green
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

#ifndef BT_HCI_H
#define BT_HCI_H

#include <stdbool.h>

/* Enables and registers the HCI driver for the target.
 * Can spawn up to one thread for this.
 * Returns false, leaving the controller off, if it can't be brought up. */
bool bt_hci_enable(void);

/* Whether the controller is fed an external sleep clock (debug info) */
bool bt_hci_sleep_clock(void);

/* Called as playback moves to (false) or away from (true) Bluetooth, so the
 * target can power down its own output stage meanwhile */
void bt_target_local_output(bool enable);

/* Disables the HCI driver for the target.
 * If bt_hci_enable spawned any threads, they must exit now.
 * Guaranteed to be called after Bluedroid deinit. */
void bt_hci_disable(void);

#endif
