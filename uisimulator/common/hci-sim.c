/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
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

#include <stdint.h>
#include <file.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include "bt_hci.h"
#include "esp_bluedroid_hci.h"
#include "esp_bt_main.h"
#include "kernel.h"
#include "thread-sdl.h"
#include "panic.h"
#include "semaphore.h"

#undef open
#undef close
#undef read
#undef write
#undef fsync

#include "mutex.h"
#include "tick.h"

static int hci_log_fd = -1;
static struct mutex hci_log_mutex;

typedef struct {
    uint32_t seconds;
    uint32_t microseconds;
    uint32_t captured_packet_length;
    uint32_t original_packet_length;
    uint32_t direction;
} pcap_header_t;

static int controller_fd = -1;

void hci_send_uart(uint8_t *data, uint16_t len) {
    if (hci_log_fd != -1) {
        mutex_lock(&hci_log_mutex);
        pcap_header_t header = {
            .seconds = current_tick,
            .microseconds = 0,
            .captured_packet_length = len + 4,
            .original_packet_length = len + 4,
            .direction = 0x01000000,
        };
        write(hci_log_fd, &header, 20);
        write(hci_log_fd, data, len);
        fsync(hci_log_fd);
        mutex_unlock(&hci_log_mutex);
    }
    void *mythread = sim_thread_unlock();
    write(controller_fd, data, len);
    sim_thread_lock(mythread);
}

size_t uart_rx(uint8_t *data, size_t len) {
    void *mythread = sim_thread_unlock();
    size_t cnt = read(controller_fd, data, len);
    sim_thread_lock(mythread);
    return cnt;
}

size_t hci_recv_uart(uint8_t *data, size_t len) {
    if (len == 0) {
        panicf("hci recv buffer too small, got %u but wanted at least 1", len);
    }
    size_t recvd = uart_rx(data, 1);
    if (recvd > 0) {
        size_t expected;
        if (data[0] == 0x01) {
            if (len < 4) {
                panicf("hci recv buffer too small, got %u but wanted at least 4", len);
            }
            while (recvd < 4) {
                recvd += uart_rx(recvd + data, 4 - recvd);
            }
            expected = (((size_t) data[3]) & 0xFF) + 4;
        } else if (data[0] == 0x02) {
            if (len < 5) {
                panicf("hci recv buffer too small, got %u but wanted at least 5", len);
            }
            while (recvd < 5) {
                recvd += uart_rx(recvd + data, 5 - recvd);
            }
            expected = ((((size_t) data[3]) & 0xFF) | ((((size_t) data[4]) << 8) & 0xFF)) + 5;
        } else if (data[0] == 0x03) {
            if (len < 4) {
                panicf("hci recv buffer too small, got %u but wanted at least 4", len);
            }
            while (recvd < 4) {
                recvd += uart_rx(recvd + data, 4 - recvd);
            }
            expected = (((size_t) data[3]) & 0xFF) + 4;
        } else if (data[0] == 0x04) {
            if (len < 3) {
                panicf("hci recv buffer too small, got %u but wanted at least 3", len);
            }
            while (recvd < 3) {
                recvd += uart_rx(recvd + data, 3 - recvd);
            }
            expected = (((size_t) data[2]) & 0xFF) + 3;
        } else if (data[0] == 0x05) {
            if (len < 5) {
                panicf("hci recv buffer too small, got %u but wanted at least 5", len);
            }
            while (recvd < 5) {
                recvd += uart_rx(recvd + data, 5 - recvd);
            }
            expected = ((((size_t) data[3]) & 0xFF) | ((((size_t) data[4]) << 8) & 0xFF)) + 5;
        } else {
            // Unknown frame type.
            // Panic.
            panicf("hci got unknown frame type %d", data[0]);
        }
        if (len < expected) {
            panicf("hci recv buffer too small, got %u but wanted at least %u", len, expected);
        }
        while (recvd < expected) {
            recvd += uart_rx(recvd + data, expected - recvd);
        }
        if (hci_log_fd != -1) {
            mutex_lock(&hci_log_mutex);
            pcap_header_t header = {
                .seconds = current_tick,
                .microseconds = 0,
                .captured_packet_length = recvd + 4,
                .original_packet_length = recvd + 4,
                .direction = 0x00000000,
            };
            write(hci_log_fd, &header, 20);
            write(hci_log_fd, data, recvd);
            fsync(hci_log_fd);
            mutex_unlock(&hci_log_mutex);
        }
    }
    return recvd;
}

bool hci_check_send_available_uart(void) {
    return true;
}

const esp_bluedroid_hci_driver_callbacks_t *hci_callbacks;
struct semaphore hci_callbacks_sem;

esp_err_t hci_register_host_callback_uart(const esp_bluedroid_hci_driver_callbacks_t *callback) {
    if (hci_callbacks != NULL) {
        return ESP_FAIL;
    }
    hci_callbacks = callback;
    semaphore_release(&hci_callbacks_sem);
    return ESP_OK;
}

#define HCI_RECV_BUF_SIZE 1026

uint32_t hci_stack[(DEFAULT_STACK_SIZE + HCI_RECV_BUF_SIZE) / sizeof(uint32_t)];

void bluetooth_hci_task(void)
{
    uint8_t buf[HCI_RECV_BUF_SIZE];

    semaphore_wait(&hci_callbacks_sem, TIMEOUT_BLOCK);

    while (true) {
        size_t recvd = hci_recv_uart(buf, HCI_RECV_BUF_SIZE);
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
            close(controller_fd);
            controller_fd = -1;
            hci_callbacks = NULL;
            thread_exit();
            return;
        }
        hci_callbacks->notify_host_recv(buf, recvd);
    }
}

void bt_hci_enable(void)
{
    controller_fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    struct termios termios;
    tcflush(controller_fd, TCIOFLUSH);
	tcgetattr(controller_fd, &termios);
	termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON);
	termios.c_oflag &= ~OPOST;
	termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	termios.c_cflag &= ~(CSIZE | PARENB);
	termios.c_cflag |= CS8;
	termios.c_cflag |= CRTSCTS;
	tcsetattr(controller_fd, TCSANOW, &termios);

    hci_log_fd = open("hci_log.pcap", O_WRONLY | O_CREAT | O_TRUNC, 0666);
    write(hci_log_fd, (uint8_t[]){0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xc9, 0x00, 0x00, 0x00}, 24);
    mutex_init(&hci_log_mutex);

    hci_callbacks = NULL;
    semaphore_init(&hci_callbacks_sem, 1, 0);

    esp_bluedroid_hci_driver_operations_t operations = {
        .send = hci_send_uart,
        .check_send_available = hci_check_send_available_uart,
        .register_host_callback = hci_register_host_callback_uart,
    };
    esp_bluedroid_attach_hci_driver(&operations);
    create_thread(bluetooth_hci_task, hci_stack, (DEFAULT_STACK_SIZE + HCI_RECV_BUF_SIZE), 0, "bt_hci" IF_PRIO(, PRIORITY_BLUETOOTH));
}

void bt_hci_disable(void)
{
    int old_fd = hci_log_fd;
    hci_log_fd = -1;
    if (old_fd != -1)
        close(old_fd);
    hci_send_uart((uint8_t[]){0x01, 0x03, 0x0c, 0x00}, 4);
}

void *smp_get_local_oob_data = NULL;
void *smp_clear_local_oob_data = NULL;
void *smp_encrypt_data = NULL;
void *smp_calculate_long_term_key_from_link_key = NULL;
void *smp_save_secure_connections_long_term_key = NULL;