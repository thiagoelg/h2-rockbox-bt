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

/* HCI driver for the simulator: talks H4 to a controller on /dev/ttyACM0,
 * e.g. a USB dongle running a UART HCI firmware */

#include <stdint.h>
#include <errno.h>
#include <file.h>
#include <poll.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include "bt_hci.h"
#include "esp_bluedroid_hci.h"
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

#define HCI_RECV_BUF_SIZE 1026
#define HCI_STACK_SIZE (DEFAULT_STACK_SIZE + HCI_RECV_BUF_SIZE)

static int controller_fd = -1;
static volatile bool hci_closing = false;

static int hci_log_fd = -1;
static struct mutex hci_log_mutex;

/* LINKTYPE_BLUETOOTH_HCI_H4_WITH_PHDR record header */
struct pcap_record_header {
    uint32_t seconds;
    uint32_t microseconds;
    uint32_t captured_packet_length;
    uint32_t original_packet_length;
    uint32_t direction; /* big-endian, 0 = sent, 1 = received */
};

static void hci_log_packet(const uint8_t *data, size_t len, bool received)
{
    struct pcap_record_header header = {
        .seconds = current_tick / HZ,
        .microseconds = (current_tick % HZ) * (1000000 / HZ),
        .captured_packet_length = len + 4,
        .original_packet_length = len + 4,
        .direction = received ? 0x01000000 : 0,
    };

    mutex_lock(&hci_log_mutex);
    if (hci_log_fd >= 0) {
        write(hci_log_fd, &header, sizeof(header));
        write(hci_log_fd, data, len);
    }
    mutex_unlock(&hci_log_mutex);
}

static void hci_send_uart(uint8_t *data, uint16_t len)
{
    hci_log_packet(data, len, false);
#ifdef HAVE_SDL_THREADS
    void *me = sim_thread_unlock();
#endif
    write(controller_fd, data, len);
#ifdef HAVE_SDL_THREADS
    sim_thread_lock(me);
#endif
}

/* Waits for data without holding up the other threads, which share the
 * host thread unless they're SDL threads. Returns the amount read, or
 * 0 on error or once bt_hci_disable() was called. */
static size_t uart_rx(uint8_t *data, size_t len)
{
    while (!hci_closing) {
        struct pollfd pfd = { .fd = controller_fd, .events = POLLIN };
#ifdef HAVE_SDL_THREADS
        void *me = sim_thread_unlock();
        int ready = poll(&pfd, 1, 100);
        sim_thread_lock(me);
#else
        int ready = poll(&pfd, 1, 0);
#endif
        if (ready > 0) {
            ssize_t n = read(controller_fd, data, len);
            return n > 0 ? (size_t)n : 0;
        }
        if (ready < 0 && errno != EINTR)
            return 0;
#ifndef HAVE_SDL_THREADS
        sleep(1);
#endif
    }
    return 0;
}

/* Reads exactly len bytes */
static bool hci_read(uint8_t *data, size_t len)
{
    size_t recvd = 0;
    while (recvd < len) {
        size_t n = uart_rx(data + recvd, len - recvd);
        if (n == 0)
            return false;
        recvd += n;
    }
    return true;
}

/* Returns the length of the H4 packet read, or 0 on failure. Same framing
 * as the erosqnative driver. */
static size_t hci_recv_uart(uint8_t *data, size_t len)
{
    /* Header length by packet type: command, ACL, SCO, event, ISO */
    static const uint8_t header_len[] = { 0, 4, 5, 4, 3, 5 };

    if (len < 5) {
        panicf("hci recv buffer too small, got %lu but wanted at least 5",
               (unsigned long)len);
    }
    if (!hci_read(data, 1))
        return 0;

    uint8_t type = data[0];
    if (type == 0 || type >= ARRAYLEN(header_len)) {
        panicf("hci got unknown frame type %d", type);
    }
    if (!hci_read(data + 1, header_len[type] - 1))
        return 0;

    size_t expected;
    switch (type) {
        case 0x02:
            expected = (((size_t) data[3]) | (((size_t) data[4]) << 8)) + 5;
            break;
        case 0x04:
            expected = ((size_t) data[2]) + 3;
            break;
        case 0x05:
            expected = (((size_t) data[3]) |
                        ((((size_t) data[4]) & 0x3F) << 8)) + 5;
            break;
        default:
            expected = ((size_t) data[3]) + 4;
            break;
    }
    if (len < expected) {
        panicf("hci recv buffer too small, got %lu but wanted at least %lu",
               (unsigned long)len, (unsigned long)expected);
    }
    if (!hci_read(data + header_len[type], expected - header_len[type]))
        return 0;

    hci_log_packet(data, expected, true);
    return expected;
}

static bool hci_check_send_available_uart(void)
{
    return true;
}

static const esp_bluedroid_hci_driver_callbacks_t *hci_callbacks;
static struct semaphore hci_callbacks_sem;

static esp_err_t hci_register_host_callback_uart(
    const esp_bluedroid_hci_driver_callbacks_t *callback)
{
    if (hci_callbacks != NULL) {
        return ESP_FAIL;
    }
    hci_callbacks = callback;
    semaphore_release(&hci_callbacks_sem);
    return ESP_OK;
}

static uint32_t hci_stack[HCI_STACK_SIZE / sizeof(uint32_t)];
static unsigned int hci_thread_id = 0;

static void hci_thread(void)
{
    uint8_t buf[HCI_RECV_BUF_SIZE];

    semaphore_wait(&hci_callbacks_sem, TIMEOUT_BLOCK);

    while (!hci_closing) {
        size_t recvd = hci_recv_uart(buf, HCI_RECV_BUF_SIZE);
        if (recvd > 0 && !hci_closing)
            hci_callbacks->notify_host_recv(buf, recvd);
    }

    hci_callbacks = NULL;
}

bool bt_hci_enable(void)
{
    /* pcap file header: little-endian v2.4, snaplen 65535, link type 201
     * (LINKTYPE_BLUETOOTH_HCI_H4_WITH_PHDR) */
    static const uint8_t pcap_file_header[24] = {
        0xd4, 0xc3, 0xb2, 0xa1, 0x02, 0x00, 0x04, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xff, 0xff, 0x00, 0x00, 0xc9, 0x00, 0x00, 0x00,
    };

    if (controller_fd >= 0)
        return false;

    controller_fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    if (controller_fd < 0)
        return false;

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

    mutex_init(&hci_log_mutex);
    hci_log_fd = open("hci_log.pcap", O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (hci_log_fd >= 0)
        write(hci_log_fd, pcap_file_header, sizeof(pcap_file_header));

    hci_closing = false;
    hci_callbacks = NULL;
    semaphore_init(&hci_callbacks_sem, 1, 0);

    hci_thread_id = create_thread(hci_thread, hci_stack, HCI_STACK_SIZE, 0,
                                  "bt_hci" IF_PRIO(, PRIORITY_BLUETOOTH));
    if (hci_thread_id == 0) {
        bt_hci_disable();
        return false;
    }

    esp_bluedroid_hci_driver_operations_t operations = {
        .send = hci_send_uart,
        .check_send_available = hci_check_send_available_uart,
        .register_host_callback = hci_register_host_callback_uart,
    };
    esp_bluedroid_attach_hci_driver(&operations);
    return true;
}

void bt_hci_disable(void)
{
    static const uint8_t hci_reset_cmd[] = {0x01, 0x03, 0x0c, 0x00};

    if (controller_fd < 0)
        return;

    /* The receive thread notices within a poll */
    hci_closing = true;
    if (hci_thread_id != 0) {
        semaphore_release(&hci_callbacks_sem);
        thread_wait(hci_thread_id);
        hci_thread_id = 0;
    }

    /* Leave the controller idle for the next run */
    hci_send_uart((uint8_t *)hci_reset_cmd, sizeof(hci_reset_cmd));
    close(controller_fd);
    controller_fd = -1;

    mutex_lock(&hci_log_mutex);
    if (hci_log_fd >= 0)
        close(hci_log_fd);
    hci_log_fd = -1;
    mutex_unlock(&hci_log_mutex);
}

void *smp_get_local_oob_data = NULL;
void *smp_clear_local_oob_data = NULL;
void *smp_encrypt_data = NULL;
void *smp_calculate_long_term_key_from_link_key = NULL;
void *smp_save_secure_connections_long_term_key = NULL;

bool bt_hci_sleep_clock(void)
{
    return false;
}

void bt_target_local_output(bool enable)
{
    (void)enable;
}
