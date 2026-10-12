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

#include <stdint.h>
#include "bt_hci.h"
#include "devicedata.h"
#include "esp_bluedroid_hci.h"
#include "gpio-ingenic.h"
#include "kernel.h"
#include "panic.h"
#include "thread.h"
#include "uart-x1000.h"
#include "semaphore.h"
#include "bcm_firmware.h"
#include "x1000/cpm.h"

#if defined(BT_DEBUG) && !defined(BT_DEBUG_PCAP)
#define BT_DEBUG_PCAP
#endif

/* Big enough for any event, and for ACL packets of up to 1021 bytes */
#define HCI_RECV_BUF_SIZE 1026

#ifdef BT_DEBUG_PCAP
#include "file.h"
#include "mutex.h"
#include "tick.h"

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

/* ACL data is truncated (enough for L2CAP, AVDTP and RTP headers), so that
 * capturing doesn't starve the audio stream */
#define HCI_LOG_ACL_SNAPLEN 128

static void hci_log_open(void)
{
    /* pcap file header: little-endian v2.4, snaplen 65535, link type 201
     * (LINKTYPE_BLUETOOTH_HCI_H4_WITH_PHDR) */
    static const uint8_t pcap_file_header[24] = {
        0xd4, 0xc3, 0xb2, 0xa1, 0x02, 0x00, 0x04, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xff, 0xff, 0x00, 0x00, 0xc9, 0x00, 0x00, 0x00,
    };

    mutex_init(&hci_log_mutex);

    /* Keep the previous capture, e.g. from before a crash */
    remove("/hci_log.old.pcap");
    rename("/hci_log.pcap", "/hci_log.old.pcap");
    hci_log_fd = open("/hci_log.pcap", O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (hci_log_fd >= 0)
        write(hci_log_fd, pcap_file_header, sizeof(pcap_file_header));
}

static void hci_log_close(void)
{
    mutex_lock(&hci_log_mutex);
    if (hci_log_fd >= 0)
        close(hci_log_fd);
    hci_log_fd = -1;
    mutex_unlock(&hci_log_mutex);
}

static void hci_log_packet(const uint8_t *data, size_t len, bool received)
{
    static long next_sync;

    bool is_acl = len > 0 && data[0] == 0x02;
    size_t caplen = is_acl ? MIN(len, HCI_LOG_ACL_SNAPLEN) : len;
    long tick = current_tick;
    struct pcap_record_header header = {
        .seconds = tick / HZ,
        .microseconds = (tick % HZ) * (1000000 / HZ),
        .captured_packet_length = caplen + 4,
        .original_packet_length = len + 4,
        .direction = received ? 0x01000000 : 0,
    };

    mutex_lock(&hci_log_mutex);
    if (hci_log_fd >= 0) {
        write(hci_log_fd, &header, sizeof(header));
        write(hci_log_fd, data, caplen);
        /* Syncing every event (each Number_Of_Completed_Packets while
         * streaming) starved this thread, which hands out the send
         * credits */
        if (TIME_AFTER(current_tick, next_sync)) {
            fsync(hci_log_fd);
            next_sync = current_tick + HZ / 10;
        }
    }
    mutex_unlock(&hci_log_mutex);
}

#define HCI_STACK_SIZE (DEFAULT_STACK_SIZE + HCI_RECV_BUF_SIZE + 1024)
#else
#define hci_log_open() do {} while (0)
#define hci_log_close() do {} while (0)
#define hci_log_packet(data, len, received) do {} while (0)
#define HCI_STACK_SIZE (DEFAULT_STACK_SIZE + HCI_RECV_BUF_SIZE)
#endif

static void hci_send_uart(uint8_t *data, uint16_t len)
{
    hci_log_packet(data, len, false);
    uart_tx(PORT_UART0, data, len);
}

/* Reads during bring-up time out, so a controller that doesn't answer can't
 * hang Bluetooth startup. The receive thread waits indefinitely. */
#define HCI_INIT_TIMEOUT (HZ / 2)
static int hci_rx_timeout = TIMEOUT_BLOCK;

/* Set by bt_hci_disable() to stop the receive thread */
static volatile bool hci_closing = false;

/* Reads exactly len bytes. Fails on timeout, or when bt_hci_disable() wakes
 * up a blocked read. */
static bool hci_read(uint8_t *data, size_t len)
{
    size_t recvd = 0;
    while (recvd < len) {
        if (hci_closing)
            return false;
        size_t n = uart_rx_timeout(PORT_UART0, data + recvd, len - recvd,
                                   hci_rx_timeout);
        if (n == 0)
            return false;
        recvd += n;
    }
    return true;
}

/* Returns the length of the H4 packet read, or 0 on failure */
static size_t hci_recv_uart(uint8_t *data, size_t len)
{
    /* Header length by packet type: command, ACL, SCO, event, ISO */
    static const uint8_t header_len[] = { 0, 4, 5, 4, 3, 5 };
    /* While bringing the controller up, garbage can be a speed mismatch,
     * which the caller recovers from */
    bool recoverable = hci_rx_timeout != TIMEOUT_BLOCK;

    if (len < 5) {
        panicf("hci recv buffer too small, got %u but wanted at least 5", len);
    }
    if (!hci_read(data, 1))
        return 0;

    uint8_t type = data[0];
    if (type == 0 || type >= ARRAYLEN(header_len)) {
        if (recoverable)
            return 0;
        panicf("hci got unknown frame type %d", type);
    }
    if (!hci_read(data + 1, header_len[type] - 1))
        return 0;

    size_t expected;
    switch (type) {
        case 0x02:
            /* ACL data length is a full 16-bit little-endian field */
            expected = (((size_t) data[3]) | (((size_t) data[4]) << 8)) + 5;
            break;
        case 0x04:
            expected = ((size_t) data[2]) + 3;
            break;
        case 0x05:
            /* ISO data length is the low 14 bits of a little-endian field */
            expected = (((size_t) data[3]) |
                        ((((size_t) data[4]) & 0x3F) << 8)) + 5;
            break;
        default:
            expected = ((size_t) data[3]) + 4;
            break;
    }
    if (len < expected) {
        if (recoverable)
            return 0;
        panicf("hci recv buffer too small, got %u but wanted at least %u",
               len, expected);
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

    /* Bluedroid registers its callbacks during init */
    semaphore_wait(&hci_callbacks_sem, TIMEOUT_BLOCK);

    while (!hci_closing) {
        size_t recvd = hci_recv_uart(buf, HCI_RECV_BUF_SIZE);
        if (recvd > 0 && !hci_closing)
            hci_callbacks->notify_host_recv(buf, recvd);
    }

    hci_callbacks = NULL;
}

/* Sends a command during bring-up and waits for it to complete
 * successfully. Returns false if the controller doesn't answer or reports
 * an error. */
static bool hci_init_cmd(const uint8_t *cmd, size_t len, uint8_t *buf)
{
    hci_send_uart((uint8_t *)cmd, len);

    /* Skip anything else the controller might send meanwhile. A Command
     * Complete event is 04 0e <len> <credits> <opcode, 2> <status> ... */
    for (int tries = 0; tries < 4; tries++) {
        size_t n = hci_recv_uart(buf, HCI_RECV_BUF_SIZE);
        if (n == 0)
            return false;
        if (n >= 7 && buf[0] == 0x04 && buf[1] == 0x0e &&
            buf[4] == cmd[1] && buf[5] == cmd[2])
            return buf[6] == 0;
    }
    return false;
}

static const uint8_t hci_reset_cmd[] = {0x01, 0x03, 0x0c, 0x00};

/* The controller acknowledges at the current speed, then switches */
static bool hci_set_fast_baud(uint8_t *buf)
{
    /* Update_UART_Baud_Rate
     * Encoded_Baud_Rate: 0x0000 (unused)
     * Explicit_Baud_Rate: 0x002dc6c0 (DEC 3000000) */
    static const uint8_t baud_cmd[] = {
        0x01, 0x18, 0xfc, 0x06, 0x00, 0x00, 0xc0, 0xc6, 0x2d, 0x00
    };
    static const uint8_t read_version_cmd[] = {0x01, 0x01, 0x10, 0x00};

    if (!hci_init_cmd(baud_cmd, sizeof(baud_cmd), buf))
        return false;

    uart_set_baud(PORT_UART0, 3000000);
    sleep(HZ / 100);

    /* Make sure both ends switched */
    return hci_init_cmd(read_version_cmd, sizeof(read_version_cmd), buf);
}

/* The controller's 32.768 kHz sleep clock: the RTC clock output, function 0
 * of PB26. Like the original firmware, only enable it while powered. */
#define GPIO_BT_CLK32K GPIO_PB(26)

static bool hci_clk32k_on = false;

bool bt_hci_sleep_clock(void)
{
    return hci_clk32k_on;
}

/* Follows the original firmware's power sequence. It also holds BT_WAKE
 * asserted for good, so the controller never enters its UART sleep mode. */
static void hci_power(bool on)
{
    if (on) {
        /* The RTC clock defaults to EXCLK/512 (46.875 kHz). Select the
         * 32.768 kHz oscillator, which the RTC keeps running anyway; nothing
         * else uses the RTC clock (timers, PWM and the watchdog run from
         * EXCLK or PCLK). Check it took: a wrong sleep clock is worse than
         * none. */
        jz_writef(CPM_OPCR, ERCS(1));
        hci_clk32k_on = jz_readf(CPM_OPCR, ERCS) != 0;
        if (hci_clk32k_on) {
            gpio_set_function(GPIO_BT_CLK32K, GPIOF_DEVICE(0));
            sleep(HZ / 100);
        }
        gpio_set_level(GPIO_HOST_WAKE_BT, 1);
        gpio_set_level(GPIO_BT_REG_ON_HW4, 1);

        /* Give the controller time to boot before talking to it */
        sleep(HZ / 10);
    } else {
        gpio_set_level(GPIO_BT_REG_ON_HW4, 0);
        gpio_set_level(GPIO_HOST_WAKE_BT, 0);
        if (hci_clk32k_on) {
            gpio_set_function(GPIO_BT_CLK32K, GPIOF_INPUT);
            hci_clk32k_on = false;
        }
    }
}

/* Powers up the controller, downloads the patch and leaves the controller
 * running it at 3 Mbaud. The download stays at 115200 (~3 s): unlike with
 * brcm_patchram_plus --use_baudrate_for_download in the original firmware,
 * the ROM never acknowledges Update_UART_Baud_Rate here. */
static bool hci_bring_up(uint8_t *buf)
{
    uart_init(PORT_UART0, 115200);
    hci_power(true);

    if (!hci_init_cmd(hci_reset_cmd, sizeof(hci_reset_cmd), buf))
        return false;

    /* Download_Minidriver */
    static const uint8_t minidriver_cmd[] = {0x01, 0x2e, 0xfc, 0x00};
    if (!hci_init_cmd(minidriver_cmd, sizeof(minidriver_cmd), buf))
        return false;

    sleep(HZ / 20);

    for (uint8_t **i = hcd_init_commands; *i != NULL; i++) {
        if (!hci_init_cmd(*i, ((size_t) (*i)[3]) + 4, buf))
            return false;
    }

    /* The final record is Launch_RAM, which restarts the controller with the
     * patched firmware; brcm_patchram_plus waits 200 ms here as well */
    sleep(HZ / 4);

    return hci_init_cmd(hci_reset_cmd, sizeof(hci_reset_cmd), buf) &&
           hci_set_fast_baud(buf);
}

static bool hci_enabled = false;

bool bt_hci_enable(void)
{
    uint8_t buf[HCI_RECV_BUF_SIZE];

    if (device_data.hw_rev != 4 || hci_enabled)
        return false;

    hci_enabled = true;
    hci_closing = false;
    hci_callbacks = NULL;
    semaphore_init(&hci_callbacks_sem, 1, 0);
    hci_log_open();

    /* Power cycle and try once more if the controller doesn't answer */
    hci_rx_timeout = HCI_INIT_TIMEOUT;
    bool ok = false;
    for (int attempt = 0; attempt < 2 && !ok; attempt++) {
        ok = hci_bring_up(buf);
        if (!ok) {
            uart_deinit(PORT_UART0);
            hci_power(false);
            sleep(HZ / 10);
        }
    }
    hci_rx_timeout = TIMEOUT_BLOCK;

    if (ok) {
        hci_thread_id = create_thread(hci_thread, hci_stack, HCI_STACK_SIZE,
                                      0, "bt_hci"
                                      IF_PRIO(, PRIORITY_BLUETOOTH));
        ok = hci_thread_id != 0;
    }
    if (!ok) {
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
    if (!hci_enabled)
        return;

    /* Stop the receive thread: deinitialising the UART wakes up a blocked
     * read, and it might still be waiting for Bluedroid's callbacks */
    hci_closing = true;
    uart_deinit(PORT_UART0);
    if (hci_thread_id != 0) {
        semaphore_release(&hci_callbacks_sem);
        thread_wait(hci_thread_id);
        hci_thread_id = 0;
    }

    hci_power(false);
    hci_log_close();
    hci_enabled = false;
}
