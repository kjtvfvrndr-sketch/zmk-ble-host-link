/*
 * SPDX-License-Identifier: MIT
 *
 * A debugging check of what this keyboard brings to a new pairing
 * (ZMK_BLE_HOST_LINK_CHECK_OWN_PAIRING_STATE). Pairing with LE Secure
 * Connections ends in a check both sides compute from a shared key (DHKey):
 * the keyboard from its private key and the host's public key, the host from
 * its private key and the keyboard's public key, both together with the two
 * addresses of the connection. When every new pairing fails at that check
 * (SMP 0x0b, DHKey check failed) while the hosts paired before keep working,
 * either the keyboard's key pair no longer holds together, or the keyboard
 * connects with another address than the one it computes with.
 *
 * The key pair: the stack computes a DHKey from the keyboard's private key
 * and a test public key; this file computes the same DHKey independently,
 * from the test private key and the keyboard's public key. The two agree
 * exactly when the keyboard's private key still belongs to its public key.
 * The test key pair is the specification's debug pair (Core Vol 3, Part H,
 * 2.3.5.6.1), public anyway. The stack runs one DHKey at a time: a pairing
 * starting during the check's own DHKey (about a quarter of a second) fails,
 * so the check runs only at start, on request and a few seconds after a
 * pairing failed.
 *
 * The address: the random address the stack last gave the controller, and
 * the one its legacy advertiser keeps, against the identity it computes with.
 *
 * It reaches into the stack's internals (bt_pub_key_get(), bt_dh_key_gen(),
 * bt_dev): for debugging builds, with the host's own ECC emulation
 * (BT_TINYCRYPT_ECC), which this file also computes with.
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

#include <tinycrypt/constants.h>
#include <tinycrypt/ecc.h>
#include <tinycrypt/ecc_dh.h>

/* The stack's internal headers (subsys/bluetooth/host). */
#include "ecc.h"
#include "hci_core.h"

#include "own_pairing_state_check.h"

LOG_MODULE_DECLARE(zmk_ble_host_link, CONFIG_ZMK_BLE_HOST_LINK_LOG_LEVEL);

#define COORD_LEN 32

/* The specification's debug key pair: the private key big-endian, as the
   crypto library takes it; the public key little-endian per coordinate, as
   HCI carries it. */
static const uint8_t test_private_key_be[COORD_LEN] = {
    0x3f, 0x49, 0xf6, 0xd4, 0xa3, 0xc5, 0x5f, 0x38, 0x74, 0xc9, 0xb3, 0xe3, 0xd2, 0x10, 0x3f, 0x50,
    0x4a, 0xff, 0x60, 0x7b, 0xeb, 0x40, 0xb7, 0x99, 0x58, 0x99, 0xb8, 0xa6, 0xcd, 0x3c, 0x1a, 0xbd,
};
static const uint8_t test_public_key_le[2 * COORD_LEN] = {
    /* X */
    0xe6, 0x9d, 0x35, 0x0e, 0x48, 0x01, 0x03, 0xcc, 0xdb, 0xfd, 0xf4, 0xac, 0x11, 0x91, 0xf4, 0xef,
    0xb9, 0xa5, 0xf9, 0xe9, 0xa7, 0x83, 0x2c, 0x5e, 0x2c, 0xbe, 0x97, 0xf2, 0xd2, 0x03, 0xb0, 0x20,
    /* Y */
    0x8b, 0xd2, 0x89, 0x15, 0xd0, 0x8e, 0x1c, 0x74, 0x24, 0x30, 0xed, 0x8f, 0xc2, 0x45, 0x63, 0x76,
    0x5c, 0x15, 0x52, 0x5a, 0xbf, 0x9a, 0x32, 0x63, 0x6d, 0xeb, 0x2a, 0x65, 0x49, 0x9c, 0x80, 0xdc,
};

/* The stack is computing a pairing's DHKey: tried again this often, this
   many times. */
#define STACK_BUSY_RETRY_MS 500
#define STACK_BUSY_RETRIES 10
/* The stack's answer takes about a quarter of a second. */
#define STACK_ANSWER_TIMEOUT_MS 3000

/* The independent computation takes as long: on a queue of its own, below
   ZMK's work, so keys and the split link do not wait for it. */
#define CHECK_STACK_SIZE 2048
#define CHECK_PRIORITY K_LOWEST_APPLICATION_THREAD_PRIO

static K_THREAD_STACK_DEFINE(check_stack, CHECK_STACK_SIZE);
static struct k_work_q check_queue;

static const char *check_occasion;
static struct k_work check_work;

static K_SEM_DEFINE(stack_dhkey_ready, 0, 1);
static uint8_t stack_dhkey_le[COORD_LEN];
static bool stack_dhkey_valid;

/* From the Bluetooth RX context. */
static void on_stack_dhkey(const uint8_t *dhkey) {
    stack_dhkey_valid = dhkey != NULL;
    if (dhkey != NULL) {
        memcpy(stack_dhkey_le, dhkey, sizeof(stack_dhkey_le));
    }
    k_sem_give(&stack_dhkey_ready);
}

void own_pairing_state_public_key_fingerprint(char *buf, size_t len) {
    const uint8_t *key = bt_pub_key_get();

    if (key == NULL) {
        snprintf(buf, len, "none");
        return;
    }
    snprintf(buf, len, "%02x%02x%02x%02x", key[0], key[1], key[2], key[3]);
}

static void check_key_pair_cb(struct k_work *work) {
    ARG_UNUSED(work);

    const char *occasion = check_occasion;
    const uint8_t *keyboard_public_le = bt_pub_key_get();
    char fingerprint[12];

    own_pairing_state_public_key_fingerprint(fingerprint, sizeof(fingerprint));
    if (keyboard_public_le == NULL) {
        LOG_WRN("Pairing key pair (%s): the keyboard has no public key", occasion);
        return;
    }

    /* This side: the test private key with the keyboard's public key. */
    uint8_t keyboard_public_be[2 * COORD_LEN];
    uint8_t expected_be[COORD_LEN];

    sys_memcpy_swap(keyboard_public_be, keyboard_public_le, COORD_LEN);
    sys_memcpy_swap(&keyboard_public_be[COORD_LEN], &keyboard_public_le[COORD_LEN], COORD_LEN);
    if (uECC_valid_public_key(keyboard_public_be, uECC_secp256r1()) != 0) {
        LOG_WRN("Pairing key pair BROKEN (%s): the keyboard's public key %s is not on the "
                "curve",
                occasion, fingerprint);
        return;
    }
    if (uECC_shared_secret(keyboard_public_be, test_private_key_be, expected_be,
                           uECC_secp256r1()) != TC_CRYPTO_SUCCESS) {
        LOG_WRN("Pairing key pair (%s): the check's own computation failed", occasion);
        return;
    }

    /* The stack's side: the keyboard's private key with the test public key. */
    int err;

    k_sem_reset(&stack_dhkey_ready);
    for (int tries = 0;; tries++) {
        err = bt_dh_key_gen(test_public_key_le, on_stack_dhkey);
        if (err != -EBUSY || tries >= STACK_BUSY_RETRIES) {
            break;
        }
        k_sleep(K_MSEC(STACK_BUSY_RETRY_MS));
    }
    if (err != 0) {
        LOG_WRN("Pairing key pair (%s): not checked, the stack did not take the request (%d)",
                occasion, err);
        return;
    }
    if (k_sem_take(&stack_dhkey_ready, K_MSEC(STACK_ANSWER_TIMEOUT_MS)) != 0) {
        LOG_WRN("Pairing key pair (%s): not checked, the stack did not answer", occasion);
        return;
    }
    if (!stack_dhkey_valid) {
        LOG_WRN("Pairing key pair BROKEN (%s): the stack could not compute a DHKey", occasion);
        return;
    }

    uint8_t stack_be[COORD_LEN];

    sys_memcpy_swap(stack_be, stack_dhkey_le, COORD_LEN);
    if (memcmp(stack_be, expected_be, COORD_LEN) != 0) {
        LOG_WRN("Pairing key pair BROKEN (%s): the private key no longer matches public key %s "
                "-- every new pairing fails its DHKey check",
                occasion, fingerprint);
    } else {
        LOG_INF("Pairing key pair OK (%s), public key %s", occasion, fingerprint);
    }
}

void own_pairing_state_check_key_pair(const char *occasion) {
    check_occasion = occasion;
    k_work_submit_to_queue(&check_queue, &check_work);
}

void own_pairing_state_log_address(const char *occasion, bool only_if_differs) {
    bt_addr_le_t identity[CONFIG_BT_ID_MAX];
    size_t count = ARRAY_SIZE(identity);
    char id_text[BT_ADDR_LE_STR_LEN];
    char random_text[BT_ADDR_LE_STR_LEN];
    char adv_text[BT_ADDR_LE_STR_LEN];

    bt_id_get(identity, &count);
    if (count == 0) {
        LOG_WRN("Own address (%s): no identity yet", occasion);
        return;
    }
    bt_addr_le_to_str(&identity[BT_ID_DEFAULT], id_text, sizeof(id_text));
    bt_addr_le_to_str(&bt_dev.random_addr, random_text, sizeof(random_text));
#if !defined(CONFIG_BT_EXT_ADV)
    bt_addr_le_to_str(&bt_dev.adv.random_addr, adv_text, sizeof(adv_text));
#else
    snprintf(adv_text, sizeof(adv_text), "(extended advertising)");
#endif

    const bool random_differs =
        identity[BT_ID_DEFAULT].type == BT_ADDR_LE_RANDOM &&
        !bt_addr_eq(&bt_dev.random_addr.a, &identity[BT_ID_DEFAULT].a);

    if (random_differs) {
        LOG_WRN("Own address (%s): the controller has %s, the identity is %s -- hosts pairing "
                "now compute with another address; advertiser keeps %s",
                occasion, random_text, id_text, adv_text);
    } else if (!only_if_differs) {
        LOG_INF("Own address (%s): %s, the identity; advertiser keeps %s", occasion,
                random_text, adv_text);
    }
}

static int own_pairing_state_check_init(void) {
    k_work_init(&check_work, check_key_pair_cb);
    k_work_queue_start(&check_queue, check_stack, K_THREAD_STACK_SIZEOF(check_stack),
                       CHECK_PRIORITY, NULL);
    k_thread_name_set(&check_queue.thread, "pairing_check");
    return 0;
}

SYS_INIT(own_pairing_state_check_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
