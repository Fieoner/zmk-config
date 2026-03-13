/*
 * Charybdis BLE-to-USB HID Dongle
 *
 * Connects to a ZMK keyboard over BLE, discovers the HID service,
 * subscribes to keyboard/consumer/mouse report notifications,
 * and forwards them over USB HID.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/settings/settings.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dongle, LOG_LEVEL_INF);

/* ---------- HID Report IDs (must match ZMK) ---------- */

#define REPORT_ID_KEYBOARD 0x01
#define REPORT_ID_CONSUMER 0x02
#define REPORT_ID_MOUSE    0x03

/* Report body sizes (without report ID) */
#define KEYBOARD_REPORT_SIZE 8   /* 1 mod + 1 reserved + 6 keys (HKRO) */
#define CONSUMER_REPORT_SIZE 12  /* 6 × uint16_t (FULL) */
#define MOUSE_REPORT_SIZE    9   /* 1 buttons + 4×int16 (x,y,scroll_y,scroll_x) */

#define MAX_REPORT_SIZE 13       /* max(8,12,9) + 1 for report_id */

/* Target keyboard name (overridable via -DDONGLE_TARGET_NAME="...") */
#ifndef DONGLE_TARGET_NAME
#define DONGLE_TARGET_NAME "Charybdis"
#endif
#define TARGET_NAME DONGLE_TARGET_NAME

/* Fast BLE connection params: 7.5ms interval, 0 latency, 4s timeout */
#define CONN_PARAM_FAST BT_LE_CONN_PARAM(6, 6, 0, 400)

/* ---------- HID Report Descriptor (matches ZMK HKRO + Consumer FULL + Mouse) ---------- */

static const uint8_t hid_report_desc[] = {
    /* ---- Keyboard (Report ID 1) ---- */
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x06,       /* Usage (Keyboard) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, 0x01,       /*   Report ID (1) */
    /* Modifier keys (8 bits) */
    0x05, 0x07,       /*   Usage Page (Key Codes) */
    0x19, 0xE0,       /*   Usage Min (Left Control) */
    0x29, 0xE7,       /*   Usage Max (Right GUI) */
    0x15, 0x00,       /*   Logical Min (0) */
    0x25, 0x01,       /*   Logical Max (1) */
    0x75, 0x01,       /*   Report Size (1) */
    0x95, 0x08,       /*   Report Count (8) */
    0x81, 0x02,       /*   Input (Data, Variable, Absolute) */
    /* Reserved byte */
    0x05, 0x07,       /*   Usage Page (Key Codes) */
    0x75, 0x08,       /*   Report Size (8) */
    0x95, 0x01,       /*   Report Count (1) */
    0x81, 0x03,       /*   Input (Constant, Variable, Absolute) */
    /* Key array (6 keys, HKRO) */
    0x05, 0x07,       /*   Usage Page (Key Codes) */
    0x15, 0x00,       /*   Logical Min (0) */
    0x26, 0xFF, 0x00, /*   Logical Max (255) */
    0x19, 0x00,       /*   Usage Min (0) */
    0x29, 0xFF,       /*   Usage Max (255) */
    0x75, 0x08,       /*   Report Size (8) */
    0x95, 0x06,       /*   Report Count (6) */
    0x81, 0x00,       /*   Input (Data, Array, Absolute) */
    0xC0,             /* End Collection */

    /* ---- Consumer (Report ID 2) ---- */
    0x05, 0x0C,       /* Usage Page (Consumer) */
    0x09, 0x01,       /* Usage (Consumer Control) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, 0x02,       /*   Report ID (2) */
    0x05, 0x0C,       /*   Usage Page (Consumer) */
    /* Consumer keys (6 × 16-bit, FULL mode) */
    0x15, 0x00,       /*   Logical Min (0) */
    0x26, 0xFF, 0x0F, /*   Logical Max (0x0FFF) */
    0x19, 0x00,       /*   Usage Min (0) */
    0x2A, 0xFF, 0x0F, /*   Usage Max (0x0FFF) */
    0x75, 0x10,       /*   Report Size (16) */
    0x95, 0x06,       /*   Report Count (6) */
    0x81, 0x00,       /*   Input (Data, Array, Absolute) */
    0xC0,             /* End Collection */

    /* ---- Mouse (Report ID 3) ---- */
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x02,       /* Usage (Mouse) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, 0x03,       /*   Report ID (3) */
    0x09, 0x01,       /*   Usage (Pointer) */
    0xA1, 0x00,       /*   Collection (Physical) */
    /* 5 buttons */
    0x05, 0x09,       /*     Usage Page (Button) */
    0x19, 0x01,       /*     Usage Min (1) */
    0x29, 0x05,       /*     Usage Max (5) */
    0x15, 0x00,       /*     Logical Min (0) */
    0x25, 0x01,       /*     Logical Max (1) */
    0x75, 0x01,       /*     Report Size (1) */
    0x95, 0x05,       /*     Report Count (5) */
    0x81, 0x02,       /*     Input (Data, Variable, Absolute) */
    /* 3-bit padding */
    0x75, 0x03,       /*     Report Size (3) */
    0x95, 0x01,       /*     Report Count (1) */
    0x81, 0x03,       /*     Input (Constant, Variable, Absolute) */
    /* X, Y (16-bit signed, relative) */
    0x05, 0x01,       /*     Usage Page (Generic Desktop) */
    0x09, 0x30,       /*     Usage (X) */
    0x09, 0x31,       /*     Usage (Y) */
    0x16, 0x00, 0x80, /*     Logical Min (-32768) */
    0x26, 0xFF, 0x7F, /*     Logical Max (32767) */
    0x75, 0x10,       /*     Report Size (16) */
    0x95, 0x02,       /*     Report Count (2) */
    0x81, 0x06,       /*     Input (Data, Variable, Relative) */
    /* Vertical scroll (16-bit signed, relative) */
    0xA1, 0x02,       /*     Collection (Logical) */
    0x09, 0x38,       /*       Usage (Wheel) */
    0x16, 0x00, 0x80, /*       Logical Min (-32768) */
    0x26, 0xFF, 0x7F, /*       Logical Max (32767) */
    0x35, 0x00,       /*       Physical Min (0) */
    0x45, 0x00,       /*       Physical Max (0) */
    0x75, 0x10,       /*       Report Size (16) */
    0x95, 0x01,       /*       Report Count (1) */
    0x81, 0x06,       /*       Input (Data, Variable, Relative) */
    0xC0,             /*     End Collection */
    /* Horizontal scroll (16-bit signed, relative) */
    0xA1, 0x02,       /*     Collection (Logical) */
    0x05, 0x0C,       /*       Usage Page (Consumer) */
    0x0A, 0x38, 0x02, /*       Usage (AC Pan) */
    0x16, 0x00, 0x80, /*       Logical Min (-32768) */
    0x26, 0xFF, 0x7F, /*       Logical Max (32767) */
    0x35, 0x00,       /*       Physical Min (0) */
    0x45, 0x00,       /*       Physical Max (0) */
    0x75, 0x10,       /*       Report Size (16) */
    0x95, 0x01,       /*       Report Count (1) */
    0x81, 0x06,       /*       Input (Data, Variable, Relative) */
    0xC0,             /*     End Collection */
    0xC0,             /*   End Collection (Physical) */
    0xC0,             /* End Collection (Application) */
};

/* ---------- USB HID with report accumulation ---------- */

static const struct device *hid_dev;
static volatile bool usb_ready;

/* Pending report buffers — accumulated while USB is busy */
static uint8_t pending_kb[KEYBOARD_REPORT_SIZE];
static uint8_t pending_consumer[CONSUMER_REPORT_SIZE];

/* Mouse accumulator: deltas are summed, buttons take latest state */
struct mouse_accum {
    uint8_t buttons;
    int32_t d_x;
    int32_t d_y;
    int32_t d_scroll_y;
    int32_t d_scroll_x;
};
static struct mouse_accum pending_mouse;

static bool dirty_kb;
static bool dirty_consumer;
static bool dirty_mouse;

static void flush_next_report(void);

static void usb_hid_ready(const struct device *dev)
{
    usb_ready = true;
    /* Immediately flush any pending accumulated report */
    flush_next_report();
}

static const struct hid_ops usb_hid_ops = {
    .int_in_ready = usb_hid_ready,
};

static void hid_write_report(uint8_t report_id, const uint8_t *body, uint16_t len)
{
    uint8_t buf[MAX_REPORT_SIZE];
    buf[0] = report_id;
    memcpy(buf + 1, body, len);
    uint32_t wrote;
    usb_ready = false;
    hid_int_ep_write(hid_dev, buf, len + 1, &wrote);
}

/* Clamp int32 to int16 range */
static int16_t clamp16(int32_t v)
{
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

static void flush_next_report(void)
{
    if (!usb_ready) {
        return;
    }

    /* Priority: keyboard > consumer > mouse */
    if (dirty_kb) {
        dirty_kb = false;
        hid_write_report(REPORT_ID_KEYBOARD, pending_kb, KEYBOARD_REPORT_SIZE);
        return;
    }
    if (dirty_consumer) {
        dirty_consumer = false;
        hid_write_report(REPORT_ID_CONSUMER, pending_consumer, CONSUMER_REPORT_SIZE);
        return;
    }
    if (dirty_mouse) {
        dirty_mouse = false;
        /* Pack accumulated mouse data into report body */
        uint8_t body[MOUSE_REPORT_SIZE];
        body[0] = pending_mouse.buttons;
        int16_t x = clamp16(pending_mouse.d_x);
        int16_t y = clamp16(pending_mouse.d_y);
        int16_t sy = clamp16(pending_mouse.d_scroll_y);
        int16_t sx = clamp16(pending_mouse.d_scroll_x);
        memcpy(&body[1], &x, 2);
        memcpy(&body[3], &y, 2);
        memcpy(&body[5], &sy, 2);
        memcpy(&body[7], &sx, 2);
        /* Reset accumulator (keep buttons — they're absolute state) */
        pending_mouse.d_x = 0;
        pending_mouse.d_y = 0;
        pending_mouse.d_scroll_y = 0;
        pending_mouse.d_scroll_x = 0;
        hid_write_report(REPORT_ID_MOUSE, body, MOUSE_REPORT_SIZE);
        return;
    }
}

static void queue_keyboard(const uint8_t *data, uint16_t len)
{
    if (len > KEYBOARD_REPORT_SIZE) len = KEYBOARD_REPORT_SIZE;
    memcpy(pending_kb, data, len);
    dirty_kb = true;
    flush_next_report();
}

static void queue_consumer(const uint8_t *data, uint16_t len)
{
    if (len > CONSUMER_REPORT_SIZE) len = CONSUMER_REPORT_SIZE;
    memcpy(pending_consumer, data, len);
    dirty_consumer = true;
    flush_next_report();
}

static void queue_mouse(const uint8_t *data, uint16_t len)
{
    if (len < MOUSE_REPORT_SIZE) return;
    /* buttons: absolute state — overwrite */
    pending_mouse.buttons = data[0];
    /* deltas: accumulate (little-endian int16) */
    int16_t x, y, sy, sx;
    memcpy(&x, &data[1], 2);
    memcpy(&y, &data[3], 2);
    memcpy(&sy, &data[5], 2);
    memcpy(&sx, &data[7], 2);
    pending_mouse.d_x += x;
    pending_mouse.d_y += y;
    pending_mouse.d_scroll_y += sy;
    pending_mouse.d_scroll_x += sx;
    dirty_mouse = true;
    flush_next_report();
}

/* ---------- BLE GATT Discovery ---------- */

static struct bt_conn *kb_conn;

/* HID Service: 0x1812 */
static struct bt_uuid_16 hid_svc_uuid = BT_UUID_INIT_16(0x1812);
/* HID Report Characteristic: 0x2A4D */
static struct bt_uuid_16 report_char_uuid = BT_UUID_INIT_16(0x2A4D);
/* HID Report Reference Descriptor: 0x2908 */
static struct bt_uuid_16 report_ref_uuid = BT_UUID_INIT_16(0x2908);

/* Track discovered report characteristics */
struct report_info {
    uint16_t value_handle;
    uint8_t report_id;
    uint8_t report_type; /* 1=input, 2=output, 3=feature */
};

#define MAX_REPORTS 6
static struct report_info reports[MAX_REPORTS];
static int num_reports;

/* Discovery params */
static struct bt_gatt_discover_params disc_params;

/* Subscribe params (one per input report we care about) */
static struct bt_gatt_subscribe_params sub_keyboard;
static struct bt_gatt_subscribe_params sub_consumer;
static struct bt_gatt_subscribe_params sub_mouse;

/* Read params for report reference */
static struct bt_gatt_read_params read_params;

/* Handles for the HID service range */
static uint16_t hid_start_handle;
static uint16_t hid_end_handle;

/* Temporary storage during char discovery */
struct char_pending {
    uint16_t value_handle;
    uint16_t end_handle;
};

#define MAX_PENDING 6
static struct char_pending pending_chars[MAX_PENDING];
static int num_pending;
static int ref_read_idx;

/* Forward declarations */
static void start_scan(void);
static void discover_hid_service(void);
static void discover_report_chars(void);
static void read_next_report_ref(void);
static void subscribe_reports(void);

/* ---------- BLE Notification Callbacks ---------- */

static uint8_t keyboard_notify_cb(struct bt_conn *conn,
                                   struct bt_gatt_subscribe_params *params,
                                   const void *data, uint16_t length)
{
    if (!data) {
        LOG_WRN("Keyboard unsubscribed");
        return BT_GATT_ITER_STOP;
    }
    queue_keyboard(data, length);
    return BT_GATT_ITER_CONTINUE;
}

static uint8_t consumer_notify_cb(struct bt_conn *conn,
                                   struct bt_gatt_subscribe_params *params,
                                   const void *data, uint16_t length)
{
    if (!data) {
        LOG_WRN("Consumer unsubscribed");
        return BT_GATT_ITER_STOP;
    }
    queue_consumer(data, length);
    return BT_GATT_ITER_CONTINUE;
}

static uint8_t mouse_notify_cb(struct bt_conn *conn,
                                struct bt_gatt_subscribe_params *params,
                                const void *data, uint16_t length)
{
    if (!data) {
        LOG_WRN("Mouse unsubscribed");
        return BT_GATT_ITER_STOP;
    }
    queue_mouse(data, length);
    return BT_GATT_ITER_CONTINUE;
}

/* ---------- Subscribe to Input Reports ---------- */

static void subscribe_reports(void)
{
    LOG_INF("Subscribing to %d report(s)", num_reports);

    for (int i = 0; i < num_reports; i++) {
        if (reports[i].report_type != 1) {
            continue; /* Only subscribe to Input reports */
        }

        struct bt_gatt_subscribe_params *sub = NULL;
        bt_gatt_notify_func_t cb = NULL;

        switch (reports[i].report_id) {
        case REPORT_ID_KEYBOARD:
            sub = &sub_keyboard;
            cb = keyboard_notify_cb;
            break;
        case REPORT_ID_CONSUMER:
            sub = &sub_consumer;
            cb = consumer_notify_cb;
            break;
        case REPORT_ID_MOUSE:
            sub = &sub_mouse;
            cb = mouse_notify_cb;
            break;
        default:
            LOG_WRN("Unknown report ID %d", reports[i].report_id);
            continue;
        }

        sub->notify = cb;
        sub->value_handle = reports[i].value_handle;
        /* CCC handle is value_handle + 1 for standard GATT layout */
        sub->ccc_handle = reports[i].value_handle + 1;
        sub->end_handle = hid_end_handle;
        sub->disc_params = NULL; /* We already know the CCC handle */
        sub->value = BT_GATT_CCC_NOTIFY;

        int err = bt_gatt_subscribe(kb_conn, sub);
        if (err && err != -EALREADY) {
            LOG_ERR("Subscribe failed for report %d (err %d)",
                    reports[i].report_id, err);
        } else {
            LOG_INF("Subscribed to report ID %d (handle 0x%04x)",
                    reports[i].report_id, reports[i].value_handle);
        }
    }
}

/* ---------- Report Reference Read ---------- */

static uint8_t report_ref_read_cb(struct bt_conn *conn, uint8_t err,
                                   struct bt_gatt_read_params *params,
                                   const void *data, uint16_t length)
{
    if (err || !data || length < 2) {
        LOG_WRN("Report ref read failed (err %d, len %d)", err, length);
    } else {
        const uint8_t *ref = data;
        reports[num_reports].value_handle = pending_chars[ref_read_idx].value_handle;
        reports[num_reports].report_id = ref[0];
        reports[num_reports].report_type = ref[1];
        LOG_INF("Report: handle=0x%04x id=%d type=%d",
                reports[num_reports].value_handle, ref[0], ref[1]);
        num_reports++;
    }

    ref_read_idx++;
    if (ref_read_idx < num_pending) {
        read_next_report_ref();
    } else {
        subscribe_reports();
    }

    return BT_GATT_ITER_STOP;
}

static void read_next_report_ref(void)
{
    /* Find the Report Reference descriptor for this characteristic.
     * It's typically at value_handle + 2 (after CCC descriptor),
     * but we search within the char's handle range to be safe. */
    read_params.func = report_ref_read_cb;
    read_params.handle_count = 0;
    read_params.by_uuid.uuid = &report_ref_uuid.uuid;
    read_params.by_uuid.start_handle = pending_chars[ref_read_idx].value_handle;
    read_params.by_uuid.end_handle = pending_chars[ref_read_idx].end_handle;

    int err = bt_gatt_read(kb_conn, &read_params);
    if (err) {
        LOG_ERR("Report ref read start failed (err %d)", err);
        ref_read_idx++;
        if (ref_read_idx < num_pending) {
            read_next_report_ref();
        } else {
            subscribe_reports();
        }
    }
}

/* ---------- Characteristic Discovery ---------- */

static uint8_t char_discover_cb(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_INF("Characteristic discovery done, found %d", num_pending);
        if (num_pending > 0) {
            ref_read_idx = 0;
            num_reports = 0;
            read_next_report_ref();
        }
        return BT_GATT_ITER_STOP;
    }

    const struct bt_gatt_chrc *chrc = attr->user_data;

    if (num_pending < MAX_PENDING) {
        pending_chars[num_pending].value_handle = chrc->value_handle;
        /* End handle is either the next attr handle - 1, or we approximate */
        pending_chars[num_pending].end_handle = chrc->value_handle + 3;
        num_pending++;
    }

    return BT_GATT_ITER_CONTINUE;
}

static void discover_report_chars(void)
{
    num_pending = 0;

    disc_params.uuid = &report_char_uuid.uuid;
    disc_params.func = char_discover_cb;
    disc_params.start_handle = hid_start_handle;
    disc_params.end_handle = hid_end_handle;
    disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(kb_conn, &disc_params);
    if (err) {
        LOG_ERR("Char discovery failed (err %d)", err);
    }
}

/* ---------- Service Discovery ---------- */

static uint8_t svc_discover_cb(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_ERR("HID service not found");
        return BT_GATT_ITER_STOP;
    }

    const struct bt_gatt_service_val *svc = attr->user_data;
    hid_start_handle = attr->handle;
    hid_end_handle = svc->end_handle;

    LOG_INF("HID service found: 0x%04x - 0x%04x", hid_start_handle, hid_end_handle);

    discover_report_chars();
    return BT_GATT_ITER_STOP;
}

static void discover_hid_service(void)
{
    disc_params.uuid = &hid_svc_uuid.uuid;
    disc_params.func = svc_discover_cb;
    disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    disc_params.type = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(kb_conn, &disc_params);
    if (err) {
        LOG_ERR("Service discovery failed (err %d)", err);
    }
}

/* ---------- Security ---------- */

static void security_changed(struct bt_conn *conn, bt_security_t level,
                              enum bt_security_err err)
{
    if (err) {
        LOG_WRN("Security failed (err %d) — deleting bond and reconnecting", err);
        const bt_addr_le_t *addr = bt_conn_get_dst(conn);
        bt_unpair(BT_ID_DEFAULT, addr);
        bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
        return;
    }

    LOG_INF("Security level %d", level);

    /* Now that we're encrypted, discover HID service */
    discover_hid_service();
}

/* ---------- Connection Callbacks ---------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %d)", err);
        kb_conn = NULL;
        start_scan();
        return;
    }

    LOG_INF("Connected");
    kb_conn = bt_conn_ref(conn);

    /* Request 2M PHY for lower air time */
    const struct bt_conn_le_phy_param phy = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
    };
    bt_conn_le_phy_update(conn, &phy);

    /* Request fast connection interval (7.5ms) */
    bt_conn_le_param_update(conn, CONN_PARAM_FAST);

    /* HID requires encryption — request security */
    err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (err) {
        LOG_ERR("Set security failed (err %d)", err);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %d)", reason);

    if (kb_conn) {
        bt_conn_unref(kb_conn);
        kb_conn = NULL;
    }

    /* Clear subscription state */
    memset(&sub_keyboard, 0, sizeof(sub_keyboard));
    memset(&sub_consumer, 0, sizeof(sub_consumer));
    memset(&sub_mouse, 0, sizeof(sub_mouse));

    /* Send empty reports to release all keys/buttons */
    static const uint8_t empty_kb[KEYBOARD_REPORT_SIZE] = {0};
    static const uint8_t empty_consumer[CONSUMER_REPORT_SIZE] = {0};
    static const uint8_t empty_mouse[MOUSE_REPORT_SIZE] = {0};
    queue_keyboard(empty_kb, sizeof(empty_kb));
    queue_consumer(empty_consumer, sizeof(empty_consumer));
    queue_mouse(empty_mouse, sizeof(empty_mouse));

    /* Restart scanning */
    start_scan();
}

BT_CONN_CB_DEFINE(conn_cbs) = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed,
};

/* ---------- Scanning ---------- */

static bool parse_ad_name(struct bt_data *data, void *user_data)
{
    bool *found = user_data;

    if (data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) {
        if (data->data_len == sizeof(TARGET_NAME) - 1 &&
            memcmp(data->data, TARGET_NAME, data->data_len) == 0) {
            *found = true;
            return false; /* Stop parsing */
        }
    }
    return true; /* Continue parsing */
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                     uint8_t adv_type, struct net_buf_simple *buf)
{
    if (kb_conn) {
        return; /* Already connected */
    }

    bool found = false;
    bt_data_parse(buf, parse_ad_name, &found);

    if (!found) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("Found %s (%s, RSSI %d)", TARGET_NAME, addr_str, rssi);

    /* Stop scanning */
    bt_le_scan_stop();

    /* Connect */
    struct bt_conn *conn;
    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                                 CONN_PARAM_FAST, &conn);
    if (err) {
        LOG_ERR("Create conn failed (err %d)", err);
        start_scan();
        return;
    }
    bt_conn_unref(conn);
}

static void start_scan(void)
{
    int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
    if (err) {
        LOG_ERR("Scan start failed (err %d)", err);
    } else {
        LOG_INF("Scanning for %s...", TARGET_NAME);
    }
}

/* ---------- Main ---------- */

int main(void)
{
    LOG_INF("Dongle starting (target: %s)", TARGET_NAME);

    /* Initialize USB HID */
    hid_dev = device_get_binding("HID_0");
    if (!hid_dev) {
        LOG_ERR("HID device not found");
        return -1;
    }

    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc),
                            &usb_hid_ops);
    usb_hid_init(hid_dev);

    int err = usb_enable(NULL);
    if (err && err != -EALREADY) {
        LOG_WRN("USB enable returned %d (may already be enabled)", err);
    }
    /* Wait for USB enumeration then mark endpoint ready */
    k_msleep(500);
    usb_ready = true;
    LOG_INF("USB enabled");

    /* Initialize BLE */
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("BLE enable failed (err %d)", err);
        return -1;
    }
    LOG_INF("BLE enabled");

    /* Load settings (stored bonds) */
    settings_load();

    /* Start scanning for keyboard */
    start_scan();

    return 0;
}
