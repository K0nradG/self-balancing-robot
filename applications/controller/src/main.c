#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)

const struct device* uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#define LOG_MODULE_NAME central_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define UART_BUFFER_SIZE 1

static struct k_work_delayable uart_ble_work;
static uint8_t uart_buffer[UART_BUFFER_SIZE];

static struct bt_conn* default_conn;
static struct bt_nus_client nus_client;

static uint16_t rotate_step = 40;

static void
send_nus_message(const char* message)
{
    int err;

    if(!default_conn)
    {
        LOG_WRN("Not connected; cannot send data");
        return;
    }

    err = bt_nus_client_send(&nus_client, message, strlen(message));
    if(err)
    {
        LOG_ERR("Failed to send data over NUS (err %d)", err);
    }
    else
    {
        LOG_INF("Sent: %s", message);
    }
}

static void
uart_ble_work_handler(struct k_work* work)
{
    uint8_t c = uart_buffer[0];
    switch(c)
    {
        case 'r':
            send_nus_message(CONFIG_ROTATE_RIGHT_COMMAND);
            break;

        case 'l':
            send_nus_message(CONFIG_ROTATE_LEFT_COMMAND);
            break;

        case 'f':
            send_nus_message(CONFIG_DRIVE_FORWARD_COMMAND);
            break;

        default:
            LOG_WRN("Nieznany znak z UART: %c", c);
            break;
    }
}

static K_WORK_DELAYABLE_DEFINE(uart_ble_work, uart_ble_work_handler);

static void
uart_cb(const struct device* dev, void* user_data)
{
    uint8_t c;

    while(uart_fifo_read(dev, &c, 1))
    {
        LOG_INF("Received from UART: %c", c);
        uart_buffer[0] = c;
        k_work_submit(&uart_ble_work.work);
    }
}

static void
ble_data_sent(struct bt_nus_client* nus, uint8_t err, const uint8_t* const data, uint16_t len)
{
    ARG_UNUSED(nus);
    ARG_UNUSED(data);
    ARG_UNUSED(len);

    if(err)
    {
        LOG_WRN("ATT error code: 0x%02X", err);
    }
}

static void
button_handler(uint32_t button_state, uint32_t has_changed)
{
    if(has_changed & DK_BTN1_MSK)
    {
        if(button_state & DK_BTN1_MSK)
        {
            char message[8];
            snprintf(message, sizeof(message), "rs%d", rotate_step);
            send_nus_message(message);

            rotate_step += 40;
            if(rotate_step > 320)
            {
                rotate_step = 40;
            }
        }
    }

    if(has_changed & DK_BTN2_MSK)
    {
        if(button_state & DK_BTN2_MSK)
        {
            send_nus_message(CONFIG_ROTATE_LEFT_COMMAND);
        }
    }
}

static uint8_t
ble_data_received(struct bt_nus_client* nus, const uint8_t* data, uint16_t len)
{
    ARG_UNUSED(nus);
    LOG_INF("Received data over BLE: %.*s", len, data);
    return BT_GATT_ITER_CONTINUE;
}

static void
discovery_complete(struct bt_gatt_dm* dm, void* context)
{
    struct bt_nus_client* nus = context;
    LOG_INF("Service discovery completed");

    bt_gatt_dm_data_print(dm);

    bt_nus_handles_assign(dm, nus);
    bt_nus_subscribe_receive(nus);

    bt_gatt_dm_data_release(dm);
}

static void
discovery_service_not_found(struct bt_conn* conn, void* context)
{
    LOG_INF("Service not found");
}

static void
discovery_error(struct bt_conn* conn, int err, void* context)
{
    LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
    .completed         = discovery_complete,
    .service_not_found = discovery_service_not_found,
    .error_found       = discovery_error,
};

static void
gatt_discover(struct bt_conn* conn)
{
    int err;

    if(conn != default_conn)
    {
        return;
    }

    err = bt_gatt_dm_start(conn, BT_UUID_NUS_SERVICE, &discovery_cb, &nus_client);
    if(err)
    {
        LOG_ERR("could not start the discovery procedure, error code: %d", err);
    }
}

static void
exchange_func(struct bt_conn* conn, uint8_t err, struct bt_gatt_exchange_params* params)
{
    if(!err)
    {
        LOG_INF("MTU exchange done");
    }
    else
    {
        LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
    }
}

static void
connected(struct bt_conn* conn, uint8_t conn_err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if(conn_err)
    {
        LOG_INF("Failed to connect to %s, 0x%02x %s", addr, conn_err, bt_hci_err_to_str(conn_err));

        if(default_conn == conn)
        {
            bt_conn_unref(default_conn);
            default_conn = NULL;

            err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
            if(err)
            {
                LOG_ERR("Scanning failed to start (err %d)", err);
            }
        }

        return;
    }

    LOG_INF("Connected: %s", addr);

    static struct bt_gatt_exchange_params exchange_params;

    exchange_params.func = exchange_func;
    err                  = bt_gatt_exchange_mtu(conn, &exchange_params);
    if(err)
    {
        LOG_WRN("MTU exchange failed (err %d)", err);
    }

    gatt_discover(conn);

    err = bt_scan_stop();
    if((!err) && (err != -EALREADY))
    {
        LOG_ERR("Stop LE scan failed (err %d)", err);
    }
}

static void
disconnected(struct bt_conn* conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

    if(default_conn != conn)
    {
        return;
    }

    bt_conn_unref(default_conn);
    default_conn = NULL;

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if(err)
    {
        LOG_ERR("Scanning failed to start (err %d)", err);
    }
}

static void
security_changed(struct bt_conn* conn, bt_security_t level, enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if(!err)
    {
        LOG_INF("Security changed: %s level %u", addr, level);
    }
    else
    {
        LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err, bt_security_err_to_str(err));
    }

    gatt_discover(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected        = connected,
    .disconnected     = disconnected,
    .security_changed = security_changed,
};

static void
scan_filter_match(struct bt_scan_device_info* device_info, struct bt_scan_filter_match* filter_match, bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    LOG_INF("Filters matched. Address: %s connectable: %d", addr, connectable);
}

static void
scan_connecting_error(struct bt_scan_device_info* device_info)
{
    LOG_WRN("Connecting failed");
}

static void
scan_connecting(struct bt_scan_device_info* device_info, struct bt_conn* conn)
{
    default_conn = bt_conn_ref(conn);
}

static int
nus_client_init(void)
{
    int err;
    struct bt_nus_client_init_param init = {
        .cb =
            {
                .received = ble_data_received,
                .sent     = ble_data_sent,
            },
    };

    err = bt_nus_client_init(&nus_client, &init);
    if(err)
    {
        LOG_ERR("NUS Client initialization failed (err %d)", err);
        return err;
    }

    LOG_INF("NUS Client module initialized");
    return err;
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

static int
scan_init(void)
{
    int err;
    struct bt_scan_init_param scan_init = {
        .connect_if_match = 1,
    };

    bt_scan_init(&scan_init);
    bt_scan_cb_register(&scan_cb);

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, CONFIG_TARGET_BLE_DEVICE_NAME);
    if(err)
    {
        LOG_ERR("Scanning filters cannot be set (err %d)", err);
        return err;
    }

    err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, false);
    if(err)
    {
        LOG_ERR("Filters cannot be turned on (err %d)", err);
        return err;
    }

    LOG_INF("Scan module initialized");
    return err;
}

int
main(void)
{
    if(!device_is_ready(uart_dev))
    {
        LOG_ERR("UART device not ready");
        return 0;
    }

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    int err;

    err = dk_buttons_init(button_handler);
    if(err)
    {
        LOG_ERR("dk_buttons_init failed (err %d)", err);
        return 0;
    }

    LOG_INF("Buttons and LEDs initialized");

    err = bt_enable(NULL);
    if(err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }
    LOG_INF("Bluetooth initialized");

    err = scan_init();
    if(err != 0)
    {
        LOG_ERR("scan_init failed (err %d)", err);
        return 0;
    }

    err = nus_client_init();
    if(err != 0)
    {
        LOG_ERR("nus_client_init failed (err %d)", err);
        return 0;
    }

    printk("Starting Bluetooth Central UART example\n");

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if(err)
    {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return 0;
    }

    LOG_INF("Scanning successfully started");

    while(true)
    {
        k_sleep(K_SECONDS(1));
    }
}
