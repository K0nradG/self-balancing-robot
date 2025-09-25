#include "dfu_ble.h"

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt.h>

#include "aliro/utils.h"
#include "logger/platform_log.h"

namespace {

constexpr uint32_t kAdvertisingOptions = BT_LE_ADV_OPT_CONN;
constexpr uint8_t kAdvertisingFlags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
constexpr uint8_t kSmpUuid[] = { SMP_BT_SVC_UUID_VAL };

constexpr uint16_t kAdvertisingIntervalMin = 400;
constexpr uint16_t kAdvertisingIntervalMax = 500;

static constexpr size_t kAdvertisingItemsSize = 2;
static constexpr size_t kServiceItemsSize = 1;
std::array<bt_data, kAdvertisingItemsSize> mAdvertisingItems{};
std::array<bt_data, kServiceItemsSize> mServiceItems{};

static void AuthPasskeyDisplay(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	ALIRO_LOG_INF("Passkey for %s: %06u\n", addr, passkey);
}

static void AuthCancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	ALIRO_LOG_DBG("Pairing cancelled: %s\n", addr);
}

enum mgmt_cb_return UploadConfirmHandler(uint32_t, enum mgmt_cb_return, int32_t *rc, uint16_t *, bool *, void *data,
					 size_t)
{
	ALIRO_LOG_DBG("DFU over SMP chunk received");
	return MGMT_CB_OK;
}

enum mgmt_cb_return CommandHandler(uint32_t, enum mgmt_cb_return, int32_t *, uint16_t *, bool *, void *, size_t)
{
	return MGMT_CB_OK;
}

enum mgmt_cb_return DfuStoppedHandler(uint32_t, enum mgmt_cb_return, int32_t *, uint16_t *, bool *, void *, size_t)
{
	return MGMT_CB_OK;
}

mgmt_callback sUploadCallback = {
	.callback = UploadConfirmHandler,
	.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK,
};

mgmt_callback sCommandCallback = {
	.callback = CommandHandler,
	.event_id = (MGMT_EVT_OP_CMD_RECV | MGMT_EVT_OP_CMD_DONE),
};

mgmt_callback sDfuStopped = {
	.callback = DfuStoppedHandler,
	.event_id = (MGMT_EVT_OP_IMG_MGMT_DFU_STOPPED | MGMT_EVT_OP_IMG_MGMT_DFU_PENDING),
};

} // namespace

AliroError BleSmpManager::Init(const Callbacks &callbacks)
{
	mCallbacks = callbacks;
	static struct bt_conn_auth_cb conn_auth_callbacks = {
		.passkey_display = AuthPasskeyDisplay,
		.cancel = AuthCancel,
	};

	mgmt_callback_register(&sUploadCallback);
	mgmt_callback_register(&sCommandCallback);
	mgmt_callback_register(&sDfuStopped);

	int err = bt_enable(NULL);
	VerifyOrReturnStatus(err == 0, ALIRO_ERROR_INTERNAL, ALIRO_LOG_ERR("Bluetooth init failed (err %d)", err));

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	VerifyOrReturnStatus(err == 0, ALIRO_ERROR_INTERNAL,
			     ALIRO_LOG_ERR("Failed to register BLE authorization callbacks (err %d)", err));

	ALIRO_LOG_DBG("DFU SMP module initialized");
	return ALIRO_NO_ERROR;
}

static void StartDfuSmpAdvHandler(struct k_work *work)
{
	mAdvertisingItems[0] = BT_DATA(BT_DATA_FLAGS, &kAdvertisingFlags, sizeof(kAdvertisingFlags));
	mAdvertisingItems[1] = BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, strlen(CONFIG_BT_DEVICE_NAME));
	mServiceItems[0] = BT_DATA(BT_DATA_UUID128_ALL, kSmpUuid, sizeof(kSmpUuid));

	bt_le_adv_param adv_param =
		BT_LE_ADV_PARAM_INIT(kAdvertisingOptions, kAdvertisingIntervalMin, kAdvertisingIntervalMax, NULL);

	int err = bt_le_adv_start(&adv_param, mAdvertisingItems.data(), mAdvertisingItems.size(), mServiceItems.data(),
				  mServiceItems.size());
	if (err) {
		ALIRO_LOG_ERR("DFU SMP advertising failed to start (rc %d)", err);
	} else {
		BleSmpManager::Instance().SetAdvStatus(BleSmpManager::ADV_STARTED);
		ALIRO_LOG_DBG("DFU SMP advertising started");
	}
}

K_WORK_DELAYABLE_DEFINE(DfuSmpStartWork, StartDfuSmpAdvHandler);

static void StopDfuSmpAdvHandler(struct k_work *work)
{
	int err = bt_le_adv_stop();
	if (err) {
		ALIRO_LOG_ERR("DFU SMP advertising failed to stop (rc %d)", err);
	} else {
		BleSmpManager::Instance().SetAdvStatus(BleSmpManager::ADV_STOPPED);
		ALIRO_LOG_DBG("DFU SMP advertising stopped");
	}
}
K_WORK_DELAYABLE_DEFINE(DfuSmpStopWork, StopDfuSmpAdvHandler);

void BleSmpManager::DfuSmpStartAdv()
{
	if (mIsAdvStarted) {
		ALIRO_LOG_DBG("DFU SMP advertising already started");
		return;
	}
	k_work_submit(&DfuSmpStartWork.work);
}

void BleSmpManager::DfuSmpStopAdv()
{
	if (!mIsAdvStarted) {
		ALIRO_LOG_DBG("DFU SMP advertising already stopped");
		return;
	}
	k_work_submit(&DfuSmpStopWork.work);
}

void BleSmpManager::ConfirmNewImage()
{
	/* Check if the image is run in the REVERT mode and eventually */
	/* confirm it to prevent reverting on the next boot. */
	VerifyOrReturn(mcuboot_swap_type() == BOOT_SWAP_TYPE_REVERT);

	if (boot_write_img_confirmed()) {
		ALIRO_LOG_ERR("Failed to confirm firmware image, it will be reverted on the next boot");
	} else {
		ALIRO_LOG_DBG("New firmware image confirmed");
	}
}
