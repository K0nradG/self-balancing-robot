#pragma once

#include "aliro/errors.h"
#include <zephyr/bluetooth/bluetooth.h>

class BleSmpManager {
public:
	static constexpr bool ADV_STARTED = true;
	static constexpr bool ADV_STOPPED = false;

	static BleSmpManager &Instance()
	{
		static BleSmpManager instance;
		return instance;
	}

	AliroError Init();
	void DfuSmpStartAdv();
	void DfuSmpStopAdv();
	void ConfirmNewImage();

	void SetAdvStatus(bool status) { mIsAdvStarted = status; }
	bool GetAdvStatus() const { return mIsAdvStarted; }

private:
	BleSmpManager() = default;
	BleSmpManager(const BleSmpManager &) = delete;
	BleSmpManager &operator=(const BleSmpManager &) = delete;
	BleSmpManager(BleSmpManager &&) = delete;
	BleSmpManager &operator=(BleSmpManager &&) = delete;

	bool mIsAdvStarted{ false };
};
