// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <zephyr/kernel.h>

void
reschedule_work(k_work_delayable* dwork, k_timeout_t delay, char const* desc);