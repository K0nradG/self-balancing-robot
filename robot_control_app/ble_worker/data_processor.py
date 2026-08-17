# Copyright 2026 Filip Dymczyk and Konrad Grucel

# Class for handling processing of the data received through NUS.

import csv
import os
import re
import time
from dataclasses import dataclass, field


@dataclass
class ParsedData:
    telemetry: dict[str, float] = field(default_factory=dict)
    battery_mv: float | None = None
    pid_params: dict | None = None


class DataProcessor:

    def __init__(self, log_dir: str = "robot_data_logs"):
        self.log_dir = log_dir
        self.csv_file = None
        self.csv_writer = None

    def process(self, text: str, auto_record: bool = False) -> ParsedData:
        """Parses raw text notification and handles optional CSV recording."""
        parsed = ParsedData()

        parsed.telemetry = self.__parse_telemetry(text)
        if parsed.telemetry is not None and auto_record:
            self.__record_telemetry(parsed.telemetry)

        bat_mv = self.__parse_battery(text)
        if bat_mv is not None:
            parsed.battery_mv = bat_mv

        parsed.pid_params = self.__parse_pid_parameters(text)

        return parsed

    def close_recording(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

    def __parse_telemetry(self, text: str) -> dict[str, float] | None:
        matches = re.findall(
            r"([a-zA-Z0-9_]+):\s*(-?\d+(?:\.\d+)?)", text
        )
        if matches:
            return {key: float(val) for key, val in matches}
        return None

    def __parse_battery(self, text: str) -> float | None:
        match = re.search(
            r"bat\s+lvl\s+mv\s*:?\s*(\d+)", text, re.IGNORECASE
        )
        if match:
            return float(match.group(1))
        return None

    def __parse_pid_parameters(self, text: str) -> dict | None:
        pid_pattern = (
            r"Kp([0-9.-]+)_Ki([0-9.-]+)_Kd([0-9.-]+)_"
            r"Kp([0-9.-]+)_Ki([0-9.-]+)_Kd([0-9.-]+)_"
            r"Kp([0-9.-]+)_Ki([0-9.-]+)_Kd([0-9.-]+)_"
            r"Kp([0-9.-]+)_Ki([0-9.-]+)_Kd([0-9.-]+)_"
            r"Kp([0-9.-]+)_Ki([0-9.-]+)_Kd([0-9.-]+)"
        )
        match = re.search(pid_pattern, text)
        if match:
            vals = [float(x) for x in match.groups()]
            controllers = [
                "distance",
                "linear_speed",
                "balance",
                "rotate",
                "wheel_speed",
            ]
            return {
                ctrl_key: {
                    "kp": vals[i * 3],
                    "ki": vals[i * 3 + 1],
                    "kd": vals[i * 3 + 2],
                }
                for i, ctrl_key in enumerate(controllers)
            }
        return None

    def __record_telemetry(self, data: dict):
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        if not self.csv_file:
            filename = time.strftime(
                f"{self.log_dir}/data_%Y%m%d_%H%M%S.csv"
            )
            self.csv_file = open(filename, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            headers = ["timestamp"] + list(data.keys())
            self.csv_writer.writerow(headers)

        row = [time.time()] + list(data.values())
        self.csv_writer.writerow(row)
        self.csv_file.flush()