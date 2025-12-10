#!/usr/bin/env python3
import asyncio
import re
import sys
import logging
from gpiozero import LED

from nus_service import NUSClient


logger = logging.getLogger("BatteryMonitor")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

BATTERY_REGEX = re.compile(r"bat\s+lvl\s+mv\s+(\d+)", re.IGNORECASE)
LOW_BATTERY_THRESHOLD = 7000
BLINK_INTERVAL = 0.2
GPIO_PIN = 4


class BatterySupervisor:
    def __init__(self, addr):
        self.addr = addr
        self.client = NUSClient(addr)

        self.led = LED(GPIO_PIN)
        self.blink_task = None
        self.current_bat_mv = None

        self.client.on_data = self._handle_nus_data

    async def connect_until_success(self):
        """Próbuje łączyć aż się powiedzie."""
        while True:
            logger.info("Próba połączenia...")
            await self.client.connect()
            if self.client.connected:
                logger.info("Połączono pomyślnie.")
                return
            logger.info("Nieudane — retry za 2 sek.")
            await asyncio.sleep(2)

    async def enable_notifications(self):
        while not self.client._notify_active:
            try:
                await self.client.notifications_on()
                await asyncio.sleep(1)
            except Exception as e:
                logger.error(f"Błąd notify: {e}")
                await asyncio.sleep(2)

    def _handle_nus_data(self, text: str):
        """Callback wywoływany bezpośrednio przez NUSClient."""
        m = BATTERY_REGEX.search(text)
        if not m:
            return

        mv = int(m.group(1))
        self.current_bat_mv = mv
        logger.info(f"Poziom baterii: {mv} mV")

        if mv < LOW_BATTERY_THRESHOLD:
            self._start_blinking()
        else:
            self._stop_blinking()

    def _start_blinking(self):
        if self.blink_task is None:
            logger.warning("Bateria niska → miganie GPIO4")
            self.blink_task = asyncio.create_task(self._blink_loop())

    def _stop_blinking(self):
        if self.blink_task:
            logger.info("Bateria OK → stop miganie")
            self.blink_task.cancel()
            self.blink_task = None
            self.led.off()

    async def _blink_loop(self):
        try:
            while True:
                self.led.toggle()
                await asyncio.sleep(BLINK_INTERVAL)
        except asyncio.CancelledError:
            self.led.off()

    async def run(self):
        await self.connect_until_success()
        await self.enable_notifications()

        logger.info("Monitoring baterii uruchomiony.")
        while True:
            await asyncio.sleep(1)

async def main():
    if len(sys.argv) != 2:
        print("Użycie: python3 monitor_battery_with_nus.py AA:BB:CC:DD:EE:FF")
        sys.exit(1)

    addr = sys.argv[1]

    supervisor = BatterySupervisor(addr)
    await supervisor.run()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nZatrzymano.")
