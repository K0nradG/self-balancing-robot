#!/usr/bin/env python3
import asyncio
import logging
import sys
from bleak import BleakClient

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("NUS")

# UUID NUS
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # host -> device
NUS_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # device -> host (notify)


class NUSClient:
    def __init__(self, address):
        self.address = address
        self.client = BleakClient(address)
        self.connected = False
        self._notify_active = False
        self.on_data = None
        self.device_name = None
        self.on_trajectory_ack = None 

    async def connect(self):
        try:
            logger.info("Łączenie z %s...", self.address)
            await self.client.connect()
            self.connected = self.client.is_connected if isinstance(self.client.is_connected, bool) else await self.client.is_connected()
            if self.connected:
                try:
                    self.device_name = await self.client.read_gatt_char("00002a00-0000-1000-8000-00805f9b34fb")
                    self.device_name = self.device_name.decode("utf-8", errors="ignore")
                    logger.info("Połączono z %s (device name: %s)", self.address, self.device_name)
                except Exception:
                    self.device_name = None
                    logger.warning("Nie udało się pobrać nazwy urządzenia.")
            else:
                logger.error("Nie udało się połączyć.")
        except Exception as e:
            logger.exception("Błąd przy łączeniu: %s", e)

    async def disconnect(self):
        try:
            if self._notify_active:
                await self.notifications_off()
            await self.client.disconnect()
            self.connected = False
            logger.info("Rozłączono")
        except Exception:
            pass

    async def send(self, data: str):
        if not self.connected:
            logger.warning("Niepołączony klient")
            return
        try:
            await self.client.write_gatt_char(NUS_RX_CHAR_UUID, data.encode("utf-8"))
            logger.info("Dane wysłane: %r", data)
        except Exception as e:
            logger.exception("Błąd wysyłania: %s", e)

    async def notifications_on(self):
        if not self.connected:
            logger.warning("Niepołączony klient")
            return
        if self._notify_active:
            logger.info("Notify już włączone")
            return
        try:
            await self.client.start_notify(NUS_TX_CHAR_UUID, self.notify_handler)
            self._notify_active = True
            logger.info("Notify włączone")
        except Exception as e:
            logger.exception("Błąd przy włączaniu notify: %s", e)

    async def notifications_off(self):
        if not self.connected:
            logger.warning("Niepołączony klient")
            return
        if not self._notify_active:
            logger.info("Notify już wyłączone")
            return
        try:
            await self.client.stop_notify(NUS_TX_CHAR_UUID)
            self._notify_active = False
            logger.info("Notify wyłączone")
        except Exception as e:
            logger.exception("Błąd przy wyłączaniu notify: %s", e)

    def notify_handler(self,sender_handle, data: bytearray):
        try:
            text = data.decode("utf-8", errors="replace")
        except Exception:
            text = None
        hex_data = " ".join(f"{b:02x}" for b in data)
        logger.info("NOTIFY [%s]: hex=%s text=%r", sender_handle, hex_data, text)
        print(f"NOTIFY: {text}")

        if not text:
            return

        if self.on_data:
            try:
                self.on_data(text)
            except Exception as e:
                logger.warning("Error NUS on_data callback: %s", e)

        # special callback for wathcing trajectory ack in latest logs
        if "tc" in text.lower():
            if self.on_trajectory_ack:
                try:
                    self.on_trajectory_ack(text)
                except Exception as e:
                    logger.warning("Error NUS on_trajectory_ack callback: %s", e)

    
    def get_status(self) -> dict:
        return {
            "connected": self.connected,
            "notify_active": self._notify_active,
            "address": self.address,
            "device_name": self.device_name or "Unknown device"
        }

# Asynchroniczny input (non-blocking)
async def async_input(prompt: str = "") -> str:
    print(prompt, end="", flush=True)
    loop = asyncio.get_event_loop()
    return (await loop.run_in_executor(None, sys.stdin.readline)).rstrip("\n")


async def interactive_loop(client: NUSClient):
    print("Tryb interaktywny. Komendy:")
    print(" send \"data\"")
    print(" notifications on")
    print(" notifications off")
    print(" disconnect / exit")
    while True:
        cmd = await async_input("> ")
        cmd = cmd.strip()
        if cmd.startswith("send "):
            data = cmd[5:].strip('"')
            await client.send(data)
        elif cmd == "notifications on":
            await client.notifications_on()
        elif cmd == "notifications off":
            await client.notifications_off()
        elif cmd in ("disconnect", "exit"):
            await client.disconnect()
            break
        else:
            print("Nieznana komenda. Dostępne: send \"data\", notifications on, notifications off, disconnect")


async def main():
    import argparse
    parser = argparse.ArgumentParser(description="Interaktywny klient NUS")
    parser.add_argument("--addr", "-a", required=True, help="Adres MAC urządzenia BLE")
    args = parser.parse_args()

    client = NUSClient(args.addr)
    await client.connect()
    if client.connected:
        await interactive_loop(client)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nZakończono")

