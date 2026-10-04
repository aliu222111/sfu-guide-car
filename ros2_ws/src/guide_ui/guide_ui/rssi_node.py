"""Bluetooth signal strength of the student's phone (spec section 9, phase 6).

Scans BLE advertisements with bleak and publishes ``guide/phone_rssi``
(Float32, dBm) for advertisements whose name or address matches. Phones
randomise their BLE address, so the student's phone should advertise a fixed
name: on Android, nRF Connect's advertiser; on iPhone, LightBlue's virtual
peripheral. Set ``device_name`` to that name.
"""

from __future__ import annotations

import asyncio
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RssiNode(Node):
    def __init__(self):
        super().__init__("phone_rssi")
        self.name = self.declare_parameter("device_name", "SFU-GUIDE").value
        self.address = (self.declare_parameter("device_address", "").value or "").upper()
        self.alpha = self.declare_parameter("smoothing", 0.3).value
        self.pub = self.create_publisher(Float32, "guide/phone_rssi", 10)
        self.value = None
        self.last_seen = time.monotonic()
        self.silence = self.declare_parameter("silence_timeout", 2.0).value
        # Out of range means no advertisements at all, so silence must count as weak.
        self.create_timer(1.0, self.check_silence)
        threading.Thread(target=lambda: asyncio.run(self.scan()), daemon=True).start()

    def matches(self, device, adv) -> bool:
        if self.address and device.address.upper() == self.address:
            return True
        return bool(self.name) and (adv.local_name == self.name or device.name == self.name)

    def check_silence(self):
        if time.monotonic() - self.last_seen > self.silence:
            self.value = None
            self.pub.publish(Float32(data=-127.0))

    async def scan(self):
        from bleak import BleakScanner

        def on_adv(device, adv):
            if not self.matches(device, adv):
                return
            # Exponential smoothing: single BLE readings jump by 10 dB.
            self.last_seen = time.monotonic()
            self.value = adv.rssi if self.value is None else (1 - self.alpha) * self.value + self.alpha * adv.rssi
            self.pub.publish(Float32(data=float(self.value)))

        async with BleakScanner(on_adv):
            while rclpy.ok():
                await asyncio.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = RssiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
