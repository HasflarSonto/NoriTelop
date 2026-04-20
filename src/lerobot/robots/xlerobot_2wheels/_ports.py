"""Port resolution and serial-over-TCP bridge support for XLeRobot.

Env vars (first match wins per bus):
    XLEROBOT_BUS1_PORT  - explicit override for bus 1 (e.g. "COM5", "/dev/ttyUSB0",
                          "socket://xlerobot.local:3001")
    XLEROBOT_BUS2_PORT  - explicit override for bus 2
    XLEROBOT_BRIDGE     - hostname or IP of an RPi4 running ser2net; expands to
                          socket://{host}:3001 for bus1 and socket://{host}:3002 for bus2

Platform defaults (when no env var is set):
    Windows -> COM5 / COM6
    Linux   -> /dev/xlerobot_bus1 / /dev/xlerobot_bus2 (udev symlinks set up on the RPi4)

Side effect on import: monkey-patches scservo_sdk.port_handler.PortHandler.setupPort
to use serial.serial_for_url so `socket://host:port` URLs work transparently.
The patch is idempotent.
"""
import os
import platform

import serial

BRIDGE_BUS1_TCP_PORT = 3001
BRIDGE_BUS2_TCP_PORT = 3002


def _platform_default(bus_num: int) -> str:
    if platform.system() == "Windows":
        return "COM5" if bus_num == 1 else "COM6"
    return "/dev/xlerobot_bus1" if bus_num == 1 else "/dev/xlerobot_bus2"


def _bridge_url(bus_num: int) -> str | None:
    host = os.environ.get("XLEROBOT_BRIDGE")
    if not host:
        return None
    port = BRIDGE_BUS1_TCP_PORT if bus_num == 1 else BRIDGE_BUS2_TCP_PORT
    return f"socket://{host}:{port}"


def get_bus1_port() -> str:
    return os.environ.get("XLEROBOT_BUS1_PORT") or _bridge_url(1) or _platform_default(1)


def get_bus2_port() -> str:
    return os.environ.get("XLEROBOT_BUS2_PORT") or _bridge_url(2) or _platform_default(2)


def _install_socket_url_patch() -> None:
    try:
        import scservo_sdk.port_handler as ph
    except ImportError:
        return

    if getattr(ph.PortHandler, "_xlerobot_socket_patched", False):
        return

    def setupPort(self, cflag_baud):  # noqa: N802 - matches upstream API
        if self.is_open:
            self.closePort()
        self.ser = serial.serial_for_url(
            self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            timeout=0,
        )
        self.is_open = True
        self.ser.reset_input_buffer()
        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0
        return True

    ph.PortHandler.setupPort = setupPort
    ph.PortHandler._xlerobot_socket_patched = True


_install_socket_url_patch()
