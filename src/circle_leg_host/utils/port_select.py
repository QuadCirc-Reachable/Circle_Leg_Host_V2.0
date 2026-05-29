"""串口选择: 与 V1.0 main.py 等价行为。"""

from typing import Optional

from serial.tools import list_ports


_KEYWORDS = [
    "USB", "UART", "CH340", "CP210", "FTDI", "ARDUINO",
    "SILICON", "STM", "SERIAL",
]


def list_serial_ports():
    return list(list_ports.comports())


def port_exists(port_name: Optional[str]) -> bool:
    if not port_name:
        return False
    try:
        return any(p.device == port_name for p in list_ports.comports())
    except Exception:
        return False


def select_serial_port(preferred_port: Optional[str] = None) -> Optional[str]:
    """按 V1.0 main.py 中 select_serial_port 的策略选择串口。"""
    ports = list(list_ports.comports())
    if not ports:
        return None

    if preferred_port:
        for p in ports:
            if p.device == preferred_port:
                return p.device

    def score(p):
        text = f"{p.description} {p.manufacturer} {p.hwid}".upper()
        return sum(1 for kw in _KEYWORDS if kw in text)

    return sorted(ports, key=score, reverse=True)[0].device
