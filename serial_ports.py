"""Select the Uno by USB identity rather than its changing device path."""
from serial.tools import list_ports

UNO_USB_IDS = {(0x2341, 0x0043), (0x2341, 0x0001), (0x2341, 0x0243),
               (0x2A03, 0x0043), (0x1A86, 0x7523)}


def find_uno(configured_port, serial_number=None, ports=None):
    ports = list(list_ports.comports() if ports is None else ports)
    # macOS can expose cu/tty aliases for one device; prefer cu.
    ports = [p for p in ports if not p.device.startswith('/dev/tty.')]
    if serial_number:
        candidates = [p for p in ports if p.serial_number == serial_number
                      and (p.vid, p.pid) in UNO_USB_IDS]
    else:
        candidates = [p for p in ports if (p.vid, p.pid) in UNO_USB_IDS]
        configured = [p for p in ports if p.device == configured_port]
        if configured and (configured[0].vid, configured[0].pid) in UNO_USB_IDS:
            return configured[0]
    if len(candidates) == 1:
        return candidates[0]
    if len(candidates) > 1:
        raise RuntimeError('Multiple Uno-compatible devices found; set uno_serial_number to select yours')
    raise RuntimeError('Uno not found. Connect its USB cable, then click Tare.')
