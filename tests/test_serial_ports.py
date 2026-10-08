import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from serial_ports import find_uno


def port(device, serial='UNO', vid=0x2341, pid=0x0043):
    return SimpleNamespace(device=device, serial_number=serial, vid=vid, pid=pid)


class DetectionTests(unittest.TestCase):
    def test_changed_path_selected_by_identity(self):
        p=port('/dev/cu.usbmodem31301')
        self.assertIs(find_uno('/dev/cu.usbmodem1201','UNO',[p]),p)

    def test_unrelated_modem_and_motor_excluded(self):
        p=port('/dev/cu.usbmodem31301')
        ports=[port('/dev/cu.usbmodemBillBoard','OTHER',0x291A,0x8355),
               port('/dev/cu.usbserial-FTB8HQU7','FTB8HQU7',0x0403,0x6014),p]
        self.assertIs(find_uno('/dev/cu.old',ports=ports),p)

    def test_ambiguity_and_missing_identity_fail(self):
        ports=[port('/dev/cu.one','A'),port('/dev/cu.two','B')]
        with self.assertRaisesRegex(RuntimeError,'Multiple'):find_uno('/dev/cu.old',ports=ports)
        with self.assertRaisesRegex(RuntimeError,'not found'):find_uno('/dev/cu.old','MISSING',ports)

    def test_configured_board_and_macos_alias(self):
        p=port('/dev/cu.one','A')
        ports=[p,port('/dev/tty.one','A'),port('/dev/cu.two','B')]
        self.assertIs(find_uno('/dev/cu.one',ports=ports),p)
        self.assertIs(find_uno('/dev/cu.old','A',ports),p)
