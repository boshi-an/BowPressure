"""Select the receiver for the serial port that the user picked."""
from contextlib import closing, contextmanager
import os
import sys

import serial
from serial.tools import list_ports

from ftdi_usb import FT232R_USB_PID, FTDI_USB_VID, FtdiUsb, SUPPORTED_BAUD_RATES


TRANSPORTS = ('auto', 'ftdi-usb', 'serial')


def _ftdi_serial_number(port):
    path = os.path.realpath(port).replace('/dev/tty.', '/dev/cu.', 1)
    for device in list_ports.comports():
        candidate = os.path.realpath(device.device).replace('/dev/tty.', '/dev/cu.', 1)
        if candidate == path and (device.vid, device.pid) == (FTDI_USB_VID, FT232R_USB_PID):
            return device.serial_number
    return None


@contextmanager
def open_transport(port, baud, transport='auto'):
    if transport not in TRANSPORTS:
        raise ValueError(f'Unknown transport: {transport}')
    device_serial = None
    if transport != 'serial' and sys.platform == 'darwin' and baud in SUPPORTED_BAUD_RATES:
        device_serial = _ftdi_serial_number(port)
    if device_serial:
        with FtdiUsb(device_serial, baud) as source:
            source.transport_name = 'ftdi-usb'
            yield source
        return
    if transport == 'ftdi-usb':
        raise OSError('Direct FTDI USB requires a listed macOS FT232R port (0403:6001) '
                      'with a USB serial number at 1,500,000 or 3,000,000 baud. '
                      'Select a supported device or use --transport serial.')
    with closing(serial.Serial(port=None, baudrate=baud, timeout=.05)) as source:
        source.dtr = source.rts = False
        source.port = port
        source.transport_name = 'serial'
        source.open()
        yield source
