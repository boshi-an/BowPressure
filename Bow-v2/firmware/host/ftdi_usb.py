"""Read an FT232R directly, keeping USB IN transfers queued during host stalls."""
from collections import deque
from contextlib import ExitStack
import ctypes


FTDI_USB_VID = 0x0403
FT232R_USB_PID = 0x6001
# The FT232R uses special divisor 0 for 3 Mbaud and divisor 2 for 1.5 Mbaud.
_BAUD_DIVISOR = {1_500_000: 2, 3_000_000: 0}
SUPPORTED_BAUD_RATES = tuple(_BAUD_DIVISOR)

_LINE_ERRORS = (
    (0x02, 'ftdi_overrun_packets'),
    (0x04, 'ftdi_parity_error_packets'),
    (0x08, 'ftdi_framing_error_packets'),
    (0x10, 'ftdi_break_packets'),
    (0x80, 'ftdi_fifo_error_packets'),
)
FTDI_STATUS_FIELDS = tuple(name for _, name in _LINE_ERRORS)

_PACKET_BYTES = 64
_TRANSFER_BYTES = 4096
_TRANSFER_COUNT = 16
_READ_TIMEOUT = .05


def _load_usb():
    try:
        import libusb_package
        import usb1

        library = libusb_package.get_library_path()
        usb1.loadLibrary(ctypes.CDLL(str(library)) if library is not None else None)
    except (ImportError, OSError) as exc:
        raise OSError('Direct FTDI USB requires libusb1 and libusb-package. '
                      'Install the packages in requirements.txt.') from exc
    return usb1


class FtdiUsb:
    """Expose ordered UART payloads and FTDI status from its bulk IN endpoint."""

    def __init__(self, serial_number, baud):
        if baud not in _BAUD_DIVISOR:
            raise ValueError(f'Unsupported FT232R direct USB baud rate: {baud}')
        self.serial_number = serial_number
        self.baud = baud
        self._resources = None
        self._closing = False
        self._ordered = deque()
        self._completed = set()
        self._error = None
        self._line_counts = (0,) * len(_LINE_ERRORS)

    def __enter__(self):
        if self._resources is not None:
            raise OSError('The FT232R USB transport is already open')
        self._closing = False
        self._error = None
        self._line_counts = (0,) * len(_LINE_ERRORS)
        self._usb = _load_usb()
        try:
            with ExitStack() as stack:
                self._context = stack.enter_context(self._usb.USBContext())
                matches = []
                for device in self._context.getDeviceList(skip_on_error=True):
                    if (device.getVendorID(), device.getProductID()) != (FTDI_USB_VID, FT232R_USB_PID):
                        continue
                    try:
                        if device.getSerialNumber() == self.serial_number:
                            matches.append(device)
                    except self._usb.USBError:
                        continue
                if not matches:
                    raise OSError(f'FT232R with serial number {self.serial_number} was not found')
                if len(matches) != 1:
                    raise OSError(f'Multiple FT232R adapters have serial number {self.serial_number}')
                handle = matches[0].open()
                # libusb1 exposes close as a weak finalizer. Retain the handle
                # through cleanup so its queued USB reads stay alive.
                stack.callback(lambda: handle.close())
                handle.claimInterface(0)

                # Set the baud divisor, 8N1, and no flow control.
                for request, value in ((3, _BAUD_DIVISOR[self.baud]), (4, 8), (2, 0)):
                    handle.controlWrite(0x40, request, value, 0, b'', timeout=1000)
                # Purge the UART-to-host FIFO before starting this connection.
                # It can still hold bow data from before the last close.
                handle.controlWrite(0x40, 0, 2, 0, b'', timeout=1000)
                for _ in range(_TRANSFER_COUNT):
                    transfer = handle.getTransfer()
                    transfer.setBulk(0x81, _TRANSFER_BYTES, self._on_complete, timeout=1000)
                    self._submit(transfer)
                self._resources = stack.pop_all()
        except self._usb.USBError as exc:
            self._ordered.clear()
            self._completed.clear()
            raise OSError(f'Could not open FT232R {self.serial_number} via direct USB: {exc}') from exc
        return self

    def _submit(self, transfer):
        transfer.submit()
        self._ordered.append(transfer)

    def _on_complete(self, transfer):
        if not self._closing:
            self._completed.add(transfer)

    def read_chunk(self):
        """Return available payloads, or empty bytes after waiting at most 50 ms."""
        if self._resources is None:
            raise OSError('The FT232R USB transport is closed')
        if self._error is not None:
            raise self._error
        payload = bytearray()
        counts = list(self._line_counts)
        try:
            if self._ordered[0] not in self._completed:
                try:
                    self._context.handleEventsTimeout(_READ_TIMEOUT)
                except self._usb.USBErrorInterrupted:
                    return b''
            while self._ordered and self._ordered[0] in self._completed:
                transfer = self._ordered.popleft()
                self._completed.remove(transfer)
                status = transfer.getStatus()
                size = transfer.getActualLength()
                if status != self._usb.TRANSFER_COMPLETED:
                    raise OSError(f'FT232R USB bulk read failed (status {status}, {size} bytes)')
                data = bytes(transfer.getBuffer()[:size])
                for i in range(0, size, _PACKET_BYTES):
                    if i + 1 >= size:
                        raise OSError('Incomplete FT232R status header in a USB packet')
                    line_status = data[i + 1]
                    if line_status & 0x9e:
                        for index, (bit, _) in enumerate(_LINE_ERRORS):
                            counts[index] += bool(line_status & bit)
                    payload.extend(data[i + 2:i + _PACKET_BYTES])
                self._submit(transfer)
        except self._usb.USBError as exc:
            self._error = OSError(f'FT232R USB receive failed: {exc}')
        except OSError as exc:
            self._error = exc
        finally:
            self._line_counts = tuple(counts)
        if not payload and self._error is not None:
            raise self._error
        return bytes(payload)

    def status(self):
        """Return cumulative counts of USB packets with UART receive errors."""
        return dict(zip(FTDI_STATUS_FIELDS, self._line_counts))

    def __exit__(self, *exc_info):
        self._closing = True
        try:
            self._resources.__exit__(*exc_info)
        except self._usb.USBError as exc:
            if exc_info[0] is None:
                raise OSError(f'Could not close FT232R USB transport: {exc}') from exc
        finally:
            self._ordered.clear()
            self._completed.clear()
            self._resources = None
