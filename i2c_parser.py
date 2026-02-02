# i2c_parser.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Callable, List, Tuple

from smbus2 import SMBus, i2c_msg
import time

from nmea_decoder import NMEADecoder, EpochFrame
from ucm_calc import UCMConverter, extract_latlon_from_nmea_message


@dataclass
class UCMResult:
    epoch_id: int
    t_recv_s: float
    x_m: float
    y_m: float
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None


class I2CGNSSParser:
    """
    Read ZED-F9R DDC (I2C) stream, decode NMEA, epoch by GGA,
    and output UCM local XY.
    """

    def __init__(
        self,
        *,
        i2c_bus: int = 1,
        addr: int = 0x42,
        max_chunk: int = 32,
        fixed_zone: int = 52,
        verify_checksum: bool = False,
        avail_swap: bool = True,     # True면 avail = (lsb<<8)|msb
        drop_ff_padding: bool = False,
        sleep_s: float = 0.005,
        on_ucm: Optional[Callable[[UCMResult], None]] = None,
        on_epoch_end: Optional[Callable[[object], None]] = None,
    ):
        self.i2c_bus = i2c_bus
        self.addr = addr
        self.max_chunk = max_chunk
        self.drop_ff_padding = drop_ff_padding
        self.sleep_s = sleep_s

        self.decoder = NMEADecoder(verify_checksum=verify_checksum)
        self.decoder.begin_epoch(epoch_id=1)

        self.ucm = UCMConverter(fixed_zone=fixed_zone)
        self._origin_set = False

        self.avail_swap = avail_swap
        self.on_ucm = on_ucm
        self.on_epoch_end = on_epoch_end

        self._line_buf = bytearray()
        self._pending_ucm: List[UCMResult] = []

        self._bus: Optional[SMBus] = None

        self.last_succesful_frame: Optional[EpochFrame] = None
        self.last_succesful_ucm = [0, 0]

    # -------- lifecycle --------

    def open(self) -> None:
        if self._bus is None:
            self._bus = SMBus(self.i2c_bus)

    def close(self) -> None:
        if self._bus is not None:
            try:
                self._bus.close()
            finally:
                self._bus = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    # -------- low-level I2C --------


    def read_available(self) -> int:
        """
        Read 2 bytes from 0xFD (avail LSB/MSB) using combined transaction.
        """
        if self._bus is None:
            raise RuntimeError("I2C bus not opened. Call open() first.")

        w = i2c_msg.write(self.addr, [0xFD])
        r = i2c_msg.read(self.addr, 2)
        self._bus.i2c_rdwr(w, r)
        b0, b1 = list(r)

        # default: b0=LSB, b1=MSB  -> avail = (MSB<<8)|LSB
        # swap:    b0=MSB, b1=LSB  -> avail = (MSB<<8)|LSB = (b0<<8)|b1
        if self.avail_swap:
            msb, lsb = b0, b1
        else:
            lsb, msb = b0, b1

        return (msb << 8) | lsb

    def read_stream(self, n: int) -> bytes:
        if self._bus is None:
            raise RuntimeError("I2C bus not opened. Call open() first.")
        data = bytes(self._bus.read_i2c_block_data(self.addr, 0xFF, n))
        if self.drop_ff_padding:
            data = bytes(b for b in data if b != 0xFF)
        return data

    # -------- NMEA helpers --------

    @staticmethod
    def is_gga_sentence(line: str) -> bool:
        line = line.strip()
        if not line.startswith("$"):
            return False
        body = line[1:].split("*", 1)[0]
        head = body.split(",", 1)[0]  # e.g., "GNGGA"
        return len(head) >= 5 and head[2:5] == "GGA"

    def _frame_to_ucm(self, frame) -> Optional[UCMResult]:
        """
        Take epoch frame, pick a representative fix (GGA[0]),
        convert to local XY using UTM zone fixed_zone.
        """
        if not frame or not frame.GGA:
            return None

        gga = frame.GGA[0]
        latlon = extract_latlon_from_nmea_message(gga)
        if latlon is None:
            return None

        lat, lon = latlon

        if not self._origin_set:
            self.ucm.set_origin_from_latlon(lat, lon)
            self._origin_set = True

        x_m, y_m = self.ucm.to_local_xy(lat, lon)
        return UCMResult(
            epoch_id=frame.epoch_id,
            t_recv_s=getattr(frame, "t_recv_s", time.time()),
            x_m=x_m,
            y_m=y_m,
            lat_deg=lat,
            lon_deg=lon,
        )

    # -------- public API --------

    def poll(self) -> int:
        """
        Non-blocking-ish: read what is available now, process lines,
        and possibly produce UCM results.
        Returns number of bytes consumed from stream (approx).
        """
        if self._bus is None:
            raise RuntimeError("I2C bus not opened. Call open() first.")

        consumed = 0

        avail = self.read_available()
        if avail <= 0:
            if self.sleep_s > 0:
                time.sleep(self.sleep_s)
            return 0

        n = min(avail, self.max_chunk)
        data = self.read_stream(n)
        if not data:
            if self.sleep_s > 0:
                time.sleep(self.sleep_s)
            return 0

        consumed = len(data)
        
        for b in data:
            self._line_buf.append(b)
            if b == 0x0A:  # '\n'
                line = self._line_buf.decode("ascii", errors="replace")
                self._line_buf.clear()
                
                # epoch rollover by GGA
                if self.is_gga_sentence(line):
                    prev = self.decoder.end_epoch()
                    if prev is not None:
                        # optional raw callback
                        if self.on_epoch_end is not None:
                            self.on_epoch_end(prev)
                            self.last_succesful_frame = prev
                        # generate UCM result from finished epoch
                        u = self._frame_to_ucm(prev)
                        if u is not None:
                            if self.on_ucm is not None:
                                ucm_result = self.on_ucm(u)
                                self.last_succesful_ucm(ucm_result.x_m, ucm_result.y_m)
                            else:
                                self._pending_ucm.append(u)

                    # start new epoch (this GGA belongs to new epoch)
                    self.decoder.begin_epoch()

                # store the line into current epoch
                self.decoder.feed_line(line)

        if self.sleep_s > 0:
            time.sleep(self.sleep_s)

        return consumed

    def get_latest_ucm(self) -> Optional[UCMResult]:
        if not self._pending_ucm:
            return None
        return self._pending_ucm[-1]

    def drain_ucm(self) -> List[UCMResult]:
        out = self._pending_ucm[:]
        self._pending_ucm.clear()
        return out
