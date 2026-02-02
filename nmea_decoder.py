from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Dict, List, Type, Any, Tuple
import time
import re


# -------------------------
# Utilities
# -------------------------

_NMEA_RE = re.compile(r"^\$(?P<body>[^*]+)(?:\*(?P<cs>[0-9A-Fa-f]{2}))?\s*$")

def nmea_checksum_xor(body: str) -> int:
    """XOR of all bytes between '$' and '*' (excluded)."""
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return cs

def parse_lat_lon(nmea_value: str, hemi: str) -> Optional[float]:
    """
    NMEA lat/lon format:
      lat: ddmm.mmmm, N/S
      lon: dddmm.mmmm, E/W
    Returns decimal degrees or None.
    """
    if not nmea_value or not hemi:
        return None
    try:
        if "." not in nmea_value:
            return None

        # Determine degrees digits by hemisphere kind (lat vs lon)
        # lat has 2 deg digits, lon has 3 deg digits
        deg_digits = 2 if hemi.upper() in ("N", "S") else 3

        deg = int(nmea_value[:deg_digits])
        minutes = float(nmea_value[deg_digits:])

        dec = deg + minutes / 60.0
        if hemi.upper() in ("S", "W"):
            dec = -dec
        return dec
    except Exception:
        return None

def to_int(x: str) -> Optional[int]:
    try:
        return int(x) if x != "" else None
    except Exception:
        return None

def to_float(x: str) -> Optional[float]:
    try:
        return float(x) if x != "" else None
    except Exception:
        return None


# -------------------------
# Base / Message Types
# -------------------------

@dataclass
class NMEAMessage:
    talker: str
    msg_type: str
    raw: str
    fields: List[str]
    checksum_ok: Optional[bool] = None

    @property
    def full_type(self) -> str:
        # e.g. "GNGGA" or "GPGSA"
        return f"{self.talker}{self.msg_type}"

@dataclass
class NMEAGGA(NMEAMessage):
    """
    GGA - Global Positioning System Fix Data
    fields:
      0 time (hhmmss.sss)
      1 lat
      2 N/S
      3 lon
      4 E/W
      5 fix quality
      6 num satellites
      7 HDOP
      8 altitude
      9 altitude unit (M)
      10 geoid separation
      11 geoid unit (M)
      12 DGPS age
      13 DGPS station id
    """
    utc_time: Optional[str] = None
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    fix_quality: Optional[int] = None
    num_sats: Optional[int] = None
    hdop: Optional[float] = None
    altitude_m: Optional[float] = None
    geoid_sep_m: Optional[float] = None

@dataclass
class NMEAGSA(NMEAMessage):
    """
    GSA - GNSS DOP and Active Satellites
    fields:
      0 mode1 (M/A)
      1 mode2 (1/2/3)
      2..13 sat PRNs (up to 12)
      14 PDOP
      15 HDOP
      16 VDOP
    """
    mode1: Optional[str] = None
    mode2: Optional[int] = None
    sat_prns: List[int] = field(default_factory=list)
    pdop: Optional[float] = None
    hdop: Optional[float] = None
    vdop: Optional[float] = None

@dataclass
class NMEARMC(NMEAMessage):
    """
    RMC - Recommended Minimum Navigation Information
    fields:
      0 utc time
      1 status A/V
      2 lat
      3 N/S
      4 lon
      5 E/W
      6 speed over ground (knots)
      7 course over ground (deg)
      8 date (ddmmyy)
      9 magnetic variation
      10 mag var E/W
      11 mode indicator (optional)
    """
    utc_time: Optional[str] = None
    status: Optional[str] = None
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    sog_knots: Optional[float] = None
    cog_deg: Optional[float] = None
    date_ddmmyy: Optional[str] = None

@dataclass
class NMEAVTG(NMEAMessage):
    """
    VTG - Course Over Ground and Ground Speed
    fields:
      0 true track
      1 'T'
      2 magnetic track
      3 'M'
      4 speed knots
      5 'N'
      6 speed km/h
      7 'K'
      8 mode indicator (optional)
    """
    true_track_deg: Optional[float] = None
    mag_track_deg: Optional[float] = None
    sog_knots: Optional[float] = None
    sog_kmh: Optional[float] = None

@dataclass
class NMEAGLL(NMEAMessage):
    """
    GLL - Geographic Position (Lat/Lon)
    fields:
      0 lat
      1 N/S
      2 lon
      3 E/W
      4 utc time
      5 status A/V
      6 mode indicator (optional)
    """
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    utc_time: Optional[str] = None
    status: Optional[str] = None

@dataclass
class NMEATXT(NMEAMessage):
    """
    TXT - Text Transmission (often used by receivers for status)
    fields:
      0 total messages
      1 message number
      2 text identifier
      3 text
    """
    total: Optional[int] = None
    number: Optional[int] = None
    text_id: Optional[int] = None
    text: Optional[str] = None

@dataclass
class NMEAGeneric(NMEAMessage):
    """Fallback for unsupported/unknown sentence types."""


# -------------------------
# Epoch Frame
# -------------------------

@dataclass
class EpochFrame:
    """
    Stores messages received in the same epoch window.
    epoch_id: user-defined or monotonically increasing id
    t_recv_s: receiver-side timestamp (seconds)
    """
    epoch_id: int
    t_recv_s: float
    messages_by_type: Dict[str, List[NMEAMessage]] = field(default_factory=dict)

    def add(self, msg: NMEAMessage) -> None:
        self.messages_by_type.setdefault(msg.msg_type, []).append(msg)

    def get(self, msg_type: str) -> List[NMEAMessage]:
        return self.messages_by_type.get(msg_type, [])

    # Convenience typed accessors
    @property
    def GGA(self) -> List[NMEAGGA]:
        return [m for m in self.get("GGA") if isinstance(m, NMEAGGA)]

    @property
    def GSA(self) -> List[NMEAGSA]:
        return [m for m in self.get("GSA") if isinstance(m, NMEAGSA)]

    @property
    def RMC(self) -> List[NMEARMC]:
        return [m for m in self.get("RMC") if isinstance(m, NMEARMC)]

    @property
    def VTG(self) -> List[NMEAVTG]:
        return [m for m in self.get("VTG") if isinstance(m, NMEAVTG)]

    @property
    def GLL(self) -> List[NMEAGLL]:
        return [m for m in self.get("GLL") if isinstance(m, NMEAGLL)]

    @property
    def TXT(self) -> List[NMEATXT]:
        return [m for m in self.get("TXT") if isinstance(m, NMEATXT)]


# -------------------------
# Decoder
# -------------------------

class NMEADecoder:
    """
    - feed_line(line): parse one NMEA line and store into current epoch frame
    - begin_epoch(epoch_id=None): start a new epoch frame
    - end_epoch(): finalize current epoch frame and return it
    """

    def __init__(self, *, verify_checksum: bool = True) -> None:
        self.verify_checksum = verify_checksum
        self._epoch_counter = 0
        self._current: Optional[EpochFrame] = None

        # Registry: msg_type -> parser
        self._parsers: Dict[str, Any] = {
            "GGA": self._parse_gga,
            "GSA": self._parse_gsa,
            "RMC": self._parse_rmc,
            "VTG": self._parse_vtg,
            "GLL": self._parse_gll,
            "TXT": self._parse_txt,
            # "GCC", "GCA": custom types can be added via register_parser(...)
        }

        # Alias support (if user/dev wants to map e.g. GCC -> GGA)
        self._aliases: Dict[str, str] = {
            # Example:
            # "GCC": "GGA",
            # "GCA": "GSA",
        }

    def register_parser(self, msg_type: str, parser_fn) -> None:
        """Register or override parser for a message type (e.g., 'GCC', 'GCA')."""
        self._parsers[msg_type] = parser_fn

    def register_alias(self, msg_type: str, target_type: str) -> None:
        """
        Treat msg_type as target_type for parsing/storage (e.g., GCC -> GGA).
        """
        self._aliases[msg_type] = target_type

    def begin_epoch(self, epoch_id: Optional[int] = None, t_recv_s: Optional[float] = None) -> EpochFrame:
        if epoch_id is None:
            self._epoch_counter += 1
            epoch_id = self._epoch_counter
        if t_recv_s is None:
            t_recv_s = time.time()
        self._current = EpochFrame(epoch_id=epoch_id, t_recv_s=t_recv_s)
        return self._current

    def end_epoch(self) -> Optional[EpochFrame]:
        frame = self._current
        self._current = None
        return frame

    def current_frame(self) -> Optional[EpochFrame]:
        return self._current


    def feed_line(self, line: str, *, epoch_id: Optional[int] = None) -> Optional[NMEAMessage]:
        """
        Parse a single NMEA sentence line and store it into the current epoch frame.
        If no epoch is active, it auto-starts one (or uses provided epoch_id).
        Returns parsed message or None if line is ignored/invalid.
        """
        line = line.strip()
        if not line or not line.startswith("$"):
            return None

        msg = self._parse_line(line)
        if msg is None:
            return None

        if self._current is None:
            self.begin_epoch(epoch_id=epoch_id)

        assert self._current is not None
        self._current.add(msg)
        return msg

    # -------------------------
    # Internal parsing
    # -------------------------

    def _parse_line(self, line: str) -> Optional[NMEAMessage]:
        m = _NMEA_RE.match(line)
        if not m:
            return None

        body = m.group("body")
        cs_hex = m.group("cs")

        checksum_ok: Optional[bool] = None
        if self.verify_checksum and cs_hex is not None:
            expected = int(cs_hex, 16)
            actual = nmea_checksum_xor(body)
            checksum_ok = (expected == actual)
            if not checksum_ok:
                # checksum fail: drop (strict)
                return None

        parts = body.split(",")
        if not parts or len(parts[0]) < 5:
            return None

        header = parts[0]
        talker = header[:2]
        msg_type = header[2:]

        # Apply alias mapping if configured
        parse_type = self._aliases.get(msg_type, msg_type)

        fields = parts[1:]  # after header
        parser = self._parsers.get(parse_type, None)

        if parser is None:
            return NMEAGeneric(talker=talker, msg_type=msg_type, raw=line, fields=fields, checksum_ok=checksum_ok)

        # If aliased, we parse as parse_type but keep original msg_type for storage
        parsed = parser(talker, msg_type, line, fields, checksum_ok)
        return parsed

    # -------------------------
    # Type-specific parsers
    # -------------------------

    def _parse_gga(self, talker: str, msg_type: str, raw: str, fields: List[str], checksum_ok: Optional[bool]) -> NMEAGGA:
        # Pad to at least 14 fields
        f = (fields + [""] * 14)[:14]
        utc_time = f[0] or None
        lat = parse_lat_lon(f[1], f[2])
        lon = parse_lat_lon(f[3], f[4])
        fixq = to_int(f[5])
        nsat = to_int(f[6])
        hdop = to_float(f[7])
        alt = to_float(f[8])
        geoid = to_float(f[10])

        return NMEAGGA(
            talker=talker, msg_type=msg_type, raw=raw, fields=fields, checksum_ok=checksum_ok,
            utc_time=utc_time, lat_deg=lat, lon_deg=lon, fix_quality=fixq,
            num_sats=nsat, hdop=hdop, altitude_m=alt, geoid_sep_m=geoid
        )

    def _parse_gsa(self, talker: str, msg_type: str, raw: str, fields: List[str], checksum_ok: Optional[bool]) -> NMEAGSA:
        f = (fields + [""] * 17)[:17]
        mode1 = f[0] or None
        mode2 = to_int(f[1])
        prns: List[int] = []
        for x in f[2:14]:
            v = to_int(x)
            if v is not None:
                prns.append(v)
        pdop = to_float(f[14])
        hdop = to_float(f[15])
        vdop = to_float(f[16])

        return NMEAGSA(
            talker=talker, msg_type=msg_type, raw=raw, fields=fields, checksum_ok=checksum_ok,
            mode1=mode1, mode2=mode2, sat_prns=prns, pdop=pdop, hdop=hdop, vdop=vdop
        )

    def _parse_rmc(self, talker: str, msg_type: str, raw: str, fields: List[str], checksum_ok: Optional[bool]) -> NMEARMC:
        f = (fields + [""] * 12)[:12]
        utc_time = f[0] or None
        status = f[1] or None
        lat = parse_lat_lon(f[2], f[3])
        lon = parse_lat_lon(f[4], f[5])
        sog_kn = to_float(f[6])
        cog = to_float(f[7])
        date = f[8] or None

        return NMEARMC(
            talker=talker, msg_type=msg_type, raw=raw, fields=fields, checksum_ok=checksum_ok,
            utc_time=utc_time, status=status, lat_deg=lat, lon_deg=lon,
            sog_knots=sog_kn, cog_deg=cog, date_ddmmyy=date
        )

    def _parse_vtg(self, talker: str, msg_type: str, raw: str, fields: List[str], checksum_ok: Optional[bool]) -> NMEAVTG:
        f = (fields + [""] * 9)[:9]
        true_track = to_float(f[0])
        mag_track = to_float(f[2])
        sog_kn = to_float(f[4])
        sog_kmh = to_float(f[6])

        return NMEAVTG(
            talker=talker, msg_type=msg_type, raw=raw, fields=fields, checksum_ok=checksum_ok,
            true_track_deg=true_track, mag_track_deg=mag_track, sog_knots=sog_kn, sog_kmh=sog_kmh
        )

    def _parse_gll(self, talker: str, msg_type: str, raw: str, fields: List[str], checksum_ok: Optional[bool]) -> NMEAGLL:
        f = (fields + [""] * 7)[:7]
        lat = parse_lat_lon(f[0], f[1])
        lon = parse_lat_lon(f[2], f[3])
        utc_time = f[4] or None
        status = f[5] or None

        return NMEAGLL(
            talker=talker, msg_type=msg_type, raw=raw, fields=fields, checksum_ok=checksum_ok,
            lat_deg=lat, lon_deg=lon, utc_time=utc_time, status=status
        )

    def _parse_txt(self, talker: str, msg_type: str, raw: str, fields: List[str], checksum_ok: Optional[bool]) -> NMEATXT:
        f = (fields + [""] * 4)[:4]
        total = to_int(f[0])
        number = to_int(f[1])
        text_id = to_int(f[2])
        text = f[3] or None

        return NMEATXT(
            talker=talker, msg_type=msg_type, raw=raw, fields=fields, checksum_ok=checksum_ok,
            total=total, number=number, text_id=text_id, text=text
        )


# -------------------------
# Example usage
# -------------------------
if __name__ == "__main__":
    dec = NMEADecoder(verify_checksum=False)
    dec.begin_epoch()

    dec.feed_line("$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
    dec.feed_line("$GNRMC,123520,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A*7C")

    frame = dec.end_epoch()
    assert frame is not None
    print("GGA:", frame.GGA)
    print("RMC:", frame.RMC)
