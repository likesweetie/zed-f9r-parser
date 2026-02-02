import time
from typing import Optional

from i2c_parser import I2CGNSSParser, UCMResult
from lcm_publisher import LCMPublisher
from gps_lcm_type import gps_t

# NMEA dataclasses / frame types
from nmea_decoder import EpochFrame, NMEAGGA, NMEARMC  # 파일명은 실제에 맞게 수정하세요.

MULTI_CAST_ADDR = "udpm://239.255.76.67:7667?ttl=1"
CHANNEL = "GPS_DATA"

KNOTS_TO_MPS = 0.5144444444444444


# 최근 UCM을 콜백으로 받아서 저장해두고, 프레임이 나오면 같이 패킹하는 구조
_last_ucm: Optional[UCMResult] = None


def on_ucm(u: UCMResult):
    global _last_ucm
    _last_ucm = u
    print(f"[UCM] epoch={u.epoch_id} x={u.x_m:.3f} y={u.y_m:.3f}")


def pack_gps_msg(frame: EpochFrame, ucm: Optional[UCMResult]) -> gps_t:
    msg = gps_t()

    # --------
    # Time / epoch
    # --------
    msg.timestamp = time.time_ns()
    msg.epoch = int(frame.epoch_id)

    # --------
    # UCM
    # --------
    if ucm is not None and int(ucm.epoch_id) == int(frame.epoch_id):
        msg.is_ucm_valid = True
        msg.ucm_x = float(ucm.x_m)
        msg.ucm_y = float(ucm.y_m)
    else:
        # epoch mismatch면 "이번 프레임에 대응되는 ucm이 아직 없다"로 보는 편이 디버깅에 유리합니다.
        msg.is_ucm_valid = False
        msg.ucm_x = 0.0
        msg.ucm_y = 0.0

    # --------
    # GGA
    # --------
    gga: Optional[NMEAGGA] = frame.GGA[0] if len(frame.GGA) > 0 else None
    if gga is not None and gga.lat_deg is not None and gga.lon_deg is not None:
        msg.is_gga_alive = True
        msg.gga_lat = float(gga.lat_deg)
        msg.gga_lon = float(gga.lon_deg)
        msg.gga_fixq = int(gga.fix_quality or 0)
        msg.gga_nsat = int(gga.num_sats or 0)
        msg.gga_hdop = float(gga.hdop or 0.0)
        msg.gga_alt = float(gga.altitude_m or 0.0)
        msg.gga_geoid = float(gga.geoid_sep_m or 0.0)
    else:
        msg.is_gga_alive = False
        msg.gga_lat = 0.0
        msg.gga_lon = 0.0
        msg.gga_fixq = 0
        msg.gga_nsat = 0
        msg.gga_hdop = 0.0
        msg.gga_alt = 0.0
        msg.gga_geoid = 0.0

    # --------
    # RMC (speed/course)
    # --------
    rmc: Optional[NMEARMC] = frame.RMC[0] if len(frame.RMC) > 0 else None
    if rmc is not None:
        # rmc.status == "A" (valid)까지 체크하고 싶으면 아래 조건에 추가하셔도 됩니다.
        msg.is_rmc_alive = True

        sog_mps = 0.0
        if rmc.sog_knots is not None:
            sog_mps = float(rmc.sog_knots) * KNOTS_TO_MPS

        msg.sog_mps = sog_mps
        msg.cog_deg = float(rmc.cog_deg or 0.0)
    else:
        msg.is_rmc_alive = False
        msg.sog_mps = 0.0
        msg.cog_deg = 0.0

    return msg


def main(args=[MULTI_CAST_ADDR, CHANNEL]):
    lcm_url = args[0]
    channel = args[1]

    pub = LCMPublisher(lcm_url, channel)

    # I2CGNSSParser는 내부에서 NMEADecoder로 epoch frame을 만들고,
    # poll()이 EpochFrame을 반환한다고 가정합니다.
    with I2CGNSSParser(i2c_bus=1, addr=0x42, fixed_zone=52, on_ucm=on_ucm) as gps:
        while True:
            frame = gps.poll()  # <-- EpochFrame | None (가정)
            if frame is None:
                time.sleep(0.001)
                continue

            # 콜백으로 받은 최신 ucm을 가져옴
            ucm = _last_ucm

            msg = pack_gps_msg(frame, ucm)
            pub.publish_gps(msg)

            # 200 Hz로 쏘고 싶으면 sleep(0.005)이지만,
            # GNSS가 1~10 Hz라면 frame이 있는 경우만 발행하는 편이 더 의미 있습니다.
            # 필요시 throttling을 여기서 하시면 됩니다.
            time.sleep(0.005)


if __name__ == "__main__":
    main()
