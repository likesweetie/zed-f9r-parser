#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ucm_convert.py
- Convert NMEA lat/lon (WGS84) to UTM-like planar coordinates (meters)
- Provide UCM_X, UCM_Y = (Easting, Northing) by default
- Also supports local XY relative to an origin (e.g., first fix)
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple


# ---------------------------
# WGS84 constants
# ---------------------------
WGS84_A = 6378137.0                      # semi-major axis (m)
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)     # first eccentricity squared
K0 = 0.9996


@dataclass(frozen=True)
class UTMCoord:
    easting_m: float
    northing_m: float
    zone: int
    hemisphere: str  # 'N' or 'S'


@dataclass(frozen=True)
class UCMCoord:
    ucm_x: float
    ucm_y: float
    zone: int
    hemisphere: str


def _deg2rad(deg: float) -> float:
    return deg * 3.141592653589793 / 180.0


def utm_zone_from_lon(lon_deg: float) -> int:
    """
    UTM zone number from longitude in degrees.
    zone in [1..60]
    """
    # clamp lon to [-180, 180)
    lon = lon_deg
    if lon == 180.0:
        lon = 179.999999
    return int((lon + 180.0) // 6.0) + 1


def latlon_to_utm(lat_deg: float, lon_deg: float, zone: Optional[int] = None) -> UTMCoord:
    """
    Convert WGS84 latitude/longitude to UTM Easting/Northing (meters).
    - No special-case handling for Norway/Svalbard UTM exceptions.
    - Suitable for robotics navigation / mapping in most regions.
    """
    if zone is None:
        zone = utm_zone_from_lon(lon_deg)

    hemisphere = "N" if lat_deg >= 0.0 else "S"

    lat = _deg2rad(lat_deg)
    lon = _deg2rad(lon_deg)

    # Central meridian of the zone
    lon0_deg = (zone - 1) * 6 - 180 + 3
    lon0 = _deg2rad(lon0_deg)

    e2 = WGS84_E2
    ep2 = e2 / (1.0 - e2)

    sin_lat = __import__("math").sin(lat)
    cos_lat = __import__("math").cos(lat)
    tan_lat = __import__("math").tan(lat)

    N = WGS84_A / __import__("math").sqrt(1.0 - e2 * sin_lat * sin_lat)
    T = tan_lat * tan_lat
    C = ep2 * cos_lat * cos_lat
    A = cos_lat * (lon - lon0)

    # Meridional arc
    M = WGS84_A * (
        (1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 ** 3 / 256.0) * lat
        - (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 ** 3 / 1024.0) * __import__("math").sin(2.0 * lat)
        + (15.0 * e2 * e2 / 256.0 + 45.0 * e2 ** 3 / 1024.0) * __import__("math").sin(4.0 * lat)
        - (35.0 * e2 ** 3 / 3072.0) * __import__("math").sin(6.0 * lat)
    )

    # Easting
    easting = K0 * N * (
        A
        + (1.0 - T + C) * A**3 / 6.0
        + (5.0 - 18.0 * T + T**2 + 72.0 * C - 58.0 * ep2) * A**5 / 120.0
    ) + 500000.0

    # Northing
    northing = K0 * (
        M
        + N * tan_lat * (
            A**2 / 2.0
            + (5.0 - T + 9.0 * C + 4.0 * C**2) * A**4 / 24.0
            + (61.0 - 58.0 * T + T**2 + 600.0 * C - 330.0 * ep2) * A**6 / 720.0
        )
    )

    # False northing for southern hemisphere
    if hemisphere == "S":
        northing += 10000000.0

    return UTMCoord(easting_m=easting, northing_m=northing, zone=zone, hemisphere=hemisphere)


class UCMConverter:
    """
    UCMConverter:
    - ucm_from_latlon(): returns UCM (default = UTM Easting/Northing)
    - set_origin_from_latlon(): sets local origin and enables local XY output
    - to_local_xy(): returns (x, y) relative to origin in meters
    """

    def __init__(self, fixed_zone: Optional[int] = None):
        self.fixed_zone = fixed_zone
        self._origin_utm: Optional[UTMCoord] = None

    def ucm_from_latlon(self, lat_deg: float, lon_deg: float) -> UCMCoord:
        utm = latlon_to_utm(lat_deg, lon_deg, zone=self.fixed_zone)
        return UCMCoord(ucm_x=utm.easting_m, ucm_y=utm.northing_m, zone=utm.zone, hemisphere=utm.hemisphere)

    def set_origin_from_latlon(self, lat_deg: float, lon_deg: float) -> None:
        self._origin_utm = latlon_to_utm(lat_deg, lon_deg, zone=self.fixed_zone)

    def to_local_xy(self, lat_deg: float, lon_deg: float) -> Tuple[float, float]:
        """
        Local XY in meters relative to origin UTM.
        - X: east (m)
        - Y: north (m)
        """
        if self._origin_utm is None:
            raise RuntimeError("Origin not set. Call set_origin_from_latlon() first.")
        cur = latlon_to_utm(lat_deg, lon_deg, zone=self._origin_utm.zone)
        x = cur.easting_m - self._origin_utm.easting_m
        y = cur.northing_m - self._origin_utm.northing_m
        return x, y


# ---------------------------
# Integration helper for NMEADecoder frames/messages
# ---------------------------

def extract_latlon_from_nmea_message(msg) -> Optional[Tuple[float, float]]:
    """
    Works with the earlier NMEADecoder dataclasses:
    - NMEAGGA: lat_deg, lon_deg
    - NMEARMC: lat_deg, lon_deg
    - NMEAGLL: lat_deg, lon_deg
    Returns (lat, lon) or None
    """
    lat = getattr(msg, "lat_deg", None)
    lon = getattr(msg, "lon_deg", None)
    if lat is None or lon is None:
        print("no lat on lon!!")
        return None
    return float(lat), float(lon)
