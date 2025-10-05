import numpy as np

# BASED ON https://github.com/Auterion/px4-ros2-interface-lib/blob/396120292cc70014c37646d30fccc2c62a649458/px4_ros2_cpp/src/utils/map_projection_impl.cpp

# Constants
kRadiusOfEarth = 6378137.0  # meters (WGS84 Earth radius)
#kRadiusOfEarth = 6371000.0  # meters (usec by the above liked source)

def deg_to_rad(degrees: float) -> float:
    return np.deg2rad(degrees)

class MapProjectionImpl:
    def __init__(self, ref_lat_deg: float, ref_lon_deg: float):
        # Store reference latitude/longitude in radians
        self._ref_lat = deg_to_rad(ref_lat_deg)
        self._ref_lon = deg_to_rad(ref_lon_deg)
        self._ref_sin_lat = np.sin(self._ref_lat)
        self._ref_cos_lat = np.cos(self._ref_lat)

    def global_to_local(self, lat_deg: float, lon_deg: float):
        """
        Convert a global GPS coordinate (latitude, longitude in degrees)
        to local X/Y coordinates (in meters) relative to the reference point.
        """
        lat_rad = deg_to_rad(lat_deg)
        lon_rad = deg_to_rad(lon_deg)

        sin_lat = np.sin(lat_rad)
        cos_lat = np.cos(lat_rad)
        cos_d_lon = np.cos(lon_rad - self._ref_lon)

        arg = np.clip(
            self._ref_sin_lat * sin_lat + self._ref_cos_lat * cos_lat * cos_d_lon,
            -1.0, 1.0
        )
        c = np.arccos(arg)

        k = 1.0
        if np.abs(c) > 1e-12:
            k = c / np.sin(c)

        x = k * (self._ref_cos_lat * sin_lat - self._ref_sin_lat * cos_lat * cos_d_lon) * kRadiusOfEarth
        y = k * cos_lat * np.sin(lon_rad - self._ref_lon) * kRadiusOfEarth

        return float(x), float(y)