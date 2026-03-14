import math
import random
from dataclasses import dataclass
from typing import Optional


@dataclass
class TrueState:
    x: float
    y: float
    psi: float
    u: float
    v: float
    r: float


class ImuModel:
    def __init__(self, gyro_bias_std: float = 0.001, accel_std: float = 0.05):
        self.bg_z = 0.0
        self.gyro_bias_std = gyro_bias_std
        self.accel_std = accel_std

    def sample(self, state: TrueState, t: float, dt: float):
        # simple random walk for gyro bias
        self.bg_z += random.gauss(0.0, self.gyro_bias_std * math.sqrt(dt))
        gz = state.r + self.bg_z + random.gauss(0.0, self.gyro_bias_std)
        # approximate surge acceleration derivative (zero for now)
        ax = random.gauss(0.0, self.accel_std)
        return {
            "t": t,
            "ang_vel_b": [0.0, 0.0, gz],
            "lin_acc_b": [ax, 0.0, 0.0],
        }


class GpsModel:
    def __init__(self, pos_std: float = 1.5, rate_hz: float = 5.0):
        self.pos_std = pos_std
        self.period = 1.0 / rate_hz
        self._next_t = 0.0

    def sample(self, state: TrueState, t: float, enu_to_wgs84) -> Optional[dict]:
        if t < self._next_t:
            return None
        self._next_t += self.period
        lat, lon = enu_to_wgs84(state.x, state.y)
        lat += random.gauss(0.0, self.pos_std * 1e-5)
        lon += random.gauss(0.0, self.pos_std * 1e-5)
        return {
            "t": t,
            "lat_deg": lat,
            "lon_deg": lon,
            "alt_m": 0.0,
            "valid": True,
        }


class MagModel:
    def __init__(self, noise_std_deg: float = 2.0):
        self.noise_std = math.radians(noise_std_deg)

    def sample(self, state: TrueState, t: float):
        heading = state.psi + random.gauss(0.0, self.noise_std)
        mx = math.cos(heading)
        my = math.sin(heading)
        return {"t": t, "mag_b": [mx, my, 0.0]}

