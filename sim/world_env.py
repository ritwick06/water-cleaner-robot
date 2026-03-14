import math
import random
from dataclasses import dataclass


@dataclass
class WaveConfig:
    hs: float = 0.2  # significant wave height [m]
    tp: float = 4.0  # peak period [s]
    n_components: int = 8


class WaveField:
    def __init__(self, cfg: WaveConfig):
        self.cfg = cfg
        self.omegas = []
        self.amps = []
        self.phases = []
        g = 9.81
        wp = 2.0 * math.pi / cfg.tp
        alpha = 8.1e-3
        beta = 0.74
        dw = wp / cfg.n_components
        for k in range(cfg.n_components):
            w = dw * (k + 1)
            s_eta = alpha * g**2 * w ** (-5.0) * math.exp(
                -beta * (wp / w) ** 4.0
            )
            a = math.sqrt(2.0 * s_eta * dw)
            self.omegas.append(w)
            self.amps.append(a)
            self.phases.append(random.uniform(0.0, 2.0 * math.pi))

    def tau_wave(self, t: float):
        # Very crude: map surface elevation to surge force.
        eta = 0.0
        for w, a, ph in zip(self.omegas, self.amps, self.phases):
            eta += a * math.cos(w * t + ph)
        Xw = 20.0 * eta
        return Xw, 0.0, 0.0


class CurrentField:
    def __init__(self, u_c: float = 0.1, v_c: float = 0.0):
        self.u_c = u_c
        self.v_c = v_c

