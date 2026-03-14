import dataclasses
from dataclasses import dataclass
from typing import Callable


@dataclass
class State3Dof:
    x: float
    y: float
    psi: float
    u: float
    v: float
    r: float


def rk4_step(
    f: Callable[[State3Dof], State3Dof],
    x: State3Dof,
    dt: float,
) -> State3Dof:
    k1 = f(x)

    x2 = dataclasses.replace(
        x,
        x=x.x + 0.5 * dt * k1.x,
        y=x.y + 0.5 * dt * k1.y,
        psi=x.psi + 0.5 * dt * k1.psi,
        u=x.u + 0.5 * dt * k1.u,
        v=x.v + 0.5 * dt * k1.v,
        r=x.r + 0.5 * dt * k1.r,
    )
    k2 = f(x2)

    x3 = dataclasses.replace(
        x,
        x=x.x + 0.5 * dt * k2.x,
        y=x.y + 0.5 * dt * k2.y,
        psi=x.psi + 0.5 * dt * k2.psi,
        u=x.u + 0.5 * dt * k2.u,
        v=x.v + 0.5 * dt * k2.v,
        r=x.r + 0.5 * dt * k2.r,
    )
    k3 = f(x3)

    x4 = dataclasses.replace(
        x,
        x=x.x + dt * k3.x,
        y=x.y + dt * k3.y,
        psi=x.psi + dt * k3.psi,
        u=x.u + dt * k3.u,
        v=x.v + dt * k3.v,
        r=x.r + dt * k3.r,
    )
    k4 = f(x4)

    return State3Dof(
        x=x.x + (dt / 6.0) * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
        y=x.y + (dt / 6.0) * (k1.y + 2 * k2.y + 2 * k3.y + k4.y),
        psi=x.psi + (dt / 6.0) * (k1.psi + 2 * k2.psi + 2 * k3.psi + k4.psi),
        u=x.u + (dt / 6.0) * (k1.u + 2 * k2.u + 2 * k3.u + k4.u),
        v=x.v + (dt / 6.0) * (k1.v + 2 * k2.v + 2 * k3.v + k4.v),
        r=x.r + (dt / 6.0) * (k1.r + 2 * k2.r + 2 * k3.r + k4.r),
    )

