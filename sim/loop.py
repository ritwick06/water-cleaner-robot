import asyncio
import math
from typing import Callable, List

from sim.dynamics_integrator import State3Dof, rk4_step
from sim.sensors import ImuModel, GpsModel, MagModel, TrueState
from sim.world_env import WaveField, WaveConfig, CurrentField

from pyawscr import (
    Brain,
    VesselParams,
    ImuSample,
    GpsFix,
    MagSample,
    Command,
    CommandType,
    Dynamics3Dof,
    Input3Dof,
    State3Dof as CoreState3Dof,
)


class Simulation:
    def __init__(self, telemetry_cb: Callable[[dict], None]):
        self.telemetry_cb = telemetry_cb
        self.dt = 0.02
        self.vessel = VesselParams()
        self.vessel.m = 40.0
        self.vessel.x_G = 0.0
        self.vessel.I_z = 10.0
        self.vessel.X_u = -5.0
        self.vessel.Y_v = -20.0
        self.vessel.N_r = -5.0
        self.vessel.X_uu = -1.0
        self.vessel.Y_vv = -40.0
        self.vessel.N_rr = -2.0
        self.vessel.X_du = -5.0
        self.vessel.Y_dv = -30.0
        self.vessel.N_dr = -5.0
        self.vessel.thruster_y = 0.4

        self.brain = Brain(self.vessel)
        self.brain.set_reference_origin(13.0, 80.0, 0.0)

        self.state = State3Dof(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.dynamics = Dynamics3Dof(self.vessel)
        self.imu_model = ImuModel()
        self.gps_model = GpsModel()
        self.mag_model = MagModel()
        self.wave = WaveField(WaveConfig())
        self.current = CurrentField()

    def enu_to_wgs84(self, x: float, y: float):
        Re = 6378137.0
        lat0 = math.radians(13.0)
        lon0 = math.radians(80.0)
        lat = lat0 + y / Re
        lon = lon0 + x / (Re * math.cos(lat0))
        return math.degrees(lat), math.degrees(lon)

    async def set_mission_polygon(self, vertices: List[dict]) -> None:
        """Configure coverage polygon and start mission.

        vertices: list of {"lat": float, "lon": float}
        """
        gps_vertices = []
        for v in vertices:
            g = GpsFix()
            g.t = 0.0
            g.lat_deg = float(v["lat"])
            g.lon_deg = float(v["lon"])
            g.alt_m = 0.0
            g.valid = True
            gps_vertices.append(g)
        self.brain.set_coverage_polygon(gps_vertices)

        cmd = Command()
        cmd.type = CommandType.START_MISSION
        self.brain.handle_command(cmd)

    async def run(self):
        t = 0.0
        tick = 0

        while True:
            # Get previous control command from brain by stepping at t - dt
            cmd = self.brain.step(t)

            # Hydrodynamics using C++ 3-DOF model + RK4 integration
            Xw, Yw, Nw = self.wave.tau_wave(t)

            core_state = CoreState3Dof()
            core_state.x = self.state.x
            core_state.y = self.state.y
            core_state.psi = self.state.psi
            core_state.u = self.state.u
            core_state.v = self.state.v
            core_state.r = self.state.r

            core_input = Input3Dof()
            core_input.T_left = cmd.thrust_left_N
            core_input.T_right = cmd.thrust_right_N
            core_input.u_c = self.current.u_c
            core_input.v_c = self.current.v_c
            core_input.tau_wave_X = Xw
            core_input.tau_wave_Y = Yw
            core_input.tau_wave_N = Nw

            def f(local_state: State3Dof) -> State3Dof:
                cs = CoreState3Dof()
                cs.x = local_state.x
                cs.y = local_state.y
                cs.psi = local_state.psi
                cs.u = local_state.u
                cs.v = local_state.v
                cs.r = local_state.r
                xdot = self.dynamics.derivative(cs, core_input)
                return State3Dof(
                    x=xdot.x,
                    y=xdot.y,
                    psi=xdot.psi,
                    u=xdot.u,
                    v=xdot.v,
                    r=xdot.r,
                )

            self.state = rk4_step(f, self.state, self.dt)

            true_state = TrueState(
                x=self.state.x,
                y=self.state.y,
                psi=self.state.psi,
                u=self.state.u,
                v=self.state.v,
                r=self.state.r,
            )

            imu_dict = self.imu_model.sample(true_state, t, self.dt)
            imu = ImuSample()
            imu.t = imu_dict["t"]
            imu.ang_vel_b = imu_dict["ang_vel_b"]
            imu.lin_acc_b = imu_dict["lin_acc_b"]
            self.brain.feed_imu(imu, self.dt)

            gps_dict = self.gps_model.sample(true_state, t, self.enu_to_wgs84)
            if gps_dict:
                gps = GpsFix()
                gps.t = gps_dict["t"]
                gps.lat_deg = gps_dict["lat_deg"]
                gps.lon_deg = gps_dict["lon_deg"]
                gps.alt_m = gps_dict["alt_m"]
                gps.valid = gps_dict["valid"]
                self.brain.feed_gps(gps)

            mag_dict = self.mag_model.sample(true_state, t)
            mag = MagSample()
            mag.t = mag_dict["t"]
            mag.mag_b = mag_dict["mag_b"]
            self.brain.feed_mag(mag)

            if tick % 5 == 0:
                telem = self.brain.get_telemetry()
                self.telemetry_cb(
                    {
                        "t": telem.t,
                        "lat": telem.lat_deg,
                        "lon": telem.lon_deg,
                        "heading_deg": telem.heading_deg,
                        "speed_mps": telem.speed_mps,
                        "mode": int(telem.mode.value),
                    }
                )

            await asyncio.sleep(self.dt)
            t += self.dt
            tick += 1

