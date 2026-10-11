"""Energy model (paper Eq. 3):  E_cons(t) = sum_m P_m*u_m(t) + C(t),  E_avail(t) = E0 - integral(E_cons).

C(t) is measured compute power (RAPL package energy delta / dt). P_m are sensor power
constants from datasheets (config/sensors.yaml). There is no silent fallback: if the
energy counter cannot be read, RaplReader raises.
"""
from pathlib import Path


class EnergyCounterError(RuntimeError):
    pass


class RaplReader:
    """Cumulative package energy in joules from the Linux powercap interface (Intel RAPL; also AMD Zen)."""

    def __init__(self, root="/sys/class/powercap"):
        zones = []
        for z in sorted(Path(root).glob("intel-rapl:*")):
            name_file = z / "name"
            if z.name.count(":") == 1 and name_file.exists() and name_file.read_text().strip().startswith("package"):
                zones.append(z)
        if not zones:
            raise EnergyCounterError(f"no RAPL package zone under {root}")
        self.zones = zones
        self.max_uj = []
        for z in zones:
            try:
                (z / "energy_uj").read_text()
            except PermissionError as e:
                raise EnergyCounterError(
                    f"{z}/energy_uj is not readable by this user. Grant access with:\n"
                    f"  sudo chmod a+r {root}/intel-rapl:*/energy_uj {root}/intel-rapl:*/*/energy_uj") from e
            self.max_uj.append(int((z / "max_energy_range_uj").read_text()))
        self._last = self._raw()
        self._total_uj = 0

    def _raw(self):
        return [int((z / "energy_uj").read_text()) for z in self.zones]

    def joules(self) -> float:
        """Energy in joules since this reader was created (counter wrap-around handled)."""
        now = self._raw()
        for prev, cur, mx in zip(self._last, now, self.max_uj):
            self._total_uj += cur - prev if cur >= prev else cur + mx - prev
        self._last = now
        return self._total_uj * 1e-6


class EnergyAccount:
    """Integrates E_cons and tracks E_avail. Sensor power is modelled, compute power is supplied by the caller."""

    def __init__(self, e0_joules: float, sensor_power_w: dict):
        if e0_joules <= 0:
            raise ValueError("E0 must be positive")
        self.e0 = float(e0_joules)
        self.sensor_power_w = dict(sensor_power_w)
        self.compute_j = 0.0
        self.sensor_j = 0.0

    def sensor_power(self, enabled: dict) -> float:
        return sum(p for m, p in self.sensor_power_w.items() if enabled.get(m, False))

    def update(self, dt: float, compute_joules: float, enabled: dict) -> float:
        """Add one interval: dt seconds, measured compute energy for it, sensors on. Returns E_cons power (W)."""
        if dt <= 0:
            return 0.0
        ps = self.sensor_power(enabled)
        self.compute_j += compute_joules
        self.sensor_j += ps * dt
        return ps + compute_joules / dt

    @property
    def consumed(self) -> float:
        return self.compute_j + self.sensor_j

    @property
    def available(self) -> float:
        return self.e0 - self.consumed

    @property
    def fraction(self) -> float:
        return self.available / self.e0
