# Copyright (c) 2026, go2-fusion team
# SPDX-License-Identifier: BSD-3-Clause
"""Stale-sensor decision for the driver's motion stop (no ROS imports).

The driver feeds in "this sensor just sent data" and asks two things:
which required sensors are stale right now (if any, Oak must not move),
and whether that answer just changed (so it can report an event).
Kept free of ROS so it can be unit tested on any machine.
"""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

STALE = 'sensor_stale'
FRESH = 'sensor_fresh'


@dataclass
class WatchdogEvent:
    name: str                                   # STALE or FRESH
    sensors: List[str]                          # the sensors that were stale
    age_s: Optional[float]                      # oldest data age now (None: never seen)
    stopped_s: float = 0.0                      # FRESH only: how long the stop lasted


class SensorWatchdog:
    """Freshness of one robot's sensors.

    A sensor is stale when it has never sent data, or its last data is
    older than the timeout. A timeout of 0 or less turns the check off.

    Events: no event fires until every required sensor has been fresh at
    least once (so a node starting up, before the robot connects, is not
    a "sensor went stale" event). After that, update() returns a STALE
    event when any required sensor goes stale and a FRESH event when all
    are fresh again; otherwise None.
    """

    def __init__(self) -> None:
        self._last: Dict[str, float] = {}
        self._state: Optional[str] = None       # None, STALE or FRESH
        self._stale_since = 0.0
        self._stale_sensors: List[str] = []

    def seen(self, sensor: str, now: float) -> None:
        self._last[sensor] = now

    def age(self, sensor: str, now: float) -> Optional[float]:
        last = self._last.get(sensor)
        return None if last is None else now - last

    def stale(self, required: Iterable[str], now: float,
              timeout_sec: float) -> List[Tuple[str, Optional[float]]]:
        """(sensor, age) for each required sensor that is stale."""
        if timeout_sec <= 0:
            return []
        out = []
        for sensor in required:
            age = self.age(sensor, now)
            if age is None or age > timeout_sec:
                out.append((sensor, age))
        return out

    def update(self, required: Iterable[str], now: float,
               timeout_sec: float) -> Optional[WatchdogEvent]:
        required = list(required)
        stale = self.stale(required, now, timeout_sec)
        if stale:
            if self._state != FRESH:
                return None                     # still starting, or already stale
            self._state = STALE
            self._stale_since = now
            self._stale_sensors = [s for s, _ in stale]
            return WatchdogEvent(STALE, self._stale_sensors,
                                 _oldest(a for _, a in stale))
        previous, self._state = self._state, FRESH
        if previous != STALE:
            return None
        sensors = self._stale_sensors
        return WatchdogEvent(FRESH, sensors,
                             _oldest(self.age(s, now) for s in sensors),
                             now - self._stale_since)


def _oldest(ages) -> Optional[float]:
    ages = list(ages)
    if any(a is None for a in ages):
        return None
    return max(ages) if ages else None
