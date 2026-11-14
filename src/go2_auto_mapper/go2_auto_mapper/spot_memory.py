"""
Remember open spots the robot scanned from, for getting unstuck.

A spot is recorded when the robot finishes its 360 degree scan there and the
lidar's closest return is at least `min_clearance` away: the robot has just
proven it can stand and turn there. When the explorer gets stuck it goes back
to the most recent spot that the current SLAM map still shows as open, before
falling back to the nearest open space on the map (open_space.py). No ROS, so
it can be tested on its own.
"""

from collections import namedtuple
import math

# One remembered spot: map position (m), the lidar clearance measured there
# (m), and when (time.monotonic()).
Spot = namedtuple('Spot', 'x y clearance stamp')

# A scan this close to the last remembered spot replaces it, so turning
# twice in one place doesn't fill the memory with copies.
SAME_SPOT_M = 0.5
# Trips back to a spot that an obstacle ahead may interrupt before the spot
# is forgotten. One bump on the way says nothing about the spot itself.
OBSTACLE_STRIKES = 2
# A scan counts as all around if it spans this much of a full turn and at
# least this share of its readings are not NaN (inf means nothing in range).
FULL_TURN_SHARE = 0.95
VALID_READING_SHARE = 0.9


def scan_clearance(ranges, range_min, range_max):
    """
    Return the closest valid lidar return in meters, or None if there is none.

    Returns outside [range_min, range_max] and NaN are not measurements;
    infinity means nothing within range that way.
    """
    valid = [r for r in ranges if math.isfinite(r) and range_min <= r <= range_max]
    return min(valid) if valid else None


def scan_covers_circle(angle_min, angle_increment, ranges):
    """
    Check a scan looks all the way around, with few missing readings.

    `angle_min` is kept for the LaserScan field order; only the span
    (readings x increment) and the share of non-NaN readings matter.
    """
    if not ranges:
        return False
    span = abs(angle_increment) * len(ranges)
    readings = sum(1 for r in ranges if not math.isnan(r))
    return (
        span >= FULL_TURN_SHARE * 2.0 * math.pi
        and readings >= VALID_READING_SHARE * len(ranges)
    )


class SpotMemory:
    """The last `size` open spots, oldest first."""

    def __init__(self, size, min_clearance):
        self.size = max(1, int(size))
        self.min_clearance = min_clearance
        self.spots = ()
        self.strikes = {}

    def remember(self, x, y, clearance, stamp):
        """Add the spot if it had enough room; True if it was added."""
        if clearance is None or clearance < self.min_clearance:
            return False
        spots = self.spots
        if spots and math.hypot(spots[-1].x - x, spots[-1].y - y) < SAME_SPOT_M:
            spots = spots[:-1]
        self.spots = (spots + (Spot(x, y, clearance, stamp),))[-self.size:]
        self.strikes = {s: n for s, n in self.strikes.items() if s in self.spots}
        return True

    def forget(self, spot):
        """Drop a spot that turned out to be unreachable."""
        self.spots = tuple(s for s in self.spots if s != spot)
        self.strikes = {s: n for s, n in self.strikes.items() if s != spot}

    def strike(self, spot):
        """
        Count an obstacle interrupting a trip back to `spot`.

        Returns True if that was strike OBSTACLE_STRIKES and the spot is now
        forgotten.
        """
        count = self.strikes.get(spot, 0) + 1
        if count >= OBSTACLE_STRIKES:
            self.forget(spot)
            return True
        self.strikes = {**self.strikes, spot: count}
        return False


def choose_closest_spot(spots, walk_distance, min_move, usable):
    """
    Return the remembered spot with the shortest walk, or None.

    `walk_distance(spot)` is the walk there in meters, or None if it can't
    be walked to. Spots under `min_move` m away and ones `usable(spot)`
    rejects are skipped. Equal walks go to the most recent spot.
    """
    best = None
    for order, spot in enumerate(spots):
        walk = walk_distance(spot)
        if walk is None or walk < min_move or not usable(spot):
            continue
        if best is None or (walk, -order) < best[0]:
            best = ((walk, -order), spot)
    return None if best is None else best[1]
