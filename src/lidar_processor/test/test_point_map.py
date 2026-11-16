import pytest

from lidar_processor.point_map import CellPointMap


def pts(m):
    """The map's points as a set of tuples."""
    return set(map(tuple, m.points().tolist()))


def floor(x0, x1, y0, y1, step=0.1):
    """Floor points over a rectangle, like the robot's cloud always has."""
    pts = []
    x = x0
    while x <= x1 + 1e-9:
        y = y0
        while y <= y1 + 1e-9:
            pts.append((round(x, 3), round(y, 3), 0.0))
            y += step
        x += step
    return pts


def test_accumulates_areas_seen_one_after_another():
    m = CellPointMap(cell_size=0.25, max_points=10_000)
    m.update(floor(0, 1, 0, 1))
    m.update(floor(5, 6, 0, 1))          # a different room
    xs = {round(p[0]) for p in pts(m)}
    assert 0 in xs and 5 in xs            # both rooms kept


def test_obstacle_removed_from_view_disappears():
    m = CellPointMap(cell_size=0.25, max_points=10_000)
    box = [(0.5, 0.5, h / 10) for h in range(1, 6)]   # a box 0.5 m tall
    m.update(floor(0, 1, 0, 1) + box)
    assert (0.5, 0.5, 0.3) in pts(m)
    dropped = m.update(floor(0, 1, 0, 1))             # box carried away
    assert (0.5, 0.5, 0.3) not in pts(m)
    assert dropped == len(box)


def test_new_obstacle_appears():
    m = CellPointMap(cell_size=0.25, max_points=10_000)
    m.update(floor(0, 1, 0, 1))
    m.update(floor(0, 1, 0, 1) + [(0.7, 0.2, 0.4)])
    assert (0.7, 0.2, 0.4) in pts(m)


def test_out_of_view_area_is_not_touched():
    m = CellPointMap(cell_size=0.25, max_points=10_000)
    chair = [(5.5, 0.5, 0.4)]
    m.update(floor(5, 6, 0, 1) + chair)               # saw the chair
    m.update(floor(0, 1, 0, 1))                       # now looking elsewhere
    assert (5.5, 0.5, 0.4) in pts(m)


def test_count_matches_points():
    m = CellPointMap(cell_size=0.25, max_points=10_000)
    m.update(floor(0, 1, 0, 1) + [(0.5, 0.5, 0.3)])
    m.update(floor(0.5, 1.5, 0, 1))                   # overlaps half
    assert len(m) == len(m.points()) == len(pts(m))


def test_over_limit_drops_least_recently_seen_cells():
    m = CellPointMap(cell_size=0.25, max_points=300)
    old, newer, newest = floor(0, 1, 0, 1), floor(5, 6, 0, 1), floor(9, 10, 0, 1)
    m.update(old)                                     # 121 points each
    m.update(newer)
    m.update(newest)
    kept = pts(m)
    assert len(m) <= 300
    assert set(newest) <= kept and set(newer) <= kept   # recent areas intact
    assert len(set(old) & kept) < len(old)              # only the oldest lost points


def test_clear_and_bad_cell_size():
    m = CellPointMap(cell_size=0.25, max_points=100)
    m.update(floor(0, 0.5, 0, 0.5))
    assert m.clear() > 0 and len(m) == 0 and len(m.points()) == 0
    with pytest.raises(ValueError):
        CellPointMap(cell_size=0, max_points=10)
