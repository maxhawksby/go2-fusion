from go2_auto_mapper.open_space import find_open_space

FREE, WALL, UNKNOWN = 0, 100, -1


def make_grid(width, height, fill=FREE):
    return [fill] * (width * height)


def put(data, width, x0, x1, y0, y1, value):
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            data[y * width + x] = value


def test_finds_nearest_spot_with_enough_clearance():
    # 40 x 20 room with walls around; robot jammed against the left wall.
    w, h = 40, 20
    data = make_grid(w, h)
    put(data, w, 0, 39, 0, 0, WALL)
    put(data, w, 0, 39, 19, 19, WALL)
    put(data, w, 0, 0, 0, 19, WALL)
    put(data, w, 39, 39, 0, 19, WALL)

    x, y, room = find_open_space(data, w, h, (1, 10), clearance=6, max_radius=30)

    assert room >= 6
    # Nearest such cell is 6 cells from the left wall, no farther than needed.
    assert x == 6


def test_does_not_tunnel_through_walls():
    # A wall splits the map; the open area on the far side is unreachable.
    w, h = 40, 20
    data = make_grid(w, h)
    put(data, w, 0, 39, 0, 19, UNKNOWN)
    put(data, w, 1, 8, 1, 18, FREE)      # cramped strip where the robot is
    put(data, w, 9, 9, 0, 19, WALL)
    put(data, w, 10, 38, 1, 18, FREE)    # big room beyond the wall

    result = find_open_space(data, w, h, (4, 10), clearance=6, max_radius=30)

    # Best reachable is the strip's middle, never the room past the wall.
    assert result is None or result[0] <= 8


def test_unknown_space_counts_as_not_roomy():
    w, h = 30, 30
    data = make_grid(w, h, UNKNOWN)
    put(data, w, 10, 19, 10, 19, FREE)   # small known patch
    x, y, room = find_open_space(data, w, h, (10, 10), clearance=10, max_radius=20)
    # Can't reach clearance 10 in a 10 x 10 patch: returns the roomiest cell,
    # in the middle of the patch.
    assert room < 10
    assert 13 <= x <= 16 and 13 <= y <= 16


def test_min_move_skips_the_current_spot():
    w, h = 30, 30
    data = make_grid(w, h)   # wide open, no walls at all
    x, y, _ = find_open_space(
        data, w, h, (15, 15), clearance=3, max_radius=10, min_move=5
    )
    assert max(abs(x - 15), abs(y - 15)) >= 5


def test_returns_none_when_nothing_is_roomier():
    w, h = 10, 3
    data = make_grid(w, h, WALL)
    put(data, w, 1, 8, 1, 1, FREE)       # one-cell-wide corridor
    assert find_open_space(data, w, h, (4, 1), clearance=3, max_radius=5) is None
