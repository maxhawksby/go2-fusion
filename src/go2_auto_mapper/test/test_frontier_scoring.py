from go2_auto_mapper.frontier_scoring import (
    count_revealed_unknown,
    overlaps_dead,
    remaining_size,
    visit_utility,
    world_key,
)

FREE, WALL, UNKNOWN = 0, 100, -1


def make_grid(width, height, fill=FREE):
    return [fill] * (width * height)


def put(data, width, x0, x1, y0, y1, value):
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            data[y * width + x] = value


def test_counts_unknown_cells_in_view():
    # Known free strip on the left, unknown on the right.
    w, h = 40, 21
    data = make_grid(w, h, UNKNOWN)
    put(data, w, 0, 19, 0, 20, FREE)
    near_edge = count_revealed_unknown(data, w, h, 18, 10, 10, 72)
    deep_inside = count_revealed_unknown(data, w, h, 5, 10, 10, 72)
    assert near_edge > 0
    assert deep_inside == 0   # unknown space is out of range from there


def test_walls_block_the_view():
    w, h = 40, 21
    data = make_grid(w, h, UNKNOWN)
    put(data, w, 0, 19, 0, 20, FREE)
    open_view = count_revealed_unknown(data, w, h, 17, 10, 10, 72)
    put(data, w, 19, 19, 0, 20, WALL)   # wall along the boundary
    walled = count_revealed_unknown(data, w, h, 17, 10, 10, 72)
    assert walled == 0 < open_view


def test_prefers_the_pose_that_sees_more():
    # A doorway into a big unknown room vs. a small unknown nook.
    w, h = 60, 40
    data = make_grid(w, h, WALL)
    put(data, w, 1, 30, 1, 38, FREE)       # known corridor/room
    put(data, w, 31, 58, 1, 38, UNKNOWN)   # big unknown room...
    put(data, w, 31, 31, 1, 38, WALL)      # ...behind a wall
    put(data, w, 31, 31, 18, 22, FREE)     # with a 5-cell doorway
    put(data, w, 5, 8, 0, 0, UNKNOWN)      # small unknown nook at the top
    by_door = count_revealed_unknown(data, w, h, 29, 20, 20, 72)
    by_nook = count_revealed_unknown(data, w, h, 6, 2, 20, 72)
    assert by_door > by_nook


def test_utility_trades_gain_against_distance():
    assert visit_utility(100, 0.0, 20.0) == 100
    assert visit_utility(100, 2.0, 20.0) == 60
    # 10 more cells aren't worth an extra 1 m at 20 cells per meter.
    assert visit_utility(110, 3.0, 20.0) < visit_utility(100, 2.0, 20.0)


def test_world_key_survives_origin_shift():
    res = 0.05

    def corner(origin, gx):
        return origin + gx * res

    # A grid with origin -5.0 and one that grew by 20 cells (origin -6.0):
    # the same physical cell has index 30 in one and 50 in the other.
    assert world_key(corner(-5.0, 30), 0.0, res) == world_key(corner(-6.0, 50), 0.0, res)


def test_world_key_keeps_neighboring_cells_apart():
    res = 0.05
    origin = -5.0
    # Long enough to include corners whose ratio floats just off an integer
    # (those are the ones floor() would merge with a neighbor).
    keys = {
        world_key(origin + gx * res, origin + gy * res, res)
        for gx in range(300)
        for gy in range(4)
    }
    assert len(keys) == 300 * 4


def test_remaining_size_counts_overlapping_clusters():
    target = {(0, 0), (1, 0), (2, 0), (3, 0)}
    unchanged = [{(0, 0), (1, 0), (2, 0), (3, 0)}, {(50, 50)}]
    split = [{(0, 0)}, {(3, 0), (4, 0)}]
    gone = [{(50, 50), (51, 50)}]
    assert remaining_size(target, unchanged) == 4
    assert remaining_size(target, split) == 3
    assert remaining_size(target, gone) == 0


def test_overlaps_dead():
    dead = [{(0, 0), (1, 0), (2, 0), (3, 0)}]
    assert overlaps_dead({(0, 0), (1, 0), (2, 0)}, dead, 0.5)
    assert not overlaps_dead({(2, 0), (8, 0), (9, 0), (10, 0)}, dead, 0.5)
    assert not overlaps_dead(set(), dead, 0.5)
