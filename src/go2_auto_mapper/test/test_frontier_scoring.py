from go2_auto_mapper.frontier_scoring import (
    count_revealed_unknown,
    count_unknown_behind,
    group_frontier_pieces,
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


def _reference_frontiers(data, width, height, min_size):
    """The original per-cell implementation, kept as the reference the fast
    version must match exactly."""
    from collections import deque

    def valid(x, y):
        return 0 <= x < width and 0 <= y < height

    def is_frontier(x, y):
        if not valid(x, y) or data[y * width + x] != 0:
            return False
        return any(
            valid(nx, ny) and data[ny * width + nx] == -1
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1))
        )

    frontiers, visited = [], set()
    for y in range(height):
        for x in range(width):
            if (x, y) in visited or not is_frontier(x, y):
                continue
            queue, cluster = deque([(x, y)]), []
            visited.add((x, y))
            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                    if valid(nx, ny) and (nx, ny) not in visited and is_frontier(nx, ny):
                        visited.add((nx, ny))
                        queue.append((nx, ny))
            if len(cluster) >= min_size:
                frontiers.append(cluster)
    return frontiers


def test_pieces_within_the_link_distance_join_into_one_group():
    # Three 3-cell pieces in a row, 1 cell apart: one broken edge.
    pieces = [[(0, 0), (1, 0), (2, 0)], [(4, 0), (5, 0), (6, 0)], [(8, 0), (9, 0), (10, 0)]]
    groups = group_frontier_pieces(pieces, link_cells=2)
    assert len(groups) == 1
    assert sorted(groups[0]) == sorted(cell for piece in pieces for cell in piece)


def test_pieces_farther_apart_stay_separate_groups():
    pieces = [[(0, 0), (1, 0)], [(5, 0), (6, 0)], [(5, 9), (6, 9)]]
    groups = group_frontier_pieces(pieces, link_cells=2)
    assert len(groups) == 3


def test_groups_chain_through_nearby_pieces():
    # A and C are 4 cells apart, but B sits between them within reach of both.
    pieces = [[(0, 0)], [(2, 0)], [(4, 0)], [(30, 30)]]
    groups = group_frontier_pieces(pieces, link_cells=2)
    assert sorted(len(g) for g in groups) == [1, 3]


def test_no_pieces_no_groups():
    assert group_frontier_pieces([], link_cells=2) == []


def test_unknown_behind_counts_what_the_cells_can_see():
    # Known free strip on the left, unknown on the right: the edge sees it.
    w, h = 40, 21
    data = make_grid(w, h, UNKNOWN)
    put(data, w, 0, 19, 0, 20, FREE)
    edge = [(19, y) for y in range(8, 13)]
    assert count_unknown_behind(data, w, h, edge, 10, 48) > 0


def test_unknown_behind_stops_at_walls():
    w, h = 40, 21
    data = make_grid(w, h, UNKNOWN)
    put(data, w, 0, 19, 0, 20, FREE)
    put(data, w, 20, 20, 0, 20, WALL)   # a wall between the edge and the unknown
    edge = [(19, y) for y in range(8, 13)]
    assert count_unknown_behind(data, w, h, edge, 10, 48) == 0


def test_unknown_behind_counts_each_cell_once():
    # Every cell of a group sees the same unknown cells: count them once.
    w, h = 40, 21
    data = make_grid(w, h, UNKNOWN)
    put(data, w, 0, 19, 0, 20, FREE)
    one = count_unknown_behind(data, w, h, [(19, 10)], 10, 48)
    same_twice = count_unknown_behind(data, w, h, [(19, 10), (19, 10)], 10, 48)
    assert same_twice == one


def test_fast_frontiers_match_reference_exactly():
    import random
    from array import array
    from go2_auto_mapper.frontier_scoring import find_frontier_clusters

    rng = random.Random(7)
    for trial in range(30):
        w, h = rng.randint(5, 70), rng.randint(5, 70)
        data = [rng.choice((FREE, FREE, FREE, UNKNOWN, WALL, 50)) for _ in range(w * h)]
        for min_size in (1, 5):
            expected = _reference_frontiers(data, w, h, min_size)
            assert find_frontier_clusters(data, w, h, min_size) == expected
            # OccupancyGrid.data arrives as array('b'), not a list.
            assert find_frontier_clusters(array('b', data), w, h, min_size) == expected
