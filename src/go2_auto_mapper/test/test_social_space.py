import json
import math

from go2_auto_mapper.social_space import (
    PATH_HORIZON_STEPS,
    line_crosses,
    parse_people,
    social_zones,
    transform,
    zone_at,
)


def people_json(**people):
    return json.dumps(people)


def test_parse_people_reads_the_detector_json():
    text = people_json(person_1={
        'x': 1.0, 'y': 2.0, 'vx': 0.0, 'vy': 1.0, 'group': None,
        'path': [[1.0, 2.4], [1.0, 2.8]],
    })
    assert parse_people(text) == [(1.0, 2.0, [(1.0, 2.4), (1.0, 2.8)], None)]


def test_parse_people_skips_bad_input():
    assert parse_people('not json') == []
    assert parse_people(None) == []
    assert parse_people('[1, 2]') == []
    assert parse_people(people_json(a={'y': 1}, b={'x': 'far', 'y': 0},
                                    c={'x': 1, 'y': 1})) == [(1.0, 1.0, [], None)]


def test_transform_turns_and_shifts():
    x, y = transform(1.0, 0.0, 2.0, 3.0, math.pi / 2)
    assert math.isclose(x, 2.0, abs_tol=1e-9)
    assert math.isclose(y, 4.0)


def test_zones_cover_person_next_steps_and_group():
    path = [(0.0, 0.4 * k) for k in range(1, 13)]
    people = [(0.0, 0.0, path, 0), (1.0, 0.0, [], 0)]
    zones = social_zones(people)
    kinds = [z.kind for z in zones]
    assert kinds.count('person') == 2
    assert kinds.count('path') == PATH_HORIZON_STEPS   # only the near forecast
    group = next(z for z in zones if z.kind == 'group')
    assert (group.x, group.y, group.radius) == (0.5, 0.0, 1.0)


def test_zones_move_into_the_map_frame():
    zones = social_zones([(1.0, 0.0, [], None)], 10.0, 0.0, math.pi)
    assert math.isclose(zones[0].x, 9.0)


def test_zone_at():
    zones = social_zones([(0.0, 0.0, [], None)], personal=1.0)
    assert zone_at(0.5, 0.5, zones).kind == 'person'
    assert zone_at(2.0, 0.0, zones) is None


def test_line_through_a_person_is_caught():
    zones = social_zones([(2.0, 0.0, [], None)])
    assert line_crosses(0.0, 0.0, 4.0, 0.0, zones).kind == 'person'
    assert line_crosses(0.0, 0.0, 0.0, 4.0, zones) is None


def test_line_between_two_people_in_a_group_is_caught():
    # Two people 1.6 m apart, talking: their own 0.5 m circles leave a gap
    # in the middle, the group circle closes it.
    zones = social_zones([(2.0, 0.8, [], 0), (2.0, -0.8, [], 0)], personal=0.5)
    assert line_crosses(0.0, 0.0, 4.0, 0.0, zones).kind == 'group'


def test_a_line_leaving_a_zone_it_starts_in_is_fine():
    zones = social_zones([(0.0, 0.0, [], None)])
    assert line_crosses(0.3, 0.0, 3.0, 0.0, zones) is None
