"""ROS-dependent tests of the explorer waiting for people; run inside the
Humble container."""

from types import SimpleNamespace

from go2_auto_mapper.explorer import FrontierExplorer, YIELD_MAX_SEC
from go2_auto_mapper.social_space import social_zones


def explorer_stub(people, goal_active=True):
    node = SimpleNamespace(
        yield_to_people=True, goal_active=goal_active, yield_since=None,
        yield_spent=False, stops=[],
    )
    node.get_robot_pose = lambda frame: (0.0, 0.0, 0.0)   # facing +x
    node.people_zones = lambda: social_zones(people)
    node.get_logger = lambda: SimpleNamespace(info=lambda m: None)
    node._hold_stop = lambda: node.stops.append(True)
    return node


def wait(node):
    return FrontierExplorer.yield_for_people(node)


def test_waits_for_a_person_ahead():
    node = explorer_stub([(2.0, 0.0, [], None)])
    assert wait(node)
    assert node.stops == [True]


def test_ignores_a_person_off_to_the_side_or_when_idle():
    assert not wait(explorer_stub([(0.0, 3.0, [], None)]))
    assert not wait(explorer_stub([(2.0, 0.0, [], None)], goal_active=False))


def test_waits_for_someone_about_to_cross():
    # Standing 3 m to the left, walking right across the robot's way.
    path = [(0.5, 3.0 - 0.5 * k) for k in range(1, 13)]
    assert wait(explorer_stub([(0.5, 3.0, path, None)]))


def test_gives_up_waiting_then_waits_again_for_the_next_person():
    node = explorer_stub([(2.0, 0.0, [], None)])
    assert wait(node)
    node.yield_since -= YIELD_MAX_SEC
    assert not wait(node)        # carry on around them
    assert not wait(node)        # still the same person: no second wait
    node.people_zones = lambda: []
    assert not wait(node)        # the way is clear
    node.people_zones = lambda: social_zones([(2.0, 0.0, [], None)])
    assert wait(node)            # a new person ahead: wait again
