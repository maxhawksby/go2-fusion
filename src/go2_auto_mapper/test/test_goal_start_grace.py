from ros_stubs import load_explorer

ex = load_explorer()


def test_a_goal_just_sent_is_kept_while_oak_stands_still():
    assert ex.goal_still_starting(forward=0.0, goal_age=0.3, grace=3.5)


def test_a_goal_just_sent_is_kept_when_speed_is_unknown():
    assert ex.goal_still_starting(forward=None, goal_age=0.3, grace=3.5)


def test_walking_at_the_obstacle_ends_the_grace_at_once():
    assert not ex.goal_still_starting(forward=0.2, goal_age=0.3, grace=3.5)


def test_the_grace_runs_out():
    assert not ex.goal_still_starting(forward=0.0, goal_age=3.6, grace=3.5)


def test_turning_backward_is_not_walking_at_the_obstacle():
    assert ex.goal_still_starting(forward=-0.1, goal_age=1.0, grace=3.5)


def test_zero_grace_is_the_old_behaviour():
    assert not ex.goal_still_starting(forward=0.0, goal_age=0.0, grace=0.0)
