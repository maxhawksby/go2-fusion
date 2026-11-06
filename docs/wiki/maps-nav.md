---
title: Maps and navigation
parent: oak
---

# Maps and navigation

Welcome to Maps and navigation: how Oak draws a map of a room, knows where it is on that map, walks to a spot you pick, and can even explore a new room all by itself.

Part of: [Oak](../../README.md)

## What this part of Oak does

Think of how you would find your way around a new building:

1. You look around and sketch a floor plan as you go, keeping track of where you are on it. That is **[SLAM](slam.md)**.
2. Someone points at a spot on the plan and you plan a route around the furniture and walk it, stepping around anything in the way. That is **[Nav2](nav2.md)**.
3. Nobody points anywhere, so you go and check the doorways and corners you haven't seen yet, until the plan is complete. That is the **[Auto mapper](auto-mapper.md)**.
4. A friend watches the plan fill in on a screen and taps where you should go next. Those are the **[Map viewers](map-viewers.md)** (and RViz).

All of it runs on the laptop, inside Docker, as part of the normal robot stack (`robot.launch.py` starts SLAM and Nav2 by default). It is built from the [Senses](senses.md): the lidar's flat scan `/scan` and the robot's odometry, tied together by TF.

## Pages, in reading order

1. [SLAM](slam.md) - `slam_toolbox` builds a 2D grid map of the room (`/map`) from `/scan` and odometry, and keeps correcting where Oak is on it.
2. [Nav2](nav2.md) - plans a path to a goal on the map and steers Oak along it, publishing on `/cmd_vel` (twist_mux priority 5).
3. [Auto mapper](auto-mapper.md) - autonomous frontier exploration: finds unexplored edges of the SLAM map and sends Nav2 goals there. It moves the robot on its own.
4. [Map viewers](map-viewers.md) - a learning viewer of the live map: an SFML C++ window where a click sends a Nav2 goal.

## Three words you will meet everywhere

- **Occupancy grid** - the map: graph paper with 5 cm squares, each white (free), black (wall) or gray (unknown). Published on `/map`.
- **Costmap** - Nav2's copy of the map where the area around walls is shaded "expensive", so paths keep a safe distance.
- **Frontier** - a free square next to an unknown one: the edge of what Oak has seen so far.

## How the pieces talk

```
/scan + odometry --> SLAM --> /map, map->odom transform
                                |
     goal (RViz click, map viewer, auto mapper) --> Nav2 /navigate_to_pose
                                |
                     Nav2 --> /cmd_vel (priority 5) --> twist_mux --> driver --> legs
  auto mapper recovery --> /cmd_vel_mapper (priority 6) --^
```

[twist_mux](twist-mux.md) picks the most important driving source: a person on the gamepad (10) or keyboard (9) always beats Nav2 and the auto mapper.

## How it connects to the other hubs

- [Senses](senses.md) - provides `/scan`, `/point_cloud2` and the TF tree. Object detection's in-progress memory pins objects to this hub's `map` frame.
- [Driving](driving.md) - twist_mux and the [RViz panels](rviz-panels.md) (map viewer, AutoMapper, keyboard) live there; manual driving overrides everything here.
- [Brains](brains.md) - the [LLM agent](llm-agent.md) and [Object navigation](object-navigation.md) also send the robot places.
- [Driver](driver.md) - if the robot link drops, the map stops growing and Nav2 stops.
- [Lab setup](lab-setup.md) - how to get the stack running before any of this works.

## Where things stand

- Luis made auto mapping reliable in the week of 2026-10-18 and merged it to master; Nav2 and SLAM were retuned with it. Next: full exploration tests on Oak.
- In the 2026-10-11 lab session a Nav2 goal failed with "Starting point in lethal space" on the old code; MapViewer click-to-walk and the AutoMapper were not run yet. Both are on the retest list ([test log](test-log.md)).

## Safety

Nav2, the map viewer click and the auto mapper all move the robot. Only use them with a person watching Oak, and know the stop: Ctrl+C the explorer, cancel the Nav2 goal, or hold the gamepad / press a key to override (see [Auto mapper](auto-mapper.md)).

<!-- sources: launch/robot.launch.py, config/twist_mux.yaml, config/nav2_params.yaml, config/mapper_params_online_async.yaml, src/go2_auto_mapper/README.md, src/go2_map_viewer_cpp/README.md, go2_map_viewer_julia/README.md, docs/TEST_LOG.md, contract team updates (Luis), the four Maps and navigation program pages -->
