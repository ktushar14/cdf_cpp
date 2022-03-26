# COVERAGE-SEARCH

## Usage

### Build and run with qmake:
```
cd qt-build
qmake ../coverage-search.pro -r -spec linux-g++-64 CONFIG+=debug
make
./coverage-search
```

### Map convention
* X increases rightward, Y increases downward
* The map is visualized exactly as in a map `.txt` file (see `maps/map1.txt`), namely, a given cell at row r and column c in the map is drawn at position `x = c` and `y = r` on the screen.

* `1 =` traversable, covered
* `2 =` traversable, not covered
* `0 =` not traversable

### Angles
* Angles in motion primitive files are in the range [-pi, pi].
* Angles in motion primitive files are absolute, i.e., the start angle for a
given primitive is already added in the angles in intermediate poses. Thus, in
the search (see `XYThetaEnv::GetSuccs`), we just assign successor angles as the
angle of the last intermediate pose.

### Callbacks
* Map updates: `GetUpdatedMap` in persistent.cpp is called by `Dialog` whenever
it needs an updated map. The map is actually udpated by `Planner` every time a
planning query is completed. The map is an object owned by `RobotSpace`
(`RobotSpace Planner::env_` -> `MobingAI RobotSpace::movingai_` -> `int* MovingAI::m_map`).

* Robot state update: `GetNextState` in persistent.cpp is called by `Dialog`
whenever it needs the next robot state. This is always the last state in
`PlanClass Planner::planObj_ -> Plan PlanClass::wps_`.
