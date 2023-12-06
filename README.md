# Motion Planning Scenes

Motion planning consists of finding a path from a state A to a goal state B
while avoiding obstacles.

There is a wide range of motion planning libraries that focus mostly on motion
planning problems formulated in the configuration space
[OMPL](https://ompl.kavrakilab.org/). This approach is usually based on inverse
kinematics to transform real-world goals into suitable configurations. 

This repository formulates a generic motion planning scene, including both
moving and static obstacles and a generic formulation of goals.

## Obstacles

Obstacles are split into dynamic and static obstacles. Dynamic obstacles are
further split into movable and fixed obstacles. Movable obstacles can be pushed
by robots while fixed ones act as walls or static parts of the environment.

## Goals

Goals for motion planning should not depend on the robot's structure, neither should they
involve orientation that are generally hard to obtain and hardly human understandable.

## Installation

I have decided to change the name of the package on pypi to `mpscenes` as it
seems more convenient a name. Early versions were published under the name
`motion_planning_scenes`.

```bash
pip3 install .
```
or
```bash
pip3 install mpscenes
```

## Poetry
This package is build using [poetry](https://python-poetry.org/docs/). 
Poetry generates a virtual environment automatically.
If you have poetry installed you can test it through:
```bash
poetry install
poetry shell
```
Then you are in a virtual environment in which you can test this specific package.
Read more about poetry on the [documentation website](https://python-poetry.org/docs/).
