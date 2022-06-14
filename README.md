> :warning: **For Embotech Forces PRO user**: You need to use this branch as the newer casadi versions are
not supported in Forces PRO.
# Motion Planning Scenes

Motion planning consists of finding a path from a state A to a goal state B while avoiding
obstacles.

There is a wide range of motion planning libraries that focus mostly on motion planning problems
formulated in the configuration space [OMPL](https://ompl.kavrakilab.org/). This approach
is usually based on inverse kinematics to transform real-world goals into suitable
configurations. 

This repository formulates a generic motion planning scene, including both moving and
static obstacles and a generic formulation of goals.

## Obstacles

Obstacles can be roughly split into two categories, moving and static.

## Goals

Goals for motion planning should not depend on the robot's structure, neither should they
involve orientation that are generally hard to obtain and hardly human understandable.

## Structure (Beta)

![Structure (by mermaid)](./assets/overview.svg)

## Installation

```bash
pip3 install .
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
