Getting started
===============

This is the guide to understand how motion planning components are defined in
the package.

Pre-requisites
--------------

- Python >=3.8
- pip3

Installation from pypi
-----------------------

The package is uploaded to pypi so you can install it using

.. code:: bash

   pip3 install mpscenes


Installation from source
-------------------------

You first have to download the repository

.. code:: bash

   git clone git@github.com:maxspahn/motion_planning_scenes.git

Then, you can install the package using pip as:

.. code:: bash

   pip3 install .

The code can be installed in editible mode using

.. code:: bash

   pip3 install -e .

Note that we recommend using poetry in this case.

Optional: Installation with poetry
------------------------------------

If you want to use `poetry <https://python-poetry.org/docs/>`_, you have to install it
first. See their webpage for instructions `docs <https://python-poetry.org/docs/>`_.

.. code:: bash

    poetry install

The virtual environment is entered by

.. code:: bash

    poetry shell

Inside the virtual environment you can access all the examples.

Installing dependencies
-----------------------

Dependencies should be installed through pip or poetry, see below.

Using pip, you can use

.. code:: bash

    pip3 install

Using poetry

.. code:: bash

    poetry install


Examples
-----------

Obstacles and goals are defined as dictionaries. You could potentially also load
them in as yaml files, but the parsing is not part of this package. The examples
will be based on python dicts.


A simple spherical obstacle with radius can be constructed: 

.. code:: python
   
    config_dict = {
        "type": "sphere",
        "geometry": {
          "position": [2.0, 2.0, 1.0],
          "radius": 1.0,
        },
    }
    obstacle_1 = SphereObstacle(
      name="obstacle_1", content_dict=config_dict
    )

A box (cuboid) obstacle, that can be moved by robots, can be created as shown
below. Note, that we specify the limits for randomization here.

.. code:: python

    config_dict = {
        'type': 'box',
        'geometry': {
            'position' : [2.0, 0.0, 2.0],
            'width': 0.2,
            'height': 0.2,
            'length': 0.2,
        },
        'movable': True,
        'high': {
             'position' : [5.0, 5.0, 1.0],
            'width': 0.2,
            'height': 0.2,
            'length': 0.2,
        },
        'low': {
            'position' : [0.0, 0.0, 0.5],
            'width': 0.2,
            'height': 0.2,
            'length': 0.2,
        }
    }
    obstacle_2 = BoxObstacle(
      name="obstacle_2", content_dict=config_dict
    )

A dynamic obstacle can be defined as:

.. code:: python
   
    config_dict = {
        "type": "sphere",
        "geometry": {
          "trajectory": ["2.0 - 0.1 * t", "-0.0", "0.1"],
          "radius": 0.2
        },
    }
    obstacle_3 = DynamicSphereObstacle(
        name="obstacle_3", content_dict=config_dict
    )
