Getting Started
===============

First, clone this repo *including submodules*.

::

    git clone --recursive https://github.com/open-planning/roboplan-ros.git
    cd roboplan-ros

---

Pre-built
---------

RoboPlan ROS will soon be installable from pre-packaged binaries.
For now, users should build from source using one of the methods below.

Compiling
---------

We support a handful of methods for building the ROS 2 wrappers from source.

Pixi
~~~~

For isolated development work we recommend the `Pixi <https://pixi.sh>`_ package management tool.
ROS and all required dependencies are installable through `conda` and `Robostack <https://robostack.github.io>`_.

Different versions of ROS (humble, jazzy, and kilted) are supported with pixi environments.
The build relies on the `pixi-build-ros <https://prefix-dev.github.io/pixi-build-backends/backends/pixi-build-ros/#env>`_ backend to manage ROS dependencies from the workspace.

First, install Pixi using `these instructions <https://pixi.sh/latest/#installation>`_.

Once set up, you can run the ``pixi`` tasks as follows.

::

    # Install the frozen environment
    pixi install -e jazzy --frozen

    # Setup the environment
    pixi run -e jazzy setup-colcon

    # Build
    pixi run -e jazzy build

    # Run tests, must manually run build prior to executing
    pixi run -e jazzy test

    # Show tests results
    pixi run -e jazzy test-result

Once the workspace is built, it can be used as a "normal" ROS 2 workspace.
That is, the workspace can be sourced and used as expected.

::

    # Activate the jazzy environment
    pixi shell -e jazzy

    # Build with the existing install dirs
    colcon build

    # Source it
    source install/setup.bash

    # Or test,
    colcon test

    # Or launch an example
    ros2 launch roboplan_ros_franka franka_example_ik.launch.py

The examples from the upstream repository should now be available, as well:

::

    python3 roboplan/roboplan_examples/python/example_scene.py


**NOTE:** All ROS distros will share build/install directories by default.
Be sure to clear them out when switching between ROS versions or things will break.

**NOTE:** The ``pixi-build-ros`` backend requires that all ``package.xml`` files be referenced in the ``[dev]`` block of the ``pixi.toml`` file.
This is a slight annoyance due to the submodules, but many of those packages will be installed from Conda once RoboPlan is more stable.

---

Docker Development
~~~~~~~~~~~~~~~~~~

A `Docker Compose <https://docs.docker.com/compose/>`_ workflow is also provided.

To build and run,

::

    # Build the image, defaults to jazzy but can be overridden by setting $ROS_DISTRO
    docker compose build ros

    # Start the container
    docker compose up ros

    # Attach to the running container
    docker compose exec ros bash

This will drop the user into a pre-compiled colcon workspace with the source mounted in the image.
Users can edit, rebuild, and launch applications from inside the container as needed.

---


ROS 2 baremetal (colcon)
~~~~~~~~~~~~~~~~~~~~~~~~

Lastly, as a ``colcon`` package, the wrappers can be compiled in any valid ROS 2 workspace.

::

    mkdir -p ~/roboplan_ws/src
    cd ~/roboplan_ws/src
    git clone --recursive https://github.com/open-planning/roboplan-ros.git

**NOTE:** To compile the bindings you should install nanobind from pip:

::

    pip install nanobind

Source your favorite ROS distro and build the workspace.

::

    source /opt/ros/jazzy/setup.bash
    cd ~/roboplan_ws
    rosdep install --from-paths src -y --ignore-src
    colcon build

**NOTE:** Additional dependencies may be required outside of rosdistro.
We recommend referring to the upstream documentation, or to the included ``.docker/Dockerfile`` for information on a base system setup.

---
