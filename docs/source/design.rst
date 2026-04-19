Design Philosophy
=================

We follow the `Design Philosophy <https://roboplan.readthedocs.io/en/latest/design.html>`_ of the core RoboPlan repo
(excepting that we have a hard dependency on ROS 2 as a middleware, of course).

Every robot system deployed in the real world has a unique set of requirements and needs.
We are not intending to provide an all-in-one solution that fits each use case.
Instead, these packages are intended to be a *thin wrapper* around the motion planning tools provided by RoboPlan.
Developers can use existing examples to build solutions as necessary for their specific applications.

With that, we have the following additional tenets for the ROS ecosystem:

No nodes
--------

ROS 2 nodes inherently limit the execution model, lifecycle, parameter handling, and threading an application.
These choices should always be left to the application developer, not the library.
While we have a dependency on ROS 2 middleware, we will not commit to managing monolithic ROS 2 nodes that provide access to all of RoboPlan's capabilities.

As such, fully fleshed-out nodes belong in the ``roboplan_ros_examples`` packages only.

Prefer standard ROS tools
-------------------------

When bridging between RoboPlan and ROS, we prefer well-known message types (``sensor_msgs/JointState``, ``trajectory_msgs/JointTrajectory``, etc.) over custom message definitions.
We should leverage as much of the existing infrastructure as possible, rather than create friction with custom message definitions.
A new message type should only be introduced when no existing standard type can faithfully represent the data.

This tenet applies to other plugins like RViz widgets, etc.
The ROS 2 ecosystem has a great wealth of existing infrastructure, and we bias towards using what is there.

Examples over configuration
---------------------------

Rather than providing parameterized launch files, YAML configuration schemas, or plugin-based architectures, we provide concrete, minimal examples.
These examples should provide a sufficient starting point to let users wire up RoboPlan capabilities as needed for their specific setup.
