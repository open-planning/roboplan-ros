RoboPlan ROS Examples
=====================

As noted in :ref:`design_philosophy`, RoboPlan ROS is not intended to provide monolithic solutions that fit all planning applications.
The packages in the `roboplan_ros_examples <https://github.com/open-planning/roboplan-ros/tree/main/roboplan_ros_examples>`_ folder provide jumping off points for Developers looking to use RoboPlan in ROS 2 environments.
Each example is a self-contained application that demonstrates a specific workflow.

The examples use ``ros2_control`` mock hardware, so no physical robot is required to run them.


Sample RRT Plan and Execute
---------------------------

This example demonstrates a complete plan-preview-execute loop using RRT motion planning with the Franka FR3 arm.

The example uses an interactive marker in RViz along with ROS ``Trigger`` services to:

1. **Set a target pose** by dragging the interactive marker in the RViz panel.
2. **Plan** a trajectory by right clicking and pressing "Plan", or by calling the ``/plan_and_execute_node/plan`` service.
3. **Preview** the planned trajectory by pressing "Preview", or by calling the ``/plan_and_execute_node/preview`` service.
4. **Execute** the trajectory by pressing "Execute", or by calling the ``/plan_and_execute_node/execute`` service.
   This wraps a call to the FR3's ``JointTrajectoryController``.
5. **Reset** the iMarker by pressing "Reset", or by calling the ``/plan_and_execute_node/reset`` service.

To launch the demo,

::

    ros2 launch roboplan_ros_franka franka_example_planning.launch.yaml

.. figure:: media/sample_rrt_execution.gif
   :width: 600px

   Sample RRT planning and execution with the Franka FR3 arm.


Sample Cartesian Servoing with OInK
-----------------------------------

This example demonstrates real-time Cartesian servoing using OInK (Optimal Inverse Kinematics) with the Franka FR3 arm.

Unlike the plan-and-execute workflow, there is no trajectory planning or preview step.
A background control loop continuously solves a single-step IK optimization, then publishes joint commands directly to track the interactive marker's pose.
The solver uses a prioritized task hierarchy:

- **Priority 1**: A frame task tracks the end-effector pose set by the interactive marker.
- **Priority 2**: A configuration task that regularizes toward the starting joint configuration.

Joint position and velocity limits are enforced as QP constraints.

To launch the demo,

::

    ros2 launch roboplan_ros_franka franka_oink.launch.yaml

.. figure:: media/sample_oink.gif
   :width: 600px

   Sample OInK pose tracking with the Franka FR3 arm.
