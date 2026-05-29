RoboPlan ROS Examples
=====================

As noted in :ref:`design_philosophy`, RoboPlan ROS is not intended to provide all in one solutions that fit all planning applications.
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
   This wraps a call to the FR3's ``JointTraectoryController``.
5. **Reset** the iMarker by pressing "Reset", or by calling the ``/plan_and_execute_node/reset`` service.

To launch the demo,

::

    ros2 launch roboplan_ros_franka franka_example_planning.launch.yaml

.. figure:: media/sample_rrt_execution.gif
   :width: 600px

   Sample RRT planning and execution with the Franka FR3 arm.
