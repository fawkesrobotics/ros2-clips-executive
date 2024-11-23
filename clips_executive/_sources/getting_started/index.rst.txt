Getting Started
===============

This document shows how to install the |CX| so you are set up to explore it's :ref:`fundamentals`.

A general project description can be found on the :ref:`main landing page <main>`.

Installation
############
This document assumes you have a basic ROS 2 installation. If not, refer to the official `ROS documentation`_.

Binary Installation
+++++++++++++++++++

The easiest way to install the |CX| is via your package manager:

.. note::

   The |CX| is currently in the process of being indexed, hence binary packages might not be available yet.

.. code-block:: bash

   sudo apt intall ros-<ros2-distro>-clips-executive


Installation from Source
+++++++++++++++++++++++++

.. todo::

   dependency.repos should not be necessary

All dependencies are listed in the :source-master:`dependency.repos` file of the |CX| project.
This enables setting up a dependency workspace using `vcstool`.

The following steps will set up two workspaces, one for dependencies of the |CX| and one for the main project. Adjust target locations as needed.

.. code-block:: bash

    mkdir -p ~/ros2/{clips_executive_ws,deps_clips_executive_ws}/src
    git clone -b tviehmann/major-cleanup https://github.com/fawkesrobotics/ros2-clips-executive.git ~/ros2/clips_executive_ws/src/ros2-clips-executive
    cd ~/ros2/deps_clips_executive_ws/src
    vcs import < ~/ros2/clips_executive_ws/src/ros2-clips-executive/dependency.repos
    cd ~/ros2/deps_clips_executive_ws/
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=OFF

Then source the workspace with dependencies and build the CLIPS-Executive:

.. code-block:: bash

    source ~/ros2/deps_clips_executive_ws/install/setup.bash
    cd ~/ros2/clips_executive_ws/
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=OFF
    source ~/ros2/clips_executive_ws/install/setup.bash

Running the Example
+++++++++++++++++++


.. todo::

   we should probably provide a more convincing example

The following example showcases simple interactions between CLIPS and ROS via a topic:


1. Run the `ros_msgs` demo:

.. code-block:: bash

   ros2 launch cx_bringup cx_launch.py manager_config:=plugin_examples/ros_msgs.yaml

2. Publish a String on the `/ros_cx_in` topic:

.. code-block:: bash

   ros2 topic pub /ros_cx_in std_msgs/msg/String "{data: 'Hello'}"

3. Listen on the `/ros_cx_out` topic for responses:

.. code-block:: bash

   ros2 topic echo /ros_cx_out

You should receive a `"Hello World!"` message for each message received in the CLIPS environment.

Check out the :ref:`Tutorials` page to learn how to write your own CLIPS applications.
