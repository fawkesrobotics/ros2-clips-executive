Environment Manager Configuration
#################################
To just launch a bare unconfigured node, just run the following command:

.. code-block:: bash

  ros2 run cx_clips_env_manager clips_node

A more practical way to to start the node (including automatic transitions to activate and finally deactivate on exit) is also provided (Source code on :source-master:`GitHub <cx_bringup/src/cx_node.cpp>`).

.. code-block:: bash

  ros2 run cx_bringup cx_node

This is especially useful when combining it with a :rostut:`launch file <Tutorials/Intermediate/Launch/Creating-Launch-Files.html>` to automatically pass configuration options to the node.
An example launch file can be found on :source-master:`GitHub <cx_bringup/launch/cx_launch.py>`.

Parameters
**********
In the following, the node parameters of the environment manader node are listed.

:environments:

  ============== =======
  Type           Default
  -------------- -------
  string vector  []
  ============== =======

  Description
    Each entry should be a unique name for an environment. Each environment *<env>* is further specified with the parameters below.


:`<env>.log_clips_to_file`:

  ============== =======
  Type           Default
  -------------- -------
  bool           true
  ============== =======

  Description
    If set to *true*, each environment creates a log file <env-name>_<timestamp>.log in the ROS log directory contianing all CLIPS output.

:`<env>.watch`:

  ============== =======
  Type           Default
  -------------- -------
  string vector  []
  ============== =======

  Description
    Watch items to define verbosity of the environment.
    Possible values: ["all", "facts", "instances", "slots", "rules", "activations", "messages", "message_handlers", "generic_functions", "methods", "deffunctions", "compilations", "statistics", "globals", "focus"]


:`<env>.plugins`:

  ============== =======
  Type           Default
  -------------- -------
  string vector  []
  ============== =======

  Description
    List of unique plugin names.
    Each plugin *<plugin>* is further specified with parameters depending on the chosen specifier (plugin type).

:`<env>.redirect_stdout_to_debug`:

  ============== =======
  Type           Default
  -------------- -------
  bool           false
  ============== =======

  Description
    If set to true, all CLIPS output that logs to the ``clips::STDOUT="stdout"`` router is logged in ROS via ``RCLCPP_DEBUG`` instead of ``RCLCPP_INFO``.
    This is useful when the (often rather verbose) default CLIPS output should be captured in the log file but should not spam the main terminal.
    CLIPs programs that use other routers (such as ``info`` or ``t``) to print output still appear using ``RCLCPP_INFO``.

:`<plugin>.plugin`:

  ============== =======
  Type           Default
  -------------- -------
  string         ""
  ============== =======

  Description
    The specifier of the pluginlib plugin.
    This needs to name a plugin type registered in the ament index.

The configuration options of plugins are described :ref:`here <plugins>`.

Example
*******
.. code-block:: yaml

  clips_manager:
    ros__parameters:
      environments: ["main"]
      main:
        plugins: ["executive","files"]
        log_clips_to_file: true
        watch: ["rules", "facts"]
      # plugin definitions
      # ...
