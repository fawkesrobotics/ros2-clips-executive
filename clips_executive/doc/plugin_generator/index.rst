.. _usage_cx_ros_comm_gen:

ROS Interface Plugin Generator
==============================

Source code on :source-master:`GitHub <cx_ros_comm_gen>`.

In order to interface with ROS directly without the need for generic introspection, the |CX| offers a plugin generator for individual interfaces (messages, services or actions).

Note that the :ref:`Generic ROS Msg Plugin <usage_ros_msgs_plugin>` provides an alternative for ROS messages specifically, which works directly through introspection instead of relying on individual plugins for each message type.
It also allows to call services through clients, but can not create clients.

Generic support for services and actions is not implemented yet due to missing generic interfaces in rclcpp, hence the plugin generator is needed in order to deal with services and actions.

.. toctree::
   :maxdepth: 2

   gen_info.rst
   gen_msgs.rst
   gen_srvs.rst
   gen_action.rst
