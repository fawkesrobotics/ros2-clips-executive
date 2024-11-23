.. _fundamentals:

Fundamentals
============

The |CX| mainly provides a ROS `lifecycle`_ node (:docsite:`Package <cx_clips_env_manager>`, :source-master:`Github <cx_clips_env_manager>`) that manages a CLIPS instance and has the following responsibilities:

 - dynamically create and destroy CLIPS environments
 - log the CLIPS output to ROS and to files.
 - Extend CLIPS environments by plugins (using `pluginlib`_) that allow to externally customize CLIPS environments further via the CLIPS C++ API  (e.g., to inject C++ functions to CLIPS environments)

It provides a clear execution flow guided through the lifecycle:

 - On being configured, it parses all parameters to determine the environmnents and plugins to load. It initializes the environments and plugins.
 - On being activated, it first loads all requested plugins, before calling **(reset)**  in the environments, refreshing all agendas and calling **(run)** to start rule execution.
 - On being deactivated, it first asserts a ``(executive-finalize)`` fact before refreshing all agendas and calling **(run)** one more time to allow cleanup routines to execute before shutdown.

Similarly, each dynamically created environment goes through the same steps as on activation and each dynamically destroyed environment goes through the same steps as on deactivation.

.. toctree::
   :maxdepth: 2

   node_start.rst
   logging.rst
   dynamic.rst
