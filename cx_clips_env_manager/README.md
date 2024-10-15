# cx_clips_env_manager
This is the central package of the CLIPS-Executive that provides the CLIPS Environment Manager.

[CLIPS](https://clipsrules.net/) (C Language Integrated Production System) is a rule-based programming language designed for building expert systems. It was developed in the 1980s by NASA to provide a portable, efficient, and easy-to-use environment for developing knowledge-based applications.
CLIPS uses a production rule system, where knowledge is represented as if-then rules. These rules operate on facts, which represent data or conditions in the system. The inference engine matches rules against facts and executes the applicable rules based on a conflict resolution strategy.

The ROS 2 CLIPS-Executive is a wrapper to integrate CLIPS with the ROS 2 ecosystem.

The documentation assumes basic understanding regarding the usage of CLIPS. If you never worked with CLIPS before, check out the documentation of CLIPS and get familiar with the functionalities first using the documentation at [clipsrules.net](https://clipsrules.net/).

The [clips_vendor](https://github.com/carologistics/clips_vendor) package provides a packaged version of CLIPS suitable for the ROS ecosystem (and is also used by the CLIPS-Executive).

## Responsibilities
The CLIPS Environment Manager is the central component that allows to:
 - dynamically create and destroy CLIPS environments
 - log the CLIPS output to ROS and to files
 - Extend CLIPS environments by plugins (using the *cx_plugin_manager* and [pluginlib](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Pluginlib.html)) that allow to externally customize CLIPS environments further via the CLIPS C++ API  (e.g., to inject C++ functions to CLIPS environments)

 It provides a clear execution flow guided through the lifecycle:
 - On being configured, it parses all parameters to determine the environmnents and plugins to load. It initializes the environments and plugins.
 - On being activated, it first loads all requested plugins, before calling **(reset)**  in the environments, refreshing all agendas and calling **(run)** to start rule execution.
 - On being deactivated, it first asserts a `(executie-finalize)` before refreshing all agendas and calling **(run)** one more time to allow cleanup routines to execute before shutdown.

 Similarly, each dynamically created environment goes through the same steps  as on activateion and each dynamically destroyed environment goes through the same steps as on deactivation.

## Usage
The environment manager node can be configured via ROS parameters to automatically setup environments on startup.
Additionally, it provides services to create and destroy environments dynamically at runtime and to load and unload plugins into environments.

### Parameters
The example configuration below shows the configuration for an instance of the CLIPS environment manager that creates two environments on start.
It utilzes three different plugins, but the "executive" plugin is used for both instances.

For a more comprehensive example, check out the **cx_bringup** package.
```
clips_manager:
  ros__parameters:
    environments: ["main", "other"]
    main:
      # which plugins should be loaded. If undefined, defaults to no plugins
      plugins: ["executive","files_other"]

      # Should CLIPS activities be logged to files? Defaults to true
      log_clips_to_file: true

      # What changes should be watched (and logged). Defaults to not watching
      # anything. By watching "rules" and facts one effectively get a full
      # program trace of the execution execution, but CLIPS supports more,
      # namely:
      #   "all", "facts", "instances", "slots", "rules", "activations",
      #   "messages", "message_handlers", "generic_functions", "methods",
      #   "deffunctions", "compilations", "statistics", "globals", "focus"
      watch: ["rules", "facts"]

    # no configuration for "other" means that default values are chosen

    # plugin definitions
    # ...
```

### Services
These services are provided by each environment manager node:
- */create_env* for creating environments
- */destroy_env* for destroying environments
- */load_plugin* for loading of plugins in environments.
- */unload_plugin* for unloading of plugins from environments.
- */list_plugins* for listing all plugins loaded in an environment.

## Plugins
Plugins are specializations of of the *cx_plugin* base class and are handled as follows:
 - Each plugin is initialized exactly once before usage by calling it's `initialize()` function.
 - When an environment is loaded, all specified plugins are **loaded in order** of the respective plugins parameter.
 - Whenever an environment needs to load a plugin, it's `clips_env_init()` function is called once. Loading the same plugin again before unloading it first, results in an error (and the function is not called again).
 - Whenever an environment needs to unload a plugin, it's `clips_env_destroyed()` is called once. Unloading the same plugin again before loading it first, results in an error (and the function is not called again).
 - Before an environment is destroyed, all plugins are **unloaded in reverse order** of loading.
 - On destruction of a plugin, the `finalize()` function is called exactly once.
 - Before a plugin is destroyed, it is unloaded from all environments.

 The CLIPS-Executive provides a number of useful plugins (see the **cx_feature** package for an exhaustive list), it also provides generators to interface with ROS topics, services and actions (see the **cx_ros_comm_gen** package.
