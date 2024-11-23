# cx_plugin
Base class for all CLIPS plugins.

CLIPS plugins are plugins that can be dynamically loaded by the CX plugin
manager to interface with CLIPS Environments via the CLIPS C API (see the
*Advanced Programming Guide* at [clipsrules.net](https://clipsrules.net/).

The most common usage is to provide user-defined functions or external data
to CLIPS, e.g., protobuf messages along with functions to access their content.

As an example package that can serve as a boilerplate, please have a look at the **cx_example_plugin** package.

## Writing a CLIPS Plugin
As CLIPS plugins are realized via [pluginlib](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Pluginlib.html), it boils down to building a plugin and exporting it's information properly.

1. Write a plugin class that inherits from the ClipsPlugin base class provided here.
2. Export the plugin via `PLUGINLIB_EXPORT_CLASS`.
3. Export a plugin description via cmake `pluginlib_export_plugin_description_file` with base class package `cx_plugin`.

In the following, the virtual functions and their purpose are described.
### initialize()
This function is called exactly once when a plugin is loaded, before it actually provides it's features to CLIPS environments.

Typical uses include
1) Reading of parameters from specified in the parent node.
2) Initialization of Environmnent-agnostic class members.

### clips_env_init(LockSharedPtr<clips::Environment> &env)
Called once for every environment the plugin is loaded into.

Typically this is used to inject user-defined functions, define templates etc.

It should return `true`, if the initialization of the environment was successful. If it returns `false`, then the plugin manager will call `clips_env_destroyed` to allow proper cleanup.

#### Environment Reset
However, be aware that each environment is reset afterwards on startup.

This in particular means that all asserted facts and instances are deleted and it makes no sense to directly assert facts in this function.

If your plugin should provide initial facts, it should therefore use `deffacts` instead, which would assert the facts on reset.

#### Multithreading and CLIPS
The passed environment is wrapped in an object containing a mutex.
**Operations on CLIPS environments are not thread-safe**, hence this mutex should be used to guard any scope that accesses the environment from a different thread.

Note that it **must not guard the scope within clips_env_init**, as the plugin manager already guards the environment.

It also **must not guard the scope inside of CLIPS user-defined functions**, as they are called from within the CLIPS engine, hence are already guarded by the CLIPS-Executive when it calls the `run` command for the environment.

Also, be aware that CLIPS will evaluate the LHS of rules any time the fact base changes, which can easily cause deadlocks if not handled properly in multi-threaded plugins.
Consider this example from **cx_ros_msgs_plugin** which allows interactions with ROS topics:
1. The asynchronous subscription callbacks adds messages and meta-data to an unordered map, which needs to be guarded by a mutex `map_mtx_` as multiple write operations could occur at the same time when multi-threaded executors and reentrant callback groups are used.
Additionally, the messages are asserted as facts (holding a reference to the message) in the callback.
2. The **ros-msgs-get-field** UDF allows to retrieve fields of messages. As fields may contain messages, this again might need to store meta-data, hence it also needs to lock `map_mtx_`.

A simple implementation using a scoped lock for the entire scope of the callback and the entire scope of **ros-msgs-get-field** could cause a deadlock if the ros-msgs-get-field function is called on the left-hand side of a rule, e.g., like this:
```lisp
(defrule deadlock-example
  (ros-msgs-subscription (topic ?sub))
  ?msg-f <- (ros-msgs-message (topic ?sub) (msg-ptr ?inc-msg))
  (test (and (= 0 (ros-msgs-get-field ?inc-msg "velocity"))))
...
```
The assertion of the fact in the callback triggers the conditional check in the rule which therefore tries to lock `map_mtx_` blocked by the callback function itself.

#### Environment Context

Each environment also holds an instance of the `CLIPSEnvContext` class from the **cx_utils** package that can be retrieved via a static function:
```c++
cx::CLIPSEnvContext::get_context(clips::Environment *env)
```
This instance contains the name and the environment as well as the LockSharedPtr to the environment.

Hence, if you plan on performing any asynchronous tasks that are triggered from CLIPS user-defined functions (which provide only a raw pointer to an environment), you can retrieve the mutex via this additional context to ensure thread-safe guarding of the CLIPS environment.

### clips_env_destroyed(LockSharedPtr<clips::Environment> &env)
Called once for every environment that needs to unload a previously loaded plugin's feature.
Also is called when a `clips_env_init` call returns `false`.

Typically this is used to undefine user-defined functions, templates, etc.

Note that the environment **must not guard the scope within clips_env_destroyed**, as the plugin manager already guards the environment.

### finalize()
This function is called exactly once when a plugin is finally unloaded again, hence all resources should be freed for a graceful destruction of the object.
