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
