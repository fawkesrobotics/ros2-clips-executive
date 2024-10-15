# cx_plugin
Base class for all CLIPS plugins.

CLIPS plugins are plugins that can be dynamically loaded by the CX plugin
manager to interface with CLIPS Environments via the CLIPS C API (see the
*Advanced Programming Guide* at [clipsrules.net](https://clipsrules.net/).

The most common usage is to provide user-defined functions or external data
to CLIPS, e.g., protobuf messages along with functions to access their content.

## Writing a CLIPS Plugin
As CLIPS plugins are realized via [pluginlib](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Pluginlib.html), it boils down to building a plugin and exporting it's information properly.

1. Write a plugin class that inherits from the ClipsPlugin base class.
2. Export the plugin via `PLUGINLIB_EXPORT_CLASS`.
3. Export a plugin description via cmake `pluginlib_export_plugin_description_file` with base class package `cx_plugin`.

As an example package that can serve as a boilerplate, please have a look at the *cx_example_plugin*.
