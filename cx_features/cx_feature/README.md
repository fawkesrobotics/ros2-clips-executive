# cx_feature
Base class for all CLIPS features.

CLIPS features are plugins that can be dynamically loaded by the CX feature
manager to interface with CLIPS Environments via the CLIPS C API (see the
*Advanced Programming Guide* at [clipsrules.net](https://clipsrules.net/).

The most common usage is to provide user-defined functions or external data
to CLIPS, e.g., protobuf messages along with functions to access their content.

## Writing a CLIPS Feature
As CLIPS features are realized via [pluginlib](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Pluginlib.html), it boils down to building a plugin and exporting it's information properly.

1. Write a feature class that inherits from the ClipsFeature base class.
2. Export the feature via `PLUGINLIB_EXPORT_CLASS`.
3. Export a plugin description via cmake `pluginlib_export_plugin_description_file` with base class package `cx_feature`.

As an example package that can serve as a boilerplate, please have a look at the *cx_example_feature*.
