# cx_feature_manager
This package offers a lifecycle node that handles the dynamic loading of CLIPS features.

## Usage
A feature manager node is configured through it's parameters that specify the registered features that should be loaded.

It is tightly coupled with the CLIPS environment manager from the *cx_clips_env_manager* package.
The feature manager is responsible for loading and unloading of features as well as providing interfaces to load and unload features. These interfaces are provided to the environments provided by the environment manager.

Create an environment manager first, then create the feature manager and pass a pointer to the environment manager to it using the `pre_configure()` function:
```c++
  clips_env_manager_node = std::make_shared<cx::CLIPSEnvManagerNode>();
  clips_features_manager_node = std::make_shared<cx::ClipsFeatureManager>();
  clips_features_manager_node->pre_configure(clips_env_manager_node);
```

Then the node needs to be activated according to the [rclcpp lifecycle](https://docs.ros.org/en/rolling/Tutorials/Demos/Managed-Nodes.html).
### Parameters
For a full configuration example including all features provided by the CLIPS Executive, please see the *cx_bringup* package.

The feature manager looks for the list of features that needs to be loaded (under the `clips_feature_list` key) which then is further specified by additional information under the `clips_features` key.

A minimal working example is depicted below, where a custom feature is loaded that is realized through the `cx::ExampleFeature`, as specified throught the required `plugin` key (identifying a feature name with the associated pluginlib plugin implementation):
```yaml
clips_feature_manager:
  ros__parameters:
    # List of all features to be considered
    clips_feature_list: ["custom_feature"]
    # Each entry of the list needs to additionally specify the plugin implementation
    clips_features:
      custom_feature:
        plugin: "cx::ExampleFeature"
```

#### Passing Parameters to Features
Some features may expect additional parameters. These are passed through the feature manager and require explicit definition under the feature name key as shown below:
```yaml
clips_feature_manager:
  ros__parameters:
    clips_feature_list: ["custom_feature", "clips_protobuf"]
    clips_features:
      custom_feature:
        plugin: "cx::ExampleFeature"
      clips_protobuf:
        plugin: "cx::ProtobufFeature"
        # List all parameter names here
        feature_parameters: ["protobuf_path"]
        # Each param name needs to specify the value and type afterwards
        protobuf_path:
          value: "proto"
          type: "string"
```
The possible types are:
 - "string"
 - "integer"
 - "double"
 - "bool"
 - "byte-array"
 - "string-array"
 - "integer-array"
 - "double-array"
 - "bool-array"

### Lifecycle States
#### on_configure
Initializes all registered features according the given parameter (by calling the respective initialize() function of each feature).

Creates service to destroy a feature (TODO: why?)
```c++
"clips_feature_manager/destroy_feature_context"
```

### Registering Bindings
Register the generated features with the feature manager from the *cx_feature_manager* package via it's configuration file.

### Using the Bindings in CLIPS

##### Deftemplates
```lisp
(confval (path ?path-str) (type ?type-sym) (value ?val) (is-list ?is-list-sym) (list-value ?list-val))
```

##### Functions
```lisp
; load all config values from a file (absolute path) given a prefix
(config-load ?file-lex ?prefix-lex)
```
