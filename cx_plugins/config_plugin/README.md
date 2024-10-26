# cx_config_plugin
This package offers the `cx::ConfigPlugin` CLIPS plugin to read config values from yaml.

## Usage
Register this plugin with the plugin manager. It requires no additional configuration, an example setup is shown below:

```yaml
clips_manager:
  ros__parameters:
    environments: ["main"]
    main:
      plugins: ["config"]

    config:
      plugin: "cx::ConfigPlugin"
```
## CLIPS Features

##### Deftemplates
```lisp
; Asserted by the config-load function for each configuration value in the parsed yaml file.
(deftemplate confval
  (slot path (type STRING))
  (slot type (type SYMBOL) (allowed-values FLOAT UINT INT BOOL STRING))
  (slot value)
  (slot is-list (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (multislot list-value)
)
```

##### Functions
```lisp
; load all config values from a file (absolute path) given a prefix
(config-load ?file-lex ?prefix-lex)
```
