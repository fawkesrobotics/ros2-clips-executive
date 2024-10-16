# cx_config_plugin
This package offers the `cx::ConfigPlugin` CLIPS plugin to read config values from yaml.

## Usage
The usage involves two steps:
1. Loading this CLIPS-Executive plugin via the **cx_clips_env_manager** package
2. Using the functions and templates

### Registering Bindings
The config plugin does not need any additional configuration:
```
clips_manager:
  ros__parameters:
    environments: ["main"]
    main:
      plugins: ["config"]

    config:
      plugin: "cx::ConfigPlugin"
```
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
