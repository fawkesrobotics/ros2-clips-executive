# cx_config_feature
This package offers a CLIPS feature to read config values from yaml(https://plansys2.github.io/index.html).

## Usage
The usage involves two steps:
1. Loading the generated bindings as CLIPS Executive features
2. Using the functions and templates

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
