# cx_plansys2_plugin
This package offers a CLIPS plugin to interface with [plansys2](https://plansys2.github.io/index.html).

## Usage
The usage involves two steps:
1. Loading the generated bindings as CLIPS Executive plugins
2. Using the functions and templates

### Registering Bindings
Register the generated plugins with the plugin manager from the *cx_plugin_manager* package via it's configuration file.

### Using the Bindings in CLIPS

##### Deftemplates
This plugin does not load any deftemplates itself, but assumes the following deftemplates are defined from the *cx_clips_executive* core definitions:

1. plan-action ([plan.clp](../../cx_clips_executive/clips/core/plan.clp))
2. pddl-plan-feedback ([plan.clp](../../cx_clips_executive/clips/core/pddl.clp))

##### Functions
```lisp
; Add a domain instance to a problem client
(psys2-add-domain-instance ?name-lex ?type-lex)
; Add a predicate to the problem expert
(psys2-add-domain-predicate ?predicate-lex)
; Clear the knowledge from the problem expert
(psys2-clear-knowledge ?predicate-lex)
; Call the plansys planner with a certain goal
; asserts a plan-feedback and plan + plan-action facts if successful
(psys2-call-planner ?goal-id-lex ?goal-lex ?plan-lex)
```
