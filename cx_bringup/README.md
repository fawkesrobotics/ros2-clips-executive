# cx_bringup
---
Provides an example bringup mechanism for CX applications.

This package includes the configuration file for all main CX nodes inside the `params` folder. The CLIPS Executive environment (ClipsExecutive node) is highly configurable using the dedicated `clips_executive.yaml` config file. The developer could configure most of the CX's behavior, such as defining multiple agents and scenarios, or the debugging level of CLIPS. The rest of the core nodes are configured using the `cx_params.yaml` file.

It provides a dedicated node (`cx_node`) inside the `src` folders, which ensures the composed bringup of the three core CX nodes (Environment Manager, Plugins Manager, and Clips Executive node). It is based on the [ROS2 Composition](https://docs.ros.org/en/galactic/Tutorials/Composition.html) and ensures that the CLIPS-related nodes run inside the same process, which is a prerequisite for the interaction to CLIPS. It also provides the `plansys2_node`, which launches only the Planning-based nodes inside Plansys2 and excludes the Plansys2 Executioner, as the CX is responsible for the execution of plans.

## Launch CX standalone
The launch file uses the `cx_node.cpp` and the Lifecycle Manager.

Make sure to source the CX WS and run:

```
ros2 launch cx_bringup cx_launch.py <options>
```
Where `<options>` can be replaced by, for example:
```
model_file=<path/to/domain.pddl>
cx_params_file=<path/to/params.yaml>
clips_executive_params_file=<path/to/cx_params.yaml>
log_level=<info/debug/...>
```

## Launch CX with Plansys2 for planning
### Prerequisites:
- Install or Build [Plansys2](https://navigation.ros.org/build_instructions/index.html#) (cf. Build)

The launch file uses the `cx_node.cpp`, the `plansys2_node.cpp`, and the Lifecycle Manager.

Make sure to source the CX and the Plansys2 WS and run:

```
ros2 launch cx_bringup cx_launch_with_plansys2.py <options>
```
Where `<options>` can be replaced by similar parameters.

## Launch CX with provided skills
### Prerequisites:
- Install or Build [Plansys2](https://navigation.ros.org/build_instructions/index.html#) (cf. Build)

The launch file uses the `cx_node.cpp`, the `plansys2_node.cpp`, the Lifecycle Manager, and a provided implementation of skill nodes, which derive from the SkillExecution class (inside the Skill Execution package). Examples can be found inside the `cx_example_skill_nodes` package.

Make sure to source the CX and the Plansys2 WS and run:

```
ros2 launch cx_bringup cx_with_skills.py <options>
```
Where `<options>` can be replaced by similar parameters.
