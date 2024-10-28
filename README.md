# ROS2 Clips-Executive
The ROS 2 CLIPS-Executive (**CX**) offers an integration for CLIPS with the ROS 2 ecosystem for developing knowledge-based applications.

[CLIPS](https://clipsrules.net/) (C Language Integrated Production System) is a rule-based programming language designed for building expert systems. It was developed in the 1980s by NASA to provide a portable, efficient, and easy-to-use environment for developing knowledge-based applications.
CLIPS uses a production rule system, where knowledge is represented as if-then rules. These rules operate on facts, which represent data or conditions in the system. The inference engine matches rules against facts and executes the applicable rules based on a conflict resolution strategy.

The documentation assumes basic understanding regarding the usage of CLIPS. If you never worked with CLIPS before, check out the documentation of CLIPS and get familiar with the functionalities first using the documentation at [clipsrules.net](https://clipsrules.net/).

The [clips_vendor](https://github.com/carologistics/clips_vendor) package provides a packaged version of CLIPS suitable for the ROS ecosystem (and is also used by the CLIPS-Executive).

This project is inspired by the original CLIPS-Executive from the [Fawkes robotics framework](https://ojs.aaai.org/index.php/ICAPS/article/view/3544/3412)([github](https://github.com/fawkesrobotics/fawkes)).

## Overview

### Environment Manager

The heart of the CLIPS-Executive is the CLIPS Environment Manager node of the **cx_clips_env_manager** package that allows to create and destroy CLIPS environments as well as extending these environments through dynamically loaded plugins.

### Plugins

The CLIPS-Executive provides a number of plugins out-of-the-box, but it is also easy to write new plugins to customize CLIPS to your needs.
It also provides generators to provide plugins for communicating with ROS topics, services and action clients/servers (see the **cx_ros_comm_gen** package).

The provided plugins are (in alphabetical order):
 - **cx::ConfigPlugin**: Parse the content of yaml files and load them as facts.
 - **cx::ExecutiveFeature**: Continuously refresh the Agendas and run the inference engine while optionally also injecting ROS time facts in the environment.
 - **cx::FileLoadPlugin**: Load CLIPS code from files into the environment.
 - **cx::PddlParserPlugin**: Parse PDDL domains and represent them via facts.
 - **cx::Plansys2Plugin**: Interface with [Plansys2](https://plansys2.github.io/) for PDDL planning.
 - **cx::ProtobufPlugin**: Send and receive protobuf messages through CLIPS.
 <!---
 - **cx::SkillExecutionPlugin**: Generalized Executor interface
-->

### Goal Reasoning With CLIPS

Lastly, the **cx_goal_reasoning** package contains useful CLIPS source code for building goal reasoning (GR) applications, where the program flow is dictated through the refinement of goals through a goal lifecycle. This can be of great help when building bigger CLIPS-based applications, as the program flow in large rule-based applications can become overwhelming, if an overaching structure is missing.

1.  The central entities for goal reasoning are goals. They describe the objectives to achieve or conditions to maintain, e.g., to bring a certain object from point A to point B. The CX offers GR with the goal refinement mechanism in the form of goal lifecycles. This way, each goal goes through a predefined set of goal modes during its lifespan. This makes the program flow explicit and allows constant goal monitoring, observation of agent status and actions, as well as tracking and reacting to internal or exogenous factors.
2. Planning: Provides PDDL-based planning by interacting with and utilizing a dedicated planner/planning system (currently planning with [Plansys2](https://github.com/IntelligentRoboticsLabs/ros2_planning_system) is supported).
3. Execution: Provides the means to execute all PDDL actions in a provided plan on the side of the embedding system. Skills in our project represent the identifier of the PDDL action, for which an executor node is implemented. Multi-agent execution is also supported.
4. Execution Monitoring: The execution of a plan is monitored continuously to deal with exogenous events. It provides functionalities, such as reasoning whether a provided plan action is actually executable by taking information about the world into account, and also mechanisms to retry/fail an action, when desired.

## Build
This assumes you have a basic ROS 2 installation already, otherwise consult the official documentation for that.

All dependencies are listed in the dependency.repos file of this repository. Hence, a dependency workspace can be setup using vcstool.
Assuming the CLIPS-Executive was cloned into `~/clips_executive_ws/src` and the dependencies shouldbe built in the workspace `~/deps_clips_executive_ws/`:
```bash
mkdir -p ~/deps_clips_executive_ws/src
cd ~/deps_clips_executive_ws/src
vcs import < ~/clips_executive_ws/src/ros2-clips-executive/dependency.repos
cd ~/deps_clips_executive_ws
colcon build --sym --cmake-args -DBUILD_TESTING=OFF
```
Then you can proceed by sourcing the workspace containing the dependencies and building the CLIPS-Executive:
```bash
source ~/deps_clips_executive_ws/install/setup.bash
cd ~/clips_executive_ws/
colcon build --sym
source ~/clips_executive_ws/install/setup.bash
```


## Getting Started
The **cx_bringup** package provides examplary configurations of the CLIPS -Executive that serve as a good starting point to get familiar with the system.

In order to further customize CLIPS, make sure to check out the documentation of the **cx_clips_env_manager** package for more detailed description of plugin handling. You might also want to have a look at the **cx_example_plugin**, which may be used as a boilerplate to write custom plugins.
