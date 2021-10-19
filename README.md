# ROS2 Clips-Executive

The ROS2 CLIPS Executive (**CX**) is the integration of the Fawkes CLIPS Executive into ROS2 ([Fawkes CX](https://ojs.aaai.org/index.php/ICAPS/article/view/3544/3412)). This project pursues to provide a robust, reliable, and configurable CLIPS-based high-level task executioner for Robotics Systems implementing the ideas of Goal-Reasoning ([GR](https://ojs.aaai.org//index.php/aimagazine/article/view/2800)). The CX supports separation of concerns and covers most high-level execution tasks, such as:

1. Goal Management and Refinement: The central entities inside the CX are goals. They describe the objectives to achieve or conditions to maintain, e.g., to bring a certain object from point A to point B. The CX utilizes GR with the goal refinement mechanism in the form of goal lifecycles. This way, each goal goes through a predefined set of goal modes during its lifespan. This makes the program flow explicit and allows constant goal monitoring, observation of agent status and actions, as well as tracking and reacting to
internal/exogenous factors.
2. Planning: Provides PDDL-based planning by interacting with and utilizing a dedicated planner/planning system (currently planning with [Plansys2](https://github.com/IntelligentRoboticsLabs/ros2_planning_system) is supported). 
3. Execution: Provides the means to execute all PDDL actions in a provided plan on the side of the embedding system. Skills in our project represent the identifier of the PDDL action, for which an executor node is implemented. Multi-agent execution is also supported.     
4. Execution Monitoring: The execution of a plan is monitored continuously to deal with exogenous events. It provides functionalities, such as reasoning whether a provided plan action is actually executable by taking information about the world into account, and also mechanisms to retry/fail an action, when desired.  

The implemented packages provide tools to:
- Bridge between ROS and CLIPS in order to control and manage CLIPS instances (CX CLIPS)
- Manage and load features (as plugins), which add functionality to a CLIPS environment, such as the Plansys2 feature, which allows the interaction to the running Plansys2 framework, or the Skill Execution feature, which allows the execution of skills from inside CLIPS (CX Features)
- Feature plugin to ease the process of implementing new CLIPS features (CX Core)
- Control and execute all high-level decisions within the actual CX Environment based on the core CLIPS files and the input agent setup from the config file (CX CLIPS Executive)
- Execute skills utilizing the provided Skill Execution mechanism and by implementing custom skill nodes on the side of the client application (CX Skill Execution)
- Imitate the behavior of a Fawkes Blackboard on the side of ROS (CX Blackboard)
- Manage the core lifecycle nodes of the system (CX Lifecycle Manager)
- Configure the CX and an example bringup mechanism (CX Bringup)

## Build

### Install ROS2
Install ROS 2 via the [install instructions](https://docs.ros.org/en/galactic/Installation.html) for desired distribution. 

### Install Plansys2
The CX requires only the instances inside Plansys2, which are responsible for planning. This excludes the following packages: `plansys2_executor`, `plansys2_terminal`, `plansys2_bt_actions`, `plansys2_tests`, `plansys2_bringup`.
#### Ubuntu users
Follow the Plansys2 [build instructions](https://intelligentroboticslab.gsyc.urjc.es/ros2_planning_system.github.io/build_instructions/index.html).
#### Other OS
##### Prerequisites:
You need to install the dependencies of Plansys2 ([cascade_lifecycle](https://github.com/fmrico/cascade_lifecycle.git) and [popf](https://github.com/fmrico/popf.git))

Then run:
```
mkdir -p ~/plansys2_ws/src
cd ~/plansys2_ws/src
git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system.git
# To pull the dependency repos, you can use the [vcstool](https://github.com/dirk-thomas/vcstool) and run:
vcs import < dependency_repos.repos

# Optional
git clone https://github.com/IntelligentRoboticsLabs/plansys2_tfd_plan_solver.git
git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples.git

cd ~/plansys2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>
colcon build --packages-ignore plansys2_executor plansys2_bt_actions plansys2_tests plansys2_terminal plansys2_bringup --symlink-install
```
Note: you may need to source the ROS2 ws beforehand!

### Install ROS2 CX
#### Prerequisites:
- [clipsmm](https://github.com/timn/clipsmm)
- [pddl_parser](https://github.com/fawkesrobotics/pddl_parser)
- [clips_pddl_parser](https://github.com/fawkesrobotics/clips_pddl_parser)

Run:
```
mkdir -p ~/cx_ws/src
cd ~/cx_ws/src
git clone https://github.com/fawkesrobotics/ros2-clips-executive 
# To pull the dependency repos, you can use the [vcstool](https://github.com/dirk-thomas/vcstool) and run:
vcs import < dependency.repos

cd ~/cx_ws
# You need to source the Plansys2 WS
source ~/plansys2_ws/install/local_setup.bash 
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>
colcon build --packages-ignore cx_example_skill_nodes --symlink-install

# If you want the example skill nodes package the `nav2_msgs` package is required 
```
Note: you may need to source the ROS2 WS beforehand!

Note2: Remember to always source the ROS2 WS, Plansys2 WS and the CX WS afterwards!

---

We hope that our system finds utilization in more Robotics projects, as it offers the integration of a proven and established GR-based high-level controller on the side of Fawkes into the ROS ecosystem, which is capable of reasoning in dynamic settings and robotics competitions, such as the [RCLL](http://www.robocup-logistics.org/).   
