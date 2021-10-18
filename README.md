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

We hope that our system finds utilization in more Robotics projects, as it offers the integration of a proven and established GR-based high-level controller on the side of Fawkes into the ROS ecosystem, which is capable of reasoning in dynamic settings and robotics competitions, such as the [RCLL](http://www.robocup-logistics.org/).   
