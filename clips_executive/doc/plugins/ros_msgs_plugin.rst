.. _usage_ros_msgs_plugin:

Generic ROS Msg Plugin
######################

Source code on :source-master:`GitHub <cx_plugins/ros_msgs_plugin>`.

.. admonition:: Plugin Class

  cx::RosMsgsPlugin

This plugin provides the ability to interface with ROS topics of any type using introspection.

Configuration
*************

This plugin has no specific configuration options.

Features
********

Facts
~~~~~

.. code-block:: lisp

  ; Asserted by the create-subscription function.
  ; Retracted by the destroy-subscription function.
  (deftemplate ros-msgs-subscription
    (slot topic (type STRING)) ; example: "/cx_string_in"
    (slot type (type STRING))  ; example: "std_msgs/msg/String"
  )

  ; Asserted by the ros-msgs-create-publisher function.
  ; Retracted by the respective ros-msgs-destroy-publisher function.
  (deftemplate-ros-msgs-publisher
    (slot topic (type STRING)) ; example: "/cx_string_out"
    (slot type (type STRING))  ; example: "std_msgs/msg/String"
  )

  ; Asserted by the callback of a subscriber whenever a message arrives.
  ; Process the message and then call ros-msgs-destroy-msgs before retracting!
  (deftemplate ros-msgs-message
    (slot topic (type STRING)) ; example          : "/cx_string_in"
    (slot msg-ptr (type EXTERNAL-ADDRESS)) example: <Pointer-C-0x7f1550001d20>
  )

Functions
~~~~~~~~~

.. code-block:: lisp

  ; Create and destroy publishers and subscribers.
  (ros-msgs-create-publisher ?topic-name ?msg-type)    ; example args: "/cx_string_out" "std_msgs/msg/String"
  (ros-msgs-destroy-publisher ?topic-name)             ; example args: "/cx_string_out"
  (ros-msgs-create-subscription ?topic-name ?msg-type) ; example args: "/cx_string_in" "std_msgs/msg/String"
  (ros-msgs-destroy-subscription ?topic-name)          ; example args: "/cx_string_in"

  ; Publish a given message over a topic.
  ; Requires the publisher to be created first using create-publisher.
  (ros-msgs-publish ?msg-ptr ?topic-name)

  ; Create a message and return a pointer to it
  (bind ?msg-ptr (ros-msgs-create-message ?msg-type)) ; example args: "std_msgs/msg/String"
                                                      ; example ret: <Pointer-C-0x7f1550001d20>

  ; Destroy a message, call this after publishing a message or processing an incoming message to prevent it from staying in memory.
  (ros-msgs-destroy-message ?msg-ptr) ; example args: <Pointer-C-0x7f1550001d20>

  ; Populate the field of a message.
  ; If the field is a message, then pass a pointer to that message obtained from ros-msgs-create-message.
  (ros-msgs-set-field ?msg-ptr ?field-name ?field-value) ; example args: <Pointer-C-0x7f1550001d20> "data" "Hello World"

  ; Retrieve a field of a message.
  ; If the field is a message, then a pointer is returned that can be further inspected by passing it to ros-msgs-get-field.
  (ros-msgs-get-field ?msg-ptr ?field-name)

Message Lifetimes
~~~~~~~~~~~~~~~~~

Since clips stores objects via void pointers, dynamic object lifetime management via `std::shared_ptr` does not work direcly from within CLIPS.
Instead, object lifetimes need to be managed more explicitly through the usage of `create` and `destroy` functions.

It is advised to clean up all objects as soon as they are not needed anymore in order to free up memory.

Note that when processing nested messages, the message obtained via **ros-msgs-get-field** is not allocating new memory, but rather points to the memory of the parent message.
Calling **ros-msgs-destroy-message** is not necessary, as sub-messages retrieved via **ros-msgs-get-field** only hold a shallow reference and are cleaned up when the parent message is destroyed. Calling it anyways, will only invalidate this shallow reference.

When creating a new message via **ros-msgs-create-message**, new memory is allocated.
When using **ros-msgs-set-field** to set a nested message, dynamic memory from the sub-message is moved to the parent message, hence the nested message loses all dynamically allocated data (e.g., unbound arrays, strings).
See the example below:

.. code-block:: lisp

  (bind ?new-msg (ros-msgs-create-message "geometry_msgs/msg/Twist"))
  (bind ?sub-msg (ros-msgs-create-message "geometry_msgs/msg/Vector3"))
  (ros-msgs-set-field ?sub-msg "x" 1.0)
  (ros-msgs-set-field ?new-msg "linear" ?sub-msg)
  ; now all dynamicly allocated members of ?sub-msg are reset,
  ; as they are moved to the parent message.

In particular, obtaining a nested message from one message ``?source`` and setting it as a member to another message ``?sink`` will cause ``?source`` to lose all dynamic data within it's sub-message, as the sub-message obtained is actually pointing to memory within ``?source``:

.. code-block:: lisp

  (bind ?source (ros-msgs-create-message "geometry_msgs/msg/Twist"))
  (bind ?sub-msg (ros-msgs-create-message "geometry_msgs/msg/Vector3"))
  (ros-msgs-set-field ?sub-msg "x" 5.5)
  (ros-msgs-set-field ?source "linear" ?sub-msg)
  (bind ?sink (ros-msgs-create-message "geometry_msgs/msg/Twist"))
  (bind ?source-sub-msg (ros-msgs-get-field ?source "linear"))
  (ros-msgs-set-field ?sink "linear" ?source-sub-msg)
  ; now all dynamicly allocated members of the sub message in ?source are reset,
  ; as they are moved to ?sink.

Usage Example
*************

A minimal working example is provided by the :docsite:`cx_bringup` package. Run it via:

.. code-block:: bash

    ros2 launch cx_bringup cx_launch.py manager_config:=plugin_examples/ros_msgs.yaml

It creates a ``std_msgs/msg/String`` supscription on topic ``/ros_cx_in`` and prints out any text send over it.
Additionally, it creates a publisher on ``/ros_cx_out`` that publishes ``Hello World`` whenever something is received over the ``/ros_cx_in`` topic.

Configuration
~~~~~~~~~~~~~

File :source-master:`cx_bringup/params/plugin_examples/ros_msgs.yaml`.

.. code-block:: yaml

  clips_manager:
    ros__parameters:
      environments: ["cx_ros_msgs"]
      cx_ros_msgs:
        plugins: ["executive", "ros_msgs", "files"]
        watch: ["facts", "rules"]

      executive:
        plugin: "cx::ExecutivePlugin"
        refresh_rate: 10
      ros_msgs:
        plugin: "cx::RosMsgsPlugin"
      files:
        plugin: "cx::FileLoadPlugin"
        pkg_share_dirs: ["cx_bringup"]
        load: [
          "clips/plugin_examples/ros-msgs.clp"]


Code
~~~~

File :source-master:`cx_bringup/clips/plugin_examples/ros-msgs.clp`.

.. code-block:: lisp

  (defrule ros-msgs-pub-init
  " Create publisher for ros_cx_out."
    (not (ros-msgs-publisher (topic "ros_cx_out")))
    (not (executive-finalize))
  =>
    ; create the publisher
    (ros-msgs-create-publisher "ros_cx_out" "std_msgs/msg/String")
    (printout info "Publishing on /ros_cx_out" crlf)
  )

  (defrule ros-msgs-pub-hello
  " Whenever a message comes in, send out a Hello World message in response. "
    (declare (salience 1))
    (ros-msgs-publisher (topic ?topic))
    (ros-msgs-message)
    =>
    (printout yellow "Sending Hello World Message!" crlf)
    (bind ?msg (ros-msgs-create-message "std_msgs/msg/String"))
    (ros-msgs-set-field ?msg "data" "Hello world!")
    (ros-msgs-publish ?msg ?topic)
    (ros-msgs-destroy-message ?msg)
  )

  (defrule ros-msgs-sub-init
  " Create a simple subscriber using the generated bindings. "
    (not (ros-msgs-subscription (topic "ros_cx_in")))
    (not (executive-finalize))
  =>
    (ros-msgs-create-subscription "ros_cx_in" "std_msgs/msg/String")
    (printout info "Listening for String messages on /ros_cx_in" crlf)
  )

  (defrule ros-msgs-receive
  " React to incoming messages and answer (on a different topic). "
    (ros-msgs-subscription (topic ?sub))
    ?msg-f <- (ros-msgs-message (topic ?sub) (msg-ptr ?inc-msg))
    =>
    (bind ?recv (ros-msgs-get-field ?inc-msg "data"))
    (printout blue "Recieved via " ?sub ": " ?recv crlf)
    (ros-msgs-destroy-message ?inc-msg)
    (retract ?msg-f)
  )

  (defrule ros-msgs-pub-sub-finalize
  " Delete the publisher and subscription on executive finalize. "
    (executive-finalize)
    (ros-msgs-publisher (topic ?topic))
    (ros-msgs-subscription (topic ?in-topic))
  =>
    (printout info "Destroying publishers and subscriptions " crlf)
    (ros-msgs-destroy-publisher ?topic)
    (ros-msgs-destroy-subscription ?in-topic)
  )

  (defrule ros-msgs-message-cleanup
  " Delete the messages on executive finalize. "
    (executive-finalize)
    ?msg-f <- (ros-msgs-message (msg-ptr ?ptr))
  =>
    (ros-msgs-destroy-message ?ptr)
    (retract ?msg-f)
  )
