; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(defrule ros-msgs-pub-init
" Create publisher for ros_cx_out."
  (not (ros-msgs-publisher (topic "ros_cx_out")))
  (not (executive-finalize))
=>
  ; print welcome text
  (printout green "-------------------- ")
  (printout bold  "ros msg example")
  (printout green " -------------------" crlf)
  (printout green "| ")
  (printout blue  "Creates a subscription to /ros_cx_in and a publisher")
  (printout green " |" crlf)
  (printout green "| ")
  (printout  blue "on /ros_cx_out. Whenever a message on /ros_cx_in is ")
  (printout green " |" crlf)
  (printout green "| ")
  (printout  blue "received, a response is published on /ros_cx_out    ")
  (printout green " |" crlf)
  (printout green "| ")
  (printout  blue "with content \"Hello World!\".                        ")
  (printout green " |" crlf)
  (printout green "--------------------------------------------------------" crlf)
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

(defrule ros-msgs-sub-finalize
" Delete the subscription on executive finalize. "
  (executive-finalize)
  (ros-msgs-subscription (topic ?topic))
=>
  (printout debug "Destroying topic " ?topic crlf)
  (ros-msgs-destroy-subscription ?topic)
)

(defrule ros-msgs-pub-finalize
" Delete the publisher on executive finalize. "
  (executive-finalize)
  (ros-msgs-publisher (topic ?topic))
=>
  (printout info "Destroying topic " ?topic crlf)
  (ros-msgs-destroy-publisher ?topic)
)

(defrule ros-msgs-message-cleanup
" Delete the subscription on executive finalize. "
  (executive-finalize)
  ?msg-f <- (ros-msgs-message (msg-ptr ?ptr))
=>
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
)
