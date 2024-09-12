; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

; This file showcases intefaces with other ROS components

; --- ROS messages ---

(defrule string-pub-sub-init
" Create a simple publisher and subscriber using the generated bindings. "
  (not (cx-std-msgs-string-feature-subscriber (topic "ros_cx_in")))
  (not (cx-std-msgs-string-feature-publisher (topic "ros_cx_out")))
=>
  (cx-std-msgs-string-feature-create-publisher "ros_cx_out")
  (printout info "Created publisher for /ros_cx_out" crlf)
  (cx-std-msgs-string-feature-create-subscriber "ros_cx_in")
  (printout info "Listening for String messages on /ros_cx_in" crlf)
)

(defrule string-publisher-recv-and-answer
" React to incoming messages and answer (on a different topic) "
  (cx-std-msgs-string-feature-subscriber (topic ?sub))
  ?msg-f <- (cx-std-msgs-string-feature-msg (topic ?sub) (msg-ptr ?inc-msg))
  (cx-std-msgs-string-feature-publisher (topic ?pub))
  =>
  ; fetch the content of the message and print it
  (bind ?recv (cx-std-msgs-string-feature-get-field ?inc-msg "data"))
  (printout blue "Recieved via " ?sub " :" ?recv crlf)
  ; make sure to actually destroy the message to free heap-allocated memory for it, once the message is processed and can be removed
  (cx-std-msgs-string-feature-destroy-msg ?inc-msg)
  (retract ?msg-f)

  ; example of how to create and send a new message
  (printout green "Sending Hello World Message in response!" crlf)
  (bind ?msg (cx-std-msgs-string-feature-create-msg))
  (cx-std-msgs-string-feature-set-field ?msg "data" "Hello world!")
  (cx-std-msgs-string-feature-publish ?msg ?pub)
  ; destroy the msg after usage to free up the memory
  (cx-std-msgs-string-feature-destroy-msg ?msg)
)
