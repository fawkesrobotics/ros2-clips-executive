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

; --- ROS services ---

(defrule set-bool-client-service-init
" Create a simple client and service using the generated bindings. "
  (not (cx-std-srvs-set-bool-feature-client (service "ros_cx_client")))
  (not (cx-std-srvs-set-bool-feature-service (name "ros_cx_srv")))
=>
  (cx-std-srvs-set-bool-feature-create-client "ros_cx_client")
  (printout info "Created client for /ros_cx_client" crlf)
  (cx-std-srvs-set-bool-feature-create-service "ros_cx_srv")
  (printout info "Created service for /ros_cx_srv" crlf)
)

; this function needs to be defined in order to respond to messages
(deffunction cx-std-srvs-set-bool-feature-service-callback (?service-name ?request ?response)
  (bind ?req-data (cx-std-srvs-set-bool-feature-get-field-request ?request "data"))
  (printout info "Received request on " ?service-name ". Data: " ?req-data crlf)
  (printout info "Received " ?req-data ", responding with same value" crlf)
  (if ?req-data then
    (cx-std-srvs-set-bool-feature-set-field-response ?response "success" TRUE)
    (cx-std-srvs-set-bool-feature-set-field-response ?response "message" (str-cat "I got the request: " ?req-data))
    ;example usage of sending a request
    (printout info "Additionally, request as client with data: True" crlf)
    (bind ?new-req (cx-std-srvs-set-bool-feature-create-request))
    (cx-std-srvs-set-bool-feature-set-field-request ?new-req "data" TRUE)
    (cx-std-srvs-set-bool-feature-send-request ?new-req "ros_cx_client")
    (cx-std-srvs-set-bool-feature-destroy-request ?new-req)
   else
    (cx-std-srvs-set-bool-feature-set-field-response ?response "success" FALSE)
    (cx-std-srvs-set-bool-feature-set-field-response ?response "message" (str-cat "I got rhe request: " ?req-data))
  )
)

(defrule set-bool-client-response-received
" Create a simple client and service using the generated bindings. "
  ?msg-fact <- (cx-std-srvs-set-bool-feature-response (service ?service) (msg-ptr ?ptr))
=>
  (bind ?succ (cx-std-srvs-set-bool-feature-get-field-response ?ptr "success"))
  (bind ?msg (cx-std-srvs-set-bool-feature-get-field-response ?ptr "message"))
  (printout green "Received response from " ?service " with: " ?succ " (" ?msg ")" crlf)
  (retract ?msg-fact)
)
