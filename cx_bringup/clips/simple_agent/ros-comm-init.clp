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
  (cx-std-msgs-string-feature-msg-destroy ?inc-msg)
  (retract ?msg-f)

  ; example of how to create and send a new message
  (printout green "Sending Hello World Message in response!" crlf)
  (bind ?msg (cx-std-msgs-string-feature-msg-create))
  (cx-std-msgs-string-feature-set-field ?msg "data" "Hello world!")
  (cx-std-msgs-string-feature-publish ?msg ?pub)
  ; destroy the msg after usage to free up the memory
  (cx-std-msgs-string-feature-msg-destroy ?msg)
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
  (bind ?req-data (cx-std-srvs-set-bool-feature-request-get-field ?request "data"))
  (printout info "Received request on " ?service-name ". Data: " ?req-data crlf)
  (printout info "Received " ?req-data ", responding with same value" crlf)
  (if ?req-data then
    (cx-std-srvs-set-bool-feature-response-set-field ?response "success" TRUE)
    (cx-std-srvs-set-bool-feature-response-set-field ?response "message" (str-cat "I got the request: " ?req-data))
    ;example usage of sending a request
    (printout info "Additionally, request as client with data: True" crlf)
    (bind ?new-req (cx-std-srvs-set-bool-feature-request-create))
    (cx-std-srvs-set-bool-feature-request-set-field ?new-req "data" TRUE)
    (cx-std-srvs-set-bool-feature-send-request ?new-req "ros_cx_client")
    (cx-std-srvs-set-bool-feature-request-destroy ?new-req)
   else
    (cx-std-srvs-set-bool-feature-response-set-field ?response "success" FALSE)
    (cx-std-srvs-set-bool-feature-response-set-field ?response "message" (str-cat "I got rhe request: " ?req-data))
  )
)

(defrule set-bool-client-response-received
" Create a simple client and service using the generated bindings. "
  ?msg-fact <- (cx-std-srvs-set-bool-feature-response (service ?service) (msg-ptr ?ptr))
=>
  (bind ?succ (cx-std-srvs-set-bool-feature-response-get-field ?ptr "success"))
  (bind ?msg (cx-std-srvs-set-bool-feature-response-get-field ?ptr "message"))
  (printout green "Received response from " ?service " with: " ?succ " (" ?msg ")" crlf)
  (retract ?msg-fact)
)

; --- ROS actions ---

(deffunction cx-example-interfaces-fibonacci-feature-handle-goal-callback (?server ?goal ?uuid)
  (printout blue ?server " callback (goal " ?goal " ; id " ?uuid  " )" crlf)
  ; (return 1) ; REJECT
  (return 2) ; ACCEPT_AND_EXECUTE
  ; (return 3) ; ACCEPT_AND_DEFER
)

(deffunction cx-example-interfaces-fibonacci-feature-cancel-goal-callback (?server ?goal ?goal-handle)
  (cx-example-interfaces-fibonacci-feature-goal-destroy ?goal)
  ; (return 0) ; REJECT
  (return 1) ; ACCEPT
)

(defrule fibonacci-action-client-server-init
" Create a simple client and service using the generated bindings. "
  (not (cx-example-interfaces-fibonacci-feature-client (server "ros_cx_fibonacci")))
  (not (cx-example-interfaces-fibonacci-feature-server (name "ros_cx_fibonacci")))
=>
  (cx-example-interfaces-fibonacci-feature-create-client "ros_cx_fibonacci")
  (printout info "Created client for /ros_cx_fibonacci" crlf)
  (cx-example-interfaces-fibonacci-feature-create-server "ros_cx_fibonacci")
  (printout info "Created server for /ros_cx_fibonacci" crlf)
)

(deftemplate fibonacci
  (multislot uuid (type INTEGER))
  (slot order (type INTEGER))
  (slot progress (type INTEGER))
  (multislot sequence (type INTEGER))
  (slot result (type INTEGER))
  (slot last-computed (type FLOAT))
)

(defrule fibonacci-goal-accepted-start-compute
  (cx-example-interfaces-fibonacci-feature-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (not (fibonacci (uuid ?uuid&:(eq ?uuid (cx-example-interfaces-fibonacci-feature-server-goal-handle-get-goal-id ?ptr)))))
  =>
  (if (not (cx-example-interfaces-fibonacci-feature-server-goal-handle-is-canceling ?ptr)) then
    (bind ?goal (cx-example-interfaces-fibonacci-feature-server-goal-handle-get-goal ?ptr))
    (bind ?order (cx-example-interfaces-fibonacci-feature-goal-get-field ?goal "order"))
    (bind ?uuid (cx-example-interfaces-fibonacci-feature-server-goal-handle-get-goal-id ?ptr))
    (assert (fibonacci (uuid ?uuid) (order ?order) (progress 2) (result 0) (sequence (create$ 0 1)) (last-computed (now))))
   else
    (printout error "Somehow the goal is canceling already" crlf)
  )
  ; do not destroy the server goal handle here, only do it once the goal is fully processed and finished
  ;(cx-example-interfaces-fibonacci-feature-server-goal-handle-destroy ?ptr)
)

(defrule fibonacci-compute-next
  (cx-example-interfaces-fibonacci-feature-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  ?f <- (fibonacci (order ?order) (progress ?remaining&:(>= ?order ?remaining))
  (last-computed ?computed) (result ?old-res) (sequence $?seq) (uuid ?uuid))
  (time ?now&:(> (- ?now ?computed) 1))
  (test (eq ?uuid (cx-example-interfaces-fibonacci-feature-server-goal-handle-get-goal-id ?ptr)))
  =>
  (bind ?step (+ ?remaining 1))
  (bind ?res (+ (nth$ ?remaining ?seq) (nth$ (- ?remaining 1) ?seq)))
  (printout yellow "Computing partial result fibonacci(" ?remaining ") = "?res crlf)
  (bind ?seq (create$ ?seq ?res))
  (modify ?f (progress ?step) (result (+ ?old-res ?res)) (sequence ?seq))
  (bind ?feedback (cx-example-interfaces-fibonacci-feature-feedback-create))
  (cx-example-interfaces-fibonacci-feature-feedback-set-field ?feedback "sequence" ?seq)
  (cx-example-interfaces-fibonacci-feature-server-goal-handle-publish-feedback ?ptr ?feedback)
  (cx-example-interfaces-fibonacci-feature-feedback-destroy ?feedback)
  (modify ?f (last-computed ?now))
)

(defrule fibonacci-compute-done
  (cx-example-interfaces-fibonacci-feature-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  ?f <- (fibonacci (order ?order) (progress ?remaining&:(< ?order ?remaining)) (result ?old-res) (sequence $?seq)); (uuid $?uuid&:(eq ?uuid (cx-example-interfaces-fibonacci-feature-server-goal-handle-get-goal-id ?ptr))))
  =>
  (printout green "Final fibonacci sequence: " ?seq crlf)
  (bind ?result (cx-example-interfaces-fibonacci-feature-result-create))
  (cx-example-interfaces-fibonacci-feature-result-set-field ?result "sequence" ?seq)
  (cx-example-interfaces-fibonacci-feature-server-goal-handle-succeed ?ptr ?result)
  (cx-example-interfaces-fibonacci-feature-result-destroy ?result)
  (cx-example-interfaces-fibonacci-feature-destroy-server "ros_cx_fibonacci")
  (cx-example-interfaces-fibonacci-feature-destroy-client "ros_cx_fibonacci")
)

(defrule fibonacci-client-send-goal
  (cx-example-interfaces-fibonacci-feature-client (server ?server))
  (not (send-request))
  =>
  (assert (send-request))
  (bind ?goal (cx-example-interfaces-fibonacci-feature-goal-create))
  (cx-example-interfaces-fibonacci-feature-goal-set-field ?goal "order" 5)
  (cx-example-interfaces-fibonacci-feature-send-goal ?goal ?server)
  ; do not destroy the goal yet, it needs to persist
  ;(cx-example-interfaces-fibonacci-feature-destroy-goal ?goal)
)

(defrule fibonacci-client-get-feedback
  (declare (salience 100))
  ?f <- (cx-example-interfaces-fibonacci-feature-goal-feedback (server ?server) (client-goal-handle-ptr ?ghp) (feedback-ptr ?fp))
  =>
  (bind ?g-id (cx-example-interfaces-fibonacci-feature-client-goal-handle-get-goal-id ?ghp))
  (bind ?g-stamp (cx-example-interfaces-fibonacci-feature-client-goal-handle-get-goal-stamp ?ghp))
  (bind ?g-status (cx-example-interfaces-fibonacci-feature-client-goal-handle-get-status ?ghp))
  (bind ?g-is-f-aware (cx-example-interfaces-fibonacci-feature-client-goal-handle-is-feedback-aware ?ghp))
  (bind ?g-is-r-aware (cx-example-interfaces-fibonacci-feature-client-goal-handle-is-result-aware ?ghp))
  ; the stamp seems to be broken (looks like a rclcpp_action issue
  (printout cyan "[" (- (now) ?g-stamp) "] " ?g-status " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
  (bind ?part-seq (cx-example-interfaces-fibonacci-feature-feedback-get-field ?fp "sequence"))
  (printout blue "partial sequence: " ?part-seq   crlf)
  (cx-example-interfaces-fibonacci-feature-feedback-destroy ?fp)
  (retract ?f)
)

(defrule fibonacci-client-process-goal-reponse
  (declare (salience -100))
  ?f <- (cx-example-interfaces-fibonacci-feature-goal-response (server ?server) (client-goal-handle-ptr ?ghp))
  (time ?now)
  =>
  (bind ?g-status (cx-example-interfaces-fibonacci-feature-client-goal-handle-get-status ?ghp))
  (if (> ?g-status 3) then ; status is final in one way or another
    (bind ?g-id (cx-example-interfaces-fibonacci-feature-client-goal-handle-get-goal-id ?ghp))
    (bind ?g-stamp (cx-example-interfaces-fibonacci-feature-client-goal-handle-get-goal-stamp ?ghp))
    (bind ?g-is-f-aware (cx-example-interfaces-fibonacci-feature-client-goal-handle-is-feedback-aware ?ghp))
    (bind ?g-is-r-aware (cx-example-interfaces-fibonacci-feature-client-goal-handle-is-result-aware ?ghp))
    (printout magenta "Final goal response [" (- (now) ?g-stamp) "] " ?g-status " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
    (cx-example-interfaces-fibonacci-feature-client-goal-handle-destroy ?ghp)
    (retract ?f)
  )
)
