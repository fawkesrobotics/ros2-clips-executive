; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

; This file showcases intefaces with other ROS components

; --- ROS messages ---

(defrule string-pub-sub-init
" Create a simple publisher and subscriber using the generated bindings. "
  (not (std-msgs-string-subscriber (topic "ros_cx_in")))
  (not (std-msgs-string-publisher (topic "ros_cx_out")))
=>
  (std-msgs-string-create-publisher "ros_cx_out")
  (printout info "Created publisher for /ros_cx_out" crlf)
  (std-msgs-string-create-subscriber "ros_cx_in")
  (printout info "Listening for String messages on /ros_cx_in" crlf)
)

(defrule string-publisher-recv-and-answer
" React to incoming messages and answer (on a different topic) "
  (std-msgs-string-subscriber (topic ?sub))
  ?msg-f <- (std-msgs-string-msg (topic ?sub) (msg-ptr ?inc-msg))
  (std-msgs-string-publisher (topic ?pub))
  =>
  ; fetch the content of the message and print it
  (bind ?recv (std-msgs-string-get-field ?inc-msg "data"))
  (printout blue "Recieved via " ?sub " :" ?recv crlf)
  ; make sure to actually destroy the message to free heap-allocated memory for it, once the message is processed and can be removed
  (std-msgs-string-msg-destroy ?inc-msg)
  (retract ?msg-f)

  ; example of how to create and send a new message
  (printout green "Sending Hello World Message in response!" crlf)
  (bind ?msg (std-msgs-string-msg-create))
  (std-msgs-string-set-field ?msg "data" "Hello world!")
  (std-msgs-string-publish ?msg ?pub)
  ; destroy the msg after usage to free up the memory
  (std-msgs-string-msg-destroy ?msg)
)

; --- ROS services ---

(defrule set-bool-client-service-init
" Create a simple client and service using the generated bindings. "
  (not (std-srvs-set-bool-client (service "ros_cx_client")))
  (not (std-srvs-set-bool-service (name "ros_cx_srv")))
=>
  (std-srvs-set-bool-create-client "ros_cx_client")
  (printout info "Created client for /ros_cx_client" crlf)
  (std-srvs-set-bool-create-service "ros_cx_srv")
  (printout info "Created service for /ros_cx_srv" crlf)
)

; this function needs to be defined in order to respond to messages
(deffunction std-srvs-set-bool-service-callback (?service-name ?request ?response)
  (bind ?req-data (std-srvs-set-bool-request-get-field ?request "data"))
  (printout info "Received request on " ?service-name ". Data: " ?req-data crlf)
  (printout info "Received " ?req-data ", responding with same value" crlf)
  (if ?req-data then
    (std-srvs-set-bool-response-set-field ?response "success" TRUE)
    (std-srvs-set-bool-response-set-field ?response "message" (str-cat "I got the request: " ?req-data))
    ;example usage of sending a request
    (printout info "Additionally, request as client with data: True" crlf)
    (bind ?new-req (std-srvs-set-bool-request-create))
    (std-srvs-set-bool-request-set-field ?new-req "data" TRUE)
    (std-srvs-set-bool-send-request ?new-req "ros_cx_client")
    (std-srvs-set-bool-request-destroy ?new-req)
   else
    (std-srvs-set-bool-response-set-field ?response "success" FALSE)
    (std-srvs-set-bool-response-set-field ?response "message" (str-cat "I got rhe request: " ?req-data))
  )
)

(defrule set-bool-client-response-received
" Create a simple client and service using the generated bindings. "
  ?msg-fact <- (std-srvs-set-bool-response (service ?service) (msg-ptr ?ptr))
=>
  (bind ?succ (std-srvs-set-bool-response-get-field ?ptr "success"))
  (bind ?msg (std-srvs-set-bool-response-get-field ?ptr "message"))
  (printout green "Received response from " ?service " with: " ?succ " (" ?msg ")" crlf)
  (retract ?msg-fact)
)

; --- ROS actions ---

(deffunction example-interfaces-fibonacci-handle-goal-callback (?server ?goal ?uuid)
  (printout blue ?server " callback (goal " ?goal " ; id " ?uuid  " )" crlf)
  ; (return 1) ; REJECT
  (return 2) ; ACCEPT_AND_EXECUTE
  ; (return 3) ; ACCEPT_AND_DEFER
)

(deffunction example-interfaces-fibonacci-cancel-goal-callback (?server ?goal ?goal-handle)
  (example-interfaces-fibonacci-goal-destroy ?goal)
  ; (return 0) ; REJECT
  (return 1) ; ACCEPT
)

(defrule fibonacci-action-client-server-init
" Create a simple client and service using the generated bindings. "
  (not (example-interfaces-fibonacci-client (server "ros_cx_fibonacci")))
  (not (example-interfaces-fibonacci-server (name "ros_cx_fibonacci")))
  ;(not (client-server-already-created))
=>
  ;(assert (client-server-already-created))
  (example-interfaces-fibonacci-create-client "ros_cx_fibonacci")
  (printout info "Created client for /ros_cx_fibonacci" crlf)
  (example-interfaces-fibonacci-create-server "ros_cx_fibonacci")
  (printout info "Created server for /ros_cx_fibonacci" crlf)
)

(deftemplate fibonacci
  (slot uuid (type STRING))
  (slot order (type INTEGER))
  (slot progress (type INTEGER))
  (multislot sequence (type INTEGER))
  (slot result (type INTEGER))
  (slot last-computed (type FLOAT))
)

(defrule fibonacci-goal-accepted-start-compute
  (example-interfaces-fibonacci-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (not (fibonacci (uuid ?uuid&:(eq ?uuid (example-interfaces-fibonacci-server-goal-handle-get-goal-id ?ptr)))))
  =>
  (if (not (example-interfaces-fibonacci-server-goal-handle-is-canceling ?ptr)) then
    (bind ?goal (example-interfaces-fibonacci-server-goal-handle-get-goal ?ptr))
    (bind ?order (example-interfaces-fibonacci-goal-get-field ?goal "order"))
    (bind ?uuid (example-interfaces-fibonacci-server-goal-handle-get-goal-id ?ptr))
    (assert (fibonacci (uuid ?uuid) (order ?order) (progress 2) (result 0) (sequence (create$ 0 1)) (last-computed (now))))
   else
    (printout error "Somehow the goal is canceling already" crlf)
  )
  ; do not destroy the server goal handle here, only do it once the goal is fully processed and finished
  ;(example-interfaces-fibonacci-server-goal-handle-destroy ?ptr)
)

(defrule fibonacci-compute-next
  (example-interfaces-fibonacci-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  ?f <- (fibonacci (order ?order) (progress ?remaining&:(>= ?order ?remaining))
  (last-computed ?computed) (result ?old-res) (sequence $?seq) (uuid ?uuid))
  (time ?now&:(> (- ?now ?computed) 1))
  (test (eq ?uuid (example-interfaces-fibonacci-server-goal-handle-get-goal-id ?ptr)))
  =>
  (bind ?step (+ ?remaining 1))
  (bind ?res (+ (nth$ ?remaining ?seq) (nth$ (- ?remaining 1) ?seq)))
  (printout yellow "Computing partial result fibonacci(" ?remaining ") = "?res crlf)
  (bind ?seq (create$ ?seq ?res))
  (modify ?f (progress ?step) (result (+ ?old-res ?res)) (sequence ?seq))
  (bind ?feedback (example-interfaces-fibonacci-feedback-create))
  (example-interfaces-fibonacci-feedback-set-field ?feedback "sequence" ?seq)
  (example-interfaces-fibonacci-server-goal-handle-publish-feedback ?ptr ?feedback)
  (example-interfaces-fibonacci-feedback-destroy ?feedback)
  (modify ?f (last-computed ?now))
)

(defrule fibonacci-compute-done
  ?ag <- (example-interfaces-fibonacci-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  ?f <- (fibonacci (order ?order) (progress ?remaining&:(< ?order ?remaining)) (result ?old-res) (sequence $?seq)); (uuid $?uuid&:(eq ?uuid (example-interfaces-fibonacci-server-goal-handle-get-goal-id ?ptr))))
  =>
  (printout green "Final fibonacci sequence: " ?seq crlf)
  (bind ?result (example-interfaces-fibonacci-result-create))
  (example-interfaces-fibonacci-result-set-field ?result "sequence" ?seq)
  (example-interfaces-fibonacci-server-goal-handle-succeed ?ptr ?result)
  (example-interfaces-fibonacci-result-destroy ?result)
  (example-interfaces-fibonacci-destroy-server "ros_cx_fibonacci")
  (example-interfaces-fibonacci-destroy-client "ros_cx_fibonacci")
  (example-interfaces-fibonacci-server-goal-handle-destroy ?ptr)
  (retract ?f)
  (retract ?ag)
)

(defrule fibonacci-client-send-goal
  (example-interfaces-fibonacci-client (server ?server))
  ;(not (send-request))
  =>
  ;(assert (send-request))
  (bind ?goal (example-interfaces-fibonacci-goal-create))
  (example-interfaces-fibonacci-goal-set-field ?goal "order" 5)
  (example-interfaces-fibonacci-send-goal ?goal ?server)
  ; do nto destroy the goal yet, it needs to persist
  ;(example-interfaces-fibonacci-destroy-goal ?goal)
)

(defrule fibonacci-client-get-feedback
  (declare (salience 100))
  ?f <- (example-interfaces-fibonacci-goal-feedback (server ?server) (client-goal-handle-ptr ?ghp) (feedback-ptr ?fp))
  =>
  (bind ?g-id (example-interfaces-fibonacci-client-goal-handle-get-goal-id ?ghp))
  (bind ?g-stamp (example-interfaces-fibonacci-client-goal-handle-get-goal-stamp ?ghp))
  (bind ?g-status (example-interfaces-fibonacci-client-goal-handle-get-status ?ghp))
  (bind ?g-is-f-aware (example-interfaces-fibonacci-client-goal-handle-is-feedback-aware ?ghp))
  (bind ?g-is-r-aware (example-interfaces-fibonacci-client-goal-handle-is-result-aware ?ghp))
  ; the stamp seems to be broken (looks like a rclcpp_action issue
  (printout cyan "[" (- (now) ?g-stamp) "] " ?g-status " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
  (bind ?part-seq (example-interfaces-fibonacci-feedback-get-field ?fp "sequence"))
  (printout blue "partial sequence: " ?part-seq   crlf)
  (example-interfaces-fibonacci-feedback-destroy ?fp)
  (retract ?f)
)

(defrule fibonacci-client-process-goal-reponse
  (declare (salience -100))
  ?f <- (example-interfaces-fibonacci-goal-response (server ?server) (client-goal-handle-ptr ?ghp))
  (time ?now)
  =>
  (bind ?g-status (example-interfaces-fibonacci-client-goal-handle-get-status ?ghp))
  (if (> ?g-status 3) then ; status is final in one way or another
    (bind ?g-id (example-interfaces-fibonacci-client-goal-handle-get-goal-id ?ghp))
    (bind ?g-stamp (example-interfaces-fibonacci-client-goal-handle-get-goal-stamp ?ghp))
    (bind ?g-is-f-aware (example-interfaces-fibonacci-client-goal-handle-is-feedback-aware ?ghp))
    (bind ?g-is-r-aware (example-interfaces-fibonacci-client-goal-handle-is-result-aware ?ghp))
    (printout magenta "Final goal response [" (- (now) ?g-stamp) "] " ?g-status " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
    (example-interfaces-fibonacci-client-goal-handle-destroy ?ghp)
    (retract ?f)
  )
)
