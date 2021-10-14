;---------------------------------------------------------------------------
;  pddl.clp - Interface to a PDDL planner plansys2 planner
;
;  Created: Tue 07 Nov 2017 18:36:08 CET
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(deftemplate pddl-plan
  ; The ID of the goal.
  (slot goal-id (type SYMBOL))
  ; The ID of the PDDL Plan message.
  (slot plan-id (type SYMBOL))
  ; The current status of this plan.
  (slot status (type SYMBOL)
    (allowed-values
      NOT-STARTED RUNNING PLANNED PLAN-FAILED
    )
  )
  ; The PDDL formula to plan for.
  (slot goal (type STRING))
)

; Feedback based on planner execution status
(deftemplate pddl-plan-feedback
  (slot plan-id (type SYMBOL))
  (slot status (type SYMBOL)
    (allowed-values
      RUNNING PLANNED PLAN-FAILED
    )
  )
)

(deffunction plan-with-psys2 (?goal-id ?goal ?plan-id)
  "Add all available domain objects/facts and then
   Call the Plansys PDDL planner Client for the given 
   goal-id with the goal given as string."
  (do-for-all-facts ((?d domain-object))
    TRUE
    (psys2-add-domain-instance ?d:name ?d:type)
  )
  (do-for-all-facts ((?f domain-fact))
    TRUE
    (bind ?pred (str-cat "(" ?f:name)) 
    (foreach ?param-value ?f:param-values
      (printout t "Param-value:" ?param-value crlf)
      (bind ?pred (str-cat ?pred " " ?param-value))
    )
    (bind ?pred (str-cat ?pred ")"))
    (psys2-add-domain-predicate ?pred)
  )
  (psys2-call-planner ?goal-id ?goal ?plan-id)
)

(deffunction pddl-request-plan (?goal-id ?goal)
  (bind ?plan-id (sym-cat ?goal-id - (gensym*)))
  (assert (pddl-plan
    (goal-id ?goal-id) (goal ?goal) (status NOT-STARTED) (plan-id ?plan-id))
  )
  ; leave the printout, else tests terminate
  (printout t "Requested plan with id: " ?plan-id  crlf)
)

(defrule pddl-call
  ?g <- (goal (id ?goal-id))
  ?p <-(pddl-plan 
          (status NOT-STARTED) 
          (goal-id ?goal-id) 
          (plan-id ?plan-id)
          (goal ?goal)
        )
  (not (pddl-plan (status ~NOT-STARTED)))
  =>
  (printout t "Call psys2 feature function" crlf)
  (modify ?p (status RUNNING))
  (plan-with-psys2 ?goal-id ?goal ?plan-id)
)

(defrule pddl-check-if-planner-failed
  "Check whether the planner finished but has not found a plan."
  ?g <- (goal (id ?goal-id))
  ?pf <- (pddl-plan-feedback (status PLAN-FAILED) (plan-id ?plan-id))
  ?p <- (pddl-plan 
          ; (goal-id ?goal-id) 
          (plan-id ?plan-id)
        )
  =>
  ; (printout error "Planning failed for goal " ?goal-id crlf)
  (modify ?g (mode FINISHED) (outcome FAILED) (message "Planning failed") (error PLANNING-FAILED))
  (retract ?pf)
  (retract ?p)
  (psys2-clear-knowledge)
)

(defrule pddl-expand-goal
  "expand the goal after generated plan"
  ?g <- (goal (id ?goal-id) (mode SELECTED))
  ?pf <- (pddl-plan-feedback (status PLANNED) (plan-id ?plan-id))
  ?p <- (pddl-plan
          (goal-id ?goal-id)
          (plan-id ?plan-id)
        )
  =>
  (printout t "Fetched a new plan!" crlf)
  (modify ?g (mode EXPANDED))
  (retract ?pf)
  (retract ?p)
  (psys2-clear-knowledge)
)

; (defrule pddl-clean-up-stale-plans
;   "Clean up pddl-plan facts that were left behind.
;    This may happen if a goal get unselected while the planner is running."
;   ?p <- (pddl-plan (goal-id ?goal-id))
;   (not (goal (id ?goal-id) (mode SELECTED)))
;   =>
;   (retract ?p)
; )

(deffunction pddl-get-max-action-id ()
  "Get the max ID of all current action"
  (bind ?i 0)
  (do-for-all-facts ((?a plan-action)) (> ?a:id ?i) (bind ?i ?a:id))
  (return ?i)
)


; (deffunction pddl-call (?goal-id ?goal)
;   "Call the PDDL planner for the given goal-id with the goal given as string."
;   (bind ?m
;     (blackboard-create-msg "PddlGenInterface::pddl-gen" "GenerateMessage")
;   )
;   (blackboard-set-msg-field ?m "goal" ?goal)
;   (printout info "Calling PDDL planner for goal '" ?goal "'" crlf)
;   (bind ?gen-id (blackboard-send-msg ?m))
;   (assert (pddl-plan
;     (goal-id ?goal-id) (goal ?goal) (status GEN-PENDING) (gen-id ?gen-id))
;   )
; )

; (defrule pdll-call-psys2
;   (using-psys2)
;   =>
;   (printout t "Calling psys2" crlf)
;   (plan-with-psys2 TESTGOAL "(and (robot_talk ivo message1 francisco))")
; )

; (defrule pddl-check-if-generation-running
;   "Check whether the PDDL generator started."
;   ?p <- (pddl-plan (status GEN-PENDING) (gen-id ?gen-id))
;   (PddlGenInterface (id "pddl-gen") (msg_id ?gen-id))
;   =>
;   (printout t "PDDL problem generation started for ID " ?gen-id crlf)
;   (modify ?p (status GEN-RUNNING))
; )

; (defrule pddl-check-if-generation-finished
;   "Check whether the PDDL generator finished."
;   ?p <- (pddl-plan (status GEN-RUNNING) (gen-id ?gen-id))
;   (PddlGenInterface (id "pddl-gen") (msg_id ?gen-id) (final TRUE))
;   =>
;   (printout t "PDDL problem generation finished for ID " ?gen-id crlf)
;   (modify ?p (status GENERATED))
; )

; (defrule pddl-start-planner
;   "Start the actual planner after generating the PDDL problem."
;   ?p <- (pddl-plan (status GENERATED) (gen-id ?gen-id))
;   =>
;   (printout t "Starting to plan " ?gen-id crlf)
;   (bind ?m
;     (blackboard-create-msg "PddlPlannerInterface::pddl-planner" "PlanMessage")
;   )
;   (bind ?plan-id (blackboard-send-msg ?m))
;   (modify ?p (plan-id ?plan-id) (status PENDING))
; )

; (defrule pddl-check-if-planner-running
;   "Check whether the planner started to plan."
;   ?p <- (pddl-plan (status PENDING) (plan-id ?plan-id))
;   (PddlPlannerInterface (id "pddl-planner") (msg_id ?plan-id))
;   =>
;   (modify ?p (status RUNNING))
; )

; (defrule pddl-check-if-planner-finished
;   "Check whether the planner finished planning."
;   ?p <- (pddl-plan (status RUNNING) (plan-id ?plan-id))
;   (PddlPlannerInterface (id "pddl-planner") (msg_id ?plan-id) (final TRUE)
;     (success TRUE))
;   =>
;   (modify ?p (status PLANNED))
; )

