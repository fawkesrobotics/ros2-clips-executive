
(defrule action-selection-select
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state FORMULATED)
											(action-name ?action-name))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	(not (plan-action (state ~FORMULATED&~FINAL)))
	(not (plan-action (state FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
	(modify ?pa (state PENDING))
)

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(not (plan-action (plan-id ?plan-id) (state ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan-action (state FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)

(defrule action-execute-exogenous-noops
  ?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state PENDING)
                   (action-name ?action&:(and (neq ?action navigate) (neq ?action sample_rock) (neq ?action sample_soil) (neq ?action drop)))
                   ;(action-name ?action&:(neq ?action navigate))
         (executable TRUE)
         (param-values $?param-values))
  =>
  (printout t "Executing " ?action ?param-values crlf)
  (modify ?pa (state EXECUTION-SUCCEEDED))
)
