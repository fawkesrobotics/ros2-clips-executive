;---------------------------------------------------------------------------
;  goal-expander-pddl.clp - Expand a goal with a PDDL planner
;
;---------------------------------------------------------------------------


(defrule goal-expander-call-pddl
	?g <- (goal (mode SELECTED) (id TESTGOAL))
	=>
  (pddl-request-plan TESTGOAL "(and (robot_at tb3 wp_final))")
)
