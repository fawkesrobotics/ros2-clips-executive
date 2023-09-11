;---------------------------------------------------------------------------
;  goal-expander-pddl.clp - Expand a goal with a PDDL planner
;
;---------------------------------------------------------------------------


(defrule goal-expander-call-pddl-demo
	?g <- (goal (mode SELECTED) (id DEMO-SCENARIO-GOAL))
	=>
  (pddl-request-plan DEMO-SCENARIO-GOAL "
  	(and
		(communicated_soil_data waypoint2)
		(communicated_rock_data waypoint3)
		(communicated_image_data objective1 high_res)
	)
  ")
)

(defrule goal-expander-call-pddl-cleanup
	?g <- (goal (mode SELECTED) (id CLEANUP-GOAL))
	=>
  (pddl-request-plan CLEANUP-GOAL "
  	(and
		(empty rover0store)
	)
  ")

)