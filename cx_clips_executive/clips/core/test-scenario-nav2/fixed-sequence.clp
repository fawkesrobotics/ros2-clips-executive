
; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(defrule goal-expander-create-sequence
	?g <- (goal (mode SELECTED) (id TESTGOAL))
	=>
	(assert
		(plan (id TESTGOAL-PLAN) (goal-id TESTGOAL))
		(plan-action (id 1) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name move) (goal-id TESTGOAL)
		             (param-names r1 w1 w2) (param-values "tb3" "wp_init" "wp1"))
		(plan-action (id 2) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name move) (goal-id TESTGOAL)
		             (param-names r1 w1 w2) (param-values "tb3" "wp1" "wp2"))
		(plan-action (id 3) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name move) (goal-id TESTGOAL)
		             (param-names r1 w1 w2) (param-values "tb3" "wp2" "wp3"))
		(plan-action (id 4) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name move) (goal-id TESTGOAL)
		             (param-names r1 w1 w2) (param-values "tb3" "wp3" "wp_4"))
		(plan-action (id 5) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name move) (goal-id TESTGOAL)
		             (param-names r1 w1 w2) (param-values "tb3" "wp4" "wp5"))
		(plan-action (id 6) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name move) (goal-id TESTGOAL)
		             (param-names r1 w1 w2) (param-values "tb3" "wp5" "wp_6"))
		(plan-action (id 7) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name move) (goal-id TESTGOAL)
		             (param-names r1 w1 w2) (param-values "tb3" "wp_6" "wp_final"))
	)
	(modify ?g (mode EXPANDED))
)
