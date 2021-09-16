(defrule fire-skill
  (executive-init)
  (ff-feature-loaded skill_execution)
  =>
  (assert
    (plan (id TESTPLAN) (goal-id TESTGOAL) (type SEQUENTIAL))
    (plan-action (id 10) (goal-id TESTGOAL) (plan-id TESTPLAN)
                 (duration 4.0) (dispatch-time 0.0) (state PENDING)
                 (action-name move) (executable TRUE)
                 (param-names name) (param-values r2d2 steering_wheels_zone assembly_zone))
  )
)