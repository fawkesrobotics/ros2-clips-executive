;---------------------------------------------------------------------------
;  test-domain.clp - Test Domain
;---------------------------------------------------------------------------

(defrule load-domain
  (executive-init)
  (not (domain-loaded))
=>
  (parse-pddl-domain (path-resolve "test-scenario-nav2/domain.pddl"))
  (assert (domain-loaded))
)

; (defrule test-domain-set-sensed-predicates
;   (executive-init)
;   (domain-loaded)
;   ?p <- (domain-predicate (name robot_at) (sensed FALSE))
; =>
;   (modify ?p (sensed TRUE))
;)

(defrule load-initial-facts
  (executive-init)
  (domain-loaded)
  =>
  (assert (domain-fact (name robot_at) (param-values tb3 wp_init)))
  (assert (domain-fact (name connected) (param-values wp_init wp1)))
  (assert (domain-fact (name connected) (param-values wp1 wp2)))
  (assert (domain-fact (name connected) (param-values wp2 wp3)))
  (assert (domain-fact (name connected) (param-values wp3 wp4)))
  (assert (domain-fact (name connected) (param-values wp4 wp5)))
  (assert (domain-fact (name connected) (param-values wp5 wp6)))
  (assert (domain-fact (name connected) (param-values wp6 wp_final)))
  (assert
          (domain-object (name tb3) (type robot))
          (domain-object (name wp_init) (type waypoint))
          (domain-object (name wp1) (type waypoint))
          (domain-object (name wp2) (type waypoint))
          (domain-object (name wp3) (type waypoint))
          (domain-object (name wp4) (type waypoint))
          (domain-object (name wp5) (type waypoint))
          (domain-object (name wp6) (type waypoint))
          (domain-object (name wp_final) (type waypoint))
  )
  (assert (domain-facts-loaded))
)