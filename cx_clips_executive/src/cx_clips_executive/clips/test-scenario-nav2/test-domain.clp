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
)

(defrule load-initial-facts
  (executive-init)
  (domain-loaded)
  =>
  (assert (domain-fact (name robot_at) (param-values robotino1 wp_init)))
  (assert (domain-fact (name connected) (param-values wp_init wp1)))
  (assert (domain-fact (name connected) (param-values wp_init wp2)))
  (assert (domain-fact (name connected) (param-values wp_init wp3)))
  (assert (domain-fact (name connected) (param-values wp_init wp4)))
  (assert (domain-fact (name connected) (param-values wp1 wp_init)))
  (assert (domain-fact (name connected) (param-values wp2 wp_init)))
  (assert (domain-fact (name connected) (param-values wp3 wp_init)))
  (assert (domain-fact (name connected) (param-values wp4 wp_init)))
  (assert
          (domain-object (name robotino1) (type robot))
          (domain-object (name wp_init) (type waypoint))
          (domain-object (name wp1) (type waypoint))
          (domain-object (name wp2) (type waypoint))
          (domain-object (name wp3) (type waypoint))
          (domain-object (name wp4) (type waypoint))
  )
  (assert (domain-facts-loaded))
)

; (defrule test-domain-set-domain-fact-said-hello
;   (plan-action (action-name move) (param-values "robotino1" "wp_init" "wp1") (state SENSED-EFFECTS-WAIT))
; =>
;   (assert (domain-fact (name robot_at) (param-values robotino1 wp1)))
; )
; (defrule test-domain-set-domain-fact-said-hello
;   (plan-action (action-name move) (param-values "robotino1" "wp1" "wp_init") (state SENSED-EFFECTS-WAIT))
; =>
;   (assert (domain-fact (name robot_at) (param-values robotino1 wp_init)))
; )
; (defrule test-domain-set-domain-fact-said-hello
;   (plan-action (action-name say-hello) (param-values peggy) (state SENSED-EFFECTS-WAIT))
; =>
;   (assert (domain-fact (name said) (param-values peggy hello)))
; )

; (defrule test-domain-set-domain-fact-said-goodbye
;   (plan-action (action-name say-goodbye) (param-values PEGGY GOODBYE) (state SENSED-EFFECTS-WAIT))
; =>
;   (assert (domain-fact (name said) (param-values PEGGY GOODBYE)))
; )