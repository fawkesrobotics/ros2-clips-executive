;---------------------------------------------------------------------------
;  test-domain.clp - Test Domain
;
;  Created: Wed 04 Oct 2017 18:14:57 CEST
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(defrule load-domain
  (executive-init)
  (not (domain-loaded))
=>
  (parse-pddl-domain (path-resolve "test-scenario-pddl/domain.pddl"))
  (assert (domain-loaded))
  ; (assert (skiller-control))
)

(defrule test-domain-set-sensed-predicates
  (executive-init)
  (domain-loaded)
  ?p <- (domain-predicate (name said) (sensed FALSE))
=>
  (modify ?p (sensed TRUE))
)

(defrule load-initial-facts
  (executive-init)
  (domain-loaded)
  =>
  (assert (domain-fact (name unsaid) (param-values peggy hello)))
  ; (assert (domain-fact (name locked) (param-values peggy)))
  ; (assert (domain-fact (name locked) (param-values bob)))
  (assert
          (domain-object (name hello) (type text))
          ; (domain-object (name goodbye) (type text))
          ; (domain-object (name meh) (type text))
          ; (domain-object (name bob) (type name))
          (domain-object (name peggy) (type name))
  )
  (assert (domain-facts-loaded))
)

(defrule test-domain-set-domain-fact-said-hello
  (plan-action (action-name say-hello) (param-values peggy) (state SENSED-EFFECTS-WAIT))
=>
  (assert (domain-fact (name said) (param-values peggy hello)))
)

; (defrule test-domain-set-domain-fact-said-goodbye
;   (plan-action (action-name say-goodbye) (param-values PEGGY GOODBYE) (state SENSED-EFFECTS-WAIT))
; =>
;   (assert (domain-fact (name said) (param-values PEGGY GOODBYE)))
; )
