; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

;---------------------------------------------------------------------------
;  domain.clp - Representation of a planning domain
;
;  Created: Fri 22 Sep 2017 11:35:49 CEST
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffunction domain-retract-grounding-user-extension (?grounding-id)
(do-for-all-facts ((?plan-action plan-action))
                  (and (eq ?plan-action:precondition ?grounding-id) (neq ?plan-action:precondition nil))
  (modify ?plan-action (precondition nil))
)
)

(defrule domain-ground-plan-action-precondition
  "Create grounded pddl formulas for the precondition of a plan-action if it
  has not been grounded yet and add a reference to the plan-action fact"
  (declare (salience ?*SALIENCE-DOMAIN-GROUND*))
  ?p <- (plan-action (id ?action-id) (action-name ?operator-id)
                     (param-names $?param-names) (param-values $?param-values)
                     (precondition nil))
  (domain-operator (name ?operator-id) (param-names $?op-param-names&:(= (length$ ?param-names) (length$ ?op-param-names))))
	(pddl-formula (part-of ?operator-id))
  =>
  (bind ?grounding (ground-pddl-formula ?operator-id ?param-names ?param-values nil))
  (modify ?p (precondition ?grounding))
)

(defrule domain-retract-grounding-for-plan-action-if-precondition-mismatch
  "Sometimes it is practical to switch the params of a plan-action.
  Remove grounding to trigger the grounding process again in such a case."
  ?g <- (pddl-grounding (param-values $?param-values) (id ?grounding))
  ?a <- (plan-action (precondition ?grounding) (param-values ~$?param-values))
  =>
  (retract ?g)
  (modify ?a (precondition nil))
)

(defrule domain-check-if-action-precondition-is-satisfied
  "Check if there is a referenced precondition formula that is satisfied,
  if yes make the action executable."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?p <- (plan-action (executable FALSE) (id ?id) (precondition ?grounding-id)
                     (action-name ?operator-id) (state ?state))
  (pddl-formula (part-of ?operator-id) (id ?formula-id))
  (grounded-pddl-formula (is-satisfied TRUE) (formula-id ?formula-id) (grounding ?grounding-id))
  (pddl-grounding (id ?grounding-id))
  =>
  (modify ?p (executable TRUE))
  (if (eq ?state PENDING) then
    (printout t "Action " ?id " is executable based on " ?formula-id crlf)
  )
)

(defrule domain-check-if-action-precondition-is-unsatisfied
  "Check if all referenced precondition formulas are not satisfied,
  if yes make the action not executable."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?p <- (plan-action (executable TRUE) (id ?id) (precondition ?grounding-id)
                     (action-name ?operator-id) (state ?state))
  (pddl-formula (part-of ?operator-id) (id ?formula-id))
  (not (grounded-pddl-formula (is-satisfied TRUE) (formula-id ?formula-id) (grounding ?grounding-id)))
  (pddl-grounding (id ?grounding-id))
  =>
  (modify ?p (executable FALSE))
  (if (eq ?state PENDING) then
    (printout t "Action " ?id " is no longer executable" crlf)
  )
)


(defrule domain-amend-action-params
  "If a plan action has no action-params specified, copy the params from the
   operator."
  ?a <- (plan-action
          (action-name ?op-name)
          (param-names $?ap-names&:(= (length$ ?ap-names) 0))
          (param-values $?ap-values&:(> (length$ ?ap-values) 0))
        )
  ?op <- (domain-operator (name ?op-name)
          (param-names $?param-names&:(> (length$ ?param-names) 0)))
  =>
  (modify ?a (param-names ?param-names))
)

(defrule domain-ground-effect-precondition
  "Ground a non-atomic precondition. Grounding here merely means that we
   duplicate the precondition and tie it to one specific effect-id."
  (declare (salience ?*SALIENCE-DOMAIN-GROUND*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?pa <- (plan-action (action-name ?op)
                      (id ?action-id)
                      (goal-id ?g)
                      (plan-id ?p)
                      (state EXECUTION-SUCCEEDED)
                      (param-names $?param-names)
                      (param-values $?param-values)
                      (precondition nil))
  (domain-effect (name ?effect-name) (part-of ?op))
  ?precond <- (pddl-formula (id ?precond-id) (part-of ?effect-name))
=>
  (bind ?grounding (ground-pddl-formula ?effect-name ?param-names ?param-values nil))
  (modify ?pa (precondition ?grounding))
)

(defrule domain-effects-check-for-sensed
  "Apply effects of an action after it succeeded."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?pa <- (plan-action (id ?id) (goal-id ?g) (plan-id ?p) (action-name ?op)
                      (state EXECUTION-SUCCEEDED)
                      (param-names $?action-param-names)
                      (param-values $?action-param-values)
                      (precondition ?grounding-id))
  (domain-operator (name ?op))
  =>
  (bind ?next-state SENSED-EFFECTS-HOLD)
  (do-for-all-facts ((?e domain-effect) (?pred domain-predicate))
    (and ?pred:sensed (eq ?e:part-of ?op) (eq ?e:predicate ?pred:name))
    ; apply if this effect is unconditional or the condition is satisfied
    (if (or (not (any-factp ((?cep pddl-formula)) (eq ?cep:part-of ?e:name)))
            (any-factp ((?cep grounded-pddl-formula))
                       (and (eq ?cep:part-of ?e:name) ?cep:is-satisfied
                            (any-factp ((?grounding pddl-grounding))
                                       (and (eq ?cep:grounding ?grounding:id)
                                            (eq ?grounding:id ?grounding-id)
                                       )
                             )
                       )
            )
        )
    then
      (bind ?values
            (domain-ground-effect ?e:param-names ?e:param-constants
                                  ?action-param-names ?action-param-values))

      (assert (domain-pending-sensed-fact (name ?pred:name) (action-id ?id) (goal-id ?g) (plan-id ?p)
                                          (param-values ?values) (type ?e:type)))
      (bind ?next-state SENSED-EFFECTS-WAIT)
    )
  )
  (modify ?pa (state ?next-state))
)

(defrule domain-effects-ignore-sensed
  "Do not wait for sensed effects if the operator is not a waiting operator."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?pa <- (plan-action	(id ?id) (action-name ?op) (state SENSED-EFFECTS-WAIT))
	(domain-operator (name ?op) (wait-sensed FALSE))
	=>
	(modify ?pa (state SENSED-EFFECTS-HOLD))
)

(defrule domain-effects-apply
  "Apply effects of an action after it succeeded."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?pa <- (plan-action	(id ?id) (goal-id ?g) (plan-id ?p) (action-name ?op)
                      (state SENSED-EFFECTS-HOLD)
                      (param-names $?action-param-names)
                      (param-values $?action-param-values)
                      (precondition ?grounding-id))
  (domain-operator (name ?op))
  =>
  (do-for-all-facts ((?e domain-effect) (?pred domain-predicate))
    (and (not ?pred:sensed) (eq ?e:part-of ?op) (eq ?e:predicate ?pred:name))

    ; apply if this effect is unconditional or the condition is satisfied
    (if (or (not (any-factp ((?cep pddl-formula)) (eq ?cep:part-of ?e:name)))
            (any-factp ((?cep grounded-pddl-formula))
                       (and (eq ?cep:part-of ?e:name) ?cep:is-satisfied
                            (any-factp ((?grounding pddl-grounding))
                                       (and (eq ?cep:grounding ?grounding:id)
                                            (eq ?grounding:id ?grounding-id)
                                       )
                             )
                       )
            )
        )
    then
      (bind ?values
            (domain-ground-effect ?e:param-names ?e:param-constants
                                  ?action-param-names ?action-param-values))

      (if (eq ?e:type POSITIVE)
        then
          (assert (domain-fact (name ?pred:name) (param-values ?values)))
        else
          ; Check if there is also a positive effect for the predicate with the
          ; same values. Only apply the negative effect if no such effect
          ; exists.
          ; NOTE: This does NOT work for conditional effects.
          (if (not (any-factp
                    ((?oe domain-effect))
                    (and
                      (eq ?oe:part-of ?op) (eq ?oe:predicate ?pred:name)
                      (eq ?oe:type POSITIVE)
                      (eq ?values
                          (domain-ground-effect ?oe:param-names
                            ?oe:param-constants ?action-param-names
                            ?action-param-values))
                    )))
            then
              (delayed-do-for-all-facts ((?df domain-fact))
                (and (eq ?df:name ?pred:name) (eq ?df:param-values ?values))
                (retract ?df)
              )
        )
      )
    )
  )
  (modify ?pa (state EFFECTS-APPLIED))
)

(defrule domain-effect-wait-sensed-done
  "After the effects of an action have been applied, change it to SENSED-EFFECTS-HOLD."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?a <- (plan-action (id ?action-id) (state SENSED-EFFECTS-WAIT) (plan-id ?p) (goal-id ?g))
  (not (domain-pending-sensed-fact (action-id ?action-id) (goal-id ?g) (plan-id ?p)))
  =>
  (modify ?a (state SENSED-EFFECTS-HOLD))
)

(defrule domain-effect-sensed-remove-on-removed-action
  "Remove domain-pending-sensed-fact when the corresponding action was removed"
  ?ef <- (domain-pending-sensed-fact (action-id ?action-id) (goal-id ?g) (plan-id ?p))
  (not (plan-action (id ?action-id) (plan-id ?p) (goal-id ?g)))
  =>
  (retract ?ef)
)

(defrule domain-action-final
  "After the effects of an action have been applied, change it to FINAL."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?a <- (plan-action (id ?action-id) (state EFFECTS-APPLIED) (precondition ?pre))
  =>
  (modify ?a (state FINAL))
  (domain-retract-grounding ?pre)
)

(defrule domain-action-failed
  "An action has failed."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?a <- (plan-action (id ?action-id) (state EXECUTION-FAILED) (precondition ?pre))
  =>
  (modify ?a (state FAILED))
  (domain-retract-grounding ?pre)
)

(defrule domain-check-if-action-is-executable-without-precondition
  "If the precondition of an action does not exist, the action is alwaysexecutable."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?action <- (plan-action (id ?action-id)
                          (action-name ?action-name)
                          (executable FALSE)
                          (precondition nil))
=>
  (modify ?action (executable TRUE))
)

(defrule domain-check-operator-of-action-exists
  "Make sure that for each action in a plan, the respective operator exists."
  (plan-action (action-name ?op))
  (not (domain-operator (name ?op)))
  =>
  (assert (domain-error (error-type operator-of-action-does-not-exist)
    (error-msg (str-cat "Operator of action " ?op " does not exist"))))
)
