(defrule load-domain
  (executive-init)
  (not (domain-loaded))
=>
  (parse-pddl-domain (path-resolve "simple-agent/domain.pddl"))
  (assert (domain-loaded))
)

(defrule domain-set-sensed-predicates
  " Mark some predicates as sensed predicates.
    That means, the truth value of these predicates can be changed not directly but by some external trigger
  "
  (domain-loaded)
  ?p <- (domain-predicate (name mps-state|zone-content) (sensed FALSE))
=>
  (modify ?p (sensed TRUE))
)


(defrule domain-set-value-predicates
  ?p <- (domain-predicate (name mps-state|zone-content) (value-predicate FALSE))
=>
  (modify ?p (value-predicate TRUE))
)


(defrule domain-nowait-actions
  " Mark some actions that have a sensed effect as non-waiting. That means the effect is applied without sensing for it "
  (domain-loaded)
	?o <- (domain-operator (name wp-put|prepare-bs|prepare-rs|prepare-ds|prepare-cs|location-unlock) (wait-sensed ~FALSE))
=>
	(modify ?o (wait-sensed FALSE))
)


(defrule domain-exogenous-actions
  "Mark all actions, that model state changes of the machines, as exogenous"
  ?op <- (domain-operator
    (name bs-dispense | cs-mount-cap | cs-retrieve-cap | rs-mount-ring1 |
          rs-mount-ring2 | rs-mount-ring3 | fulfill-order-c0 | fulfill-order-discard |
          fulfill-order-c1 | fulfill-order-c2 | fulfill-order-c3 | ss-retrieve-c0)
    (exogenous FALSE)
  )
=>
  (modify ?op (exogenous TRUE))
)

(deffunction domain-load-local-facts (?team-color)
" Initialize facts that are not synced."
  (if (eq ?team-color CYAN)
    then
        (bind ?bs C-BS)
        (bind ?cs1 C-CS1)
        (bind ?cs2 C-CS2)
        (bind ?rs1 C-RS1)
        (bind ?rs2 C-RS2)
        (bind ?ds C-DS)
        (bind ?ss C-SS)
    else
        (bind ?bs M-BS)
        (bind ?cs1 M-CS1)
        (bind ?cs2 M-CS2)
        (bind ?rs1 M-RS1)
        (bind ?rs2 M-RS2)
        (bind ?ds M-DS)
        (bind ?ss M-SS)
  )
  (assert
    (domain-fact (name at) (param-values robot1 START INPUT))
    (domain-fact (name at) (param-values robot2 START INPUT))
    (domain-fact (name at) (param-values robot3 START INPUT))
    (domain-fact (name mps-team) (param-values ?bs ?team-color))
    (domain-fact (name can-hold) (param-values robot1))
    (domain-fact (name can-hold) (param-values robot2))
    (domain-fact (name can-hold) (param-values robot3))
    (domain-fact (name mps-team) (param-values ?ds ?team-color))
    (domain-fact (name mps-team) (param-values ?ss ?team-color))
    (domain-fact (name mps-team) (param-values ?cs1 ?team-color))
    (domain-fact (name mps-team) (param-values ?cs2 ?team-color))
    (domain-fact (name mps-team) (param-values ?rs1 ?team-color))
    (domain-fact (name mps-team) (param-values ?rs2 ?team-color))

    (domain-fact (name mps-type) (param-values C-BS BS))
    (domain-fact (name mps-type) (param-values C-DS DS))
    (domain-fact (name mps-type) (param-values C-SS SS))
    (domain-fact (name mps-type) (param-values C-CS1 CS))
    (domain-fact (name mps-type) (param-values C-CS2 CS))
    (domain-fact (name mps-type) (param-values C-RS1 RS))
    (domain-fact (name mps-type) (param-values C-RS2 RS))

    (domain-fact (name mps-type) (param-values M-BS BS))
    (domain-fact (name mps-type) (param-values M-DS DS))
    (domain-fact (name mps-type) (param-values M-SS SS))
    (domain-fact (name mps-type) (param-values M-CS1 CS))
    (domain-fact (name mps-type) (param-values M-CS2 CS))
    (domain-fact (name mps-type) (param-values M-RS1 RS))
    (domain-fact (name mps-type) (param-values M-RS2 RS))
    (domain-fact (name rs-sub) (param-values THREE TWO ONE))
    (domain-fact (name rs-sub) (param-values THREE ONE TWO))
    (domain-fact (name rs-sub) (param-values THREE ZERO THREE))
    (domain-fact (name rs-sub) (param-values TWO TWO ZERO))
    (domain-fact (name rs-sub) (param-values TWO ONE ONE))
    (domain-fact (name rs-sub) (param-values TWO ZERO TWO))
    (domain-fact (name rs-sub) (param-values ONE ONE ZERO))
    (domain-fact (name rs-sub) (param-values ONE ZERO ONE))
    (domain-fact (name rs-sub) (param-values ZERO ZERO ZERO))
    (domain-fact (name rs-inc) (param-values ZERO ONE))
    (domain-fact (name rs-inc) (param-values ONE TWO))
    (domain-fact (name rs-inc) (param-values TWO THREE))
    (domain-fact (name cs-color) (param-values ?cs1 CAP_GREY))
    (domain-fact (name cs-color) (param-values ?cs2 CAP_BLACK))

    (domain-fact (name mirror-orientation) (param-values 0 180))
    (domain-fact (name mirror-orientation) (param-values 45 135))
    (domain-fact (name mirror-orientation) (param-values 90 90))
    (domain-fact (name mirror-orientation) (param-values 135 45))
    (domain-fact (name mirror-orientation) (param-values 180 0))
    (domain-fact (name mirror-orientation) (param-values 225 315))
    (domain-fact (name mirror-orientation) (param-values 270 270))
    (domain-fact (name mirror-orientation) (param-values 315 225))

    (domain-object (name CCB1) (type cap-carrier))
    (domain-object (name CCB2) (type cap-carrier))
    (domain-object (name CCB3) (type cap-carrier))
    (domain-object (name CCG1) (type cap-carrier))
    (domain-object (name CCG2) (type cap-carrier))
    (domain-object (name CCG3) (type cap-carrier))
    (domain-object (name ?bs) (type mps))
    (domain-object (name ?cs1) (type mps))
    (domain-object (name ?cs2) (type mps))
    (domain-object (name ?ds) (type mps))
    (domain-object (name ?rs1) (type mps))
    (domain-object (name ?rs2) (type mps))
    (domain-object (name ?ss) (type mps))
    (domain-object (name INPUT) (type mps-side))
    (domain-object (name OUTPUT) (type mps-side))
    (domain-object (name WAIT) (type mps-side))
    (domain-object (name ?team-color) (type team-color))
    (domain-object (name O1) (type order))
    (domain-object (name O2) (type order))
    (domain-object (name O3) (type order))
    (domain-object (name O4) (type order))
    (domain-object (name O5) (type order))
    (domain-object (name O6) (type order))
    (domain-object (name O7) (type order))
    (domain-object (name O8) (type order))
    (domain-object (name O9) (type order))
    (domain-fact (name rs-ring-spec) (param-values ?rs1 RING_NONE ZERO))
    (domain-fact (name rs-ring-spec) (param-values ?rs2 RING_NONE ZERO))
  )
)

(defrule domain-load-initial-facts
" Load all initial domain facts on startup of the game "
  (domain-loaded)
  =>
  (printout info "Initializing worldmodel" crlf)
  (bind ?team-color MAGENTA)
  (if (eq ?team-color CYAN)
    then
        (bind ?bs C-BS)
        (bind ?cs1 C-CS1)
        (bind ?cs2 C-CS2)
        (bind ?rs1 C-RS1)
        (bind ?rs2 C-RS2)
        (bind ?ds C-DS)
        (bind ?ss C-SS)
    else
        (bind ?bs M-BS)
        (bind ?cs1 M-CS1)
        (bind ?cs2 M-CS2)
        (bind ?rs1 M-RS1)
        (bind ?rs2 M-RS2)
        (bind ?ds M-DS)
        (bind ?ss M-SS)
  )

  (domain-load-local-facts ?team-color)
  (foreach ?mps (create$ ?bs ?cs1 ?cs2 ?rs1 ?rs2 ?ds ?ss)
    (assert
      (domain-fact (name mps-side-free) (param-values ?mps INPUT))
      (domain-fact (name mps-side-free) (param-values ?mps OUTPUT))
      (domain-fact (name mps-side-approachable) (param-values ?mps INPUT))
      (domain-fact (name mps-side-approachable) (param-values ?mps OUTPUT))
      (domain-fact (name mps-side-approachable) (param-values ?mps WAIT))
    )
  )
  (foreach ?robot (create$ robot1 robot2 robot3)
    (assert
      (domain-fact (name at) (param-values ?robot START INPUT))
      (domain-fact (name can-hold) (param-values ?robot))
    )
  )
  (foreach ?cc (create$ CCB1 CCB2 CCB3 CCG1 CCG2 CCG3)
    (assert
      (domain-fact (name wp-base-color) (param-values ?cc BASE_CLEAR))
      (domain-fact (name wp-ring1-color) (param-values ?cc RING_NONE))
      (domain-fact (name wp-ring2-color) (param-values ?cc RING_NONE))
      (domain-fact (name wp-ring3-color) (param-values ?cc RING_NONE))
    )
  )
  (assert
    (domain-fact (name wp-cap-color) (param-values CCB1 CAP_BLACK))
    (domain-fact (name wp-cap-color) (param-values CCB2 CAP_BLACK))
    (domain-fact (name wp-cap-color) (param-values CCB3 CAP_BLACK))
    (domain-fact (name wp-cap-color) (param-values CCG1 CAP_GREY))
    (domain-fact (name wp-cap-color) (param-values CCG2 CAP_GREY))
    (domain-fact (name wp-cap-color) (param-values CCG3 CAP_GREY))
    (domain-fact (name wp-on-shelf) (param-values CCB1 ?cs2 LEFT))
    (domain-fact (name wp-on-shelf) (param-values CCB2 ?cs2 MIDDLE))
    (domain-fact (name wp-on-shelf) (param-values CCB3 ?cs2 RIGHT))
    (domain-fact (name wp-on-shelf) (param-values CCG1 ?cs1 LEFT))
    (domain-fact (name wp-on-shelf) (param-values CCG2 ?cs1 MIDDLE))
    (domain-fact (name wp-on-shelf) (param-values CCG3 ?cs1 RIGHT))
    (domain-fact (name cs-can-perform) (param-values ?cs1 RETRIEVE_CAP))
    (domain-fact (name cs-can-perform) (param-values ?cs2 RETRIEVE_CAP))
    (domain-fact (name cs-free) (param-values ?cs1))
    (domain-fact (name cs-free) (param-values ?cs2))
    (domain-fact (name cs-color) (param-values ?cs1 CAP_GREY))
    (domain-fact (name cs-color) (param-values ?cs2 CAP_BLACK))
    (domain-fact (name rs-filled-with) (param-values ?rs1 ZERO))
    (domain-fact (name rs-filled-with) (param-values ?rs2 ZERO))
  )
  (assert (domain-facts-loaded))
)
