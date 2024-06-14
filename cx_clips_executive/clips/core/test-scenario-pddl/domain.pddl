; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

;---------------------------------------------------------------------------
;  domain.pddl - A simple hello world domain
;---------------------------------------------------------------------------

(define (domain hello-world)
  (:requirements :adl :strips :typing)
  (:types
    name text
  )
  (:predicates
    (said ?n - name ?t - text)
    (unsaid ?n - name ?t - text)
    (locked ?n - name)
  )
  (:action say-hello
    :parameters (?name - name ?hello - text)
    :precondition (and (unsaid ?name ?hello))
    :effect (and (said ?name ?hello))
  )
  ; (:action lock
  ;   :parameters (?name - name)
  ;   :precondition (not (locked ?name))
  ;   :effect (and (locked ?name))
  ; )
  ; (:action say-goodbye
  ;   :parameters (?name - name ?hello - text)
  ;   :precondition (and (said ?name ?hello))
  ;   :effect (and (said ?name ?hello))
  ; )
)
