;Header and description

(define (domain move_trutlebot3)

  ;remove requirements that are not needed
  (:requirements :strips :fluents :typing :adl)

  (:types
    robot waypoint
  )

  (:predicates
    (robot_at ?r - robot ?w - waypoint)
    (connected ?w1 ?w2 - waypoint)
  )

  (:action move
    :parameters (?r - robot ?w1 ?w2 - waypoint)
    :precondition (and (robot_at ?r ?w1) (connected ?w1 ?w2))
    :effect (and (robot_at ?r ?w2) (not (robot_at ?r ?w1)))
  )

)