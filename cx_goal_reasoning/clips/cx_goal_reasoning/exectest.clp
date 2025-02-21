
; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(defrule start
	(start)
	=>
	(printout t "Spec starting" crlf)
	(assert (plan-action (id 1) (plan-name foo) (duration 4.0)
											 (action-name say) (param-names text wait))
                       (param-values "Hello world" true))
	(assert (plan-action (id 2) (plan-name foo) (duration 4.0)
											 (action-name say) (param-names text wait)
                       (param-values "Good bye" true)))
)

(defrule done
	(plan-action (id ?id) (state ?s))
	=>
	(printout t "Action " ?id " is now " ?s crlf)
)
