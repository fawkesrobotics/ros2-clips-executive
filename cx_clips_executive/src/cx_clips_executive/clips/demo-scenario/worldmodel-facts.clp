; (defrule test-wm-facts
; 	(executive-init)
; 	=>
; 	(assert

; 	)
; )


(defrule state-robot-location
	;(wm-fact (key "/domain/fact/location?R-1&C-BS") (value TRUE))
	(wm-fact (key domain fact at args? rover ?r waypoint ?p))
	=>
	(printout error "Robot " ?r " is at " ?p crlf)
)
