
; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

;---------------------------------------------------------------------------
;  lock-actions.clp - CLIPS executive - lock action executors
;
;  Created: Tue Apr 24 21:32:19 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Handlers for the following actions:
;
; lock ?mutex-name
;
; Acquires the lock for a specific mutex.
; Example PDDL operator:
; (:action lock
;   :parameters (?m - mutex)
;   :precondition (not (locked ?m))
;   :effect (and (locked ?m))
; )
;
;
; unlock ?mutex-name
;
; Release a lock which is currently been held.
; Example PDDL operator:
; (:action unlock
;   :parameters (?m - mutex)
;   :precondition (locked ?m)
;   :effect (not (locked ?m))
; )
;
;
; flush-locks
;
; Remove all locks currently in the robot memory. Useful during initialization.
; Example PDDL operator:
; (:action flush-locks
;   :precondition (is-initialization)
;   :effect (forall (?m - mutex) (not (locked ?m)))
; )
;
;
; expire-locks
;
; Remove locks of mutexes which have been acquired a configurable
; duration in the past. The value is taken from the spec configuration
; value coordination/mutex/max-age-sec.
; The action could be used in a MAINTAIN goal.

(defrule lock-actions-lock-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state PENDING)
	                    (action-name lock) (executable TRUE)
	                    (param-names $?param-names)
	                    (param-values $?param-values))
	=>
	(bind ?lock-name (plan-action-arg name ?param-names ?param-values))
	(if (any-factp ((?mutex mutex))
	               (and (eq ?mutex:name ?lock-name)
	                    (or (neq ?mutex:request NONE)
	                        (eq ?mutex:state LOCKED))))
	 then
		(modify ?pa (state EXECUTION-FAILED))
	else
		(mutex-try-lock-async ?lock-name)
		(modify ?pa (state RUNNING))
	)
)

(defrule lock-actions-lock-acquired
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
	                    (action-name lock) (state RUNNING)
	                    (param-names $?param-names) (param-values $?param-values))
	?mf <- (mutex (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
	              (request LOCK) (response ACQUIRED))
	=>
	(modify ?pa (state EXECUTION-SUCCEEDED))
	(modify ?mf (request NONE) (response NONE))
)

(defrule lock-actions-lock-rejected
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
	                    (action-name lock) (state RUNNING)
	                    (param-names $?param-names) (param-values $?param-values))
	?mf <- (mutex (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
	              (request LOCK) (response REJECTED|ERROR) (error-msg ?error-msg))
	=>
	(modify ?pa (state EXECUTION-FAILED) (error-msg ?error-msg))
	(modify ?mf (request NONE) (response NONE) (error-msg ""))
)

(defrule lock-actions-unlock-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state PENDING)
	                    (action-name unlock) (executable TRUE)
	                    (param-names $?param-names)
	                    (param-values $?param-values))
	=>
	(bind ?lock-name (plan-action-arg name ?param-names ?param-values))
	; The following performs a synchronous/blocking call
	;(bind ?rv (robmem-mutex-unlock (str-cat ?lock-name)))
	;(modify ?pa (state (if ?rv then EXECUTION-SUCCEEDED else EXECUTION-FAILED)))
	(mutex-unlock-async ?lock-name)
	(modify ?pa (state RUNNING))
)

(defrule lock-actions-unlock-done
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
	                    (action-name unlock) (state RUNNING)
	                    (param-names $?param-names) (param-values $?param-values))
	?mf <- (mutex (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
	              (state OPEN) (request UNLOCK))
	=>
	(modify ?pa (state EXECUTION-SUCCEEDED))
	(modify ?mf (request NONE) (response NONE))
)

(defrule lock-actions-flush-locks-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state PENDING)
	                    (action-name flush-locks) (executable TRUE))
	=>
	(mutex-flush-locks-async)
	(modify ?pa (state RUNNING))
)

(defrule lock-actions-flush-locks-succeeded
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
	                    (action-name flush-locks) (state RUNNING))
	?mf <- (mutex-expire-task (task FLUSH) (state COMPLETED))
	=>
	(modify ?pa (state EXECUTION-SUCCEEDED))
	(retract ?mf)
)

(defrule lock-actions-flush-locks-failed
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
	                    (action-name flush-locks) (state RUNNING))
	?mf <- (mutex-expire-task (task FLUSH) (state FAILED))
	=>
	(modify ?pa (state EXECUTION-FAILED))
	(retract ?mf)
)

(defrule lock-actions-expire-locks-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state PENDING)
	                    (action-name expire-locks) (executable TRUE))
	=>
	(mutex-expire-locks-async)
	(modify ?pa (state RUNNING))
)

(defrule lock-actions-expire-locks-succeeded
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
	                    (action-name expire-locks) (state RUNNING))
	?mf <- (mutex-expire-task (task EXPIRE) (state COMPLETED))
	=>
	(modify ?pa (state EXECUTION-SUCCEEDED))
	(retract ?mf)
)

(defrule lock-actions-expire-locks-failed
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
	                    (action-name expire-locks) (state RUNNING))
	?mf <- (mutex-expire-task (task EXPIRE) (state FAILED))
	=>
	(modify ?pa (state EXECUTION-FAILED))
	(retract ?mf)
)
