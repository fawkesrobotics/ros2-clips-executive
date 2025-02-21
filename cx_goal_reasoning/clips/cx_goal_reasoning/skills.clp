
; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

;---------------------------------------------------------------------------
;  skills.clp - CLIPS skill utilities
;
;  Created: Thu Dec 20 12:06:02 2012 (Train from Freiburg to Aachen)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;							2021  Ivaylo Doychev
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
	?*SKILL-INIT-TIMEOUT-SEC* = 5
	?*SKILL-ACQUIRE-CONTROL-RETRY-INTERVAL-SEC* = 3
)

(deftemplate skill
	; Unique ID, meaningful only in the realm of this skill executor
	; and for components that interface with it.
	; It is guaranteed to be unique, also for the case when executing the
	; exact same skill a second time.
	(slot id (type SYMBOL))
	(slot name (type SYMBOL))
	(slot parameters (type SYMBOL))
	(slot status (type SYMBOL) (allowed-values S_IDLE S_RUNNING S_FINAL S_FAILED))
	(slot error-msg (type STRING))
	(slot skill-string (type STRING))
	(slot start-time (type FLOAT))
	(slot last-updated (type FLOAT))
	(slot robot (type STRING) (default ""))
	(slot executor (type STRING) (default ""))
)

(deftemplate skill-feedback
; Populated by the Skill Execution Feature
  (slot skill-id (type SYMBOL))
  (slot status (type SYMBOL) (allowed-values S_IDLE S_RUNNING S_FINAL S_FAILED))
  (slot error (type STRING))
  (slot robot (type STRING) (default ""))
  (slot executor (type STRING) (default ""))
  (slot time (type FLOAT))
)

(deffunction skill-call (?action-name ?param-names ?param-values ?robot ?executor)
	(bind ?sks (map-action-skill ?action-name ?param-names ?param-values))

	(bind ?action_params "")
	(foreach ?param-value ?param-values

		(if (eq ?action_params "")
			then
			(bind ?action_params (str-cat ?action_params ?param-value))
		else
		  (bind ?action_params (str-cat ?action_params " " ?param-value)))
	)
	(bind ?id (sym-cat ?action-name "_" (gensym*)))
	(printout t "Calling mapped skill '" ?sks "'" crlf)
	(printout t "of action '" ?action-name " " ?action_params"'" crlf)

	(call-skill-execution ?id ?action-name ?action_params ?sks ?robot ?executor)
	(assert (skill (id ?id) (name (sym-cat ?action-name)) (parameters ?action_params)
						(skill-string ?sks) (robot ?robot) (executor ?executor) (status S_IDLE)
						(start-time (now))))
	(return ?id)
)

(defrule skill-status-update
  (declare (salience ?*SALIENCE-HIGH*))
  ?sf <- (skill-feedback (skill-id ?skill-id) (robot ?robot) (status ?new-status)
                           (error ?error-msg) (time ?t))
  ?s <- (skill (name ?n) (id ?skill-id) (status ?old-status&~?new-status&~S_FINAL&~S_FAILED)
                (robot ?robot) (last-updated ?l&:(> ?t ?l)))
  =>
  (printout t "Skill " ?n " is " ?new-status", was: " ?old-status crlf)
  (retract ?sf)
  (modify ?s (status ?new-status) (error-msg ?error-msg))
)

(defrule skill-status-update-nochange
	?sf <- (skill-feedback (skill-id ?skill-id) (robot ?robot) (status ?new-status))
  (skill (name ?n) (id ?skill-id) (status ?new-status) (robot ?robot))
  =>
  (retract ?sf)
)

(defrule skill-start-timeout
	(time ?now)
  	?sf <- (skill (name ?n) (status S_IDLE)
	(start-time ?st&:(timeout ?now ?st ?*SKILL-INIT-TIMEOUT-SEC*)))
  =>
	(printout warn "Timeout starting skill " ?n " (" ?*SKILL-INIT-TIMEOUT-SEC* " sec): assuming failure" crlf)
	(modify ?sf (status S_FAILED) (error-msg "Start timeout"))
)

(defrule skill-status-update-no-skill
	?sf <- (skill-feedback (skill-id ?skill-id) (robot ?robot) (status ?new-status))
  (not (skill (name ?n) (id ?skill-id) (robot ?robot)))
  =>
  (printout warn "Received unexpected skill feedback " ?new-status " for " ?skill-id " of " ?robot crlf)
  (retract ?sf)
)

; (defrule skill-idle
;  (declare (salience -4000))
;  ?sf <- (skill (status S_FINAL|S_FAILED))
;  =>
;  (retract ?sf)
; )

; (defrule skill-control-acquire
; 	(time $?now)
; 	?sc <- (skiller-control (skiller ?skiller) (acquired FALSE) (acquiring FALSE)
; 	       (last-try $?lt&:(timeout ?now ?lt ?*SKILL-ACQUIRE-CONTROL-RETRY-INTERVAL-SEC*)))
; 	=>
; 	(printout t "Acquiring exclusive skiller control for " ?skiller crlf)
; 	(bind ?m (blackboard-create-msg (str-cat "SkillerInterface::" ?skiller) "AcquireControlMessage"))

; 	(if (any-factp ((?c confval)) (and (eq ?c:path "/clips-executive/steal-skiller-control")
; 		(eq ?c:value TRUE)))
; 	then
; 		(blackboard-set-msg-field ?m "steal_control" TRUE)
; 	)
; 	(bind ?msgid (blackboard-send-msg ?m))
; 	(if (neq ?msgid 0)
; 	then
; 		(modify ?sc (acquiring TRUE) (last-try ?now))
; 	else
; 		(modify ?sc (last-try ?now))
; 	)
; )

; (defrule skill-control-acquired
; 	?sc <- (skiller-control (skiller ?skiller) (acquired FALSE) (acquiring TRUE))
; 	(blackboard-interface (type "SkillerInterface") (id ?skiller) (serial ?s))
;   (SkillerInterface (id ?skiller) (exclusive_controller ?s))
; 	=>
; 	(printout t "Acquired exclusive skiller control" crlf)
; 	(modify ?sc (acquired TRUE) (acquiring FALSE))
; )

; (defrule skill-control-lost
; 	?sc <- (skiller-control (skiller ?skiller) (acquired TRUE))
; 	(blackboard-interface (type "SkillerInterface") (id ?skiller) (serial ?s))
; 	(SkillerInterface (id ?skiller) (exclusive_controller ~?s))
; 	=>
; 	(printout t "Lost exclusive skiller control" crlf)
; 	(modify ?sc (acquired FALSE))
; )

; (defrule skill-control-release
; 	(declare (salience 1000))
; 	(executive-finalize)
; 	?sc <- (skiller-control (skiller ?skiller) (acquired TRUE))
; 	?bi <- (blackboard-interface (type "SkillerInterface") (id ?skiller))
; 	=>
; 	(printout t "Releasing control on finalize" crlf)
; 	(bind ?m (blackboard-create-msg (str-cat "SkillerInterface::" ?skiller) "ReleaseControlMessage"))
; 	(blackboard-send-msg ?m)
; 	(blackboard-close "SkillerInterface" ?skiller)
; 	(modify ?sc (acquired FALSE))
; 	(retract ?bi)
; )



; Quick skill execution test
;(defrule skill-test
;	(start)
;	(skiller-control (acquired TRUE))
;	=>
;	(skill-call say (create$ text "Hello world" wait true))
;)
