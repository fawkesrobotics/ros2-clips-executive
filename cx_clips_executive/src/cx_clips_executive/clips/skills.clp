
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
	; It is guaranteed to be unique, also for the case when executin the
	; exact same skill a second time.
	(slot id (type SYMBOL))
  (slot name (type SYMBOL))
  (slot parameters (type SYMBOL))
  (slot status (type SYMBOL) (allowed-values S_IDLE S_RUNNING S_FINAL S_FAILED))
  (slot error-msg (type STRING))
  (slot skill-string (type STRING))
	(multislot start-time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
	(slot agent-id (type STRING) (default ""))
)

(deftemplate skill-feedback
; Populated by the Skill Execution Feature
	(slot skill-id (type SYMBOL))
  (slot status (type SYMBOL) (allowed-values S_IDLE S_RUNNING S_FINAL S_FAILED))
  (slot error (type STRING))
	(slot agent-id (type STRING) (default ""))
)

; (deftemplate skiller-control
; 	(slot skiller (type STRING) (default ""))
; 	(slot acquired (type SYMBOL) (allowed-values FALSE TRUE))
; 	(slot acquiring (type SYMBOL) (allowed-values FALSE TRUE))
;   (multislot last-try (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
; )

(assert (ff-feature-loaded skill_execution))

(deffunction skill-call (?action-name ?param-names ?param-values $?opt-agent-id)
	(bind ?agent-id "")
	; (if (> (length$ ?opt-agent-id) 0) then (bind ?agent-id (nth$ 1 ?opt-agent-id)))
	; (if (> (length$ ?opt-agent-id) 1)
	;  then
	; 	(printout warn "skill-call: ignore unexpected params " (rest$ ?opt-agent-id) crlf)
	; )
	; rely on a function provided from the outside providing
	; a more sophisticated mapping.
	(bind ?sks (map-action-skill ?action-name ?param-names ?param-values))
	(printout logwarn "sks='" ?sks "'" crlf)

	; (bind ?id UNKNOWN)
	; (if ( and (eq ?sks "") (using fawkes))
	; 		then
	; 	(bind ?id (sym-cat ?action-name (gensym*)))
	; 	(assert (skill (id ?id) (action-name (sym-cat ?action-name)) (status S_FAILED) (start-time (now))
	; 	        (error-msg (str-cat "Failed to convert action '" ?action-name "' to skill string"))))
	; else
	(bind ?action_params "") 
	(foreach ?param-value ?param-values
		(printout t "Param-value:" ?param-value crlf)

		(if (eq ?action_params "")
			then 
			(bind ?action_params (str-cat ?action_params ?param-value))
		else
		  (bind ?action_params (str-cat ?action_params " " ?param-value)))
	)
	(bind ?id (sym-cat ?action-name "_" (gensym*)))
	(printout logwarn "Calling mapped skill '" ?sks "'" crlf)
	(printout logwarn "Calling skill '" ?action-name " " ?action_params"'" crlf)

	(call-skill-execution ?id ?action-name ?action_params ?sks ?agent-id)
	(assert (skill (id ?id) (name (sym-cat ?action-name)) (parameters ?action_params) 
						(skill-string ?sks) (agent-id ?agent-id) (status S_IDLE)
						(start-time (now))))
	(return ?id)
)
; Read skill-feedback coming from the skill-execution
(defrule skill-read-information
	(declare (salience ?*SALIENCE-HIGH*))
	(time $?)
	=>
	(call-skill-read)
)

(defrule skill-status-update
  ?sf <- (skill-feedback (skill-id ?skill-id) (agent-id ?agent-id) (status ?new-status)
                           (error ?error-msg))
  ?s <- (skill (name ?n) (id ?skill-id) (status ?old-status&~?new-status&~S_FINAL&~S_FAILED)
                (agent-id ?agent-id))
  =>
  (printout t "Skill " ?n " is " ?new-status", was: " ?old-status crlf)
  (retract ?sf)
  (modify ?s (status ?new-status) (error-msg ?error-msg))
)

(defrule skill-status-update-nochange
	?sf <- (skill-feedback (skill-id ?skill-id) (agent-id ?agent-id) (status ?new-status))
  (skill (name ?n) (id ?skill-id) (status ?new-status) (agent-id ?agent-id)
  =>
  (retract ?su)
)

(defrule skill-start-timeout
	(time $?now)
  ?sf <- (skill (name ?n) (status S_IDLE)
	(start-time $?st&:(timeout ?now ?st ?*SKILL-INIT-TIMEOUT-SEC*)))
  =>
	(printout warn "Timeout starting skill " ?n " (" ?*SKILL-INIT-TIMEOUT-SEC* " sec): assuming failure" crlf)
	(modify ?sf (status S_FAILED) (error-msg "Start timeout"))
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
