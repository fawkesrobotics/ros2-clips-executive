
;---------------------------------------------------------------------------
;  skills-actions.clp - CLIPS executive - execute skill actions
;
;  Created: Wed Sep 20 15:46:48 2017
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;							2021       Ivaylo Doychev
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate skill-action-exec-info
	(slot goal-id (type SYMBOL))
	(slot plan-id (type SYMBOL))
	(slot action-id (type INTEGER))
	;(slot channel (type INTEGER))
	(slot skill-name (type SYMBOL))
	(slot skill-id (type SYMBOL))
	(slot robot (type STRING) (default ""))
	(slot executor (type STRING) (default ""))
	(multislot skill-args)
)

(deftemplate skill-action-mapping
	(slot name (type SYMBOL) (default ?NONE))
	(slot executor (type STRING) (default ""))
	(slot map-string (type STRING))
)

(defrule skill-action-init-map-to
	(confval (path "/clips_executive/spec") (type STRING) (value ?spec))
	(domain-operator (name ?action))
	(confval (path ?p&:(eq (str-index (str-cat "/clips_executive/specs/" ?spec "/action-mapping/" ?action "/mapped-to") ?p) 1))
	         (type STRING) (value ?s))
	=>
	(assert (skill-action-mapping (name ?action) (map-string ?s)))
)
(defrule skill-action-set-mapping-executor
	?sam <- (skill-action-mapping (name ?action) (executor ""))
	(confval (path "/clips_executive/spec") (type STRING) (value ?spec))
	(confval (path ?p&:(eq (str-index (str-cat "/clips_executive/specs/" ?spec "/action-mapping/" ?action "/executor") ?p) 1))
	         (type STRING) (value ?s))
	=>
	(if (neq ?s "") then
		(modify ?sam (name ?action) (executor ?s))
	)
)

(defrule skill-action-set-action-executor
	(declare (salience ?*SALIENCE-HIGH*))
	?pa <- (plan-action (state FORMULATED)
	                    (action-name ?action-name)
	                    (executor ""))
	(skill-action-mapping (name ?action-name) (executor ?exec&:(neq ?exec "")))
	=>
	(modify ?pa (executor ?exec))
)

(defrule skill-action-start
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (state PENDING)
	                    (action-name ?action-name) (executable TRUE)
	                    (robot ?robot)
	                    (executor ?executor)
	                    (param-names $?params)
	                    (param-values $?param-values))
	(skill-action-mapping (name ?action-name))
	(not (skill-action-exec-info (robot ?robot)))
	=>
	(bind ?skill-id (skill-call ?action-name ?params ?param-values ?robot ?executor))
	(modify ?pa (state WAITING))
	(bind ?args (create$))
	(loop-for-count (?i (length$ ?params))
		(bind ?args (append$ ?args (nth$ ?i ?params) (nth$ ?i ?param-values)))
	)
	(assert (skill-action-exec-info (goal-id ?goal-id) (plan-id ?plan-id)
	                               (action-id ?id) (skill-id ?skill-id)
	                               (skill-name ?action-name) (executor ?executor)
	                               (skill-args ?args) (robot ?robot)))
)

(defrule skill-action-running
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (executor ?exec)
	                    (action-name ?action-name) (state WAITING) (robot ?robot))
	?pe <- (skill-action-exec-info (goal-id ?goal-id) (plan-id ?plan-id) (executor ?exec)
	                              (action-id ?id) (skill-id ?skill-id) (robot ?robot))
	(skill (id ?skill-id) (status S_RUNNING) (executor ?exec))
	=>
	(printout t "Action " ?action-name " is running" crlf)
	(modify ?pa (state RUNNING))
)

(defrule skill-action-final
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (executor ?exec)
	                    (action-name ?action-name) (state WAITING|RUNNING) (robot ?robot))
	?pe <- (skill-action-exec-info (goal-id ?goal-id) (plan-id ?plan-id) (executor ?exec)
	                              (action-id ?id) (skill-id ?skill-id) (robot ?robot))
	?sf <- (skill (id ?skill-id) (status S_FINAL) (robot ?robot) (executor ?exec))
	=>
	(printout t "Execution of " ?action-name " completed successfully" crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
	(retract ?sf ?pe)
)

(defrule skill-action-failed
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
	                    (action-name ?action-name) (state WAITING|RUNNING) (robot ?robot) (executor ?exec))
	?pe <- (skill-action-exec-info (goal-id ?goal-id) (plan-id ?plan-id)
	                              (action-id ?id) (skill-id ?skill-id) (robot ?robot) (executor ?exec))
	?sf <- (skill (id ?skill-id) (status S_FAILED) (error-msg ?error) (robot ?robot) (executor ?exec))
	=>
	(printout warn "Execution of " ?action-name " FAILED (" ?error ")" crlf)
	(modify ?pa (state EXECUTION-FAILED) (error-msg ?error))
	(retract ?sf ?pe)
)

(defrule skill-action-cancel-if-action-does-not-exist
	?pe <- (skill-action-exec-info (goal-id ?goal-id) (plan-id ?plan-id)
	                              (action-id ?id) (skill-id ?skill-id)
	                              (robot ?robot) (executor ?exec))
	(skill (id ?skill-id) (status S_RUNNING) (robot ?robot) (executor ?exec))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (executor ?exec)
	                  (robot ?robot)))
	=>
	(printout warn
		  "Cancelling Skill Execution, corresponding action does not exist" crlf)
	(call-skill-cancel ?robot ?exec)
	(retract ?pe)
)

(defrule skill-action-retract-execinfo-without-action
	?pe <- (skill-action-exec-info (goal-id ?goal-id) (plan-id ?plan-id) (executor ?exec)
	                              (action-id ?id) (skill-id ?skill-id) (robot ?robot))
	(not (skill (status S_RUNNING) (id ?skill-id) (executor ?exec)))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?action-id) (executor ?exec)))
	=>
	(retract ?pe)
)
