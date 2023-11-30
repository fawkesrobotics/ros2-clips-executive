
;---------------------------------------------------------------------------
;  init.clp - CLIPS executive
;
;  Created: Tue Sep 19 16:49:42 2017
;  Copyright  2012-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate executive-init-request
	(slot name (type SYMBOL))
	(slot order (type INTEGER))
	(slot feature (type SYMBOL) (allowed-values FALSE TRUE))
	(slot wait-for (type SYMBOL) (default nil))
	(multislot files (type STRING))
	(slot state (type SYMBOL) (allowed-values PENDING FEATURE-REQUESTED FEATURE-DONE
	                                          WAIT-FOR COMPLETED ERROR))
	(multislot error-msgs (type STRING))
)

(deftemplate executive-init-signal
	(slot id (type SYMBOL))
	(slot ok (type SYMBOL) (allowed-values TRUE FALSE))
	(slot error-msg (type STRING))
)

(defrule executive-load-config
  (declare (salience ?*SALIENCE-INIT*))
  (executive-init)
  =>
  (config-load ?*CX_CONFIG* "/clips_executive")
  (config-load ?*FEATURES_CONFIG* "/clips_features_manager")
)

(deffunction cx-debug-unwatch-facts ($?templates)
	(bind ?deftemplates (get-deftemplate-list))
	(printout debug "Unwatching fact templates " ?templates crlf)
	(foreach ?v ?templates
		(bind ?v-sym (sym-cat ?v))
		(if (member$ ?v-sym ?deftemplates)
		 then (unwatch facts ?v-sym)
		 else (printout warn "Cannot unwatch " ?v " (deftemplate not defined)" crlf)
		 )
	)
)

(deffunction cx-debug-unwatch-rules ($?rules)
	(bind ?defrules (get-defrule-list))
	(printout debug "Unwatching rules " ?rules crlf)
	(foreach ?v ?rules
		(bind ?v-sym (sym-cat ?v))
		(if (member$ ?v-sym ?defrules)
		 then (unwatch rules ?v-sym)
		 else (printout warn "Cannot unwatch " ?v " (defrule not defined)" crlf)
		)
	)
)

(defrule executive-configurable-unwatch
  (declare (salience ?*SALIENCE-INIT*))
  (executive-init)
  =>
	(do-for-fact ((?c confval)) (and (eq ?c:path "/clips_executive/unwatch-facts")
																	 (eq ?c:type STRING) ?c:is-list)
	 (cx-debug-unwatch-facts ?c:list-value)
  )
	(do-for-fact ((?c confval)) (and (eq ?c:path "/clips_executive/unwatch-rules")
																	 (eq ?c:type STRING) ?c:is-list)
	 (cx-debug-unwatch-rules ?c:list-value)
  )
)

(deffunction cx-init-indexes ()
	(bind ?rv (create$))
	(do-for-all-facts ((?c confval)) (str-prefix (str-cat "/clips_executive/init/")
																							 ?c:path)
		(bind ?path-elements (str-split ?c:path "/"))
		(bind ?idx (integer (eval (nth$ 3 ?path-elements))))
		(if (not (member$ ?idx ?rv)) then	(bind ?rv (append$ ?rv ?idx)))
	)
	(return (sort > ?rv))
)

(deffunction cx-assert-feature-requests ()
	(bind ?cfgpfx (str-cat "/clips_features_manager/clips_features_list"))
	(bind ?state PENDING)
	(bind ?wait-for nil)
	(bind ?request-num 0)
	(if (not (do-for-fact ((?c confval)) (eq ?c:path ?cfgpfx)
		(foreach ?name ?c:list-value
			(assert (executive-init-request (state PENDING)
					(order ?request-num)
					(name (sym-cat ?name)) (feature TRUE)))
			(bind ?request-num (+ ?request-num 1))
		)))
	then
		(printout warn "No features were loaded" cflf)
	)
	(return ?request-num)
)

(deffunction cx-assert-init-requests (?offset)
	(bind ?cfgpfx (str-cat "/clips_executive/init/"))
	(foreach ?i (cx-init-indexes)
		(bind ?feature FALSE)
		(bind ?name "MISSING")
		(bind ?files (create$))
		(bind ?error-msgs (create$))
		(bind ?state PENDING)
		(bind ?wait-for nil)
		(if (not (any-factp ((?c confval)) (eq (str-cat ?cfgpfx ?i "/name") ?c:path)))
		 then
			(bind ?state ERROR)
			(bind ?error-msgs (append$ ?error-msgs (str-cat " entry " ?i " is missing name entry")))
		 else
			(do-for-fact ((?c confval)) (eq (str-cat ?cfgpfx ?i "/name") ?c:path)
				(bind ?name ?c:value)
			)
			(do-for-fact ((?c confval)) (eq (str-cat ?cfgpfx ?i "/feature-request") ?c:path)
				(bind ?feature ?c:value)
			)
			(do-for-fact ((?c confval)) (eq (str-cat ?cfgpfx ?i "/file") ?c:path)
				(bind ?files (append$ ?files ?c:value))
			)
			(do-for-fact ((?c confval)) (eq (str-cat ?cfgpfx ?i "/wait-for") ?c:path)
				(bind ?wait-for (sym-cat ?c:value))
			)
			(do-for-fact ((?c confval)) (eq (str-cat ?cfgpfx ?i "/files") ?c:path)
				(if (not ?c:is-list)
				 then
					(printout warn "Config entry " (str-cat ?cfgpfx ?i "/files") " is not a list value, ignoring")
				 else
					(bind ?files (append$ ?files ?c:list-value))
				)
			)
		)
		(assert (executive-init-request (state ?state) (error-msgs ?error-msgs)
																		(order (+ ?i-index ?offset))
																		(name (sym-cat ?name)) (feature ?feature)
																		(files ?files) (wait-for ?wait-for)))
	)
)

(defrule executive-init-start
  (declare (salience ?*SALIENCE-INIT-LATE*))
	(executive-init)
	=>
	(bind ?offset (cx-assert-feature-requests))
	(cx-assert-init-requests ?offset)
)

(defrule executive-init-failed
  (declare (salience ?*SALIENCE-INIT*))
	(executive-init)
	(executive-init-request (state ERROR) (order ?i) (error-msgs $?error-msgs))
	?sf <- (executive-init-stage)
	=>
	(printout error crlf)
	(printout error "***********************************************************" crlf)
	(printout error crlf)
	(printout error " request " ?i " failed: " ?error-msgs crlf)
	(printout error crlf)
	(printout error "***********************************************************" crlf)
	(printout error crlf)
	(retract ?sf)
	(assert (executive-init-stage FAILED))
)

(defrule executive-init-stage-request-feature
	(executive-init)
	?ir <- (executive-init-request (state PENDING) (order ?order)
																 (name ?name) (feature TRUE))
	(not (executive-init-request (state ~COMPLETED) (order ?order2&:(< ?order2 ?order))))
	(ff-feature ?name)
	=>
	(printout t "Init: requesting feature " ?name crlf)
	(ff-feature-request (str-cat ?name))
	(modify ?ir (state FEATURE-REQUESTED))
)

(defrule executive-init-stage-request-no-feature
	(executive-init)
	?ir <- (executive-init-request (state PENDING) (order ?order)
																 (name ?name) (feature FALSE))
	(not (executive-init-request (state ~COMPLETED) (order ?order2&:(< ?order2 ?order))))
	=>
	(modify ?ir (state FEATURE-DONE))
)

(defrule executive-init-stage-request-feature-unavailable
	(executive-init)
	?ir <- (executive-init-request (state PENDING) (order ?order)
																 (name ?name) (feature TRUE))
	(not (executive-init-request (state ~COMPLETED) (order ?order2&:(< ?order2 ?order))))
	;(not (ff-feature ?name))
	=>
	(printout error "Init: feature " ?name " is not available" crlf)
	(modify ?ir (state ERROR) (error-msgs (str-cat "Feature " ?name " is not available")))
)

(defrule executive-init-stage-request-feature-fulfilled
	(executive-init)
	?ir <- (executive-init-request (state FEATURE-REQUESTED) (name ?name))
	(ff-feature-loaded ?name)
	=>
	(printout debug "Init: feature request for " ?name " has been fulfilled" crlf)
	(modify ?ir (state FEATURE-DONE))
)

(defrule executive-init-stage-request-files
	(executive-init)
	?ir <- (executive-init-request (state FEATURE-DONE) (name ?name)
																 (files $?files) (wait-for ?wait-for))
	=>
	(if (> (length$ ?files) 0)
	 then
		(printout t "Init: loading files for " ?name " " ?files crlf)
		(foreach ?f ?files
		    (printout t "Init: " ?f crlf)
			(bind ?pipepos (str-index "|" ?f))
			(bind ?file-op "LOAD")
			(bind ?file-name ?f)
			(if ?pipepos then
				(bind ?file-op (sub-string 1 (- ?pipepos 1) ?f))
				(bind ?file-name (sub-string (+ ?pipepos 1)  (str-length ?f) ?f))
			)
			(switch ?file-op
				(case "BATCH" then  (path-batch* ?file-name))
				(case "BATCH*" then (path-batch* ?file-name))
				(default (path-load ?file-name))
			)
		)
	)
	(if (eq ?wait-for nil)
	then
		(modify ?ir (state COMPLETED))
	else
		(printout t "Init: waiting for signal " ?wait-for " by " ?name crlf)
		(modify ?ir (state WAIT-FOR))
	)
)

(defrule executive-init-stage-wait-for-ok
	(executive-init)
	?ir <- (executive-init-request (state WAIT-FOR) (name ?name)
																 (wait-for ?wait-for))
	?ws <- (executive-init-signal (id ?wait-for) (ok TRUE))
	=>
	(retract ?ws)
	(printout info "Init: signal " ?wait-for " by " ?name " received" crlf)
	(modify ?ir (state COMPLETED))
)

(defrule executive-init-stage-wait-for-fail
	(executive-init)
	?ir <- (executive-init-request (state WAIT-FOR) (name ?name)
																 (wait-for ?wait-for))
	?ws <- (executive-init-signal (id ?wait-for) (ok FALSE) (error-msg ?error-msg))
	=>
	(retract ?ws)
	(modify ?ir (state ERROR) (error-msgs ?error-msg))
)

(defrule executive-init-stage-finished
	(executive-init)
	?sf <- (executive-init-stage)
	(not (executive-init-request (state ~COMPLETED)))
	=>
	(retract ?sf)
	(assert (executive-initialized))
)
