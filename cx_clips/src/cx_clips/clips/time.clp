
;---------------------------------------------------------------------------
;  time.clp - time utils
;
;  Created: Sat Jun 16 15:45:57 2012 (Mexico City)
;  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
;             2011       SRI International
;             2011       RWTH Aachen University (KBSG)
;             2011       Carnegie Mellon University
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

;(defmodule TIME-UTILS)

(defglobal
  ?*PRIORITY-TIME-RETRACT*    = -10000
)

; This assumes Fawkes-style time, i.e. sec and usec

(deffunction time-diff (?t1 ?t2)
  (return (- ?t1 ?t2))
)

(deffunction time-diff-sec (?t1 ?t2)
  (bind ?td (time-diff ?t1 ?t2))
  (return ?td)
)

(deffunction timeout (?now ?time ?timeout)
  (return (> (time-diff-sec ?now ?time) ?timeout))
)

(deffunction time> (?t1 ?t2)
	(bind ?rv FALSE)
  (if (> ?t1 ?t2) then (bind ?rv TRUE))
	(return ?rv)
)

(deffunction time-trunc-ms (?time)
	(bind ?rv ?time)
	(if (= (length$ ?time) 2) then
		(bind ?rv (create$ (nth$ 1 ?time) (* (div (nth$ 2 ?time) 1000) 1000)))
	)
	(return ?rv)
)

; Timer deftemplate, to be used with the timeout function.
; An example for a periodically triggered rule (assuming that you do have
; periodic assertion of a (time (now)) fact at suitable intervals) could be:
; @code
; (defrule timer-trigger
;   (time $?now)
;   ?tf <- (timer (name my-timer) (time $?t&:(timeout ?now ?t ?*TIMER-PERIOD*)) (seq ?seq))
;   =>
;   (modify ?tf (time ?now) (seq (+ ?seq 1)))
; )
; @endcode
; This triggers the rule every ?*TIMER-PERIOD* seconds and updates the
; last trigger time and the sequence number (which you can also skip if you
; never need it yourself.
(deftemplate timer
  (slot name)
  (slot time (type FLOAT))
  (slot seq (type INTEGER) (default 1))
)


; --- RULES - general housekeeping
(defrule time-retract
  (declare (salience ?*PRIORITY-TIME-RETRACT*))
  ?f <- (time $?)
  =>
  (retract ?f)
)
