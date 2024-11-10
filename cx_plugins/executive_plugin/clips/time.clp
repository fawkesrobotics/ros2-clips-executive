
; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(defglobal
  ?*PRIORITY-TIME-RETRACT*    = -10000
)

(defrule time-retract
  (declare (salience ?*PRIORITY-TIME-RETRACT*))
  ?f <- (time $?)
  =>
  (retract ?f)
)

(unwatch facts time)
(unwatch rules time-retract)
