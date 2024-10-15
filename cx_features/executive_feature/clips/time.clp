
; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

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

; (deffunction time-diff (?t1 ?t2)
;   (return (- ?t1 ?t2))
; )
;
; (deffunction timeout (?now ?time ?timeout)
;   (return (> (time-diff ?now ?time) ?timeout))
; )

; (deftemplate timer
;   (slot name)
;   (slot time (type FLOAT))
;   (slot seq (type INTEGER) (default 1))
; )


(defrule time-retract
  (declare (salience ?*PRIORITY-TIME-RETRACT*))
  ?f <- (time $?)
  =>
  (retract ?f)
)
