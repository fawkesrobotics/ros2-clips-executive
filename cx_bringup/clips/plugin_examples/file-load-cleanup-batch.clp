; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(printout yellow "cleanup batch" crlf)
(undefrule hello-world)
(undefrule goodbye-world)
(do-for-all-facts ((?h hello))
  (retract ?h)
)
