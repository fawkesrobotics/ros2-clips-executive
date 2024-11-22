; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(defrule hello-world
   (not (hello))
   =>
   (printout green "Hello world" crlf)
   (assert (hello))
)
(defrule goodbye-world
   (executive-finalize)
   =>
   (printout blue "Goodbye world" crlf)
)
