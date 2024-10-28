; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(defrule spam-print-time
  (time ?now)
  =>
  (printout info "time between agenda refresh and rule fire: " (- (now) ?now) crlf)
  (printout info "ROS time: " (now) crlf)
  (printout info "sys time: " (now-systime) crlf)
)
