; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(defrule print-time
  (time ?now)
  =>
  (printout info "time between agenda refresh and rule fire: " (- (now) ?now) crlf)
  (printout green "ROS time: " (now) crlf)
  (printout blue "SYS time: " (now-systime) crlf)
)
