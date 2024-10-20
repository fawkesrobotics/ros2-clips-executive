; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(foreach ?r (get-defrule-list)
  (if (eq (str-index "ros-msgs-" ?r) 0) then (undefrule ?r))
)
