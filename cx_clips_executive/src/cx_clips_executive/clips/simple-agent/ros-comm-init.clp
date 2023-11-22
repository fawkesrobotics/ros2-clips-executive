;---------------------------------------------------------------------------
;  ros-comm-init.clp - Initialize ROS2 based communication
;
;  Created: Wed 22 Nov 2023 12:04:00 CET
;  Copyright  2023 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;

(defrule ros-comm-subscribe
    (executive-init)
    =>
    (ros-default-subscribe "test")
)

(defrule ros-comm-publish
    (executive-init)
    (ros-default-message (topic "test"))
    =>
    (ros-default-create-publisher "/test2")
    (ros-default-create-message "/test2")
    (ros-default-set-field-publish "/test2" "data" "TEST")
    (ros-default-publish "/test2")
)

(defrule ros-comm-request
    (ros-default-message (topic "test"))
    =>
    (ros-default-create-request "testservice")
    (ros-default-set-field-request "testservice" "data" "TRUE")
    (ros-default-request "testservice")
)

(defrule ros-comm-print-response
    (ros-default-response (service "testservice") (success ?succ))
    =>
    (printout t ?succ crlf)
)
