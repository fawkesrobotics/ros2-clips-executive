
; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

;---------------------------------------------------------------------------
;  protobuf.clp - protobuf message templates
;
;  Created: Fri Feb 08 15:42:52 2013
;  Copyright  2013  Tim Niemueller [www.niemueller.de]
;  Licensed under BSD license, cf. LICENSE file
;---------------------------------------------------------------------------
(defglobal
  ?*PRIORITY-PROTOBUF-RETRACT*    = -10000
)

(deftemplate protobuf-msg
  (slot type (type STRING))
  (slot comp-id (type INTEGER))
  (slot msg-type (type INTEGER))
  (slot rcvd-via (type SYMBOL) (allowed-values STREAM BROADCAST))
  (multislot rcvd-from (cardinality 2 2))
  (slot rcvd-at (type FLOAT))
  (slot client-type (type SYMBOL) (allowed-values SERVER CLIENT PEER))
  (slot client-id (type INTEGER))
  (slot ptr (type EXTERNAL-ADDRESS))
)

(deftemplate protobuf-receive-failed
  (slot comp-id (type INTEGER))
  (slot msg-type (type INTEGER))
  (slot rcvd-via (type SYMBOL) (allowed-values STREAM BROADCAST))
  (multislot rcvd-from (cardinality 2 2))
  (slot client-id (type INTEGER))
  (slot message (type STRING))
)

(deftemplate protobuf-server-receive-failed
  (slot comp-id (type INTEGER))
  (slot msg-type (type INTEGER))
  (slot rcvd-via (type SYMBOL) (allowed-values STREAM BROADCAST))
  (multislot rcvd-from (cardinality 2 2))
  (slot client-id (type INTEGER))
  (slot message (type STRING))
)

(defrule protobuf-cleanup-receive-failed
  (declare (salience ?*PRIORITY-PROTOBUF-RETRACT*))
  ?f <- (protobuf-receive-failed (comp-id ?cid) (msg-type ?mt)
    (rcvd-from ?host ?port) (message ?msg))
  =>
  (retract ?f)
  (printout t "Protobuf rcv fail for " ?cid ":" ?mt " from " ?host ":" ?port ": " ?msg crlf)
)

(defrule protobuf-cleanup-server-receive-failed
  (declare (salience ?*PRIORITY-PROTOBUF-RETRACT*))
  ?f <- (protobuf-server-receive-failed (comp-id ?cid) (msg-type ?mt)
    (rcvd-from ?host ?port) (message ?msg))
  =>
  (retract ?f)
  (printout t "Protobuf server rcv fail for " ?cid ":" ?mt " from " ?host ":" ?port ": " ?msg crlf)
)

(defrule protobuf-cleanup-message
  (declare (salience ?*PRIORITY-PROTOBUF-RETRACT*))
  ?pf <- (protobuf-msg (ptr ?p))
  =>
  (pb-destroy ?p)
  (retract ?pf)
)
