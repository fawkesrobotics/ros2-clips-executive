
; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(defrule protobuf-init-example-peer
  (not (peer ?any-peer-id))
  =>
  (bind ?peer-1 (pb-peer-create-local 127.0.0.1 4444 4445))
  (bind ?peer-2 (pb-peer-create-local 127.0.0.1 4445 4444))
  (assert (peer ?peer-1))
  (assert (peer ?peer-2))
)

(defrule peer-send-msg
  (peer ?peer-id)
  (not (protobuf-msg))
  =>
  (bind ?msg (pb-create "SearchRequest"))
  (pb-set-field ?msg "query" "hello")
  (pb-set-field ?msg "page_number" ?peer-id)
  (pb-set-field ?msg "results_per_page" ?peer-id)
  (pb-broadcast ?peer-id ?msg)
  (pb-destroy ?msg)
)

(defrule protobuf-msg-read
  (protobuf-msg (type ?type) (comp-id ?comp-id) (msg-type ?msg-type)
    (rcvd-via ?via) (rcvd-from ?address ?port) (rcvd-at ?rcvd-at)
    (client-type ?c-type) (client-id ?c-id) (ptr ?ptr))
  =>
  (printout blue ?c-id "("?c-type") received" ?type
    " (" ?comp-id " " ?msg-type ") from " ?address ":" ?port "
    " (- (now)  ?rcvd-at) "s ago" crlf
  )
  (bind ?var (pb-tostring ?ptr))
  (printout yellow ?var crlf)
)


(defrule protobuf-close-peer
  (executive-finalize)
  ?f <- (peer ?any-peer-id)
  =>
  (pb-peer-destroy ?any-peer-id)
  (retract ?f)
)
