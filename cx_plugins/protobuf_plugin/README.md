# cx_protobuf_plugin
This package offers the `cx::ProtobufPlugin' plugin that allows to communicate via protobuf through [protobuf_comm](https://github.com/fawkesrobotics/protobuf_comm).

## Usage
Register this plugin with the plugin manager.
It's configuration parameters are depicted in this example setup below.

```yaml
clips_manager:
  ros__parameters:
    environments: ["main"]
    main:
      plugins: ["protobuf"]

    protobuf:
      plugin: "cx::ProtobufPlugin"
      # When specifying relative paths, look at the share directories of the listed packages to resolve them.
      # Attempts to resolve the relative paths in order of the listed packages
      # Defaults to an empty list
      pkg_share_dirs: ["cx_bringup"]
      # Specify directories to look for protobuf messages.
      # Supports absolute paths or relative paths using the share directories specified above.
      # Defaults to an empty list
      proto_paths: ["proto"]
```
## CLIPS Features
This plugin defines deftemplates, rules and user-defined functions that are described below.

##### Deftemplates
```lisp
; Asserted by the create-subscription function.
; Retracted by the destroy-subscription function.
(deftemplate protobuf-msg
  (slot type (type STRING))            ; package + "." + message-name
  (slot comp-id (type INTEGER))
  (slot msg-type (type INTEGER))
  (slot rcvd-via (type SYMBOL)
    (allowed-values STREAM BROADCAST)
  )
  (multislot rcvd-from                 ; address and port
    (cardinality 2 2)
  )
  (slot rcvd-at (type FLOAT))          ; ros timestamp in seconds 
  (slot client-type (type SYMBOL)
    (allowed-values SERVER CLIENT PEER)
  )
  (slot client-id (type INTEGER))
  (slot ptr (type EXTERNAL-ADDRESS))
)

(deftemplate protobuf-receive-failed
  (slot comp-id (type INTEGER))
  (slot msg-type (type INTEGER))
  (slot rcvd-via (type SYMBOL)
    (allowed-values STREAM BROADCAST)
  )
  (multislot rcvd-from (cardinality 2 2))
  (slot client-id (type INTEGER))
  (slot message (type STRING))
)

(deftemplate protobuf-server-receive-failed
  (slot comp-id (type INTEGER))
  (slot msg-type (type INTEGER))
  (slot rcvd-via (type SYMBOL)
    (allowed-values STREAM BROADCAST)
  )
  (multislot rcvd-from (cardinality 2 2))
  (slot client-id (type INTEGER))
  (slot message (type STRING))
)
```
##### Defrules
It defines defrules and a defglobal to clean up the fact once the end of the agenda is reached:
```lisp
(defglobal
  ?*PRIORITY-PROTOBUF-RETRACT*    = -10000
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
```
##### Functions
```lisp
(bind ?res (pb-register-type ?full-name))    ; returns TRUE if successful, FALSE otherwise
(bind ?res (pb-field-names ?msg))
(bind ?res (pb-field-type ?msg ?field-name))
(bind ?res (pb-has-field ?msg ?field-name))
(bind ?res (pb-field-label ?msg ?field-name))
(bind ?res (pb-field-value ?msg ?field-name))
(bind ?res (pb-field-list ?msg ?field-name))
(bind ?res (pb-field-is-list ?msg ?field-name))
(bind ?res (pb-create ?full-name))
(bind ?res (pb-ref ?msg))
(pb-set-field ?msg ?field-name ?value)
(pb-add-list ?msg ?field-name ?list)
(pb-send ?client-id ?msg)
(bind ?res (pb-tostring ?msg))
(pb-server-enable ?port)
(pb-server-disable)
(bind ?res (pb-peer-create ?address ?port))
(bind ?res (pb-peer-create-local ?address ?send-port ?recv-port))
(bind ?res (pb-peer-create-crypto ?address ?port ?crypto ?cypher))
(bind ?res (pb-peer-create-local-crypto ?address ?send-port ?recv-port ?crypto ?cypher))
(pb-peer-destroy ?peer-id)
(pb-peer-setup-crypto ?peer-id ?key ?cypher)
(pb-broadcast ?peer-id ?msg)
(bind ?res (pb-connect ?host ?port))
(pb-disconnect ?host ?port)
```

## Object Lifetimes and CLIPS
Since clips stores objects via void pointers, dynamic object lifetime management via `std::shared_ptr` does not work direcly from within CLIPS.
Instead, object lifetimes need to be managed more explicitly through the usage of `create` and `destroy` functions.

It is advised to clean up all objects as soon as they are not needed anymore in order to free up memory.

Note that when processing nested messages, the message obtained via **pb-field-value** is allocating new memory, hence the returned external address also needs to be cleaned up afterwards.