; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(defrule load-example-pddl
  (not (domain-loaded))
  =>
  (assert (domain-loaded))
  (bind ?share-dir (ament-index-get-package-share-directory "cx_bringup"))
  (bind ?file (str-cat ?share-dir "/clips/plugin_examples/domain.pddl"))
  (parse-pddl-domain ?file)
)
