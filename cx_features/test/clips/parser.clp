; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(defrule parse-test
  (executive-init)
  =>
  (parse-pddl-domain (path-resolve "../pddl/domain.pddl"))
)
