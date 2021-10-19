(defrule parse-test
  (executive-init)
  =>
  (parse-pddl-domain (path-resolve "../pddl/domain.pddl"))
)