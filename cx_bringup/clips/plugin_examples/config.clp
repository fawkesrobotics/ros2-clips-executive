; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(defrule load-bringup-config
  (not (confval))
  =>
  (bind ?share-dir (ament-index-get-package-share-directory "cx_bringup"))
  (bind ?file (str-cat ?share-dir "/params/plugin_examples/config.yaml"))
  (printout green "Loading yaml file: " ?file crlf)
  (config-load ?file "/")
  (delayed-do-for-all-facts ((?cv confval))
    (ppfact ?cv blue)
  )
)
