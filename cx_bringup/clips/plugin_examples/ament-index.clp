; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(deffunction print-pairs (?pairs ?key-str ?value-str)
  (bind ?length (length$ ?pairs))
   (bind ?index 1)

   (while (< ?index ?length)
      (bind ?key (nth$ ?index ?pairs))
      (bind ?value (nth$ (+ ?index 1) ?pairs))
      (printout green "    " ?key-str ": " ?key " "?value-str ": " ?value crlf)
      (bind ?index (+ ?index 2))
   )
)
(deffunction print-in-lines (?multi)
  (bind ?length (length$ ?multi))
   (bind ?index 1)

   (while (< ?index ?length)
      (bind ?key (nth$ ?index ?multi))
      (printout green "    " ?key crlf)
      (bind ?index (+ ?index 1))
   )
)
(printout info "Package prefix of cx_bringup: " crlf)
(printout green "    " (ament-index-get-package-prefix "cx_bringup") crlf)
(printout info "Packages with prefixes: " crlf)
(print-pairs (ament-index-get-packages-with-prefixes) "Package" "Prefix")

(printout info "Package share dir of rclcpp: " crlf)
(printout green "    " (ament-index-get-package-share-directory "rclcpp") crlf)

(printout info "Resource vendor packages clips_vendor: " crlf)
(print-pairs (ament-index-get-resource "vendor_packages" "clips_vendor") "Content" "Path")

(printout info "Get resources for type vendor_packages" crlf)
(print-pairs (ament-index-get-resources "vendor_packages") "Package" "Prefix")

(printout info "Get search paths" crlf)
(print-in-lines (ament-index-get-search-paths))
