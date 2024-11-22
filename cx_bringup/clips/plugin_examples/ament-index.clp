; Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

(deffunction print-pairs (?pairs ?key-str ?value-str)
  (bind ?length (length$ ?pairs))
  (bind ?index 1)

  (while (< ?index (min 10 ?length))
     (bind ?key (nth$ ?index ?pairs))
     (bind ?value (nth$ (+ ?index 1) ?pairs))
     (printout green "    " ?key-str ": " ?key " "?value-str ": " ?value crlf)
     (bind ?index (+ ?index 2))
  )
  (if (> ?length 10) then
   (printout green "    " " ... " crlf)
  )
)
(deffunction print-in-lines (?multi)
  (bind ?length (length$ ?multi))
  (bind ?index 1)

  (while (< ?index (min 5 ?length))
     (bind ?key (nth$ ?index ?multi))
     (printout green "    " ?key crlf)
     (bind ?index (+ ?index 1))
  )
  (if (> ?length 5) then
   (printout green "    " " ... " crlf)
  )
)

(printout info "(ament-index-get-package-prefix \"cx_bringup\")" crlf)
(printout green "    " (ament-index-get-package-prefix "cx_bringup") crlf)

(printout info "(ament-index-get-packages-with-prefixes)" crlf)
(print-pairs (ament-index-get-packages-with-prefixes) "Package" "Prefix")

(printout info "(ament-index-get-package-share-directory \"rclcpp\")" crlf)
(printout green "    " (ament-index-get-package-share-directory "rclcpp") crlf)

(printout info "(ament-index-get-resource \"vendor_packages\" \"clips_vendor\")" crlf)
(print-pairs (ament-index-get-resource "vendor_packages" "clips_vendor") "Content" "Path")

(printout info "(ament-index-get-resources \"vendor_packages\")" crlf)
(print-pairs (ament-index-get-resources "vendor_packages") "Package" "Prefix")

(printout info "(ament-index-get-search-paths)" crlf)
(print-in-lines (ament-index-get-search-paths))
