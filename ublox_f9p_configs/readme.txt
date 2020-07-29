ublox f9p configuration files

1) alex_rover_f9p.txt
-default configuration as described in Wiki
-tested in extreme conditions
-large input filter to avoid false-fix positions (if a fix is found it is always correct)


2) Hartmut__rover_f9_HGP_113.txt
https://forum.ardumower.de/threads/welche-gps-antennen-nutzt-ihr-wie-sieht-eure-grundplatte-aus-alles-rund-um-den-gps-empfang.23709/post-41762
-reduced NAV5 input filter
-optimized for large trees and bushes
-enabled all GNSS satellites
