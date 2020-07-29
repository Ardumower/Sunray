ublox f9p (rover) configuration files

NOTE: see Wiki for details on how to transfer a ublox configuration file to your ublox f9p GPS receiver. 
Don't forget to save a sent configuration after transferring (ucenter->View->configuratiion view->CFG->Send).


1) alex_rover_f9p_HPG_111.txt
-default configuration as described in Wiki
-tested in extreme conditions
-large input filter to avoid false-fix positions (if a fix is found it is always correct)


2) Hartmut__rover_f9_HGP_113.txt
-use this configuration if you experience RTK FIX issues 
https://forum.ardumower.de/threads/welche-gps-antennen-nutzt-ihr-wie-sieht-eure-grundplatte-aus-alles-rund-um-den-gps-empfang.23709/post-41762
-reduced NAV5 input filter
-optimized for large trees and bushes
-enabled all GNSS satellites


----------------------------------------------
ublox f9p firmware updates are available here:
https://www.u-blox.com/en/product/zed-f9p-module#tab-documentation-resources


