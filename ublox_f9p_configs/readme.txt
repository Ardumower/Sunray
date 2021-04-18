


--------------------------------------------------------------------------------------

WARNING!  DO NOT APPLY ANY CONFIG FILES TO YOUR BASE/ROVER MODULES!
Sunray firmware will automatically configure your rover module. Only apply the
supplied rover config files if you absolutely need to! Do not apply rover config 
files to your base (it will configure your base as rover...). 

--------------------------------------------------------------------------------------

A) ublox f9p (rover) configuration files


NOTE: see Wiki ( https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Rover_configuration_.28messages.29 ) 
 for details on how to transfer a ublox configuration file to your ublox f9p GPS receiver. 
 
 Don't forget to permanently save a configuration after transferring (ucenter->View->configuratiion view->CFG->Send) 
-----------------------------------------------
 
1) alex_rover_f9p_HPG_113.txt (for firmware 1.13)
-default configuration as described in Wiki
-tested in extreme conditions
-large input filter to avoid false-fix positions (if a fix is found it is always correct)

2) Hartmut__rover_f9_HGP_113.txt (for firmware 1.13)
-use this configuration if you experience RTK FIX issues 
https://forum.ardumower.de/threads/welche-gps-antennen-nutzt-ihr-wie-sieht-eure-grundplatte-aus-alles-rund-um-den-gps-empfang.23709/post-41762
-reduced NAV5 input filter

----------------------------------------------

B) ublox f9p firmware updates are available here:

https://www.u-blox.com/en/product/zed-f9p-module#tab-documentation-resources

After updating, you will need to tansfer (and save) the configuration again.


