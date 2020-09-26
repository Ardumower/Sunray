/*--------------------------------------------------------------------
This file is part of the Arduino WiFiEsp library.

The Arduino WiFiEsp library is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

The Arduino WiFiEsp library is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with The Arduino WiFiEsp library.  If not, see
<http://www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#ifndef EspDebug_H
#define EspDebug_H

#include <stdio.h>
#include "../../config.h"

// Change _ESPLOGLEVEL_ to set tracing and logging verbosity
// 0: DISABLED: no logging
// 1: ERROR: errors
// 2: WARN: errors and warnings
// 3: INFO: errors, warnings and informational (default)
// 4: DEBUG: errors, warnings, informational and debug

#ifndef _ESPLOGLEVEL_
#define _ESPLOGLEVEL_ 3
#endif


#define LOGERROR(x)    if(_ESPLOGLEVEL_>0) { CONSOLE.print("[WiFiEsp] "); CONSOLE.println(x); }
#define LOGERROR1(x,y) if(_ESPLOGLEVEL_>2) { CONSOLE.print("[WiFiEsp] "); CONSOLE.print(x); CONSOLE.print(" "); CONSOLE.println(y); }
#define LOGWARN(x)     if(_ESPLOGLEVEL_>1) { CONSOLE.print("[WiFiEsp] "); CONSOLE.println(x); }
#define LOGWARN1(x,y)  if(_ESPLOGLEVEL_>2) { CONSOLE.print("[WiFiEsp] "); CONSOLE.print(x); CONSOLE.print(" "); CONSOLE.println(y); }
#define LOGINFO(x)     if(_ESPLOGLEVEL_>2) { CONSOLE.print("[WiFiEsp] "); CONSOLE.println(x); }
#define LOGINFO1(x,y)  if(_ESPLOGLEVEL_>2) { CONSOLE.print("[WiFiEsp] "); CONSOLE.print(x); CONSOLE.print(" "); CONSOLE.println(y); }

#define LOGDEBUG(x)      if(_ESPLOGLEVEL_>3) { CONSOLE.println(x); }
#define LOGDEBUG0(x)     if(_ESPLOGLEVEL_>3) { CONSOLE.print(x); }
#define LOGDEBUG1(x,y)   if(_ESPLOGLEVEL_>3) { CONSOLE.print(x); CONSOLE.print(" "); CONSOLE.println(y); }
#define LOGDEBUG2(x,y,z) if(_ESPLOGLEVEL_>3) { CONSOLE.print(x); CONSOLE.print(" "); CONSOLE.print(y); CONSOLE.print(" "); CONSOLE.println(z); }


#endif
