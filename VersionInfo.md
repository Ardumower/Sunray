### This Version is a fork from the sunray release version 1.0.267 with the following added options:
- **Set error if bumper stays permanently triggered:**
set bumper error in case of continious triggering (time can be adjusted in config.h "BUMPER_MAX_TRIGGER_TIME".
- **map setSpeed as maximum speed for navigation by joystick from sunray-app**
It is possible to navigate the mower by touch-joystick in sunray-app.
In some cases it could be neccessary to navigate the mower very soften, especially when your connected by wifi to the mower.
If parameter is set to true, the speed value from app will be used for maximum speed by joystick control. To navigate soften, change the speed slider for example to 0.10. If you need to let the mower drive long distance without accurate positioning change the speed slider to 0.30.
- **Adjustable speed and time values in config.h from MrTreeBark**
see "MOW_SPINUPTIME; OVERLOADSPEED; ROTATETOTARGETSPEED; TRACKSLOWSPEED; APPROACHWAYPOINTSPEED; FLOATSPEED; SONARSPEED; DOCKANGULARSPEED; OBSTACLEAVOIDANCESPEED; MOTOR_MAX_SPEED; MOTOR_MIN_SPEED;
- **Map Stanley Control parameters to actual linearspeedset from motor.setlinearangularspeed from MrTreeBark**

- **Reboot GPS at a specific docking point**
  - Bei Fahrt zur Docking-Station wird über den Parameter Wert von "DOCK_SLOW_ONLY_LAST_POINTS" die Position des Dockingpunktes angegeben (betrachtet aus Richtung Dockingstation), ab welchem mit langsamer Geschwindigkeit (linear = 0,1) weiter gefahren wird. Alle Dockingpunkte vorher werden mit der normalen (setspeed) Geschwindigkeit angefahren. Ein Wert von "Null" bewirkt, dass alle Punkte mit langsamer Geschwindigkeit angefahren werden.
  - Bei Fahrt aus der Docking-Station wird über den Parameter Wert von "DOCK_POINT_GPS_REBOOT" die Position des Dockingpunktes angegeben (betrachtet aus Richtung Dockingstation), ab welchem ein GPS-Reboot durchgeführt werden soll. Der Mäher wartet dann, bis ein GPS-Fix vorhanden ist, und setzt das undocking fort. Da der Mäher dann eine korrekte Position hat, wird der Rest der Dockingstrecke vorwärts gerichtet, mit normaler Geschwindigkeit und GPS-Unterstützung fortgesetzt. Bei aktiviertem "DOCK_IGNORE_GPS" wird nur bis zum GPS-Reset Punkt ohne GPS-Unterstützung gefahren. Ein Wert von "Null" bewirkt keinen GPS-Reboot beim undocking.
  - Der Dockingpunkt für den GPS-Reboot sollte so gewählt werden, dass dieser sich an einer Stelle befindet, wo der Mäher einfach einen guten GPS-FIX bekommen kann.

  - **Detailierter Ablauf bei Verwendung von "DOCK_POINT_GPS_REBOOT" in Kombination mit "DOCK_IGNORE_GPS" = true:**
    - Mäher fährt rückwärts mit langsamer Geschwindigkeit und nur mit IMU/ODO bis zu dem bei "DOCK_POINT_GPS_REBOOT" eingestelltem Dockingpoint.
    - Dort angekommen wird ein GPS-Reset durchgeführt und der Mäher stoppt.
    - Während auf ein GPS-FIX gewartet wird, ertönt alle 5 Sek. ein kurzer Doppelton durch den Buzzer als akustisches Feedback.
    - Ist ein GPS-FIX vorhanden, muss dieses für mindestens 20 Sek. stabil bleiben. Ein einfacher Ton wird alle 5 Sek. abgespielt. Schwankt die GPS-Position zu stark, wird die Wartezeit wieder resetet und ein Doppelton mit längerer Impulsdauer wird abgespielt.
    - Ist die GPS-Position für mehr als 20 Sek. stabil, wird der Undockingprozess fortgesetzt. Da eine stabile Position besteht, erfolgt die weitere Fahrt vorwärts gerichtet, mit normaler Geschwindigkeit (setSpeed) und mit GPS-Unterstützung.

  Wer kein akustisches Feedback möchte, kann alle Zeilen die mit "if (!buzzer.isPlaying())" beginnen einfach auskommentieren.
  Zur Zeit sind auch noch Consolen-Ausgaben vorhanden, um eine bessere Kontrolle der Funktion beim testen zu haben.
