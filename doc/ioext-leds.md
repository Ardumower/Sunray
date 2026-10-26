# IO extension LED control

IO board project: `owlrobotics/FollowMe_Ben_Rover1_IOextBoard_04`.
96 WS2812 pixels on RP2040 GPIO 4. Boot: rear red/front white at 20% for
5 seconds, then off. A valid CAN command cancels the boot test immediately.
Normal default and diagnostic brightness: 5%. Brightness is a ceiling for
all scenes, including the full-charge marker. Charging fill is additionally
dimmed (16/255 background, 64/255 moving head); full is the top pixel green.
The board renders at 25 Hz; blinking is 500 ms on / 500 ms off. No ACKs.

## Physical layout needs verification

Four provisional 24-pixel segments: rear left, rear right, front right, front
left. Order, direction and indicator pixels are in `LedLayout.h`.
Observed on 2026-10-26: physical segment 3 is front left; its local pixels
20..23 illuminate the upper INNER corner. The requested indicator position is
always upper OUTER. For initial operation the user approved the assumption
that local pixels 0..3 are upper outer on all four lamps. This has NOT been
visually verified; the other segment positions/directions remain provisional.
The interrupted four-pixel diagnostic yielded no location confirmation.
Mapping may be changed without changing the protocol or Sunray.

`IOEXT_LIGHTS_ENABLE` is enabled in Sunray config_xlmower.h for initial operation
with this user-approved provisional mapping. Other configurations default to
false. This requires deploying both the controller configuration and IO firmware;
a source commit alone does not change the running robot.
Use `IOEXT_LIGHTS_BRIGHTNESS` (0..100, default 5),
`IOEXT_LIGHTS_CHARGE_SELECTION` (1 front, 2 rear, 3 both) and
`IOEXT_LIGHTS_EMPTY_VOLTAGE` (default GO_HOME_VOLTAGE) as needed.

Sunray uses station/charger state first, then the dock path, then turn direction.
On MOW undocking or DOCK docking paths: hazards. On other routes: left or right
indicators when |linear| < 0.03 m/s and |angular| > 0.06 rad/s for 250 ms.
Positive angular velocity means left. Indicators stop immediately when leaving
that condition. Active MOW/DOCK uses front white and rear red; otherwise off,
unless manually moving. Charging replaces all driving lights and indicators.
Only battery.chargingHasCompleted() selects the full-charge marker. Until then
voltage is linearly mapped from the empty display voltage to batFullVoltage,
clamped to 0..99%. This is an approximation, NOT measured battery SOC and does
not change any charging or undervoltage protection.

Sunray sends a complete state on change and once per second for recovery after
an IO-board restart. Repeated commands do not reset animation phase. Without
further commands, the board retains the last scene (there is no CAN timeout).
Do not run manual LED tests with automatic Sunray lighting enabled: its next
snapshot overwrites the manual command. Boot tests may be ended early by Sunray.

## Wire format

Standard CAN ID **700 (0x2BC)**, destination node **1**, usual source **60**.
8-byte frame: LE16 address `source | (dest << 6)`, SET=2, command, 4 payload bytes.
Reserved address bits and unused payload bytes must be zero. No broadcasts.
Commands 16..18 remain reserved for the existing firmware-update protocol.
`IoLightsProtocol.h` is identical in Sunray and the IO-board project.

| Command/value | Payload bytes 0..3 | Meaning |
|---|---|---|
| 32 State | flags, brightness, percent, selection | Atomic complete scene |
| 33 Brightness | 0..100, 0, 0, 0 | Global ceiling |
| 34 Front | 0/1, 0, 0, 0 | White front; switches to road mode |
| 35 Rear | 0/1, 0, 0, 0 | Red rear; switches to road mode |
| 36 Indicators | 0..3, 0, 0, 0 | Off/left/right/hazards; road mode |
| 37 Charging | 0..100, selection, 0, 0 | Charge animation; 100 selects full |
| 38 Full | selection, 0, 0, 0 | Top pixel bright green |
| 39 Off | 0, 0, 0, 0 | Clear all scenes and indicators, retain brightness |
| 40 Test | physical segment 0..3, 0, 0, 0 | One physical segment for 30 seconds |

Flags: bits 0..2 mode (0 off, 1 road, 2 charge, 3 full, 4 test), bit 3 front,
bit 4 rear, bits 5..6 blink (0 off, 1 left, 2 right, 3 hazards); bit 7 zero.
Selection: 1 front, 2 rear, 3 both, or physical segment 0..3 for test mode.
The test marks its first physical pixel red, last green, middle dim white,
independently of the provisional logical layout. At 30 seconds it goes dark;
switch to Off before repeating the same segment to restart the test timer.

On the Pi, using the board project's tools directory:

```sh
python3 tools/led_control.py brightness 5
python3 tools/led_control.py test 0
python3 tools/led_control.py off
python3 tools/led_control.py test 1
python3 tools/led_control.py charge 50 --selection 3
python3 tools/led_control.py off
```

Host tests: `sh linux/tests/run-io-lights-tests.sh` in Sunray and
`sh tests/run-led-tests.sh` in the IO-board project. Firmware build/update
instructions remain in the board's `README-CAN.txt`.

For the combined Sunray-to-board rendering test, set `IOEXT_PROJECT` to the
IO-board project directory when running `run-io-lights-tests.sh`. This also
verifies that the two wire-protocol headers match byte for byte.
