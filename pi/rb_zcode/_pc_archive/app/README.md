<!-- inkspace:view {"font":"serif"} -->
# LeArm ID1 diagnostic files

These files diagnose the original LeArm BUS servo on ID1 after the claw body
has been replaced. They do not require a firmware flash.

## Before any test

1. Keep the arm fixed and keep hands clear of the claw.
2. Close every LeArm V2.1 window, the phone app, UartAssist, and every other
   program that could use the LeArm COM port before using the PowerShell probe.
3. Power-cycle the control board, wait for its automatic reset to finish, then
   press K1 once. Continue only after the board gives two short beeps for PC
   mode.
4. Stop and remove servo power if the claw hits a stop, chatters, or becomes
   hot.
5. The probe intentionally has no offset-read command while diagnosing this
   arm. The factory firmware reads all six offsets without a timeout; a
   missing response can leave the board unable to answer later USB requests
   until it is power-cycled.

## Re-phase the replacement claw before motion

The original LeArm BUS ID1 servo remains the actuator. The replacement Black B
claw body is a gear-and-linkage mechanism designed around a different PWM
servo. Its PWM values are not used here, but its mechanics establish the
meaning of the poses: the original claw opens at its low-side command and
closes at its high-side command.

The factory LeArm code also uses lower ID1 positions for opening and higher
positions for closing. It always commands `P226` during boot, so `P226` must
be a safe *near-open* position after the new gear/linkage is assembled. It is
not automatically the Black B claw's 13-14 mm center gap.

1. With servo power off, separate the driven gear or linkage from ID1 so the
   original servo can rotate without loading the claw.
2. Power-cycle, wait for boot reset, press K1 once for PC mode, then command
   ID1 `226`. Cut servo power without twisting the output shaft.
3. Reinstall the new claw gear/linkage so the jaws are visibly short of their
   widest-open hard stop at that position. Do not assemble at a hard stop.
4. Reconnect and test `226 -> 236 -> 226`. The jaws should begin to close at
   `236`. If they open instead, shifting a gear tooth cannot reverse the
   direction: the gear train must be mechanically reversed or a later
   firmware mapping must be added.
5. Only after that, measure a separate `P_neutral` where the jaw-tip gap is
   13-14 mm, then measure safe open, grasp, and close positions.

### Position estimate for the Black B mechanism

The bus protocol states that `P=0..1000` represents `0..240 degrees`, or
`0.24 degrees/P`. The Black B claw documentation states that its mechanism
uses `0 degrees` for maximum opening, `90 degrees` for the 13-14 mm center
gap, and `108 degrees` for the closed reference. If the new gear is installed
so that factory reset `P226` is the safe near-open reference, the *initial
estimates* are:

| Mechanical state | Estimate | Calculation |
| --- | ---: | --- |
| Near-open reset | `P226` | factory BUS reset value |
| 13-14 mm center gap | `P601` | `226 + 90 / 0.24` |
| Closed reference | `P676` | `226 + 108 / 0.24` |

These are geometry estimates, not measured calibration values. They assume a
1:1 gear ratio, no stored ID1 deviation, and the same direction as the source
code. The claw documents do not provide the gear tooth counts or a tooth-zero
mark, so the real values must be measured after assembly. Do not send `P601`
or `P676` until the unloaded `226 -> 236 -> 226` test succeeds, the gear
direction is confirmed (`P` increasing must close), and the new claw has been
reinstalled with clearance from both hard stops. Then approach the estimated
center and close values in 10-unit steps, recording the first safe contact and
backing off before any sustained stall.

## GUI test first

Open `LeArm V2.1.exe`, select the CH341 COM port, wait for the connection icon
to become green, and use slider-control mode. A green icon only confirms that
the COM port is open; the board must also have been switched to PC mode with
K1 (two short beeps) after power-up.

The slider is the live-control path. Set ID1 to `226`, then `236`, observe the
output gear, and return to `226`. Do not use `Add action`, `Download`, or
`Action group run` for this first observation; those controls belong to the
separate stored-action workflow.

`id1_safe_motion_test.xml` is an offline record of the same three targets:

| Frame | ID1 | ID2 | ID3 | ID4 | ID5 | ID6 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 226 | 500 | 177 | 129 | 408 | 500 |
| 2 | 236 | 500 | 177 | 129 | 408 | 500 |
| 3 | 226 | 500 | 177 | 129 | 408 | 500 |

The factory BUS firmware ignores the GUI action-time field for direct/online
commands and sends each servo position using a roughly 20 ms movement time.
The `T1500` values in this XML therefore do not guarantee slow motion. Use the
individual slider values above for observation, and stop immediately if the
claw hits a stop or chatters.

The GUI slider follows `CMD3 -> robot_arm_knot_run()` in the factory source.
The separate source helper `robot_arm_claw_set()` is not on this path, so
changing its angle formula would not change LeArm V2.1 slider behavior. A
firmware mapping change, if eventually required after mechanical validation,
must be made at the `robot_arm_knot_run()` ID1 branch and must include tested
minimum and maximum limits.

## Serial probe if the GUI test does not move ID1

Close `LeArm V2.1.exe` before opening the serial port with PowerShell.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\id1_protocol_probe.ps1 -ListPorts
powershell -NoProfile -ExecutionPolicy Bypass -File .\id1_protocol_probe.ps1 -Port COMx
```

Replace `COMx` with the port shown by the first command or in the LeArm GUI.
The second command is read-only: it queries the source-compatible firmware
version and BUS/PWM type. It deliberately does not ask every servo for its
position, because the factory firmware can wait indefinitely if any one bus
servo fails to reply.

To request the six positions after the version check succeeds, run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\id1_protocol_probe.ps1 -Port COMx -ReadPositions
```

The script permits position and movement tests only after the board reports
factory BUS firmware: `servo type: BUS` and `Firmware version: 1`.

If the position query times out, capture the raw read-only response instead:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\id1_protocol_probe.ps1 -Port COMx -RawPosition
```

For a first physical check, use only the explicit, narrow target range around
the factory reset value after the claw has been re-phased:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\id1_protocol_probe.ps1 -Port COMx -MovePosition 226
powershell -NoProfile -ExecutionPolicy Bypass -File .\id1_protocol_probe.ps1 -Port COMx -MovePosition 236
powershell -NoProfile -ExecutionPolicy Bypass -File .\id1_protocol_probe.ps1 -Port COMx -MovePosition 226
```

`-MovePosition` accepts only `226..246`, moves ID1 only, and does not save any
servo setting. The direct factory BUS path can execute in about 20 ms even
though the packet carries 1500 ms. Never widen the range until the new claw
has been re-phased and its hard stops are measured. The raw position response
is returned as raw bytes because the extracted STM32 source casts the two
position bytes to `int16_t`, while the C51 examples treat the same field as
unsigned. A negative signed rendering is therefore a protocol/servo-coordinate
diagnostic, not proof that the output shaft has crossed a physical zero or hit
a stop. Do not convert it to a motion target; first repeat the test with ID1
unloaded and check whether the raw bytes follow `226 -> 236`.

Offset reads are intentionally not exposed by this probe. Although an offset
read does not write a setting, the factory command can wait forever for a
failed servo reply. If an external tool has already issued it and the version
query no longer answers, power-cycle the controller before continuing.

## Interpreting the result

- Gear and claw move: the electrical control path works; record the safe open,
  grasp, and closed positions for the new claw body.
- ID1 raw feedback is negative or does not follow `226 -> 236`: stop and test
  ID1 unloaded; then inspect the servo coordinate/mode/limit or capture the
  inner 115200-baud bus frame. Do not infer a mechanical stop from the signed
  value alone.
- Gear moves but claw does not: check gear-to-linkage engagement or a claw hard
  stop; do not change STM32 firmware first.
- Neither gear nor claw moves, while the probe reports a BUS firmware: the PC
  mode/COM command path is the remaining issue.
- The probe cannot receive a version reply: verify K1 PC mode, the selected
  COM port, and that no other program owns the port.
- Firmware version `2` with BUS type: this repository's competition firmware
  expects its heartbeat and RESET protocol, so the vendor GUI is not the
  correct controller for that firmware. Do not flash anything until the
  current firmware and the intended control path have been confirmed.
