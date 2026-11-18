# Config2 build and flash

Canonical configuration: config/config2.json. CMake compiles enabled limits,
reference modes and encoder offsets into a read-only Flash table.
machines.json is historical. Qt reads the same Config2 file and preserves
coordinate metadata when saving. Rebuild/reflash after changing permanent values.

From the workspace root:

```powershell
$CMakeExe = "$env:LOCALAPPDATA\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe"
foreach ($preset in @(
 'excavator-controlled-servo-debug','excavator-controlled-servo-release',
 'excavator-conctrler-servo-debug','excavator-conctrler-servo-release',
 'f103c8-chip-debug','f103c8-chip-release'
)) {
 & $CMakeExe --preset $preset
 & $CMakeExe --build --preset $preset
}
```

Output: build/<preset>/excavator_controlled_servo.{elf,hex,bin} and
excavator_conctrler_servo.{elf,hex,bin}. Update both endpoints: radio v2 is15
bytes and rejects v1. Controller title: LUXI CONFIG2.

Disconnect motor power for ordinary flashing. MCU reset does not clear
retained servo targets. Bring-up must start neutral with supported joints,
lifted tracks and safe rotation/cables. Startup first reads feedback. Since 2026-10-21, neutral/stable startup may prepare
unpowered servos into the proper modes and latch fresh current holds; it never
sends an endpoint/old center or unloads a powered wrong-mode axis. PREPARE_REMOTE explicitly verifies torque off, stable mode and
feedback, reference-position invariance and a fresh hold goal before enabling
torque. No ID, center or encoder-offset write is performed.
Read FLASH_LIMITS to verify Config2; reset must preserve limits.

Optional host check after receiver Debug configure:

```powershell
./Projects/excavator_controlled_servo/tests/run_host_tests.ps1 -CompileOnly
```

CompileOnly does not run binaries. Execute only where local policy allows;
never change execution policy to bypass a rejection. The suite includes
previous-stage safeguards, Config2 remote boundaries/ownership/recovery,
delayed coordinate changes and UART driver restrictions with simulated IO.

Projects/Qt_centers/scripts/build.cmd builds/tests and deploys QtCenters.exe,
DLLs, plugins and config2.json to Projects/Qt_centers for double-click use.
Hardware serial smoke runs separately and uses no position preparation/move;
it may temporarily narrow ID1 RAM limits and pauses radio via stops.
Use RESUME_REMOTE afterward when neutral and modes are ready.
See config2_control.md and hardware_test_2026-10-15.md. The latest hardware-verified
package is build/config2-normal-speed-2026-10-15/; it supersedes the October 14 package.
