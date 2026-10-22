# Failure injection

The small always-on-top **Failure Injection** window (opens at the top left on startup) triggers equipment and sensor failures. Each click on a button cycles that target through its modes and back to `ok`. **Clear all failures** resets everything, including failures left in X-Plane.

| Target | Modes | Effect |
|---|---|---|
| Lift 0-3, Pusher | `failed` | Writes the X-Plane engine failure DREF (`rel_engfai0..4`) |
| GPS, IMU, LIDAR | `bias`, `drift`, `freeze`, `dropout` | Corrupts that sensor's position |
| Altimeter | `bias`, `drift`, `freeze`, `dropout` | Corrupts `^altitude` for the agent only |
| Radar alt | `bias`, `drift`, `freeze`, `dropout` | Corrupts `^agl` for the agent only |

Active failures are published to the Soar agent on the input link, all `ok` by default:

```
^io.input-link.flightdata
   ^allEnginesOK   true|false
   ^failures
      ^active-count <int>   ^last-injected <target>|none
      ^engine-0 ... ^engine-4   ok|failed          (0-3 lift, 4 pusher)
      ^gps ^imu ^lidar          ok|bias|drift|freeze|dropout
      ^altimeter                ok|bias|drift|freeze|dropout
      ^radar-altimeter          ok|bias|drift|freeze|dropout
```

Injected failures are not cleared by the sensor-warning timeouts. They stay until cleared or the program exits. Intended for non-learning mode.

## Code

| File | Role |
|---|---|
| `java/src/xplane/FailureInjector.java` | Holds failure state, applies the faults, writes the engine failure DREF |
| `java/src/xplane/FailurePanel.java` | The button window |
| `java/src/agents/CopilotTakeoff/CopilotTakeoffAgent.java` | Publishes `^failures` and applies the altimeter overrides |
| `java/src/xplane/XPCUserInterface.java` | Applies sensor faults to the GPS, IMU and LIDAR positions |
