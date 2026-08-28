# AHMIIAS

**Assured Human-Machine Interface for Increasingly Autonomous Systems** — a NASA project.

A Soar cognitive agent flies an aircraft in X-Plane. Java sits in the middle, passing sensor data to the agent and flight commands back to the simulator.

```
X-Plane  ⇄  Java (CCAT)  ⇄  Soar agent
   ↑            ↓
 aircraft   QGroundControl ( via MAVLink)
```

---

## What you need

| | Version / Notes |
|---|---|
| **Java JDK** | 8 or newer |
| **Maven** | any recent version |
| **X-Plane** | 11 (AW609) or 12 (Beta ALIA-250) |
| **XPlaneConnect plugin** | NASA's [XPC plugin](https://github.com/nasa/XPlaneConnect) — copy into `X-Plane/Resources/plugins/` |
| **IDE** | IntelliJ recommended for running; VS Code fine for editing |

> The JSoar libraries, FreeTTS and XPlaneConnect JARs ship with the repo — Maven pulls the rest.

---

## Run it in 5 steps

**1. Clone and build**

```bash
git clone <repo-url>
cd AHMIIAS/java
mvn clean compile
```

**2. Start X-Plane** and load your aircraft at an airport. Leave it sitting on the runway.

Confirm the XPC plugin loaded: X-Plane menu → *Plugins* should list **X-Plane Connect**.

**3. Point the code at your Soar agent**

Open [CopilotTakeoffAgent.java:182](java/src/agents/CopilotTakeoff/CopilotTakeoffAgent.java#L182) and set the path to the `load.soar` file you want to run. A working agent lives in this repo at:

```
agents/Full Learning Agent with Macros/load.soar
```

⚠️ These paths are currently hardcoded to one developer's machine. Also update the two log paths at [CopilotTakeoffAgent.java:167-168](java/src/agents/CopilotTakeoff/CopilotTakeoffAgent.java#L167-L168).

**4. Run the main class**

```bash
mvn exec:java -Dexec.mainClass="main.StartAgents"
```

Or in IntelliJ: open `java/`, right-click [StartAgents.java](java/src/main/StartAgents.java) → **Run**.

**5. Watch it fly.** The Soar debugger window opens, the agent loads its rules, and the aircraft starts responding.

---

## Optional: QGroundControl

[MAVLinkBridge.java](java/src/main/MAVLinkBridge.java) streams telemetry to QGC. Start QGC and it auto-connects.

- Java binds `127.0.0.1:14551`
- QGC listens on `127.0.0.1:14550`

---

## Repo map

| Folder | What's in it |
|---|---|
| **java/** | The **CCAT** — Java bridge between Soar and X-Plane. Start here. |
| **agents/** | Soar agents (rules for decision-making + reinforcement learning) |
| **soar/** | Runtime output: preference values and test results |
| **Verification/** | nuXmv formal verification models |
| **architecture/** | AADL models of pilot ⇄ agent communication |
| **tools/** | Soar → nuXmv translator and helper scripts |
| **examples/**, **legacy-examples/** | Sample and historical code |

### Inside `java/src/`

| Package | Role |
|---|---|
| `main/` | Entry point, MAVLink bridge, engine-failure injection |
| `agents/` | Soar agent wrappers — `CopilotTakeoffAgent` is the main one |
| `xplane/` | `XPlaneConnector` (all DREF reads/writes), waypoints, UI panels |
| `data/`, `util/` | Flight data model, voting logic, small widgets |

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `Is X-Plane running?` | X-Plane isn't up, or the XPC plugin isn't installed |
| `FileNotFoundException` on startup | Fix the hardcoded paths in step 3 |
| Agent loads but nothing moves | Check the aircraft matches the DREFs in use — see `.claude/CLAUDE.md` for the AW609 vs ALIA-250 differences |
| `0 productions loaded` | Wrong `load.soar` path |

---

## Aircraft support

Currently transitioning from **AW609** (X-Plane 11, 2 tiltrotors) to **Beta ALIA-250** (X-Plane 12, 4 lift rotors + 1 pusher). The two use different datarefs for throttle and VTOL mode — full comparison table in [.claude/CLAUDE.md](.claude/CLAUDE.md).

---

## License

See [LICENSE](LICENSE).
