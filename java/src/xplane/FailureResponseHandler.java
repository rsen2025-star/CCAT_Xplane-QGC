package xplane;

import gov.nasa.xpc.XPlaneConnect;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

/**
 * Java side of failure-handling.soar.
 *
 * SOAR announces what it decided on io.output-link ^failure-response (nominal, abort-takeoff,
 * abort-climb-and-land, precautionary-landing, forced-vertical-landing, maintain-wing-borne-cruise,
 * emergency-descent, switch-position-sensor, use-radar-altimeter, cautious-vertical-descent).
 * The commands themselves (throttle, pusher-throttle, autopilot, VTOLMode, sensor-to-use) already
 * travel through the normal output-link handlers in CopilotTakeoffAgent. This class:
 *   1. logs each response change,
 *   2. waits for the commands to settle, then reads X-Plane back and checks the aircraft is actually
 *      doing what the response says (throttles, autopilot, active position sensor),
 *   3. writes one PASS/FAIL line per check to the console and to failure-response-log.csv
 *      so results can be pasted into the test plan.
 * Checks run on their own thread, so the agent loop is never blocked.
 */
public class FailureResponseHandler
{
    private static final String DREF_THROTTLE = "sim/cockpit2/engine/actuators/throttle_ratio"; // [0-3] lift rotors, [4] pusher
    private static final String DREF_AP_SERVOS = "sim/cockpit2/autopilot/servos_on";            // 1 = autopilot engaged
    private static final String LOG_FILE = "failure-response-log.csv";

    private static final long SETTLE_MS = 2000;   // let the agent's next cycles apply the commands
    private static final long TIMEOUT_MS = 8000;  // keep re-checking this long before calling it a FAIL
    private static final long POLL_MS = 500;
    private static final double OFF = 0.05;       // throttle at or below this counts as "off"
    private static final double ON = 0.05;        // throttle above this counts as "on"
    private static final double SENSOR_MAX_ERR_DEG = 0.02; // active sensor must be this close to the true position

    private final XPlaneConnect raw = XPlaneConnector.getConnection();
    private final XPCUserInterface ui;
    private final ScheduledExecutorService exec = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread t = new Thread(r, "failure-response-verifier");
        t.setDaemon(true);
        return t;
    });
    private ScheduledFuture<?> pending;
    private String current = "";
    private PrintWriter csv;

    private static class Check
    {
        final String name;
        final String expected;
        final Supplier<String> actual;   // observed value, as text for the log
        final Supplier<Boolean> passes;  // evaluated against the latest observation
        Check(String name, String expected, Supplier<String> actual, Supplier<Boolean> passes)
        {
            this.name = name; this.expected = expected; this.actual = actual; this.passes = passes;
        }
    }

    public FailureResponseHandler(XPCUserInterface ui)
    {
        this.ui = ui;
        try
        {
            boolean fresh = !new java.io.File(LOG_FILE).exists();
            csv = new PrintWriter(new FileWriter(LOG_FILE, true), true);
            if (fresh) csv.println("time,response,check,expected,actual,result");
        }
        catch (IOException e)
        {
            System.err.println("[FailureResponse] could not open " + LOG_FILE + ": " + e.getMessage());
        }
    }

    /** Called from the output-link handler whenever ^failure-response is seen. */
    public synchronized void onFailureResponse(String response)
    {
        if (response.equals(current)) return; // output events re-deliver unchanged WMEs
        System.out.println("[FailureResponse] " + (current.isEmpty() ? "(start)" : current) + " -> " + response);
        current = response;
        if (pending != null) pending.cancel(false);

        List<Check> checks = checksFor(response);
        if (checks.isEmpty())
        {
            log(response, "(no X-Plane observable effect)", "", "", "INFO");
            return;
        }
        final String resp = response;
        pending = exec.schedule(() -> verify(resp, checks), SETTLE_MS, TimeUnit.MILLISECONDS);
    }

    private void verify(String response, List<Check> checks)
    {
        long end = System.currentTimeMillis() + TIMEOUT_MS;
        List<Check> open = new ArrayList<>(checks);
        while (!open.isEmpty() && System.currentTimeMillis() < end)
        {
            open.removeIf(c -> {
                try
                {
                    if (c.passes.get()) { log(response, c.name, c.expected, c.actual.get(), "PASS"); return true; }
                }
                catch (RuntimeException e) { /* read failed this poll, try again */ }
                return false;
            });
            if (open.isEmpty()) break;
            try { Thread.sleep(POLL_MS); } catch (InterruptedException e) { return; } // cancelled by a newer response
        }
        for (Check c : open)
        {
            String actual;
            try { actual = c.actual.get(); } catch (RuntimeException e) { actual = "read failed"; }
            log(response, c.name, c.expected, actual, "FAIL");
        }
    }

    private List<Check> checksFor(String response)
    {
        List<Check> c = new ArrayList<>();
        FailureInjector fi = FailureInjector.get();
        boolean pusherFailed = fi.mode("engine-4") != FailureInjector.Mode.OK;
        int liftFailed = 0;
        for (int i = 0; i < 4; i++) if (fi.mode("engine-" + i) != FailureInjector.Mode.OK) liftFailed++;
        // floor the agent's lift controller keeps per capability (params min-descent-lift in failure-handling.soar)
        double liftFloor = liftFailed >= 2 ? 0.65 : liftFailed == 1 ? 0.45 : 0.20;

        switch (response)
        {
            case "abort-takeoff":
                c.add(liftThrottle("lift throttle off", true, 0));
                c.add(pusherThrottle("pusher throttle off", true));
                break;
            case "abort-climb-and-land":
                c.add(autopilot("autopilot off", false));
                c.add(pusherThrottle("pusher throttle off", true));
                c.add(liftThrottle("lift throttle >= " + liftFloor, false, liftFloor - 0.05));
                break;
            case "emergency-descent":
                c.add(autopilot("autopilot off", false));
                c.add(pusherThrottle("pusher throttle off", true));
                c.add(liftThrottle("lift throttle >= " + liftFloor, false, liftFloor - 0.05));
                break;
            case "maintain-wing-borne-cruise":
                c.add(liftThrottle("lift throttle off", true, 0));
                c.add(pusherThrottle("pusher throttle on", false));
                c.add(autopilot("autopilot on", true));
                break;
            case "forced-vertical-landing":
            case "precautionary-landing":
                if (pusherFailed) c.add(pusherThrottle("failed pusher throttle off", true));
                break;
            case "switch-position-sensor":
                c.add(activeSensorHealthy());
                break;
            default:
                break; // nominal, use-radar-altimeter, cautious-vertical-descent: internal to the agent
        }
        return c;
    }

    // ---- checks -------------------------------------------------------------------------------

    private Check liftThrottle(String name, boolean wantOff, double minOn)
    {
        return new Check(name, name, () -> fmt(lift()), () -> wantOff ? lift() <= OFF : lift() >= minOn);
    }

    private Check pusherThrottle(String name, boolean wantOff)
    {
        return new Check(name, name, () -> fmt(pusher()), () -> wantOff ? pusher() <= OFF : pusher() > ON);
    }

    private Check autopilot(String name, boolean wantOn)
    {
        return new Check(name, name, () -> apOn() ? "on" : "off", () -> apOn() == wantOn);
    }

    /** The sensor SOAR switched to must be unfaulted and close to the aircraft's true position. */
    private Check activeSensorHealthy()
    {
        String[] names = {"gps", "imu", "lidar"};
        return new Check("active sensor healthy", "active sensor ok and within " + SENSOR_MAX_ERR_DEG + " deg of true position",
                () -> names[ui.activeSensor] + " err=" + fmt(sensorErrorDeg()),
                () -> FailureInjector.get().mode(names[ui.activeSensor]) == FailureInjector.Mode.OK
                        && sensorErrorDeg() <= SENSOR_MAX_ERR_DEG);
    }

    // ---- X-Plane reads ------------------------------------------------------------------------

    private double lift()
    {
        float[] t = read(DREF_THROTTLE);
        double max = 0;
        for (int i = 0; i < 4; i++) max = Math.max(max, t[i]);
        return max;
    }

    private double pusher() { return read(DREF_THROTTLE)[4]; }

    private boolean apOn() { return read(DREF_AP_SERVOS)[0] > 0.5f; }

    private double sensorErrorDeg()
    {
        double lat, lon;
        switch (ui.activeSensor)
        {
            case 1: lat = ui.getIMULat(); lon = ui.getIMULon(); break;
            case 2: lat = ui.getLidarLat(); lon = ui.getLidarLon(); break;
            default: lat = ui.getGPSLat(); lon = ui.getGPSLon(); break;
        }
        data.FlightData truth = XPlaneConnector.getFlightData();
        return Math.max(Math.abs(lat - truth.lat), Math.abs(lon - truth.lon));
    }

    private float[] read(String dref)
    {
        try
        {
            return raw.getDREF(dref);
        }
        catch (IOException e)
        {
            throw new RuntimeException(e);
        }
    }

    // ---- logging ------------------------------------------------------------------------------

    private static String fmt(double v) { return String.format("%.2f", v); }

    private synchronized void log(String response, String check, String expected, String actual, String result)
    {
        String line = LocalTime.now().withNano(0) + "," + response + "," + check + "," + expected + "," + actual + "," + result;
        System.out.println("[VERIFY " + result + "] " + response + ": " + check
                + (actual.isEmpty() ? "" : " (expected " + expected + ", got " + actual + ")"));
        if (csv != null) csv.println(line.replace("\"", "'"));
    }
}