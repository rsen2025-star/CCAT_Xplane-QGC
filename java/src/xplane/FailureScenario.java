package xplane;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// S.CHANGE - M3
/**
 * Scripted failure scenario for the Milestone 3 demo: a complete flight plan with one or more
 * equipment failures injected at fixed points along the route, so the run is repeatable instead
 * of depending on someone clicking the Failure Injection buttons at the right moment.
 *
 * Each step has
 *   - a trigger (fraction of the route flown, or within X km of the destination),
 *   - one or more failures to inject through FailureInjector (same path the buttons use), and
 *   - the ^failure-response the Soar agent is expected to announce within EXPECT_TIMEOUT_S.
 * Whether the agent announced the expected response is logged PASS/FAIL next to the X-Plane
 * readback checks that FailureResponseHandler already writes to failure-response-log.csv.
 * The run ends with a check that the aircraft actually landed at the destination (scenarios that
 * are supposed to finish the route) or stays airborne (wing-borne scenario).
 *
 * Pick the scenario with a JVM option:   -Dfailure.scenario=sensor-cascade
 *   sensor-cascade   (default demo) GPS dropout -> altimeter freeze -> lift rotor out on approach;
 *                    the flight plan is still completed with a precautionary landing
 *   pusher-loss      pusher fails in cruise -> forced vertical landing where it is
 *   lift-critical    two lift rotors fail in cruise -> stays wing-borne on the pusher
 *   none             (default when the option is absent) buttons only, as before
 *
 * Call tick() once per agent cycle.
 */
public class FailureScenario
{
    private static final long EXPECT_TIMEOUT_S = 12;    // time the agent gets to announce the response
    private static final long LANDING_TIMEOUT_S = 300;  // time allowed to land after the last injection
    private static final long WING_BORNE_HOLD_S = 60;   // how long lift-critical must stay airborne

    private enum Trigger { AT_PROGRESS, WITHIN_KM_OF_DEST, SECONDS_AFTER_LAST_STEP }

    private static class Step
    {
        final String name;
        final Trigger trigger;
        final double value;
        final String[][] inject;      // {target, mode}
        final String expectResponse;  // ^failure-response the agent must announce
        boolean fired = false;
        long firedAtMs;
        boolean resolved = false;

        Step(String name, Trigger trigger, double value, String[][] inject, String expectResponse)
        {
            this.name = name; this.trigger = trigger; this.value = value;
            this.inject = inject; this.expectResponse = expectResponse;
        }
    }

    private enum Ending { LAND_AT_DESTINATION, STAY_AIRBORNE, LAND_IN_PLACE }

    private final String name;
    private final FailureResponseHandler handler;
    private final List<Step> steps = new ArrayList<>();
    private final Ending ending;
    private long lastFiredMs = 0;
    private double routeTotalKm = Double.NaN;   // distance along the route when the scenario started
    private long endingStartMs = 0;
    private boolean finished = false;
    private int passes = 0, fails = 0;

    private FailureScenario(String name, FailureResponseHandler handler, Ending ending)
    {
        this.name = name; this.handler = handler; this.ending = ending;
    }

    /** Scenario named by -Dfailure.scenario (or env FAILURE_SCENARIO); null when none is requested. */
    public static FailureScenario fromProperty(FailureResponseHandler handler)
    {
        String n = System.getProperty("failure.scenario", System.getenv("FAILURE_SCENARIO"));
        if (n == null || n.isEmpty() || n.equalsIgnoreCase("none")) return null;
        FailureScenario s;
        switch (n.toLowerCase())
        {
            case "sensor-cascade":
                s = new FailureScenario(n, handler, Ending.LAND_AT_DESTINATION);
                // 1. a quarter of the route flown: GPS gone -> agent must fail over to the IMU
                s.steps.add(new Step("GPS dropout", Trigger.AT_PROGRESS, 0.25,
                        new String[][]{{FailureInjector.GPS, "dropout"}}, "switch-position-sensor"));
                // 2. just over half the route flown (GPS still failed): barometric altimeter freezes
                s.steps.add(new Step("Altimeter freeze", Trigger.AT_PROGRESS, 0.55,
                        new String[][]{{FailureInjector.ALTIMETER, "freeze"}}, "use-radar-altimeter"));
                // 3. approach (inside the agent's 3.5 km land-at-target-km): one lift rotor fails
                s.steps.add(new Step("Lift rotor 1 failed on approach", Trigger.WITHIN_KM_OF_DEST, 3.0,
                        new String[][]{{"engine-1", "failed"}}, "precautionary-landing"));
                break;
            case "pusher-loss":
                s = new FailureScenario(n, handler, Ending.LAND_IN_PLACE);
                s.steps.add(new Step("Pusher failed in cruise", Trigger.AT_PROGRESS, 0.30,
                        new String[][]{{"engine-4", "failed"}}, "forced-vertical-landing"));
                break;
            case "lift-critical":
                s = new FailureScenario(n, handler, Ending.STAY_AIRBORNE);
                // both rotors in the same tick so the agent never sits in the 1-rotor "degraded" state
                s.steps.add(new Step("Lift rotors 0 and 2 failed in cruise", Trigger.AT_PROGRESS, 0.30,
                        new String[][]{{"engine-0", "failed"}, {"engine-2", "failed"}}, "maintain-wing-borne-cruise"));
                break;
            default:
                System.err.println("[Scenario] unknown scenario '" + n + "' (sensor-cascade, pusher-loss, lift-critical)");
                return null;
        }
        System.out.println("[Scenario] loaded '" + s.name + "' with " + s.steps.size() + " step(s)");
        return s;
    }

    public void tick(FlightPlan plan, double distanceToTargetKm)
    {
        if (finished || !plan.isDeparted()) return;
        long now = System.currentTimeMillis();

        // route length along the plan, taken from the first cycle in cruise
        if (Double.isNaN(routeTotalKm)) routeTotalKm = Math.max(distanceToTargetKm, 0.001);

        for (Step st : steps)
        {
            if (!st.fired)
            {
                if (triggered(st, plan, distanceToTargetKm, now)) fire(st, now);
                break;   // steps run in order, one at a time
            }
            if (!st.resolved) checkExpectation(st, now);
        }

        boolean allFired = true;
        for (Step st : steps) if (!st.fired) allFired = false;
        boolean allResolved = true;
        for (Step st : steps) if (!st.resolved) allResolved = false;
        if (allFired && allResolved) checkEnding(plan, now);
    }

    private boolean triggered(Step st, FlightPlan plan, double distKm, long now)
    {
        switch (st.trigger)
        {
            case AT_PROGRESS:
                return 1.0 - distKm / routeTotalKm >= st.value;
            case WITHIN_KM_OF_DEST:
                return plan.getIndex() >= plan.getPoints().length - 1 && distKm <= st.value;
            default:
                return now - lastFiredMs >= st.value * 1000;
        }
    }

    private void fire(Step st, long now)
    {
        st.fired = true;
        st.firedAtMs = now;
        lastFiredMs = now;
        System.out.println("[Scenario] step '" + st.name + "' -> injecting " + Arrays.deepToString(st.inject));
        final FailureInjector fi = FailureInjector.get();
        // off the agent thread: engine failures go through the XPlaneConnector lock
        new Thread(() -> {
            for (String[] f : st.inject) fi.inject(f[0], FailureInjector.Mode.valueOf(f[1].toUpperCase()));
        }, "scenario-injector").start();
    }

    private void checkExpectation(Step st, long now)
    {
        String actual = handler.currentResponse();
        if (actual.equals(st.expectResponse))
        {
            handler.record("scenario: " + st.name, "response " + st.expectResponse, actual, "PASS");
            st.resolved = true; passes++;
        }
        else if (now - st.firedAtMs > EXPECT_TIMEOUT_S * 1000)
        {
            handler.record("scenario: " + st.name, "response " + st.expectResponse,
                    actual.isEmpty() ? "none" : actual, "FAIL");
            st.resolved = true; fails++;
        }
    }

    private void checkEnding(FlightPlan plan, long now)
    {
        if (endingStartMs == 0) endingStartMs = now;
        long waited = (now - endingStartMs) / 1000;
        switch (ending)
        {
            case LAND_AT_DESTINATION:
                if (plan.isDestinationReached()) finish("landed at destination", "landed at destination", "PASS");
                else if (waited > LANDING_TIMEOUT_S) finish("landed at destination", "not landed after " + waited + " s", "FAIL");
                break;
            case LAND_IN_PLACE:
                // the route is abandoned on purpose: pass when the agent reports it has landed
                if (handler.landed()) finish("landed in place (route abandoned)", "landed", "PASS");
                else if (waited > LANDING_TIMEOUT_S) finish("landed in place (route abandoned)", "not landed after " + waited + " s", "FAIL");
                break;
            case STAY_AIRBORNE:
                if (waited >= WING_BORNE_HOLD_S)
                    finish("still airborne " + WING_BORNE_HOLD_S + " s after failures", "airborne", handler.landed() ? "FAIL" : "PASS");
                break;
        }
    }

    private void finish(String check, String actual, String result)
    {
        finished = true;
        if (result.equals("PASS")) passes++; else fails++;
        handler.record("scenario: " + check, check, actual, result);
        System.out.println("[Scenario] '" + name + "' finished: " + passes + " PASS, " + fails + " FAIL (see failure-response-log.csv)");
    }
}
// E.CHANGE - M3
