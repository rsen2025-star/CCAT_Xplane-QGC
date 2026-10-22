package xplane;

import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

// S.CHANGE - ZR
public class FailureInjector
{
    public enum Mode
    {
        OK, FAILED, BIAS, DRIFT, FREEZE, DROPOUT;

        @Override
        public String toString() { return name().toLowerCase(); }
    }

    // Engines 0-3 lift rotors, 4 pusher
    public static final String GPS = "gps", IMU = "imu", LIDAR = "lidar",
            ALTIMETER = "altimeter", RADAR_ALTIMETER = "radar-altimeter";
    public static final String[] ALL_TARGETS = {
            "engine-0", "engine-1", "engine-2", "engine-3", "engine-4",
            GPS, IMU, LIDAR, ALTIMETER, RADAR_ALTIMETER};

    private static final Mode[] ENGINE_MODES = {Mode.OK, Mode.FAILED};
    private static final Mode[] DATA_MODES = {Mode.OK, Mode.BIAS, Mode.DRIFT, Mode.FREEZE, Mode.DROPOUT};

    // degrees lat/lon and feet, tune in sim
    private static final double POSITION_BIAS_DEG = 0.065;
    private static final double POSITION_DRIFT_DEG_PER_S = 0.002, POSITION_MAX_DRIFT_DEG = 0.1;
    private static final double ALTITUDE_BIAS_FT = 300;
    private static final double ALTITUDE_DRIFT_FT_PER_S = 20, ALTITUDE_MAX_DRIFT_FT = 1500;

    private static class Fault
    {
        final Mode mode;
        final long startMs = System.currentTimeMillis();
        volatile double[] frozen;

        Fault(Mode mode) { this.mode = mode; }
    }

    private static final FailureInjector INSTANCE = new FailureInjector();

    public static FailureInjector get() { return INSTANCE; }

    private final Map<String, Fault> faults = new ConcurrentHashMap<>();
    private volatile String lastInjected = "none";

    private FailureInjector() {}

    // ------------------ STATE ------------------------

    public Mode mode(String target)
    {
        Fault f = faults.get(target);
        return f == null ? Mode.OK : f.mode;
    }

    public int activeCount() { return faults.size(); }

    public String lastInjected() { return lastInjected; }

    public boolean allEnginesOk()
    {
        for (String t : faults.keySet())
        {
            if (t.startsWith("engine-")) return false;
        }
        return true;
    }

    // ------------------ CONTROL ------------------------

    public void inject(String target, Mode mode)
    {
        if (mode == Mode.OK)
        {
            clear(target);
            return;
        }
        faults.put(target, new Fault(mode));
        lastInjected = target;
        System.out.println("[FailureInjector] " + target + " -> " + mode);
        writeEngine(target, true);
    }

    public void clear(String target)
    {
        faults.remove(target);
        System.out.println("[FailureInjector] " + target + " cleared");
        writeEngine(target, false);
    }

    // also resets failures X-Plane kept from a previous run
    public void clearAll()
    {
        faults.clear();
        lastInjected = "none";
        for (String t : ALL_TARGETS) writeEngine(t, false);
    }

    public Mode cycle(String target)
    {
        Mode[] modes = target.startsWith("engine-") ? ENGINE_MODES : DATA_MODES;
        Mode next = modes[(Arrays.asList(modes).indexOf(mode(target)) + 1) % modes.length];
        inject(target, next);
        return next;
    }

    // engines only; the XPC call takes the XPlaneConnector lock
    private void writeEngine(String target, boolean failed)
    {
        if (target.startsWith("engine-"))
        {
            XPlaneConnector.setEngineFailure(Integer.parseInt(target.substring("engine-".length())), failed);
        }
    }

    // ------------------ APPLYING FAULTS ------------------------

    // Returns {lat, lon} as the sensor would report them
    public double[] applyPosition(String sensor, double lat, double lon)
    {
        Fault f = faults.get(sensor);
        if (f == null) return new double[]{lat, lon};
        switch (f.mode)
        {
            case BIAS:
                return new double[]{lat + POSITION_BIAS_DEG, lon + POSITION_BIAS_DEG};
            case DRIFT:
            {
                double d = Math.min(POSITION_MAX_DRIFT_DEG, POSITION_DRIFT_DEG_PER_S * elapsedSeconds(f));
                return new double[]{lat + d, lon + d};
            }
            case FREEZE:
                if (f.frozen == null) f.frozen = new double[]{lat, lon};
                return f.frozen;
            case DROPOUT:
                return new double[]{0.0, 0.0};   // not NaN, VotingLogic.truncate() throws on it
            default:
                return new double[]{lat, lon};
        }
    }

    // Altimeter / radar altimeter reading (ft)
    public double applyValue(String target, double trueValue)
    {
        Fault f = faults.get(target);
        if (f == null) return trueValue;
        switch (f.mode)
        {
            case BIAS:
                return trueValue + ALTITUDE_BIAS_FT;
            case DRIFT:
                return trueValue + Math.min(ALTITUDE_MAX_DRIFT_FT, ALTITUDE_DRIFT_FT_PER_S * elapsedSeconds(f));
            case FREEZE:
                if (f.frozen == null) f.frozen = new double[]{trueValue};
                return f.frozen[0];
            case DROPOUT:
                return 0.0;
            default:
                return trueValue;
        }
    }

    private static double elapsedSeconds(Fault f)
    {
        return (System.currentTimeMillis() - f.startMs) / 1000.0;
    }
}
// E.CHANGE - ZR
