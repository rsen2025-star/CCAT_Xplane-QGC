package xplane;

import data.FlightData;
import gov.nasa.xpc.XPlaneConnect;

import java.io.IOException;
import java.net.SocketException;
import java.util.Arrays;
// S.E.CHANGE - BM
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
// S.E.CHANGE - BM

public class XPlaneConnector
{
    public static int HEADING_HOLD = 0x2, WING_LEVELER = 0x4, VVI_CLIMB = 0x10;


    private static XPlaneConnect xpc = getConnection();

    private enum ControlSurface
    {
        LAT_STICK(0, "Latitudinal Stick (Roll)"),
        LONG_STICK(1, "Longitudinal Stick (Pitch)"),
        RUDDER(2, "Rudder Pedals (Yaw)"),
        THROTTLE(3, "Throttle"),
        GEAR(4, "Landing Gear"),
        FLAPS(5, "Flaps");

        final int index;
        final String description;

        ControlSurface(int i, String desc)
        {
            this.index = i;
            this.description = desc;
        }
    }



    // S.CHANGE - BM
    /*
        The agent loop, the Soar output handlers and the MAVLink bridge all share one XPC
        connection. XPC sends a request and reads the next reply, so concurrent calls can
        receive each other's replies (e.g. an altitude read as "on ground"). This makes
        each request/reply pair run one at a time.
     */
    private static class LockedXPlaneConnect extends XPlaneConnect
    {
        LockedXPlaneConnect() throws SocketException { super(); }

        @Override public float[] getDREF(String dref) throws IOException
        { synchronized (XPlaneConnector.class) { return super.getDREF(dref); } }
        @Override public float[][] getDREFs(String[] drefs) throws IOException
        { synchronized (XPlaneConnector.class) { return super.getDREFs(drefs); } }
        @Override public void sendDREF(String dref, float value) throws IOException
        { synchronized (XPlaneConnector.class) { super.sendDREF(dref, value); } }
        @Override public void sendDREF(String dref, float[] value) throws IOException
        { synchronized (XPlaneConnector.class) { super.sendDREF(dref, value); } }
        @Override public void sendDREFs(String[] drefs, float[][] values) throws IOException
        { synchronized (XPlaneConnector.class) { super.sendDREFs(drefs, values); } }
        @Override public float[] getCTRL(int ac) throws IOException
        { synchronized (XPlaneConnector.class) { return super.getCTRL(ac); } }
        @Override public void sendCTRL(float[] values) throws IOException
        { synchronized (XPlaneConnector.class) { super.sendCTRL(values); } }
    }
    // E.CHANGE - BM
    public static XPlaneConnect getConnection()
    {
        if (xpc == null)
        {
            try
            {
                xpc = new LockedXPlaneConnect(); // S.E.CHANGE - BM
            }
            catch (SocketException e)
            {
                e.printStackTrace();
                System.exit(1);
            }
        }
        return xpc;
    }

    private static XPlaneConnect refresh()
    {
        try
        {
            xpc = new LockedXPlaneConnect(); // S.E.CHANGE - BM
        }
        catch (SocketException e)
        {
            e.printStackTrace();
        }
        return xpc;
    }

    // ------------------ GET INFO ------------------------

    public float currentAirspeed()
    {
        return getValueFromSim("sim/flightmodel/position/indicated_airspeed2");
    }

    public float currentAltitude()
    {
        return getValueFromSim("sim/cockpit2/gauges/indicators/altitude_ft_pilot");
    }

    public float currentHeading()
    {
        return getValueFromSim("sim/flightmodel/position/true_psi");
    }

    public float currentRollAngle()
    {
        return getValueFromSim("sim/flightmodel/position/true_phi");
    }

    public float currentPitch()
    {
        return getValueFromSim("sim/flightmodel/position/true_theta");
    }

    // ------------------ CONTROL AIRPLANE -------------------

    private static void changeControlSurfacePositions(ControlSurface controlSurface, float value)
    {
        try
        {
            float[] positions = xpc.getCTRL(0);
            positions[controlSurface.index] = value;
            xpc.sendCTRL(positions);
        }
        catch (IOException e)
        {
            failConcisely(e, controlSurface.description);
        }
    }

    public void setWheelBrake(boolean isBraking)
    {
        setValueOnSim("sim/flightmodel/controls/parkbrake", isBraking ? 1 : 0);
    }
    // AW609 only — inactive on ALIA 250 (no speedbrake)
    public void setAirBrake(float brakeDeployment){ }
    // S.CHANGE - BM
    // ALIA 250: engines 0-3 are lift rotors, PUSHER_ENGINE is the forward pusher prop.
    public static int PUSHER_ENGINE = 4;
    private float liftThrottle = 0.0f;
    private float pusherThrottle = -1.0f; // < 0 means the pusher follows the lift throttle (takeoff)

    public void setEngineThrottle(float throttle)
    {
        throttle = Math.max(0.0f, Math.min(1.0f, throttle));
        liftThrottle = throttle;
        float[] throttles = new float[]{throttle, throttle, throttle, throttle, throttle};
        if (pusherThrottle >= 0) {
            throttles[PUSHER_ENGINE] = pusherThrottle;
        }
        setValueOnSim("sim/cockpit2/engine/actuators/throttle_ratio", throttles);
    }

    // Set by the Soar agent's ^pusher-throttle output; -1 ties the pusher to the lift throttle.
    // Treating them as -1 put the pusher at lift (takeoff) power while trying to slow down.
    public void setPusherThrottle(float throttle)
    {
        pusherThrottle = throttle <= -0.5f ? -1.0f : Math.max(0.0f, Math.min(1.0f, throttle));
        setEngineThrottle(liftThrottle);
    }
    // E.CHANGE - BM
    // S.CHANGE - ZR
    public static void setEngineFailure(int engine, boolean failed)
    {
        setValueOnSim("sim/operation/failures/rel_engfai" + engine, failed ? 6 : 0);
    }
    // E.CHANGE - ZR
    public int getAutopilotState(){
        try {
            return (int)xpc.getDREF("sim/cockpit/autopilot/autopilot_state")[0];
        } catch (IOException e) {
            e.printStackTrace();
        }
        return -1;
    }

    public void setAutopilot(int mode){
        try {
            xpc.sendDREF("sim/cockpit/autopilot/autopilot_state", mode);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
    public void setAutopilotMode(int mode){
        try {
            int state = (int)xpc.getDREF("sim/cockpit/autopilot/autopilot_state")[0];
            System.err.println("state: "+(state));
            if((state & mode) == 0){
                xpc.sendDREF("sim/cockpit/autopilot/autopilot_state",state | mode);
            }
            state = (int)xpc.getDREF("sim/cockpit/autopilot/autopilot_state")[0];
            System.err.println("state: "+(state));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    // S.CHANGE - BM
    // Same as pressing AP on the Garmin. XPC can only write datarefs, so this goes through
    // X-Plane's built-in UDP command interface (port 49000) instead.
    public void engageAutopilot(){
        sendCommand("sim/autopilot/servos_on");
        autopilotEngaged = true;
        holdHeadingMode();
    }

    public void disengageAutopilot(){
        autopilotEngaged = false;
        sendCommand("sim/autopilot/servos_off_any");
    }

    // True between engageAutopilot and disengageAutopilot (the agent's cruise/decelerate)
    private volatile boolean autopilotEngaged = false;

    /*
        Keeps HDG mode on while the agent has the AP engaged, so the AP always follows the
        heading bug instead of dropping back to ROL and flying straight. Called on engage and
        every agent cycle. HDG is a toggle, so only press it when it is off
        (0 = off, 1 = armed, 2 = active) - pressing while armed would turn it off
    */
    public void holdHeadingMode(){
        if (!autopilotEngaged) return;
        try {
            if (xpc.getDREF("sim/cockpit2/autopilot/heading_status")[0] == 0) {
                sendCommand("sim/autopilot/heading");
            }
        } catch (IOException | IndexOutOfBoundsException e) {
            failConcisely(e, "holdHeadingMode");
        }
    }

    // Same as pressing VS on the Garmin and dialing in fpm: the AP climbs/descends toward the
    // selected altitude (setTargetAltitude), captures it and switches to ALT hold by itself.
    public void setVerticalSpeed(float fpm){
        try {
            // VS is a toggle, so only press it when VS isn't already active (0 = off, 1 = armed, 2 = active)
            if (xpc.getDREF("sim/cockpit2/autopilot/vvi_status")[0] != 2) {
                sendCommand("sim/autopilot/vertical_speed");
                // Engaging VS syncs the VS target to the current climb rate; overwrite it after
                Thread.sleep(300);
            }
            xpc.sendDREF("sim/cockpit2/autopilot/vvi_dial_fpm", fpm);
        } catch (IOException | InterruptedException e) {
            failConcisely(e, "setVerticalSpeed");
        }
    }

    // Height above ground (ft), vertical speed (fpm), on ground (1 = yes), pitch (deg, nose up +), ground speed (kt)
    public float[] getVerticalState(){
        try {
            float[][] v = xpc.getDREFs(new String[]{
                    "sim/flightmodel/position/y_agl",
                    "sim/flightmodel/position/vh_ind_fpm",
                    "sim/flightmodel/failures/onground_any",
                    "sim/flightmodel/position/true_theta",
                    "sim/flightmodel/position/groundspeed"});
            return new float[]{v[0][0] * 3.28084f, v[1][0], v[2][0], v[3][0], v[4][0] * 1.94384f};
        } catch (IOException | IndexOutOfBoundsException e) {
            failConcisely(e, "getVerticalState");
        }
        // Ground speed defaults high so a failed read never looks like "stopped"
        return new float[]{0.0f, 0.0f, 0.0f, 0.0f, 999.0f};
    }

    private static void sendCommand(String command){
        byte[] cmd = command.getBytes(StandardCharsets.US_ASCII);
        byte[] msg = new byte[5 + cmd.length + 1];
        System.arraycopy("CMND".getBytes(StandardCharsets.US_ASCII), 0, msg, 0, 4);
        System.arraycopy(cmd, 0, msg, 5, cmd.length);
        try (DatagramSocket socket = new DatagramSocket()) {
            socket.send(new DatagramPacket(msg, msg.length, InetAddress.getByName(xpc.getXPlaneAddr()), 49000));
        } catch (IOException e) {
            failConcisely(e, "CMND: " + command);
        }
    }
    // E.CHANGE - BM
    // AW609 only — inactive on ALIA 250 (no flaps, no thrust vectoring, fixed-angle rotors)
    public void setFlapsMode(int mode){ }
    public void setVTOLModeVertical(){ }
    public void setVTOLModeHorizontal(){ }
    public void setRotorPosition(float position){ }

    // S.CHANGE - BM
    // Takes a True heading. X-Plane 12's autopilot follows the magnetic heading bug, so the
    // current true->magnetic offset is applied before setting it (legacy true dataref kept too)
    public void setTargetHeading(double heading){
        try {
            float[][] psi = xpc.getDREFs(new String[]{
                    "sim/flightmodel/position/true_psi",
                    "sim/flightmodel/position/mag_psi"});
            double magHeading = heading + (psi[1][0] - psi[0][0]);
            magHeading = ((magHeading % 360.0) + 360.0) % 360.0;
            xpc.sendDREF("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot",(float)magHeading);
            // E.CHANGE - BM
            xpc.sendDREF("sim/cockpit/autopilot/heading",(float)heading);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
//sim/cockpit/autopilot/airspeed
    public void setTargetAirspeed(int speed){
        try {
            xpc.sendDREF("sim/cockpit/autopilot/airspeed",speed);

        } catch (IOException e) {
            e.printStackTrace();
        }

    }
    public void setTargetAltitude(int altitude){
        try {
            xpc.sendDREF("sim/cockpit2/autopilot/altitude_dial_ft",altitude); // S.E.CHANGE - BM
            xpc.sendDREF("sim/cockpit/autopilot/altitude",altitude);
            xpc.sendDREF("sim/cockpit/autopilot/current_altitude",altitude);
            xpc.sendDREF("sim/cockpit/autopilot/alt_hold_ft_fms",altitude);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
    public void setAutoHeading(){
        try {
            int state = (int)xpc.getDREF("sim/cockpit/autopilot/autopilot_state")[0];
            if((state & 0x2) == 0) {
                xpc.sendDREF("sim/cockpit/autopilot/autopilot_state", 0x2);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void setRoll(float ratio)
    {
        changeControlSurfacePositions(ControlSurface.LONG_STICK, ratio);
    }

    public void setPitch(float ratio)
    {
        changeControlSurfacePositions(ControlSurface.LAT_STICK, ratio);
    }

    public void setThrottle(float positiveRatio)
    {
        changeControlSurfacePositions(ControlSurface.THROTTLE, positiveRatio);
    }

    public void setGear(boolean deployed)
    {
        changeControlSurfacePositions(ControlSurface.GEAR, deployed ? 1 : 0);
    }

    public void setYaw(float ratio)
    {
        changeControlSurfacePositions(ControlSurface.RUDDER, ratio);
    }

    public void setFlaps(float positiveRatio)
    {
        changeControlSurfacePositions(ControlSurface.FLAPS, positiveRatio);
    }

    // ------------------------------------------------------------

    private float[][] getValuesFromSim(String[] strs)
    {
        try
        {
            return xpc.getDREFs(strs);
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        return new float[1][1];
    }

    private float getValueFromSim(String str)
    {
        try
        {
            return xpc.getDREF(str)[0];
        }
        catch (IOException | IndexOutOfBoundsException | NegativeArraySizeException e)
        {
            failConcisely(e, "DREF: "+str);
        }
        return 0.0f;
    }

    private static void failConcisely(Exception e, String extra)
    {
        System.err.println(e.toString() + " " + extra);
    }

    private static void setValueOnSim(String s, float val)
    {
        try
        {
            xpc.sendDREF(s, val);
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    private void setValueOnSim(String s, float[] val)
    {
        try
        {
            xpc.sendDREF(s, val);
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    private void setValuesOnSim(String[] strings, float[][] vals)
    {
        try
        {
            xpc.sendDREFs(strings, vals);
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    static float[][] values;
    static float[][] engineStats;
    static float[][] controlSurfaces;
    static float[] reversers;
    static float[] wheelBrakes;
    static boolean enginesOK = true;
    static boolean airBrakesON = false;
    static boolean wheelBrakesON = false;
    static boolean reverseON = false;

    public static synchronized FlightData getFlightData()
    {

        try {

            values = xpc.getDREFs(new String[]{
                    "sim/flightmodel/position/indicated_airspeed",
                    "sim/cockpit2/gauges/indicators/altitude_ft_pilot",
                    "sim/flightmodel/position/latitude",
                    "sim/flightmodel/position/longitude",
                    //"sim/flightmodel/engine/ENGN_thro"
                    "sim/cockpit2/engine/actuators/throttle_ratio"
            });

            // AW609 DREFs removed — rel_engfai0..7, speedbrake_ratio, annunciators/reverse
            // are broken/irrelevant on ALIA 250 (electric engines, no speedbrake)
            wheelBrakes = xpc.getDREF("sim/flightmodel/controls/parkbrake");
            wheelBrakesON = wheelBrakes[0] == 1.0f;
            enginesOK = true;
            airBrakesON = false;
            reverseON = false;
        }
        catch (Throwable e)
        {
            System.err.println(e.getMessage());
            xpc = XPlaneConnector.refresh();
        }

//        if ( values.length < 4)
//        {
//            System.err.println("Somehow, values.length < 4");
//        }


        return new FlightData((int) values[0][0], (int) values[1][0], values[2][0], values[3][0], values[4][0], enginesOK, wheelBrakesON, airBrakesON, reverseON);
    }
}