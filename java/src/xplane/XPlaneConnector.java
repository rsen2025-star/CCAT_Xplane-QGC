package xplane;

import data.FlightData;
import gov.nasa.xpc.XPlaneConnect;

import java.io.IOException;
import java.net.SocketException;
import java.util.Arrays;

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



    public static XPlaneConnect getConnection()
    {
        if (xpc == null)
        {
            try
            {
                xpc = new XPlaneConnect();
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
            xpc = new XPlaneConnect();
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
    public void setEngineThrottle(float throttle)
    {
        setValueOnSim("sim/cockpit2/engine/actuators/throttle_ratio", new float[]{throttle, throttle, throttle, throttle, throttle});
    }
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
    // AW609 only — inactive on ALIA 250 (no flaps, no thrust vectoring, fixed-angle rotors)
    public void setFlapsMode(int mode){ }
    public void setVTOLModeVertical(){ }
    public void setVTOLModeHorizontal(){ }
    public void setRotorPosition(float position){ }

    // Uses True heading to set the plane's target heading, cockpit displays magnetic heading
    public void setTargetHeading(double heading){
        try {
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