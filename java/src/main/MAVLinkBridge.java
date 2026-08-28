package main;

import data.FlightData;
import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.common.*;
import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;
import xplane.XPlaneConnector;

import java.io.PipedInputStream;
import java.io.PipedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

/**
 * Bridges X-Plane 12 state to QGroundControl via MAVLink over UDP.
 * Binds to local port 14551, sends to QGC at 127.0.0.1:14550.
 * QGC auto-connects when it receives the first HEARTBEAT.
 * Sends: HEARTBEAT (1 Hz), GLOBAL_POSITION_INT (5 Hz), ATTITUDE (5 Hz).
 * Listens and replies to: ParamRequestList, MissionRequestList, CommandLong.
 */
public class MAVLinkBridge implements Runnable {

    private static final int LOCAL_PORT           = 14551; // we bind here — QGC replies to this port
    private static final int QGC_PORT             = 14550; // QGC listens here for auto-connect
    private static final int HEARTBEAT_INTERVAL_MS = 1000; // 1 Hz
    private static final int POSITION_INTERVAL_MS  = 200;  // 5 Hz

    private final DatagramSocket recvSocket; // bound to LOCAL_PORT — receives QGC replies
    private final DatagramSocket sendSocket; // OS-assigned port — sends to QGC
    private final InetAddress    qgcAddress;
    private OutputStream         udpOut;    // shared across threads — all access via sendMessage()

    public MAVLinkBridge() throws Exception {
        recvSocket = new DatagramSocket(LOCAL_PORT);
        sendSocket = new DatagramSocket();
        qgcAddress = InetAddress.getByName("127.0.0.1");
    }

    @Override
    public void run() {
        System.out.println("[MAVLinkBridge] Started — local:" + LOCAL_PORT + " → QGC:" + QGC_PORT);
        try {
            // Pipe: UDP receiver writes bytes here → MavlinkConnection reads from the other end
            PipedOutputStream udpToPipe = new PipedOutputStream();
            PipedInputStream  pipeIn   = new PipedInputStream(udpToPipe);

            // Thread-safe OutputStream: buffers bytes, sends UDP packet on flush()
            udpOut = new OutputStream() {
                private final byte[] buf = new byte[512];
                private int pos = 0;

                @Override
                public synchronized void write(int b) {
                    buf[pos++] = (byte) b;
                }

                @Override
                public synchronized void flush() throws IOException {
                    if (pos > 0) {
                        sendSocket.send(new DatagramPacket(buf, pos, qgcAddress, QGC_PORT));
                        pos = 0;
                    }
                }
            };

            MavlinkConnection connection = MavlinkConnection.create(pipeIn, udpOut);

            startUdpReceiverThread(udpToPipe);
            startHeartbeatThread(connection);
            startTelemetryThread(connection);
            startListenerThread(connection);

            // Keep run() alive so daemon threads stay up
            Thread.currentThread().join();

        } catch (Exception e) {
            System.err.println("[MAVLinkBridge] Error: " + e.getMessage());
        }
    }

    // Receives UDP packets from QGC on recvSocket, writes raw bytes into pipe for MavlinkConnection to decode
    private void startUdpReceiverThread(PipedOutputStream pipe) {
        Thread t = new Thread(() -> {
            byte[] buf = new byte[512];
            DatagramPacket packet = new DatagramPacket(buf, buf.length);
            while (true) {
                try {
                    recvSocket.receive(packet);
                    pipe.write(packet.getData(), 0, packet.getLength());
                    pipe.flush();
                } catch (Exception e) {
                    System.err.println("[MAVLinkBridge] Receiver error: " + e.getMessage());
                    return;
                }
            }
        });
        t.setDaemon(true);
        t.start();
    }

    // Synchronized: prevents heartbeat/telemetry/listener threads from interleaving bytes
    private synchronized void sendMessage(MavlinkConnection connection, Object payload) throws Exception {
        connection.send1(1, 1, payload);
        udpOut.flush();
    }

    private void startHeartbeatThread(MavlinkConnection connection) {
        Thread t = new Thread(() -> {
            while (true) {
                try {
                    Heartbeat heartbeat = Heartbeat.builder()
                            .type(MavType.MAV_TYPE_QUADROTOR)
                            .autopilot(MavAutopilot.MAV_AUTOPILOT_GENERIC)
                            .baseMode(io.dronefleet.mavlink.util.EnumValue.create(0))
                            .systemStatus(MavState.MAV_STATE_ACTIVE)
                            .mavlinkVersion(3)
                            .build();
                    sendMessage(connection, heartbeat);
                    Thread.sleep(HEARTBEAT_INTERVAL_MS);
                } catch (Exception e) {
                    System.err.println("[MAVLinkBridge] Heartbeat error: " + e.getMessage());
                    return;
                }
            }
        });
        t.setDaemon(true);
        t.start();
    }

    private void startTelemetryThread(MavlinkConnection connection) {
        Thread t = new Thread(() -> {
            long startTime = System.currentTimeMillis();
            XPlaneConnector connector = new XPlaneConnector();
            while (true) {
                try {
                    FlightData fd = XPlaneConnector.getFlightData();
                    if (fd != null) {
                        long timeBootMs = System.currentTimeMillis() - startTime;

                        // GLOBAL_POSITION_INT — aircraft dot on map
                        int latInt  = (int)(fd.lat * 1e7);
                        int lonInt  = (int)(fd.lon * 1e7);
                        int altMm   = (int)(fd.altitude * 304.8); // feet → mm
                        int hdgCdeg = (int)(connector.currentHeading() * 100);
                        hdgCdeg     = ((hdgCdeg % 36000) + 36000) % 36000;

                        GlobalPositionInt pos = GlobalPositionInt.builder()
                                .timeBootMs(timeBootMs)
                                .lat(latInt)
                                .lon(lonInt)
                                .alt(altMm)
                                .relativeAlt(altMm)
                                .vx(0).vy(0).vz(0)
                                .hdg(hdgCdeg)
                                .build();
                        sendMessage(connection, pos);

                        // ATTITUDE — arrow direction on map
                        float pitchRad = (float) Math.toRadians(connector.currentPitch());
                        float rollRad  = (float) Math.toRadians(connector.currentRollAngle());
                        float yawRad   = (float) Math.toRadians(connector.currentHeading());

                        Attitude attitude = Attitude.builder()
                                .timeBootMs(timeBootMs)
                                .pitch(pitchRad)
                                .roll(rollRad)
                                .yaw(yawRad)
                                .pitchspeed(0f).rollspeed(0f).yawspeed(0f)
                                .build();
                        sendMessage(connection, attitude);
                    }
                    Thread.sleep(POSITION_INTERVAL_MS);
                } catch (InterruptedException e) {
                    return;
                } catch (Exception e) {
                    System.out.println("[MAVLinkBridge] Telemetry warning: " + e.getMessage());
                }
            }
        });
        t.setDaemon(true);
        t.start();
    }

    private void startListenerThread(MavlinkConnection connection) {
        Thread t = new Thread(() -> {
            try {
                io.dronefleet.mavlink.MavlinkMessage message;
                while ((message = connection.next()) != null) {
                    Object payload = message.getPayload();

                    // Reply to param request — QGC hangs on connecting screen without this
                    if (payload instanceof ParamRequestList) {
                        ParamValue param = ParamValue.builder()
                                .paramId("SYS_AUTOSTART")
                                .paramValue(1.0f)
                                .paramType(MavParamType.MAV_PARAM_TYPE_REAL32)
                                .paramCount(1)
                                .paramIndex(0)
                                .build();
                        sendMessage(connection, param);
                    }

                    // Reply to mission request — QGC retries forever without this
                    if (payload instanceof MissionRequestList) {
                        MissionCount missionCount = MissionCount.builder()
                                .targetSystem(255)
                                .targetComponent(0)
                                .count(0)
                                .build();
                        sendMessage(connection, missionCount);
                    }

                    // ACK all commands — QGC resends without this
                    if (payload instanceof CommandLong) {
                        CommandLong cmd = (CommandLong) payload;
                        CommandAck ack = CommandAck.builder()
                                .command(cmd.command())
                                .result(MavResult.MAV_RESULT_ACCEPTED)
                                .build();
                        sendMessage(connection, ack);
                    }
                }
            } catch (Exception e) {
                System.err.println("[MAVLinkBridge] Listener ended: " + e.getMessage());
            }
        });
        t.setDaemon(true);
        t.start();
    }
}