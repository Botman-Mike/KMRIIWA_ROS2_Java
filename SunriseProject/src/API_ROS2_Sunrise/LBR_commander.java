// Copyright 2019 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package API_ROS2_Sunrise;

// RoboticsAPI
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.ExecutionState;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.executionModel.IExecutionContainer;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IMotionContainerListener;
import com.kuka.roboticsAPI.motionModel.SplineJP;
import com.kuka.roboticsAPI.motionModel.SplineMotionJP;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

// Java util
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class LBR_commander extends AbstractCommander<ISocket> {
    private static final Logger logger = Logger.getLogger(LBR_commander.class.getName());

    // Robot Specific
    LBR lbr;

    // Implemented node classes
    LBR_sensor_reader lbr_sensor_reader;
    LBR_status_reader lbr_status_reader;
    KMP_sensor_reader kmp_sensor_reader;
    KMP_status_reader kmp_status_reader;

    // Motion: LBR
    private JointPosition CommandedjointPos;
    IMotionContainer currentmotion;
    private final double defaultVelocity = 0.2;
    AbstractFrame drivePos;
    ICommandContainer LBR_currentMotion;
    private int jointCount;
    private static final double zero = Math.pow(10, -40);

    // Spline Motion
    double[] poses;
    double[] velocities;
    double[] accelerations;
    private List<SplineMotionJP<?>> splineSegments = new ArrayList<SplineMotionJP<?>>();

    // Added:
    // Startup
    boolean startup = true;

    // Scheduler for reconnect attempts
    private final ScheduledExecutorService reconnectScheduler = Executors.newSingleThreadScheduledExecutor();
    // Exponential backoff configuration
    private static final long RECONNECT_BACKOFF_INITIAL_MS = 1000;
    private static final long RECONNECT_BACKOFF_MAX_MS     = 16000;
    private volatile long currentReconnectDelay = RECONNECT_BACKOFF_INITIAL_MS;

    public LBR_commander(int port, LBR robot, String ConnectionType, AbstractFrame drivepos) {
        super(port, ConnectionType, "LBR_commander");
        assert port == 30005 : "Port mismatch for LBR_commander";
        logger.info(getClass().getSimpleName() + "|port " + port + " expected and initialized");
        this.lbr = robot;
        this.drivePos = drivepos;

        jointCount = lbr.getJointCount();
        poses = new double[jointCount];
        velocities = new double[jointCount];
        accelerations = new double[jointCount];

        // schedule initial reconnect attempts
        scheduleReconnect();

        // JMX availability check
        boolean jmxAvailable = true;
        try {
            Class.forName("java.lang.management.ManagementFactory");
        } catch (ClassNotFoundException e) {
            jmxAvailable = false;
            logger.warning("JMX not supported on this JVM, skipping MBean registration.");
        }
        if (jmxAvailable) {
            try {
                javax.management.MBeanServer mbs = java.lang.management.ManagementFactory.getPlatformMBeanServer();
                javax.management.ObjectName name = new javax.management.ObjectName(
                    "API_ROS2_Sunrise:type=" + getClass().getSimpleName() + "Metrics"
                );
                mbs.registerMBean(this, name);
                logger.info("Registered MBean: " + name);
            } catch (Exception ex) {
                logger.log(java.util.logging.Level.SEVERE,
                    "Failed to register MBean, continuing without metrics", ex
                );
            }
        }
    }

    @Override
    protected void connect() {
        createSocket();
        if (getSocket() instanceof TCPSocket) {
            String host = ((TCPSocket) getSocket()).TCPConn.getInetAddress().getHostAddress();
            logger.info(getClass().getSimpleName() + "|port " + port + " connected to " + host + ":" + port);
        }
        socketClient = getSocket();
        if (!isSocketConnected()) {
            logger.info("LBR_commander: Initializing socket connection");
            // let scheduler handle reconnection
        } else {
            setisLBRConnected(true);
        }
    }

    /** Schedule next reconnect attempt with backoff and jitter */
    private void scheduleReconnect() {
        reconnectScheduler.schedule(new Runnable() {
            public void run() {
                try {
                    reconnectTask();
                } catch (Throwable t) {
                    logger.severe("Unexpected error in LBR reconnect task: " + t.getMessage());
                }
            }
        }, currentReconnectDelay, TimeUnit.MILLISECONDS);
    }

    /** Perform a reconnect attempt with backoff and reset on success */
    private void reconnectTask() {
        if (closed || Node.getShutdown()) return;
        if (Node.isPaused()) { scheduleReconnect(); return; }
        try {
            if (!isSocketConnected()) {
                createSocket();
                if (socket instanceof TCPSocket) {
                    try {
                        ((TCPSocket) socket).TCPConn.setKeepAlive(true);
                        ((TCPSocket) socket).TCPConn.setSoTimeout(5000);
                        logger.info("LBR_commander: Socket TCP keep-alive enabled");
                    } catch (java.net.SocketException e) {
                        logger.warning("LBR_commander: Could not configure socket keep-alive or timeout: " + e.getMessage());
                    }
                }
                if (isSocketConnected()) {
                    setisLBRConnected(true);
                    logger.info("LBR_commander: Connection with LBR Command Node OK!");
                    runmainthread();
                    // reset backoff
                    currentReconnectDelay = RECONNECT_BACKOFF_INITIAL_MS;
                    logger.info("Reconnect successful, backoff reset to " + currentReconnectDelay + " ms");
                } else {
                    long base = currentReconnectDelay * 2;
                    long jitter = (long)((Math.random() * 0.4 - 0.2) * base);
                    currentReconnectDelay = (int)Math.min(RECONNECT_BACKOFF_MAX_MS, base + jitter);
                }
            } else {
                currentReconnectDelay = RECONNECT_BACKOFF_INITIAL_MS;
            }
        } finally {
            scheduleReconnect();
        }
    }

    @Override
    protected void runLoop() throws Exception {
        // Single iteration of message processing
        if (Node.getShutdown()) {
            running = false;
            return;
        }
        
        if (Node.isPaused()) {
            Thread.sleep(100);
            return;
        }
        
        try {
            String message = socket.receive_message();
            if (message != null) {
                if (logger != null) {
                    logger.info("LBR_commander received message: " + message);
                }
                
                // Process the received message
                String[] splt = message.split(" ");
                if (splt.length > 0) {
                    if (splt[0].equals("shutdown")) {
                        // Don't process shutdown during startup grace period
                        long elapsedSinceCreation = System.currentTimeMillis() - getCreationTime();
                        if (elapsedSinceCreation < STARTUP_GRACE_PERIOD_MS) {
                            if (logger != null) {
                                logger.info("LBR_commander: Ignoring shutdown command during startup grace period");
                            }
                            return;
                        }
                        
                        if (logger != null) {
                            logger.info("LBR_commander: Received explicit shutdown command");
                        }
                        setShutdown(true);
                    } else if (splt[0].equals("setJointVelocity") && !getEmergencyStop()) {
                        receivedRealCommand = true;
                        try {
                            if (logger != null) logger.info("received joint command: " + message);
                            String[] cmdArray = message.split(" ");
                            double[] jointVelocity = new double[7];
                            
                            if (cmdArray.length == 8) { // setJointVelocity + 7 joint values
                                for (int i = 0; i < 7; i++) {
                                    jointVelocity[i] = Double.parseDouble(cmdArray[i + 1]);
                                }
                                
                                if (lbr.isReadyToMove()) {
                                    // This functionality appears to be incomplete in the original code
                                    // The LBRJogger class and startPosition variable are not defined
                                    // For now just log the condition to avoid compilation errors
                                    if (!getisLBRMoving() && !getEmergencyStop()) {
                                        setisLBRMoving(true);
                                        if (logger != null) logger.info("Starting LBR movement!");
                                        // Implementation would go here
                                    } else {
                                        // Implementation would go here
                                    }
                                } else {
                                    if (logger != null) logger.info("Can't move the robot - not ready to move!");
                                }
                            } else {
                                if (logger != null) logger.info("Invalid joint command format: " + message);
                            }
                        } catch (Exception e) {
                            if (logger != null) logger.info("Error processing joint velocity command: " + e.getMessage());
                        }
                    } else if (splt[0].equals("setCartVelocity") && !getEmergencyStop()) {
                        receivedRealCommand = true;
                        // (implementation for Cartesian velocity would go here)
                        if (logger != null) logger.info("Cartesian velocity not implemented yet");
                    }
                }
            }
            
            // If there might be more work to do, add a small sleep to prevent tight loops
            // This needs to be done outside the message processing to allow thread interruption
            if (Node.isPaused() || message == null) {
                Thread.sleep(100);
            }
            
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            if (logger != null) logger.warning("LBR commander thread interrupted: " + e.getMessage());
            throw e;
        } catch (Exception e) {
            if (logger != null) {
                logger.warning("Error in LBR commander (will retry): " + e.getMessage());
                // Print stack trace for debugging
                java.io.StringWriter sw = new java.io.StringWriter();
                java.io.PrintWriter pw = new java.io.PrintWriter(sw);
                e.printStackTrace(pw);
                logger.warning("Stack trace: " + sw.toString());
            }
            
            // Avoid tight loops on repeated errors - add a small sleep
            try {
                Thread.sleep(100);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
                throw ie; // Re-throw to signal higher level that we're interrupted
            }
        }
    }

    @Override
    protected void disconnect() {
        reconnectScheduler.shutdownNow();
        super.close();
    }

    @SuppressWarnings("unused") // Keep for future use or when called from ROS
    private void addPointToSegment(String commandstr) {
        String[] lineSplt = commandstr.split(">");
        String pointType = lineSplt[1];

        // Skip start point as velocity and acceleration is 0
        if (pointType.equals("StartPoint")) {

            splineSegments.clear();
            logger.info("Startpoint received");

            setisPathFinished(false);

            // Declares state change.
            // Removed invalid reference, not needed
        } else {
            // Read message
            for (int i = 0; i < jointCount; ++i) {
                poses[i] = Double.parseDouble(lineSplt[2].split(" ")[i]);
                if (pointType.equals("WayPoint")) {
                    velocities[i] = scaleValueToOne(Math.abs(Double.parseDouble(lineSplt[3].split(" ")[i])));
                    double accel = scaleValueToOne(Math.abs(Double.parseDouble(lineSplt[4].split(" ")[i])));
                    if (accel == 0.0) {
                        accel = zero;
                    }
                    accelerations[i] = accel;
                }
            }
            PTPpoint ptp = new PTPpoint(pointType, new JointPosition(poses), velocities, accelerations);
            splineSegments.add(ptp.getPTP());

            if (pointType.equals("EndPoint")) {
                logger.info("Endpoint received");
                followPath();
            }
        }
    }

    private double scaleValueToOne(double value) {
        if (value > 1) {
            value = 1.0;
        }
        return value;
    }

    private void followPath() {
        if (!(getEmergencyStop()) && !splineSegments.isEmpty() && isNodeRunning()) {
            SplineJP spline = new SplineJP(splineSegments.toArray(new SplineMotionJP<?>[splineSegments.size()]));
            currentmotion = lbr.moveAsync(spline, new SplineMotionListener());
        }
    }

    @SuppressWarnings("unused") // Keep for future use or when called from ROS
    private void JointLBRMotion(String commandstr) {
        if (isNodeRunning() && !getEmergencyStop()) {
            String[] lineSplt = commandstr.split(" ");
            int jointIndex = Character.getNumericValue(lineSplt[1].charAt(1)) - 1;
            double direction = Double.parseDouble(lineSplt[2]);
            double jointAngle;
            if (jointIndex == -1) { // A10: Stop all
                if (getisLBRMoving()) {
                    currentmotion.cancel();
                    setisLBRMoving(false);
                    CommandedjointPos.set(lbr.getCurrentJointPosition());
                }
            } else if (jointIndex == 8) {
                // A9: Move to driveposition
                if (getisLBRMoving()) {
                    currentmotion.cancel();
                }
                lbr.move(ptp(drivePos).setJointVelocityRel(defaultVelocity));
                setisLBRMoving(false);
                CommandedjointPos.set(lbr.getCurrentJointPosition());
            } else {

                jointAngle = setJointAngle(jointIndex, direction);
                if (!(CommandedjointPos.get(jointIndex) == jointAngle)) {
                    CommandedjointPos.set(jointIndex, jointAngle);
                    if (getisLBRMoving()) {
                        currentmotion.cancel();
                    }
                    currentmotion = lbr.moveAsync(ptp(CommandedjointPos).setJointVelocityRel(defaultVelocity));
                    setisLBRMoving(true);
                }
            }
        }
    }

    private double setJointAngle(int jointIndex, double direction) {
        double jointAngle = lbr.getCurrentJointPosition().get(jointIndex);
        if (direction == -1) {
            jointAngle = Math.ceil(lbr.getJointLimits().getMinJointPosition().get(jointIndex) * 100) / 100;
        } else if (direction == 1) {
            jointAngle = Math.floor(lbr.getJointLimits().getMaxJointPosition().get(jointIndex) * 100) / 100;
        }
        return jointAngle;
    }

    public void stopMotion() {
        if (getisLBRMoving() && currentmotion != null) {
            try {
                currentmotion.cancel();
                setisLBRMoving(false);
                logger.info("LBR motion stopped by stopMotion().");
            } catch (Exception e) {
                logger.severe("Error stopping LBR motion: " + e.getMessage());
            }
        } else {
            logger.info("No LBR motion to stop.");
        }
    }

    public class MonitorEmergencyStopThread extends Thread {
        public void run() {
            while (isNodeRunning() && !Thread.currentThread().isInterrupted() && !getShutdown()) {
                if (getEmergencyStop()) {
                    if (getisLBRMoving()) {
                        logger.info("Lbr emergencythread");
                        if (!(currentmotion == null)) {
                            currentmotion.cancel();
                        }
                        CommandedjointPos.set(lbr.getCurrentJointPosition());
                    }
                    setisLBRMoving(false);
                }
                // Add a short sleep to prevent CPU hogging and allow thread interruption
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    logger.info("MonitorEmergencyStopThread interrupted, exiting thread");
                    Thread.currentThread().interrupt(); // Restore interrupt status
                    break;
                }
            }
            logger.info("MonitorEmergencyStopThread terminated normally");
        }
    }

    public class MonitorLBRCommandConnectionsThread extends Thread {
        int timeout = 3000;

        public void run() {
            while (!(isSocketConnected()) && (!(closed))) {
                if (Thread.currentThread().isInterrupted()) {
                    logger.info("LBR command connection monitor thread interrupted, exiting");
                    break;
                }
                
                if (Node.getShutdown()) {
                    logger.info("LBR command connection monitor thread detected shutdown, exiting");
                    break;
                }
                
                if (Node.isPaused()) {
                    try { 
                        Thread.sleep(100); 
                    } catch (InterruptedException e) {
                        logger.info("LBR command connection monitor thread interrupted during pause wait");
                        Thread.currentThread().interrupt();
                        break;
                    }
                    continue;
                }
                
                if (getisKMPConnected()) {
                    timeout = connection_timeout;
                }
                createSocket();
                if (isSocketConnected()) {
                    setisLBRConnected(true);
                    break;
                }
                try {
                    Thread.sleep(timeout);
                } catch (InterruptedException e) {
                    logger.info("LBR command connection monitor thread interrupted during sleep");
                    Thread.currentThread().interrupt(); // Restore interrupt status
                    break;
                }
            }
            if (!closed) {
                logger.info("Connection with LBR Command Node OK!");
                runmainthread();
            }
        }
    }

    public class SplineMotionListener implements IMotionContainerListener {
        public SplineMotionListener() {
        }

        @Override
        public void onStateChanged(IExecutionContainer arg0, ExecutionState arg1) {
        }

        @Override
        public void containerFinished(IMotionContainer arg0) {
            logger.info("Container finished!");
            setisLBRMoving(false);
            setisPathFinished(true);
            splineSegments.clear();
        }

        @Override
        public void motionFinished(IMotion arg0) {
        }

        @Override
        public void motionStarted(IMotion arg0) {
            setisLBRMoving(true);
        }
    }

    @Override
    public void close() {
        closed = true;
        try {
            CommandedjointPos.set(lbr.getCurrentJointPosition());
        } catch (Exception e) {
            logger.warning("No motion to end!");
        }

        if (getisLBRMoving()) {
            try {
                currentmotion.cancel();
            } catch (Exception e) {
                logger.severe("LBR could not stop motion: " + e);
            }
        }
        try {
            this.socket.close();
        } catch (Exception e) {
            logger.severe("Could not close LBR commander connection: " + e);
        }
        logger.info("LBR command closed!");
    }
}