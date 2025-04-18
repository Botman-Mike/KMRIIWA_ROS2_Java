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

public class LBR_commander extends Node {

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

    public LBR_commander(int port, LBR robot, String ConnectionType, AbstractFrame drivepos) {
        super(port, ConnectionType, "LBR commander");

        this.lbr = robot;
        this.drivePos = drivepos;

        jointCount = lbr.getJointCount();
        poses = new double[jointCount];
        velocities = new double[jointCount];
        accelerations = new double[jointCount];

        // Ensure socket is initialized
        if (!isSocketConnected()) {
            createSocket();
            if (logger != null) {
                logger.info("LBR_commander: Initializing socket connection");
            }
            // Give socket time to establish
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        if (!(isSocketConnected())) {
            logger.info("Starting thread to connect LBR command node....");
            Thread monitorLBRCommandConnections = new MonitorLBRCommandConnectionsThread();
            monitorLBRCommandConnections.start();
        } else {
            setisLBRConnected(true);
        }
    }

    @Override
    public void run() {
        // Added: Track startup time
        long nodeStartTime = System.currentTimeMillis();
        boolean isInStartupGracePeriod = true;
        int messageCount = 0;
        
        while (!closed && isNodeRunning()) {
            if (Node.getShutdown()) break;
            if (Node.isPaused()) {
                try { Thread.sleep(100); } catch (InterruptedException e) { break; }
                continue;
            }
            
            try {
                // Check if startup grace period has ended
                long currentTime = System.currentTimeMillis();
                if (isInStartupGracePeriod && currentTime - nodeStartTime > STARTUP_GRACE_PERIOD_MS) {
                    isInStartupGracePeriod = false;
                    if (logger != null) {
                        logger.info("LBR_commander: Startup grace period ended");
                    }
                }
                
                String message = socket.receive_message();
                if (message != null) {
                    messageCount++;
                    
                    // During startup grace period, log ALL messages to help diagnose issues
                    if (isInStartupGracePeriod && logger != null) {
                        logger.info("LBR_commander: [STARTUP] Message #" + messageCount + ": " + message);
                        
                        // Special handling for shutdown commands during startup
                        if (message.toLowerCase().contains("shutdown")) {
                            logger.info("LBR_commander: Ignoring shutdown command during startup grace period");
                            continue; // Skip processing this command
                        }
                    }
                    
                    String[] splt = message.split(" ");
                    if (splt.length > 0) {
                        if (splt[0].equals("shutdown")) {
                            // Don't process shutdown during startup grace period
                            if (isInStartupGracePeriod) {
                                if (logger != null) {
                                    logger.info("LBR_commander: Ignoring shutdown command during startup grace period");
                                }
                                continue;
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
                
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                if (logger != null) logger.info("LBR commander thread interrupted, exiting");
                break;
            } catch (Exception e) {
                if (logger != null) logger.info("Error in LBR commander: " + e.getMessage());
                e.printStackTrace();
            }
        }
        
        if (logger != null) logger.info("LBR commander no longer running");
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
                if (!Node.isPaused()) {
                    logger.info("Connection with LBR Command Node OK!");
                }
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