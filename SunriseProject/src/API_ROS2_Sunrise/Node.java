// Copyright 2019 Nina Marie Wahl og Charlotte Heggem.
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

import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.task.ITaskLogger;
import javax.inject.Inject;

public class Node extends Thread {
    @Inject
    protected ITaskLogger logger;

    // Constants
    protected static final int MAX_RECONNECT_ATTEMPTS = 5;
    protected static final long CONNECTION_CHECK_INTERVAL = 5000; // 5 seconds
    protected static final long PAUSE_LOG_INTERVAL_MS = 2000; // 2 seconds
    protected static final int DEFAULT_CONNECTION_TIMEOUT = 600000; // 10 minutes
    protected static final long STARTUP_GRACE_PERIOD_MS = 5000; // 5 second startup grace period

    // Connection state
    protected ISocket socket = null;
    protected int port;
    protected String ConnectionType;
    protected String nodename;
    protected int connection_timeout = 2000;
    protected volatile boolean closed = false;
    protected volatile boolean isNodeRunning = true;
    protected String node_name;
    
    // Node state
    private Thread connectionMonitorThread;
    private static volatile boolean paused = false;
    private static volatile boolean shutdown = false;
    private static long lastPauseLogTime = 0;
    private volatile boolean emergencyStop = false;
    
    // Added: startup protection - tracks when this node was created
    private final long creationTime = System.currentTimeMillis();
    protected boolean receivedRealCommand = false;

    // Motion state
    private volatile boolean PathFinished = false;
    private volatile boolean isLBRmoving = false;
    private volatile boolean isKMPmoving = false;
    private volatile boolean isLBRconnected = false;
    private volatile boolean isKMPconnected = false;

    // For KMP sensor reader
    protected ISocket laser_socket;
    protected ISocket odometry_socket;
    private int KMP_laser_port;
    private int KMP_odometry_port;
    private String LaserConnectionType;
    private String OdometryConnectionType;

    protected DataController dataController;

    protected Node(int port, String ConnectionType, String nodeName) {
        this.port = port;
        this.ConnectionType = ConnectionType;
        this.nodename = nodeName;
        this.node_name = nodeName;
        initializeSocket();
    }

    protected Node(int port1, String Conn1, int port2, String Conn2, String nodeName) {
        this.KMP_laser_port = port1;
        this.KMP_odometry_port = port2;
        this.LaserConnectionType = Conn1;
        this.OdometryConnectionType = Conn2;
        this.nodename = nodeName;
        this.node_name = nodeName;
        createSocket("Laser");
        createSocket("Odom");
    }

    private void initializeSocket() {
        createSocket();
    }

    protected void createSocket() {
        try {
            if (this.ConnectionType.equals("TCP")) {
                ISocket newSocket = new TCPSocket(this.port, this.nodename);
                this.socket = newSocket;
            } else {
                ISocket newSocket = new UDPSocket(this.port, this.nodename);
                this.socket = newSocket;
            }
            if (logger != null) {
                logger.info(nodename + ": Socket initialized successfully");
            }
        } catch (Exception e) {
            if (logger != null) {
                logger.error(nodename + ": Failed to create socket: " + e.getMessage());
            }
        }
    }

    protected void createSocket(String Type) {
        if (Type.equals("Laser")) {
            if (LaserConnectionType.equals("TCP")) {
                ISocket newSocket = new TCPSocket(KMP_laser_port, this.nodename);
                this.laser_socket = newSocket;
            } else {
                ISocket newSocket = new UDPSocket(KMP_laser_port, this.nodename);
                this.laser_socket = newSocket;
            }
        } else if (Type.equals("Odom")) {
            if (OdometryConnectionType.equals("TCP")) {
                ISocket newSocket = new TCPSocket(KMP_odometry_port, this.nodename);
                this.odometry_socket = newSocket;
            } else {
                ISocket newSocket = new UDPSocket(KMP_odometry_port, this.nodename);
                this.odometry_socket = newSocket;
            }
        }
    }

    public boolean isNodeRunning() {
        return isNodeRunning;
    }

    public void runmainthread() {
        Thread nodeThread = new Thread(this, nodename + "-main");
        nodeThread.start();
    }

    @Override
    public void run() {
        try {
            while (isNodeRunning()) {
                if (Thread.currentThread().isInterrupted()) {
                    if (logger != null) logger.info(nodename + " thread was interrupted, exiting");
                    break;
                }

                try {
                    String message = socket.receive_message();
                    if (message != null) {
                        processMessage(message);
                    }
                    Thread.sleep(10);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                    if (logger != null) logger.info(nodename + " thread interrupted during sleep");
                    break;
                } catch (Exception e) {
                    if (Thread.currentThread().isInterrupted() || shutdown) {
                        break;
                    }
                    if (logger != null) logger.error(nodename + " Error processing message: " + e.getMessage());
                }
            }
        } finally {
            if (logger != null) logger.info(nodename + " thread ending");
            close();
        }
    }

    protected void processMessage(String message) {
        // Override in subclasses
    }

    public void close() {
        closed = true;
        stopConnectionMonitoring();
        if (socket != null) {
            socket.close();
        }
    }

    // Getters and setters
    public boolean isSocketConnected() {
        return this.socket != null && this.socket.isConnected();
    }

    public synchronized boolean getEmergencyStop() {
        return emergencyStop;
    }

    public synchronized void setEmergencyStop(boolean state) {
        this.emergencyStop = state;
    }

    public static boolean getShutdown() {
        return shutdown;
    }

    public void setShutdown(boolean state) {
        // Prevent immediate shutdown if within startup grace period
        if (System.currentTimeMillis() - creationTime < STARTUP_GRACE_PERIOD_MS && !receivedRealCommand) {
            if (logger != null) {
                logger.warn("Ignoring shutdown command during startup grace period");
            }
            return;
        }
        shutdown = state;
        if (logger != null) {
            logger.info("Shutdown set by " + this.nodename + " to " + state);
        }
    }

    public static void setPaused(boolean pause) {
        if (pause && !paused) {
            logInfo("=== SYSTEM PAUSED: All logging is now suppressed until resume ===");
        }
        paused = pause;
        if (!pause) {
            lastPauseLogTime = 0;
        }
    }

    public static boolean isPaused() {
        return paused;
    }

    public void setDataController(DataController controller) {
        this.dataController = controller;
    }

    // State management methods
    public boolean getisPathFinished() { return PathFinished; }
    public void setisPathFinished(boolean in) { PathFinished = in; }
    public boolean getisLBRMoving() { return isLBRmoving; }
    public void setisLBRMoving(boolean in) { isLBRmoving = in; }
    public boolean getisKMPMoving() { return isKMPmoving; }
    public void setisKMPMoving(boolean in) { isKMPmoving = in; }
    public boolean getisLBRConnected() { return isLBRconnected; }
    public void setisLBRConnected(boolean in) { isLBRconnected = in; }
    public boolean getisKMPConnected() { return isKMPconnected; }
    public void setisKMPConnected(boolean in) { isKMPconnected = in; }

    public ISocket getSocket() {
        return this.socket;
    }

    protected void startConnectionMonitoring() {
        if (connectionMonitorThread != null && connectionMonitorThread.isAlive()) {
            return;
        }

        connectionMonitorThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!closed && !Thread.currentThread().isInterrupted()) {
                    try {
                        if (!socket.isConnected()) {
                            if (logger != null) logger.warn(nodename + " Connection lost, attempting recovery");
                            int attempts = 0;
                            while (!socket.isConnected() && attempts < MAX_RECONNECT_ATTEMPTS) {
                                attempts++;
                                if (logger != null) logger.info(nodename + " Reconnection attempt " + attempts);
                                createSocket();
                                if (!socket.isConnected()) {
                                    Thread.sleep(connection_timeout);
                                }
                            }
                            if (!socket.isConnected()) {
                                if (logger != null) logger.error(nodename + " Failed to recover connection after " + MAX_RECONNECT_ATTEMPTS + " attempts");
                            }
                        }
                        Thread.sleep(CONNECTION_CHECK_INTERVAL);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    } catch (Exception e) {
                        if (logger != null) logger.error(nodename + " Error in connection monitoring: " + e.getMessage());
                    }
                }
            }
        });
        connectionMonitorThread.setName(nodename + "_monitor");

        connectionMonitorThread.setDaemon(true);
        connectionMonitorThread.start();
    }

    protected void stopConnectionMonitoring() {
        if (connectionMonitorThread != null) {
            connectionMonitorThread.interrupt();
            try {
                connectionMonitorThread.join(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            connectionMonitorThread = null;
        }
    }

    protected void ensureSocketConnection() {
        if (!isSocketConnected()) {
            if (logger != null) logger.info(nodename + ": Socket connection lost, attempting to reconnect");
            try {
                if (socket != null) {
                    try {
                        socket.close();
                    } catch (Exception e) {
                        // Ignore
                    }
                }
                createSocket();
                if (isSocketConnected()) {
                    if (logger != null) logger.info(nodename + ": Successfully reconnected socket");
                } else {
                    if (logger != null) logger.warn(nodename + ": Failed to reconnect socket");
                }
            } catch (Exception e) {
                if (logger != null) logger.error(nodename + ": Exception during socket reconnection: " + e.getMessage());
            }
        }
    }

    public void clearEmergencyStopIfSafe() {
        try {
            if (this instanceof KMP_commander) {
                KmpOmniMove kmp = ((KMP_commander)this).kmp;
                if (kmp != null && kmp.isReadyToMove() && 
                    !kmp.getSafetyState().toString().contains("WARNING_FIELD")) {
                    setEmergencyStop(false);
                    if (logger != null) logger.info("Emergency stop cleared based on safety state");
                }
            }
        } catch (Exception e) {
            if (logger != null) logger.error("Error checking if emergency stop can be cleared: " + e.getMessage());
        }
    }

    protected static void logInfo(String message) {
        // Static logging helper method
        if (!paused || (System.currentTimeMillis() - lastPauseLogTime > PAUSE_LOG_INTERVAL_MS)) {
            lastPauseLogTime = System.currentTimeMillis();
            // Note: Logger access needs to be handled carefully in static context
        }
    }
}
