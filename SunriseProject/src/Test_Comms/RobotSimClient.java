package Test_Comms;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

public class RobotSimClient {
    // Define ports matching KMRiiwaSunriseApplication
    private static final int KMP_STATUS_PORT = 30001;
    private static final int KMP_COMMAND_PORT = 30002;
    private static final int KMP_LASER_PORT = 30003;
    private static final int KMP_ODOMETRY_PORT = 30004;
    private static final int LBR_COMMAND_PORT = 30005;
    private static final int LBR_STATUS_PORT = 30006;
    private static final int LBR_SENSOR_PORT = 30007;

    private ISocket kmpStatusSocket;
    private ISocket kmpCommandSocket;
    private ISocket kmpLaserSocket;
    private ISocket kmpOdomSocket;
    private ISocket lbrCommandSocket;
    private ISocket lbrStatusSocket;
    private ISocket lbrSensorSocket;

    private final List<Thread> communicationThreads = new ArrayList<Thread>();
    private volatile boolean running = true;

    public void startCommunication() {
        // First check all ports
        checkPorts();
        System.out.println("\nAll communication tests started");

        // Initialize TCP sockets for command and status
        initializeTCPSockets();

        // Initialize UDP sockets for sensor data
        initializeUDPSockets();

        // Start communication threads
        startThreads();

        // Wait for enter to exit
        try {
            System.out.println("RobotSimClient running. Press Enter to exit.");
            new BufferedReader(new InputStreamReader(System.in)).readLine();
        } catch (Exception e) {
            System.err.println("Error reading input: " + e.getMessage());
        }

        shutdown();
    }

    private void checkPorts() {
        // Check TCP ports
        checkTCPPort(KMP_STATUS_PORT);
        checkTCPPort(KMP_COMMAND_PORT);
        checkTCPPort(LBR_COMMAND_PORT);
        checkTCPPort(LBR_STATUS_PORT);

        // Check UDP ports
        checkUDPPort(KMP_LASER_PORT);
        checkUDPPort(KMP_ODOMETRY_PORT);
        checkUDPPort(LBR_SENSOR_PORT);

        System.out.println("Port check complete.\n");
    }

    private void initializeTCPSockets() {
        kmpStatusSocket = new TCPSocket(KMP_STATUS_PORT, "KMP_STATUS");
        kmpCommandSocket = new TCPSocket(KMP_COMMAND_PORT, "KMP_COMMAND");
        lbrCommandSocket = new TCPSocket(LBR_COMMAND_PORT, "LBR_COMMAND");
        lbrStatusSocket = new TCPSocket(LBR_STATUS_PORT, "LBR_STATUS");
    }

    private void initializeUDPSockets() {
        kmpLaserSocket = new UDPSocket(KMP_LASER_PORT, "KMP_LASER");
        kmpOdomSocket = new UDPSocket(KMP_ODOMETRY_PORT, "KMP_ODOM");
        lbrSensorSocket = new UDPSocket(LBR_SENSOR_PORT, "LBR_SENSOR");

        // Set larger buffer for sensor data
        ((UDPSocket)lbrSensorSocket).setReceiveBufferSize(65535);
    }

    private void startThreads() {
        // TCP Command/Status threads
        startCommunicationThread(kmpStatusSocket);
        startCommunicationThread(kmpCommandSocket);
        startCommunicationThread(lbrCommandSocket);
        startCommunicationThread(lbrStatusSocket);

        // UDP Sensor threads
        startCommunicationThread(kmpLaserSocket);
        startCommunicationThread(kmpOdomSocket);
        startCommunicationThread(lbrSensorSocket);
    }

    private void startCommunicationThread(ISocket socket) {
        Thread thread = new Thread(new CommunicationThread(socket));
        thread.setDaemon(true);
        thread.start();
        communicationThreads.add(thread);
    }

    private void checkTCPPort(int port) {
        java.net.Socket socket = null;
        try {
            socket = new java.net.Socket("172.31.1.147", port);
            System.out.println("TCP Port " + port + " is accessible");
        } catch (Exception e) {
            System.out.println("WARNING: TCP Port " + port + " appears to be blocked. Error: " + e.getMessage());
            System.out.println("Please check your firewall settings and ensure the port is allowed.");
        } finally {
            if (socket != null) {
                try {
                    socket.close();
                } catch (Exception e) {
                    // Ignore close errors
                }
            }
        }
    }

    private void checkUDPPort(int port) {
        java.net.DatagramSocket socket = null;
        try {
            socket = new java.net.DatagramSocket();
            socket.connect(java.net.InetAddress.getByName("172.31.1.147"), port);
            System.out.println("UDP Port " + port + " is accessible");
        } catch (Exception e) {
            System.out.println("WARNING: UDP Port " + port + " appears to be blocked. Error: " + e.getMessage());
            System.out.println("Please check your firewall settings and ensure the port is allowed.");
        } finally {
            if (socket != null) {
                socket.close();
            }
        }
    }

    private void shutdown() {
        running = false;
        
        // Wait for threads to finish
        for (Thread thread : communicationThreads) {
            try {
                thread.interrupt();
                thread.join(5000); // Wait up to 5 seconds for each thread
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        communicationThreads.clear();

        // Close all sockets
        closeSocket(kmpStatusSocket, "KMP Status");
        closeSocket(kmpCommandSocket, "KMP Command");
        closeSocket(kmpLaserSocket, "KMP Laser");
        closeSocket(kmpOdomSocket, "KMP Odometry");
        closeSocket(lbrCommandSocket, "LBR Command");
        closeSocket(lbrStatusSocket, "LBR Status");
        closeSocket(lbrSensorSocket, "LBR Sensor");

        System.out.println("RobotSimClient shutdown complete");
    }

    private void closeSocket(ISocket socket, String name) {
        if (socket != null) {
            try {
                socket.close();
            } catch (Exception e) {
                System.err.println("Error closing " + name + " socket: " + e.getMessage());
            }
        }
    }

    private class CommunicationThread implements Runnable {
        private final ISocket socket;

        public CommunicationThread(ISocket socket) {
            this.socket = socket;
        }

        @Override
        public void run() {
            while (running && !Thread.currentThread().isInterrupted()) {
                try {
                    String message = socket.receive_message();
                    if (message != null) {
                        // Process received message
                        // For testing, we just echo back
                        socket.send_message("Echo: " + message);
                    }
                    Thread.sleep(10); // Small delay to prevent busy waiting
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    if (running) {
                        System.err.println("Error in communication thread: " + e.getMessage());
                    }
                }
            }
        }
    }

    public static void main(String[] args) {
        new RobotSimClient().startCommunication();
    }
}
