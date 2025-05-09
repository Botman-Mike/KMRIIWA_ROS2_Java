package Test_Comms;

import java.io.*;
import java.net.*;
import java.util.*;
import com.kuka.task.ITaskLogger;

public class CommsTester {
    // Port assignments matching server
    static final int KMP_STATUS_TCP = 30001;
    static final int KMP_COMMAND_TCP = 30002; 
    static final int KMP_LASER_UDP = 30003;
    static final int KMP_ODOM_UDP = 30004;
    static final int LBR_COMMAND_TCP = 30005;
    static final int LBR_STATUS_TCP = 30006;
    static final int LBR_SENSOR_UDP = 30007;
    
    // Configuration
    private final String host;
    private final boolean isRobot;
    private final ITaskLogger logger;
    private List<Thread> threads;
    private volatile boolean isRunning = false;

    public CommsTester(boolean runningOnRobot, ITaskLogger logger) {
        this.isRobot = runningOnRobot;
        this.logger = logger;
        // When running on robot, we connect to ROS PC
        // When running on PC, we connect to robot
        this.host = runningOnRobot ? "172.31.1.206" : "172.31.1.147";
        this.threads = new ArrayList<Thread>();
    }

    public CommsTester(boolean runningOnRobot) {
        this(runningOnRobot, null);
    }

    public void startAll() {
        if (isRunning) return;
        isRunning = true;

        // Check ports first
        checkFirewallPorts();

        // Start TCP client threads
        startThread(new TCPClientRunnable(KMP_STATUS_TCP, "KMP_STATUS"));
        startThread(new TCPClientRunnable(KMP_COMMAND_TCP, "KMP_COMMAND"));
        startThread(new TCPClientRunnable(LBR_COMMAND_TCP, "LBR_COMMAND")); 
        startThread(new TCPClientRunnable(LBR_STATUS_TCP, "LBR_STATUS"));

        // Start UDP client threads
        startThread(new UDPClientRunnable(KMP_LASER_UDP, "KMP_LASER"));
        startThread(new UDPClientRunnable(KMP_ODOM_UDP, "KMP_ODOM"));
        startThread(new UDPClientRunnable(LBR_SENSOR_UDP, "LBR_SENSOR"));

        log("All communication tests started");
    }

    private void startThread(Runnable runnable) {
        Thread thread = new Thread(runnable);
        thread.setDaemon(true);
        thread.start();
        threads.add(thread);
    }

    public void stopAll() {
        isRunning = false;
        if (threads != null) {
            for (Thread thread : threads) {
                thread.interrupt();
                try {
                    thread.join(5000); // Wait up to 5 seconds per thread
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            threads.clear();
        }
        log("All communication tests stopped");
    }

    private void checkFirewallPorts() {
        log("Checking firewall status for all ports...");
        
        // Check TCP ports
        int[] tcpPorts = new int[]{KMP_STATUS_TCP, KMP_COMMAND_TCP, LBR_COMMAND_TCP, LBR_STATUS_TCP};
        for (int i = 0; i < tcpPorts.length; i++) {
            Socket socket = null;
            try {
                socket = new Socket();
                socket.connect(new InetSocketAddress(host, tcpPorts[i]), 1000);
                log("TCP Port " + tcpPorts[i] + " is open");
            } catch (Exception e) {
                logError("WARNING: TCP Port " + tcpPorts[i] + " appears to be blocked. Error: " + e.getMessage());
                logError("Please check your firewall settings and ensure the port is allowed.");
            } finally {
                if (socket != null) {
                    try {
                        socket.close();
                    } catch (IOException e) {
                        // Ignore close errors
                    }
                }
            }
        }

        // Check UDP ports
        int[] udpPorts = new int[]{KMP_LASER_UDP, KMP_ODOM_UDP, LBR_SENSOR_UDP};
        for (int i = 0; i < udpPorts.length; i++) {
            DatagramSocket socket = null;
            try {
                socket = new DatagramSocket();
                socket.connect(InetAddress.getByName(host), udpPorts[i]);
                // Send test packet
                byte[] data = "test".getBytes();
                DatagramPacket packet = new DatagramPacket(data, data.length);
                socket.send(packet);
                log("UDP Port " + udpPorts[i] + " is accessible");
            } catch (Exception e) {
                logError("WARNING: UDP Port " + udpPorts[i] + " appears to be blocked. Error: " + e.getMessage());
                logError("Please check your firewall settings and ensure the port is allowed.");
            } finally {
                if (socket != null) {
                    socket.close();
                }
            }
        }
        log("Port check complete.\n");
    }

    private class TCPClientRunnable implements Runnable {
        private final int port;
        private final String name;

        public TCPClientRunnable(int port, String name) {
            this.port = port;
            this.name = name;
        }

        @Override
        public void run() {
            while (isRunning && !Thread.currentThread().isInterrupted()) {
                Socket socket = null;
                try {
                    log(name + " attempting connection to port " + port);
                    socket = new Socket();
                    socket.connect(new InetSocketAddress(host, port), 15000); // 15s connection timeout
                    socket.setSoTimeout(15000); // 15s read timeout
                    log(name + " connected to port " + port);
                    
                    final BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                    final BufferedWriter out = new BufferedWriter(new OutputStreamWriter(socket.getOutputStream()));

                    // Start heartbeat thread using Java 6 compatible anonymous inner class
                    Thread heartbeat = new Thread(new Runnable() {
                        public void run() {
                            while (!Thread.currentThread().isInterrupted()) {
                                try {
                                    // Fix: Add a space between the length prefix and the heartbeat message
                                    String heartbeatMsg = "heartbeat";
                                    String lengthPrefix = String.format("%010d", heartbeatMsg.length());
                                    out.write(lengthPrefix + " " + heartbeatMsg);
                                    out.flush();
                                    Thread.sleep(1000);
                                } catch (Exception e) {
                                    break;
                                }
                            }
                        }
                    });
                    heartbeat.setDaemon(true);
                    heartbeat.start();

                    String line;
                    while (isRunning && (line = in.readLine()) != null) {
                        log(name + " received: " + line);
                    }

                    heartbeat.interrupt();
                    socket.close();
                } catch (IOException e) {
                    log(name + " waiting for server... will retry in 5 seconds");
                    try {
                        Thread.sleep(5000);
                    } catch (InterruptedException ie) {
                        break;
                    }
                } finally {
                    if (socket != null) {
                        try {
                            socket.close();
                        } catch (IOException e) {
                            // Ignore close errors
                        }
                    }
                }
            }
        }
    }

    private class UDPClientRunnable implements Runnable {
        private final int port;
        private final String name;

        public UDPClientRunnable(int port, String name) {
            this.port = port;
            this.name = name;
        }

        @Override
        public void run() {
            DatagramSocket socket = null;
            try {
                socket = new DatagramSocket();
                log(name + " started on port " + port);
                socket.setSoTimeout(15000); // 15s read timeout
                
                // Start heartbeat thread
                final DatagramSocket finalSocket = socket;
                Thread heartbeat = new Thread(new Runnable() {
                    public void run() {
                        try {
                            while (!Thread.currentThread().isInterrupted() && isRunning) {
                                byte[] msg = "heartbeat".getBytes();
                                DatagramPacket packet = new DatagramPacket(msg, msg.length,
                                    InetAddress.getByName(host), port);
                                finalSocket.send(packet);
                                Thread.sleep(5000);
                            }
                        } catch (Exception e) {
                            logError(name + " heartbeat error: " + e.getMessage());
                        }
                    }
                });
                heartbeat.setDaemon(true);
                heartbeat.start();

                // Receive responses
                byte[] buf = new byte[65535];
                DatagramPacket packet = new DatagramPacket(buf, buf.length);
                
                while (isRunning && !Thread.currentThread().isInterrupted()) {
                    try {
                        socket.receive(packet);
                        String received = new String(packet.getData(), 0, packet.getLength());
                        log(name + " received: " + received);
                        
                        // Send test message
                        String msg = "test_" + name + "_msg";
                        packet = new DatagramPacket(msg.getBytes(), msg.length(),
                            InetAddress.getByName(host), port);
                        socket.send(packet);
                    } catch (SocketTimeoutException e) {
                        // Timeout is expected, continue
                    }
                }
            } catch (IOException e) {
                logError(name + " error: " + e.getMessage());
            } finally {
                if (socket != null) {
                    socket.close();
                }
            }
        }
    }

    private void log(String message) {
        if (isRobot && logger != null) {
            logger.info(message);
        } else {
            System.out.println(message);
        }
    }

    private void logError(String message) {
        if (isRobot && logger != null) {
            logger.error(message);
        } else {
            System.err.println(message);
        }
    }
}