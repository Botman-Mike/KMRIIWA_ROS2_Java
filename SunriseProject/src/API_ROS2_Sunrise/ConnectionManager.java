package API_ROS2_Sunrise;

import java.io.IOException;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import API_ROS2_Sunrise.messages.HeartbeatMessage;

public class ConnectionManager {
    // Match ROS2 node timing
    private static final int HEARTBEAT_INTERVAL_MS = 100;  // 10Hz heartbeat rate
    private static final int HEARTBEAT_TIMEOUT_MS = 1000;  // 1s timeout matches ROS2
    private static final int MAX_BACKOFF_MS = 5000;        // Max reconnect backoff
    
    private static final String HEARTBEAT_MESSAGE_PREFIX = "heartbeat";
    private static final String HEARTBEAT_DELIMITER = ",";
    
    private ScheduledExecutorService heartbeatExecutor;
    private long lastHeartbeatReceived;
    private long heartbeatsSent = 0;
    private long heartbeatsReceived = 0;
    private int consecutiveFailures = 0;
    
    // Connection fields
    private Socket tcpSocket;
    private OutputStream tcpOutputStream;
    private DatagramSocket udpSocket;
    private InetAddress udpAddress;
    private int udpPort;
    
    // Add constructor or setters for the connection objects
    public void setTCPConnection(Socket tcpSocket, OutputStream tcpOutputStream) {
        this.tcpSocket = tcpSocket;
        this.tcpOutputStream = tcpOutputStream;
    }
    
    public void setUDPConnection(DatagramSocket udpSocket, InetAddress udpAddress, int udpPort) {
        this.udpSocket = udpSocket;
        this.udpAddress = udpAddress;
        this.udpPort = udpPort;
    }
    
    public void initializeHeartbeat() {
        heartbeatExecutor = Executors.newSingleThreadScheduledExecutor();
        heartbeatExecutor.scheduleAtFixedRate(new Runnable() {
            @Override
            public void run() {
                sendHeartbeat();
                checkHeartbeatTimeout();
            }
        }, 0, HEARTBEAT_INTERVAL_MS, TimeUnit.MILLISECONDS);
    }
    
    private void sendHeartbeat() {
        try {
            HeartbeatMessage heartbeat = new HeartbeatMessage();
            byte[] message = heartbeat.toBytes();
            
            // Send via both TCP and UDP if available
            if (tcpSocket != null && tcpSocket.isConnected()) {
                tcpOutputStream.write(message);
                tcpOutputStream.flush();
                heartbeatsSent++;
            }
            
            if (udpSocket != null && !udpSocket.isClosed()) {
                DatagramPacket packet = new DatagramPacket(
                    message, 
                    message.length, 
                    udpAddress, 
                    udpPort
                );
                udpSocket.send(packet);
                heartbeatsSent++;
            }
            
            LogUtil.logInfo("heartbeat", "Heartbeat sent successfully");
        } catch (IOException e) {
            consecutiveFailures++;
            int backoff = Math.min(consecutiveFailures * 1000, MAX_BACKOFF_MS);
            LogUtil.logError("heartbeat", "Failed to send heartbeat, backing off " + backoff + "ms");
            try {
                Thread.sleep(backoff);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
            }
        }
    }
    
    private void checkHeartbeatTimeout() {
        long now = System.currentTimeMillis();
        if (now - lastHeartbeatReceived > HEARTBEAT_TIMEOUT_MS) {
            LogUtil.logInfo("Connection timeout - no heartbeat received for " + 
                          ((now - lastHeartbeatReceived)/1000.0) + " seconds");
            // Close socket to force reconnect
            if (tcpSocket != null) {
                try {
                    tcpSocket.close();
                } catch (IOException e) {
                    LogUtil.logError("Error closing TCP socket: " + e.getMessage());
                }
            }
            handleConnectionLost();
        }
    }
    
    private void handleConnectionLost() {
        // Implement reconnection logic here
        try {
            reconnect();
        } catch (Exception e) {
            LogUtil.logError("Failed to reconnect: " + e.getMessage());
        }
    }
    
    public void handleIncomingHeartbeat() {
        lastHeartbeatReceived = System.currentTimeMillis();
        heartbeatsReceived++;
        consecutiveFailures = 0;
        LogUtil.logInfo("Heartbeat received (total: " + heartbeatsReceived + ")");
    }
    
    public void reconnect() throws IOException {
        LogUtil.logInfo("reconnect", "Attempting to reconnect TCP and UDP connections...");
        // Close existing TCP socket if open
        if (tcpSocket != null) {
            try {
                tcpSocket.close();
                LogUtil.logInfo("reconnect", "Closed existing TCP socket.");
            } catch (IOException e) {
                LogUtil.logError("reconnect", "Error closing TCP socket: " + e.getMessage());
            }
            tcpSocket = null;
        }
        
        // Reinitialize TCP connection
        try {
            // IP and port must match ROS configuration (update if necessary)
            String tcpHost = "172.31.1.206";  
            int tcpPort = 30002;  
            Socket newTcpSocket = new Socket(tcpHost, tcpPort);
            newTcpSocket.setReuseAddress(true);
            setTCPConnection(newTcpSocket, newTcpSocket.getOutputStream());
            LogUtil.logInfo("reconnect", "Reconnected TCP socket successfully to " + tcpHost + ":" + tcpPort);
        } catch (IOException ex) {
            LogUtil.logError("reconnect", "TCP reconnection failed: " + ex.getMessage());
            throw ex;
        }
        
        // Reinitialize UDP connection
        try {
            // UDP config must also match ROS; update host and port as necessary
            String udpHost = "172.31.1.206";
            int udpPortConfig = 30005;
            DatagramSocket newUdpSocket = new DatagramSocket(); // use any available local port
            InetAddress address = InetAddress.getByName(udpHost);
            setUDPConnection(newUdpSocket, address, udpPortConfig);
            LogUtil.logInfo("reconnect", "Reconnected UDP socket successfully to " + udpHost + ":" + udpPortConfig);
        } catch (IOException ex) {
            LogUtil.logError("reconnect", "UDP reconnection failed: " + ex.getMessage());
            throw ex;
        }
    }
    
    // Method to validate heartbeat messages
    private boolean isValidHeartbeat(String message) {
        if (message == null || !message.startsWith(HEARTBEAT_MESSAGE_PREFIX)) {
            return false;
        }
        String[] parts = message.split(HEARTBEAT_DELIMITER);
        return parts.length == 3; // prefix,timestamp,status
    }
    
    // Add to cleanup/dispose method
    public void stopHeartbeat() {
        if (heartbeatExecutor != null) {
            heartbeatExecutor.shutdown();
            try {
                heartbeatExecutor.awaitTermination(1, TimeUnit.SECONDS);
            } catch (InterruptedException e) {
                LogUtil.logError("Error shutting down heartbeat executor: " + e.getMessage());
            }
        }
    }
}