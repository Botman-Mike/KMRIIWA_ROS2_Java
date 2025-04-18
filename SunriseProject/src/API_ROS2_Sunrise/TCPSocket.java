package API_ROS2_Sunrise;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.IOException;
import java.io.InterruptedIOException;
import java.net.SocketTimeoutException;
import java.net.SocketException;
import java.io.EOFException;
import java.net.Socket;
import java.net.InetSocketAddress;
import java.nio.charset.Charset;
import com.kuka.task.ITaskLogger;
import javax.inject.Inject;

public class TCPSocket implements ISocket {
    private volatile boolean isConnected = false;
    private Thread heartbeatThread;

    public Socket TCPConn;
    public PrintWriter outputStream;
    public BufferedReader inputStream;
    int COMport;
    String nodename;
    private boolean running = true;
    private long lastHeartbeat = System.currentTimeMillis();
    private static final long HEARTBEAT_TIMEOUT = 15000; // 15 second timeout
    private volatile boolean heartbeatRunning = false;
    private static final Charset UTF8_CHARSET = Charset.forName("UTF-8");
    private static final int CONNECTION_TIMEOUT = 5000; // Reduced to 5 seconds
    private static final int SO_TIMEOUT = 1000; // Separate socket read timeout
    private static final int MAX_RECONNECT_ATTEMPTS = 3;
    private static final long RECONNECT_DELAY = 2000; // 2 second delay between reconnection attempts

    // Default remote IP, can be overridden
    private static String DEFAULT_REMOTE_IP = "172.31.1.206";
    private String remoteIP;

    @Inject
    private ITaskLogger logger;

    public TCPSocket(int port, String node_name) {
        this(port, node_name, DEFAULT_REMOTE_IP);
    }

    public TCPSocket(int port, String node_name, String remoteIP) {
        isConnected = false;
        COMport = port;
        this.nodename = node_name;
        this.remoteIP = remoteIP;
        TCPConn = connect();
    }

    public Socket connect() {
        int attempts = 0;
        while (attempts < MAX_RECONNECT_ATTEMPTS && !Thread.currentThread().isInterrupted()) {
            try {
                if (logger != null) {
                    logger.info(this.nodename + " attempting connection to ROS over TCP on " + remoteIP + ":" + COMport + 
                              " (attempt " + (attempts + 1) + "/" + MAX_RECONNECT_ATTEMPTS + ")");
                }
                
                if (TCPConn != null) {
                    try {
                        TCPConn.close();
                    } catch (Exception e) {
                        // Ignore close errors
                    }
                }

                TCPConn = new Socket();
                TCPConn.setReuseAddress(true);
                TCPConn.setTcpNoDelay(true); // Disable Nagle's algorithm
                TCPConn.setSoTimeout(SO_TIMEOUT); // Timeout for read operations only
                
                // Connect with specific timeout
                TCPConn.connect(new InetSocketAddress(remoteIP, COMport), CONNECTION_TIMEOUT);
                
                outputStream = new PrintWriter(TCPConn.getOutputStream(), true);
                inputStream = new BufferedReader(new InputStreamReader(TCPConn.getInputStream()));
                
                isConnected = true;
                lastHeartbeat = System.currentTimeMillis(); // Reset heartbeat timer on connection
                
                if (logger != null) {
                    logger.info(this.nodename + " TCP connection established successfully");
                }
                return TCPConn;
            } catch (IOException e) {
                attempts++;
                if (attempts < MAX_RECONNECT_ATTEMPTS) {
                    if (logger != null) {
                        logger.warn(this.nodename + " connection attempt " + attempts + " failed: " + e.getMessage() + 
                                  ". Retrying in " + (RECONNECT_DELAY/1000) + " seconds...");
                    }
                    try {
                        Thread.sleep(RECONNECT_DELAY);
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                } else {
                    if (logger != null) {
                        logger.error(this.nodename + " failed to connect after " + MAX_RECONNECT_ATTEMPTS + 
                                   " attempts. Last error: " + e.getMessage());
                    }
                }
                isConnected = false;
            }
        }
        return null;
    }

    public void send_message(String buffer) {
        if (!isConnected()) {
            if (logger != null) logger.warn(nodename + " attempting to reconnect before sending message");
            TCPConn = connect();
            if (!isConnected()) {
                if (logger != null) logger.error(nodename + " failed to reconnect, message not sent");
                return;
            }
        }
        
        try {
            int len = (this.encode(buffer)).length;
            String send_string = String.format("%010d", len) + " " + buffer;
            outputStream.write(send_string);
            outputStream.flush();
        } catch (Exception e) {
            if (logger != null) logger.error(nodename + " error sending message: " + e.getMessage());
            isConnected = false;
        }
    }

    public boolean isConnected() {
        if (TCPConn == null) {
            if (logger != null) logger.warn(nodename + " TCP Socket check: Socket is null");
            return false;
        }

        try {
            if (TCPConn.isClosed()) {
                if (logger != null) logger.warn(nodename + " TCP Socket check: Socket is closed");
                return false;
            }

            if (!TCPConn.isConnected()) {
                if (logger != null) logger.warn(nodename + " TCP Socket check: Socket not connected");
                return false;
            }

            if (TCPConn.isInputShutdown() || TCPConn.isOutputShutdown()) {
                if (logger != null) logger.warn(nodename + " TCP Socket check: Input or output is shutdown");
                return false;
            }

            return (System.currentTimeMillis() - lastHeartbeat) < HEARTBEAT_TIMEOUT;
        } catch (Exception e) {
            if (logger != null) logger.error(nodename + " TCP Socket check exception: " + e.getMessage());
            return false;
        }
    }

    public String receive_message() {
        if (!isConnected()) {
            try {
                if (logger != null) logger.warn(nodename + " TCP Socket lost, attempting reconnection");
                connect();
                if (!isConnected()) {
                    return null;
                }
            } catch (Exception e) {
                if (logger != null) logger.error(nodename + " TCP Socket reconnection failed: " + e.getMessage());
                return null;
            }
        }

        try {
            TCPConn.setSoTimeout(1000);

            BufferedReader reader = new BufferedReader(new InputStreamReader(TCPConn.getInputStream()));
            String message = reader.readLine();
            if (message != null && message.trim().equals("heartbeat")) {
                updateLastHeartbeat();
                return null;
            }
            return message;
        } catch (SocketTimeoutException e) {
            return null;
        } catch (Exception e) {
            if (running && !(e instanceof InterruptedIOException)) {
                if (logger != null) logger.error(nodename + " TCP receive error: " + e.getMessage());

                if (e instanceof SocketException ||
                        e instanceof EOFException ||
                        (e.getMessage() != null && e.getMessage().contains("Connection reset"))) {

                    if (logger != null) logger.warn(nodename + " TCP Connection appears broken, forcing reconnection");
                    try {
                        TCPConn.close();
                    } catch (Exception ex) {
                    }
                    TCPConn = null;
                }
            }
            return null;
        }
    }

    public void close() {
        stopHeartbeatThread();
        if (logger != null) logger.info(nodename + " TCP connection closing");
        running = false;

        if (TCPConn != null) {
            try {
                TCPConn.close();
                if (logger != null) logger.info(nodename + " TCP socket closed successfully");
            } catch (Exception e) {
                if (logger != null) logger.error(nodename + " TCP socket close error: " + e.getMessage());
            }
        }
    }

    public void sendHeartbeat() {
        if (!isConnected || outputStream == null) {
            if (logger != null) logger.warn(nodename + " Cannot send heartbeat - socket disconnected");
            return;
        }
        String heartbeatMsg = "heartbeat";
        String formattedMsg = String.format("%010d", heartbeatMsg.length()) + " " + heartbeatMsg;
        try {
            outputStream.write(formattedMsg);
            outputStream.flush();
        } catch (Exception e) {
            if (logger != null) {
                logger.error(nodename + " TCP heartbeat error: " + e.getMessage());
                // Try to recover connection
                if (!isConnected()) {
                    logger.info(nodename + " Attempting to recover connection after heartbeat failure");
                    TCPConn = connect();
                }
            }
        }
    }

    public void startHeartbeatThread() {
        if (heartbeatThread != null && heartbeatThread.isAlive()) {
            if (logger != null) logger.info(nodename + " Heartbeat thread already running");
            return;
        }

        heartbeatRunning = true;
        heartbeatThread = new Thread(new Runnable() {
            public void run() {
                if (logger != null) logger.info(nodename + " Starting heartbeat thread");
                int consecutiveFailures = 0;
                
                while (heartbeatRunning && !Thread.currentThread().isInterrupted()) {
                    try {
                        sendHeartbeat();
                        // Reset failure counter on successful heartbeat
                        if (consecutiveFailures > 0) {
                            consecutiveFailures = 0;
                            if (logger != null) logger.info(nodename + " Heartbeat connection recovered");
                        }
                        Thread.sleep(5000);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    } catch (Exception e) {
                        consecutiveFailures++;
                        if (logger != null) logger.error(nodename + " Heartbeat error: " + e.getMessage() + 
                                                       " (Failure #" + consecutiveFailures + ")");
                        // If too many consecutive failures, try to reconnect
                        if (consecutiveFailures >= 3) {
                            if (logger != null) logger.warn(nodename + " Multiple heartbeat failures, forcing reconnection");
                            TCPConn = connect();
                            consecutiveFailures = 0;
                        }
                        try {
                            Thread.sleep(1000); // Brief pause before retry
                        } catch (InterruptedException ie) {
                            Thread.currentThread().interrupt();
                            break;
                        }
                    }
                }
                if (logger != null) logger.info(nodename + " Heartbeat thread stopped");
            }
        }, nodename + "-heartbeat");
        heartbeatThread.setDaemon(true);
        heartbeatThread.start();
    }

    public void stopHeartbeatThread() {
        heartbeatRunning = false;
        if (heartbeatThread != null) {
            if (logger != null) logger.info(nodename + " Stopping heartbeat thread");
            heartbeatThread.interrupt();
            try {
                heartbeatThread.join(1000); // Wait up to 1 second for clean shutdown
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            heartbeatThread = null;
        }
    }

    public String decode(byte[] data) {
        String message = new String(data, 0, data.length, UTF8_CHARSET);
        return message;
    }

    public byte[] encode(String string) {
        return string.getBytes(UTF8_CHARSET);
    }

    public long getLastHeartbeat() {
        return lastHeartbeat;
    }

    public void updateLastHeartbeat() {
        lastHeartbeat = System.currentTimeMillis();
    }
}
