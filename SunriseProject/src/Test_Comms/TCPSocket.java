package Test_Comms;

import java.io.*;
import java.net.*;

public class TCPSocket implements ISocket {
    private volatile boolean isConnected = false;
    private Socket TCPConn;
    private PrintWriter outputStream;
    private BufferedReader inputStream;
    private final int COMport;
    private final String nodename;
    private boolean running = true;
    private long lastHeartbeat = System.currentTimeMillis();
    private static final long HEARTBEAT_TIMEOUT = 15000; // 15 second timeout
    private volatile boolean heartbeatRunning = false;
    private Thread heartbeatThread;
    private static final int MAX_RECONNECT_ATTEMPTS = 3;
    private static final long RECONNECT_DELAY = 2000; // 2 second delay between reconnection attempts
    private static final int CONNECTION_TIMEOUT = 5000; // 5 second connection timeout
    
    // TCP keep-alive configuration
    private static final boolean KEEP_ALIVE_ENABLED = true; // Enable TCP keep-alive
    // Note: Advanced keep-alive parameters like idle time, interval, and count
    // cannot be configured through standard Java Socket API
    
    private final String remoteIP;

    public TCPSocket(int port, String node_name) {
        this(port, node_name, "172.31.1.206"); // Updated to match production IP
    }

    public TCPSocket(int port, String node_name, String remoteIP) {
        this.COMport = port;
        this.nodename = node_name;
        this.remoteIP = remoteIP;
        TCPConn = connect();
        if (TCPConn != null) {
            startHeartbeatThread();
        }
    }

    public Socket connect() {
        int attempts = 0;
        while (attempts < MAX_RECONNECT_ATTEMPTS && !Thread.currentThread().isInterrupted()) {
            try {
                System.out.println(this.nodename + " attempting connection to Robot over TCP on " + remoteIP + ":" + COMport + 
                                 " (attempt " + (attempts + 1) + "/" + MAX_RECONNECT_ATTEMPTS + ")");
                
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
                TCPConn.setSoTimeout(1000); // Read timeout
                
                // Set TCP keep-alive options
                if (KEEP_ALIVE_ENABLED) {
                    TCPConn.setKeepAlive(true);
                    System.out.println(nodename + " TCP keep-alive enabled");
                    
                    // Note: Advanced keep-alive parameters require Java 9+ and only work on some platforms
                    // If you need more control over keep-alive parameters, consider platform-specific solutions
                }
                
                // Connect with timeout
                TCPConn.connect(new InetSocketAddress(remoteIP, COMport), CONNECTION_TIMEOUT);
                
                outputStream = new PrintWriter(TCPConn.getOutputStream(), true);
                inputStream = new BufferedReader(new InputStreamReader(TCPConn.getInputStream()));
                
                isConnected = true;
                lastHeartbeat = System.currentTimeMillis(); // Reset heartbeat timer on connection
                System.out.println(this.nodename + " TCP connection established successfully");
                return TCPConn;
            } catch (IOException e) {
                attempts++;
                if (attempts < MAX_RECONNECT_ATTEMPTS) {
                    System.out.println(this.nodename + " connection attempt " + attempts + " failed: " + e.getMessage() + 
                                    ". Retrying in " + (RECONNECT_DELAY/1000) + " seconds...");
                    try {
                        Thread.sleep(RECONNECT_DELAY);
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                } else {
                    System.err.println("Could not connect " + this.nodename + " to Robot over TCP after " + MAX_RECONNECT_ATTEMPTS + 
                                     " attempts. Last error: " + e.getMessage());
                }
                isConnected = false;
            }
        }
        return null;
    }

    @Override
    public void send_message(String buffer) {
        if (!isConnected() || isHeartbeatTimedOut()) {
            try {
                System.out.println(nodename + " TCP Socket lost or heartbeat timed out, attempting reconnection");
                connect();
                if (!isConnected()) {
                    return;
                }
            } catch (Exception e) {
                System.err.println(nodename + " TCP Socket reconnection failed: " + e.getMessage());
                return;
            }
        }

        try {
            // Format the message with length prefix
            String formattedMessage = formatMessage(buffer);
            outputStream.println(formattedMessage);
            outputStream.flush();
        } catch (Exception e) {
            System.err.println(nodename + " TCP send error: " + e.getMessage());
            handleConnectionError();
        }
    }

    @Override
    public String receive_message() {
        if (!isConnected() || isHeartbeatTimedOut()) {
            try {
                System.out.println(nodename + " TCP Socket lost or heartbeat timed out, attempting reconnection");
                connect();
                if (!isConnected()) {
                    return null;
                }
            } catch (Exception e) {
                System.err.println(nodename + " TCP Socket reconnection failed: " + e.getMessage());
                return null;
            }
        }

        try {
            TCPConn.setSoTimeout(1000);
            String message = inputStream.readLine();
            
            // Check for null message
            if (message == null) {
                return null;
            }
            
            // Use our helper function to check if this is a heartbeat message
            if (isHeartbeatMessage(message)) {
                // Only log heartbeats occasionally to avoid filling logs
                if (System.currentTimeMillis() % 60000 < 1000) { 
                    System.out.println(nodename + " Received heartbeat message");
                }
                updateLastHeartbeat();
                return null;
            }
            
            // For non-heartbeat messages, check if we need to extract content
            // This ensures compatibility with both prefixed and non-prefixed formats
            String content = extractMessageContent(message);
            if (content != null) {
                return content; // Return just the message content without prefix
            }
            
            return message; // Return the original message if not in prefixed format
            
        } catch (SocketTimeoutException e) {
            return null;
        } catch (Exception e) {
            if (running && !(e instanceof InterruptedIOException)) {
                System.err.println(nodename + " TCP receive error: " + e.getMessage());
                handleConnectionError();
            }
            return null;
        }
    }

    private void handleConnectionError() {
        if (TCPConn != null) {
            try {
                TCPConn.close();
            } catch (Exception ex) {
                // Ignore close errors
            }
            TCPConn = null;
        }
        isConnected = false;
    }

    @Override
    public boolean isConnected() {
        return isConnected && TCPConn != null && TCPConn.isConnected();
    }

    @Override
    public void close() {
        stopHeartbeatThread();
        System.out.println(nodename + " TCP connection closing");
        running = false;

        if (TCPConn != null) {
            try {
                TCPConn.close();
                isConnected = false;
                System.out.println(nodename + " TCP socket closed successfully");
            } catch (Exception e) {
                System.err.println(nodename + " TCP socket close error: " + e.getMessage());
            }
        }
    }

    @Override
    public byte[] encode(String string) {
        try {
            return string.getBytes("UTF-8");
        } catch (java.io.UnsupportedEncodingException e) {
            System.err.println(nodename + " Encoding error: " + e.getMessage());
            return string.getBytes();
        }
    }

    @Override
    public String decode(byte[] data) {
        try {
            return new String(data, "UTF-8");
        } catch (java.io.UnsupportedEncodingException e) {
            System.err.println(nodename + " Decoding error: " + e.getMessage());
            return new String(data);
        }
    }

    public void sendHeartbeat() {
        send_message("heartbeat");
    }

    private void updateLastHeartbeat() {
        lastHeartbeat = System.currentTimeMillis();
    }

    public void startHeartbeatThread() {
        if (heartbeatThread != null && heartbeatThread.isAlive()) {
            return;
        }

        heartbeatRunning = true;
        heartbeatThread = new Thread(new Runnable() {
            public void run() {
                while (heartbeatRunning && !Thread.currentThread().isInterrupted()) {
                    try {
                        sendHeartbeat();
                        Thread.sleep(5000);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    } catch (Exception e) {
                        System.err.println(nodename + " Heartbeat error: " + e.getMessage());
                    }
                }
            }
        });
        heartbeatThread.setDaemon(true);
        heartbeatThread.start();
    }

    public void stopHeartbeatThread() {
        heartbeatRunning = false;
        if (heartbeatThread != null) {
            heartbeatThread.interrupt();
            try {
                heartbeatThread.join(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public void setReceiveBufferSize(int size) {
        try {
            if (TCPConn != null) {
                TCPConn.setReceiveBufferSize(size);
            }
        } catch (Exception e) {
            System.err.println("Could not set TCP receive buffer size: " + e.getMessage());
        }
    }

    public long getLastHeartbeat() {
        return lastHeartbeat;
    }

    private boolean isHeartbeatTimedOut() {
        return System.currentTimeMillis() - lastHeartbeat > HEARTBEAT_TIMEOUT;
    }

    /**
     * Format a message with the standard length prefix format
     * @param message The raw message to format
     * @return The formatted message with length prefix
     */
    private String formatMessage(String message) {
        byte[] bytes = encode(message);
        int len = bytes.length;
        return String.format("%010d", len) + " " + message;
    }
    
    /**
     * Extract the actual message content from a prefixed message
     * @param prefixedMessage The message with length prefix
     * @return The extracted message content, or null if format is invalid
     */
    private String extractMessageContent(String prefixedMessage) {
        if (prefixedMessage == null || prefixedMessage.length() < 12) {
            return null;
        }
        
        try {
            // Standard format is: <10-digit-length> <space> <message>
            // So we split on the first space
            int firstSpacePos = prefixedMessage.indexOf(" ");
            if (firstSpacePos > 0) {
                return prefixedMessage.substring(firstSpacePos + 1);
            }
        } catch (Exception e) {
            System.err.println(nodename + " Failed to extract message content: " + e.getMessage());
        }
        
        return null;
    }
    
    /**
     * Checks if a message (with or without prefix) is a heartbeat message
     * @param message The message to check
     * @return true if this is a heartbeat message
     */
    private boolean isHeartbeatMessage(String message) {
        if (message == null) {
            return false;
        }
        
        // First try to treat it as a prefixed message
        String content = extractMessageContent(message);
        if (content != null && content.equals("heartbeat")) {
            return true;
        }
        
        // Also handle unprefixed heartbeats for compatibility
        return message.equals("heartbeat");
    }
}