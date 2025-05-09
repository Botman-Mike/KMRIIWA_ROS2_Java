package API_ROS2_Sunrise;

import java.io.IOException;
import java.net.SocketTimeoutException;
import java.net.SocketException;
import java.io.EOFException;
import java.net.Socket;
import java.net.InetSocketAddress;
import java.nio.charset.Charset;
import com.kuka.task.ITaskLogger;
import javax.inject.Inject;
import java.io.InputStream;
import java.io.DataInputStream;
import java.io.OutputStream;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class TCPSocket implements ISocket {
    private Thread writerThread;
    private volatile boolean writerRunning = false;
    
    // Connection management
    private static final int KMP_STATUS_PORT = 30001;
    private static final int KMP_COMMAND_PORT = 30002;
    private static final int LBR_COMMAND_PORT = 30005;
    private static final int LBR_STATUS_PORT = 30006;
    
    // The four persistent socket connections
    private Socket kmpStatusSocket;
    private Socket kmpCommandSocket;
    private Socket lbrCommandSocket;
    private Socket lbrStatusSocket;
    
    // For backward compatibility with existing code
    public Socket TCPConn;
    
    // Default remote IP, can be overridden
    private static String DEFAULT_REMOTE_IP = "172.31.1.206";
    private String remoteIP;

    // Port info
    int COMport;
    String nodename;
    private boolean running = true;
    private long lastHeartbeat = System.currentTimeMillis();
    private final BlockingQueue<String> sendQueue = new LinkedBlockingQueue<String>();
    
    // Heartbeat timeout - confirmed 15 seconds
    private static final long HEARTBEAT_TIMEOUT = 15000; 
    private static final Charset UTF8_CHARSET = Charset.forName("UTF-8");
    public static final int SO_TIMEOUT = 1000; // Socket read timeout
    private static final int CONNECTION_TIMEOUT = 5000; // Connection timeout 
    private static final int MAX_RECONNECT_ATTEMPTS = 3;
    private static final int SOCKET_BUFFER_SIZE = 65536; // 64KB buffer size
    
    // Reconnection parameters
    private static final long INITIAL_RECONNECT_DELAY = 500; // Start with 500ms
    private static final long MAX_RECONNECT_DELAY = 30000; // Max 30 seconds between retries
    
    // Scheduler for heartbeats
    private ScheduledExecutorService scheduler;

    @Inject
    private ITaskLogger logger;

    public TCPSocket(int port, String node_name) {
        this(port, node_name, DEFAULT_REMOTE_IP);
    }

    public TCPSocket(int port, String node_name, String remoteIP) {
        COMport = port;
        this.nodename = node_name;
        this.remoteIP = remoteIP;
        
        // Initialize all four persistent sockets
        initializeSockets();
        
        // Set TCPConn for backward compatibility - pointing to the socket matching the port
        TCPConn = getSocketForPort(port);
        
        // Start sender thread
        startWriterThread();
        
        // Initialize and start the heartbeat scheduler
        initializeHeartbeatScheduler();
        
        if (logger != null) logger.info(nodename + "|Initialized with persistent connections to " + remoteIP);
    }
    
    /**
     * Initialize all four persistent socket connections
     */
    private void initializeSockets() {
        // Create and configure all four sockets
        kmpStatusSocket = createAndConfigureSocket(KMP_STATUS_PORT, "KMP status");
        kmpCommandSocket = createAndConfigureSocket(KMP_COMMAND_PORT, "KMP command");
        lbrCommandSocket = createAndConfigureSocket(LBR_COMMAND_PORT, "LBR command");
        lbrStatusSocket = createAndConfigureSocket(LBR_STATUS_PORT, "LBR status");
    }
    
    /**
     * Creates and configures a socket for a specific port
     */
    private Socket createAndConfigureSocket(int port, String socketName) {
        Socket socket = null;
        int attempts = 0;
        long reconnectDelay = INITIAL_RECONNECT_DELAY;
        
        while (attempts < MAX_RECONNECT_ATTEMPTS && !Thread.currentThread().isInterrupted()) {
            try {
                if (logger != null) {
                    logger.info(nodename + " creating persistent " + socketName + " socket on " + remoteIP + ":" + port);
                }
                
                // Create fresh socket with proper configuration
                socket = new Socket();
                socket.setReuseAddress(true);
                socket.setTcpNoDelay(true); // Disable Nagle's algorithm
                socket.setSoTimeout(SO_TIMEOUT); // Set read timeout
                
                // Connect with specific timeout
                socket.connect(new InetSocketAddress(remoteIP, port), CONNECTION_TIMEOUT);
                
                // Configure socket after connection as required
                socket.setKeepAlive(true);
                socket.setReceiveBufferSize(SOCKET_BUFFER_SIZE);
                
                if (logger != null) {
                    logger.info(nodename + " " + socketName + " CONNECTED on port " + port);
                }
                
                return socket;
            } catch (IOException e) {
                attempts++;
                if (attempts < MAX_RECONNECT_ATTEMPTS) {
                    if (logger != null) {
                        logger.warn(nodename + " " + socketName + " socket connection attempt " + attempts + 
                                   " failed: " + e.getMessage() + ". Retrying in " + (reconnectDelay/1000.0) + " seconds...");
                    }
                    try {
                        Thread.sleep(reconnectDelay);
                        // Apply exponential backoff
                        reconnectDelay = Math.min(reconnectDelay * 2, MAX_RECONNECT_DELAY);
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                } else {
                    if (logger != null) {
                        logger.error(nodename + " " + socketName + " socket failed to connect after " + MAX_RECONNECT_ATTEMPTS + 
                                   " attempts. Last error: " + e.getMessage());
                    }
                }
            }
        }
        
        return null; // Return null if all attempts failed
    }

    /**
     * Initialize the heartbeat scheduler using ScheduledExecutorService
     */
    private void initializeHeartbeatScheduler() {
        // Create single thread executor for heartbeats
        scheduler = Executors.newSingleThreadScheduledExecutor();
        
        // Schedule fixed-rate heartbeat task every 5 seconds
        scheduler.scheduleAtFixedRate(new Runnable() {
            public void run() {
                sendHeartbeat();
            }
        }, 0, 5, TimeUnit.SECONDS);
        
        if (logger != null) {
            logger.info(nodename + " Heartbeat scheduler initialized with 5-second interval");
        }
    }

    /**
     * Send heartbeat to all sockets
     */
    public void sendHeartbeat() {
        if (logger != null) {
            logger.info(nodename + " HEARTBEAT_SENT - Sending heartbeats to all ports");
        }
        
        // Send heartbeat to each socket
        sendHeartbeatToSocket(kmpStatusSocket, KMP_STATUS_PORT, "KMP status");
        sendHeartbeatToSocket(kmpCommandSocket, KMP_COMMAND_PORT, "KMP command");
        sendHeartbeatToSocket(lbrCommandSocket, LBR_COMMAND_PORT, "LBR command");
        sendHeartbeatToSocket(lbrStatusSocket, LBR_STATUS_PORT, "LBR status");
    }
    
    /**
     * Send heartbeat to a specific socket with error handling and reconnection
     */
    private void sendHeartbeatToSocket(Socket socket, int port, String socketName) {
        if (socket == null || socket.isClosed() || !socket.isConnected()) {
            if (logger != null) {
                logger.error(nodename + " ERROR - " + socketName + " socket disconnected, attempting reconnect");
            }
            socket = reconnectSocket(port);
            if (socket == null) {
                return;
            }
        }
        
        try {
            // Heartbeat message with CRLF
            String heartbeatMsg = "heartbeat\r\n";
            byte[] payload = heartbeatMsg.getBytes(UTF8_CHARSET);
            
            // Format 10-digit header + space as required
            String header = String.format("%010d", payload.length);
            
            OutputStream out = socket.getOutputStream();
            out.write(header.getBytes(UTF8_CHARSET));
            out.write(' ');
            out.write(payload);
            out.flush();
            
            // Update last heartbeat timestamp
            updateLastHeartbeat();
            
        } catch (IOException e) {
            if (logger != null) {
                logger.error(nodename + " ERROR - Failed to send heartbeat to " + socketName + ": " + e.getMessage());
            }
            
            // Close the socket
            try {
                if (!socket.isClosed()) {
                    socket.close();
                }
            } catch (IOException closeErr) {
                // Ignore close errors
            }
            
            // Attempt to reconnect the socket with exponential backoff
            reconnectSocketWithBackoff(port, socketName);
        }
    }
    
    /**
     * Reconnect socket with exponential backoff
     */
    private Socket reconnectSocketWithBackoff(int port, String socketName) {
        long reconnectDelay = INITIAL_RECONNECT_DELAY;
        int attempts = 0;
        Socket socket = null;
        
        while (attempts < MAX_RECONNECT_ATTEMPTS && socket == null) {
            try {
                if (logger != null) {
                    logger.info(nodename + " Reconnecting " + socketName + " after " + (reconnectDelay/1000.0) + " seconds");
                }
                
                // Wait before retrying
                Thread.sleep(reconnectDelay);
                
                // Attempt to reconnect
                socket = createAndConfigureSocket(port, socketName);
                
                if (socket != null) {
                    // Update socket reference
                    updateSocketReference(port, socket);
                    
                    if (logger != null) {
                        logger.info(nodename + " RECONNECTED " + socketName + " socket successfully");
                    }
                    return socket;
                }
                
                // Increase backoff delay exponentially
                reconnectDelay = Math.min(reconnectDelay * 2, MAX_RECONNECT_DELAY);
                attempts++;
                
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        
        if (socket == null && logger != null) {
            logger.error(nodename + " Failed to reconnect " + socketName + " after " + MAX_RECONNECT_ATTEMPTS + " attempts");
        }
        
        return socket;
    }
    
    /**
     * Update the appropriate socket reference
     */
    private void updateSocketReference(int port, Socket socket) {
        switch (port) {
            case KMP_STATUS_PORT: 
                kmpStatusSocket = socket;
                break;
            case KMP_COMMAND_PORT: 
                kmpCommandSocket = socket;
                break;
            case LBR_COMMAND_PORT: 
                lbrCommandSocket = socket;
                break;
            case LBR_STATUS_PORT: 
                lbrStatusSocket = socket;
                break;
        }
        
        // Update TCPConn if this is for the current port
        if (port == COMport) {
            TCPConn = socket;
        }
    }

    private void startWriterThread() {
        writerRunning = true;
        writerThread = new Thread(new Runnable() {
            public void run() {
                while (writerRunning && !Thread.currentThread().isInterrupted()) {
                    try {
                        String msg = sendQueue.take();
                        
                        // Get socket based on port
                        Socket targetSocket = getSocketForPort(COMport);
                        
                        if (targetSocket != null && targetSocket.isConnected() && !targetSocket.isClosed()) {
                            sendToSocket(targetSocket, msg);
                        } else {
                            // If socket is disconnected, try to reconnect and send
                            if (logger != null) {
                                logger.warn(nodename + " Socket for port " + COMport + " unavailable. Attempting reconnect.");
                            }
                            
                            targetSocket = reconnectSocket(COMport);
                            if (targetSocket != null) {
                                sendToSocket(targetSocket, msg);
                            } else {
                                if (logger != null) {
                                    logger.error(nodename + " Failed to send message after reconnection attempts: " + msg);
                                }
                            }
                        }
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }, nodename + "-writer");
        writerThread.setDaemon(true);
        writerThread.start();
    }
    
    /**
     * Get the appropriate socket for the given port
     */
    private Socket getSocketForPort(int port) {
        switch (port) {
            case KMP_STATUS_PORT: return kmpStatusSocket;
            case KMP_COMMAND_PORT: return kmpCommandSocket;
            case LBR_COMMAND_PORT: return lbrCommandSocket;
            case LBR_STATUS_PORT: return lbrStatusSocket;
            default: return null;
        }
    }
    
    /**
     * Get socket name for logging
     */
    private String getSocketName(int port) {
        switch (port) {
            case KMP_STATUS_PORT: return "KMP status";
            case KMP_COMMAND_PORT: return "KMP command";
            case LBR_COMMAND_PORT: return "LBR command";
            case LBR_STATUS_PORT: return "LBR status";
            default: return "Unknown";
        }
    }
    
    /**
     * Send a message to a specific socket with error handling and reconnection
     */
    private void sendToSocket(Socket socket, String message) {
        int attempts = 0;
        long reconnectDelay = INITIAL_RECONNECT_DELAY;
        boolean sent = false;
        
        while (!sent && attempts < MAX_RECONNECT_ATTEMPTS) {
            try {
                // Append CRLF to payload - maintaining the required format
                String msgWithCrlf = message + "\r\n";
                byte[] payload = msgWithCrlf.getBytes(UTF8_CHARSET);
                
                // Format 10-digit header + space - maintaining the required format
                String header = String.format("%010d", payload.length);
                
                OutputStream out = socket.getOutputStream();
                out.write(header.getBytes(UTF8_CHARSET));
                out.write(' ');
                out.write(payload);
                out.flush();
                
                sent = true;
                
            } catch (IOException e) {
                attempts++;
                
                if (logger != null) {
                    logger.warn(nodename + " Socket write failed (attempt " + attempts + "/" + MAX_RECONNECT_ATTEMPTS + 
                              "): " + e.getMessage());
                }
                
                // Close this socket as it's having problems
                try {
                    if (!socket.isClosed()) {
                        socket.close();
                    }
                } catch (IOException closeErr) {
                    // Ignore close errors
                }
                
                // Apply exponential backoff before retry
                try {
                    if (logger != null) {
                        logger.info(nodename + " STALLED - waiting " + (reconnectDelay/1000.0) + 
                                  " seconds before retry...");
                    }
                    Thread.sleep(reconnectDelay);
                    reconnectDelay = Math.min(reconnectDelay * 2, MAX_RECONNECT_DELAY);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                    break;
                }
                
                // Try to reopen the socket if needed
                if (attempts < MAX_RECONNECT_ATTEMPTS) {
                    if (logger != null) {
                        logger.info(nodename + " RECONNECTING socket...");
                    }
                    int port = getPortFromSocket(socket);
                    socket = reconnectSocket(port);
                    
                    if (socket == null) {
                        if (logger != null) {
                            logger.error(nodename + " Failed to reconnect socket for port " + port);
                        }
                        break;
                    }
                }
            }
        }
        
        if (!sent && logger != null) {
            logger.error(nodename + " Failed to send message after " + MAX_RECONNECT_ATTEMPTS + " attempts");
        }
    }
    
    /**
     * Get the port number from a socket
     */
    private int getPortFromSocket(Socket socket) {
        if (socket == kmpStatusSocket) return KMP_STATUS_PORT;
        if (socket == kmpCommandSocket) return KMP_COMMAND_PORT;
        if (socket == lbrCommandSocket) return LBR_COMMAND_PORT;
        if (socket == lbrStatusSocket) return LBR_STATUS_PORT;
        return -1;
    }
    
    /**
     * Reconnect a socket for a specific port
     */
    private Socket reconnectSocket(int port) {
        String socketName = getSocketName(port);
        
        if (logger != null) {
            logger.info(nodename + " Reconnecting " + socketName + " socket on port " + port);
        }
        
        Socket newSocket = createAndConfigureSocket(port, socketName);
        
        // Update the appropriate socket reference
        if (newSocket != null) {
            updateSocketReference(port, newSocket);
            
            if (logger != null) {
                logger.info(nodename + " " + socketName + " socket successfully reconnected");
            }
        }
        
        return newSocket;
    }

    public Socket connect() {
        // This is now just a wrapper around current socket for backward compatibility
        Socket socket = getSocketForPort(COMport);
        
        // Update the TCPConn reference for backward compatibility
        TCPConn = socket;
        
        return socket;
    }

    public void send_message(String buffer) {
        // Enqueue message for sender thread
        if (!sendQueue.offer(buffer)) {
            if (logger != null) logger.error(nodename + "|port " + COMport + " send queue full, message dropped");
        }
    }

    public boolean isConnected() {
        Socket socket = getSocketForPort(COMport);
        
        if (socket == null) {
            if (logger != null) logger.warn(nodename + " TCP Socket check: Socket is null");
            return false;
        }

        try {
            if (socket.isClosed()) {
                if (logger != null) logger.warn(nodename + " TCP Socket check: Socket is closed.");
                return false;
            }

            if (!socket.isConnected()) {
                if (logger != null) logger.warn(nodename + " TCP Socket check: Socket not connected.");
                return false;
            }

            if (socket.isInputShutdown() || socket.isOutputShutdown()) {
                if (logger != null) logger.warn(nodename + " TCP Socket check: Input or output is shutdown.");
                return false;
            }
            
            // Check heartbeat timeout
            long timeSinceLastHeartbeat = System.currentTimeMillis() - lastHeartbeat;
            if (timeSinceLastHeartbeat >= HEARTBEAT_TIMEOUT) {
                if (logger != null) logger.warn(nodename + " TCP Socket check: Heartbeat timeout (" + 
                                             timeSinceLastHeartbeat + "ms >= " + HEARTBEAT_TIMEOUT + "ms). Marking as disconnected.");
                return false;
            }

            return true;
        } catch (Exception e) {
            if (logger != null) logger.error(nodename + " TCP Socket check exception: " + e.getMessage() + ". Assuming disconnected.");
            return false;
        }
    }

    public String receive_message() {
        Socket socket = getSocketForPort(COMport);
        
        if (socket == null || socket.isClosed() || !socket.isConnected()) {
            if (logger != null) logger.warn(nodename + " TCP Socket lost, attempting reconnection");
            socket = reconnectSocket(COMport);
            if (socket == null) {
                return null;
            }
        }

        try {
            InputStream inputStream = socket.getInputStream();
            DataInputStream dataInputStream = new DataInputStream(inputStream);
            
            // Loop until we find a valid 11-byte header (10-digit length + space) - maintained as required
            int messageLength;
            byte[] headerRaw = new byte[11];
            while (true) {
                dataInputStream.readFully(headerRaw, 0, 11);
                boolean valid = true;
                for (int i = 0; i < 10; i++) {
                    if (headerRaw[i] < '0' || headerRaw[i] > '9') { valid = false; break; }
                }
                if (valid && headerRaw[10] == ' ') {
                    messageLength = Integer.parseInt(new String(headerRaw, 0, 10, UTF8_CHARSET));
                    break;
                }
                // Invalid header: discard until CRLF, but prevent runaway
                int prev = -1, curr, discarded = 0;
                while (true) {
                    curr = dataInputStream.readUnsignedByte();
                    discarded++;
                    if (prev == '\r' && curr == '\n') break;
                    prev = curr;
                    if (discarded > 1024) {
                        throw new IOException("Stream out of sync: too many bytes discarded");
                    }
                }
                // After CRLF, retry header read
            }
            
            // Read the message body including CRLF
            byte[] payloadBytes = new byte[messageLength];
            dataInputStream.readFully(payloadBytes);
            String fullPayload = new String(payloadBytes, UTF8_CHARSET);
            
            // Remove trailing CRLF if present
            String messageContent;
            if (fullPayload.endsWith("\r\n")) {
                messageContent = fullPayload.substring(0, fullPayload.length() - 2);
            } else {
                messageContent = fullPayload;
            }

            if ("heartbeat".equals(messageContent)) {
                updateLastHeartbeat();
                if (logger != null) logger.info(nodename + " Received heartbeat.");
                return null;
            }
            return messageContent;

        } catch (NumberFormatException e) {
            if (logger != null) logger.error(nodename + " Invalid message format: Cannot parse length header.");
            socket = reconnectSocket(COMport);
            return null;
        } catch (SocketTimeoutException e) {
            // Just a timeout, not an error
            return null;
        } catch (EOFException e) {
            if (logger != null) logger.warn(nodename + " TCP receive error: Connection closed by peer (EOF). STALLED.");
            socket = reconnectSocket(COMport);
            return null;
        } catch (SocketException e) {
            if (logger != null) logger.error(nodename + " TCP receive error (SocketException): " + e.getMessage());
            socket = reconnectSocket(COMport);
            return null;
        } catch (IOException e) {
            if (running && logger != null) logger.error(nodename + " TCP receive error (IOException): " + e.getMessage());
            socket = reconnectSocket(COMport);
            return null;
        } catch (Exception e) {
            if (running && logger != null) logger.error(nodename + " Unexpected error during TCP receive: " + e.getMessage(), e);
            socket = reconnectSocket(COMport);
            return null;
        }
    }
    
    /**
     * Close all socket resources
     */
    private void closeAllSockets() {
        // Close KMP status socket
        if (kmpStatusSocket != null) {
            try {
                kmpStatusSocket.close();
                if (logger != null) logger.info(nodename + " Closed KMP status socket");
            } catch (IOException e) { /* ignore */ }
            kmpStatusSocket = null;
        }
        
        // Close KMP command socket
        if (kmpCommandSocket != null) {
            try {
                kmpCommandSocket.close();
                if (logger != null) logger.info(nodename + " Closed KMP command socket");
            } catch (IOException e) { /* ignore */ }
            kmpCommandSocket = null;
        }
        
        // Close LBR command socket
        if (lbrCommandSocket != null) {
            try {
                lbrCommandSocket.close();
                if (logger != null) logger.info(nodename + " Closed LBR command socket");
            } catch (IOException e) { /* ignore */ }
            lbrCommandSocket = null;
        }
        
        // Close LBR status socket
        if (lbrStatusSocket != null) {
            try {
                lbrStatusSocket.close();
                if (logger != null) logger.info(nodename + " Closed LBR status socket");
            } catch (IOException e) { /* ignore */ }
            lbrStatusSocket = null;
        }
    }

    public void close() {
        // Stop writer thread
        writerRunning = false;
        if (writerThread != null) writerThread.interrupt();
        
        // Shutdown the scheduler gracefully
        if (scheduler != null) {
            scheduler.shutdown();
            try {
                if (!scheduler.awaitTermination(2, TimeUnit.SECONDS)) {
                    scheduler.shutdownNow();
                }
            } catch (InterruptedException e) {
                scheduler.shutdownNow();
                Thread.currentThread().interrupt();
            }
        }
        
        // Log explicit close action
        if (logger != null) logger.info(nodename + " TCP connections closing explicitly.");
        running = false;
        closeAllSockets();
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

    public static int getSoTimeout() {
        return SO_TIMEOUT;
    }

    public void startHeartbeatThread() {
        // Method kept for ISocket interface compatibility
        // Instead of creating a new thread, we're now using the scheduler
        // If scheduler is null or shutdown, re-initialize it
        if (scheduler == null || scheduler.isShutdown()) {
            initializeHeartbeatScheduler();
            if (logger != null) logger.info(nodename + " Restarted heartbeat scheduler");
        } else {
            if (logger != null) logger.info(nodename + " Heartbeat scheduler already running");
        }
    }

    public void stopHeartbeatThread() {
        // Method kept for ISocket interface compatibility
        // Shutdown the scheduler if it's running
        if (scheduler != null && !scheduler.isShutdown()) {
            scheduler.shutdown();
            try {
                if (!scheduler.awaitTermination(2, TimeUnit.SECONDS)) {
                    scheduler.shutdownNow();
                }
                
                if (logger != null) logger.info(nodename + " Heartbeat scheduler stopped");
                
            } catch (InterruptedException e) {
                scheduler.shutdownNow();
                Thread.currentThread().interrupt();
                
                if (logger != null) logger.warn(nodename + " Heartbeat scheduler interrupted while shutting down");
            }
        }
    }
}
