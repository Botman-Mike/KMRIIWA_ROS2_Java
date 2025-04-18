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
    private final String remoteIP;

    public TCPSocket(int port, String node_name) {
        this(port, node_name, "172.31.1.147"); // Default IP for backward compatibility
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
            outputStream.println(buffer);
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
            
            if (message != null && message.trim().equals("heartbeat")) {
                updateLastHeartbeat();
                return null;
            }
            return message;
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
}