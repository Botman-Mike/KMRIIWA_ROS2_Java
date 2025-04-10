package API_ROS2_Sunrise;


import java.io.IOException;

import java.net.DatagramPacket;

import java.nio.charset.Charset;
import java.net.Socket;
import java.net.ServerSocket;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.EOFException;
import java.io.InterruptedIOException;
import java.io.OutputStream;

import java.net.SocketException;
import java.net.SocketTimeoutException;

import API_ROS2_Sunrise.ISocket;

public class TCPSocket implements ISocket {
    private ConnectionManager connectionManager;
    private Socket tcpSocket;
    private OutputStream tcpOutputStream;

    public volatile boolean isConnected;
    public Socket TCPConn;
    DatagramPacket package_out;
    DatagramPacket package_in;
    public PrintWriter outputStream;
    public BufferedReader inputStream;
    private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");
    int COMport;
    String nodename;
    private boolean running = true;
    private ServerSocket serverSocket;

    public TCPSocket(int port, String node_name) {
        isConnected = false;
        COMport = port;
        this.nodename = node_name;
        TCPConn = connect();
        this.connectionManager = new ConnectionManager();
    }

    public void setConnectionManager(ConnectionManager connectionManager) {
        this.connectionManager = connectionManager;
    }

    private void initializeConnection() {
        // After establishing TCP connection
        connectionManager.setTCPConnection(tcpSocket, tcpOutputStream);
        connectionManager.initializeHeartbeat();
    }

    public Socket connect() {
        while (true) {
            try {
                String remotePC = "172.31.1.206";
                //String NUC = "192.168.10.120";

                TCPConn = new Socket(remotePC, COMport);
                TCPConn.setReuseAddress(true);
                LogUtil.logInfo(this.nodename + " connecting to ROS over TCP on port: " + COMport);
                break;
            } catch (IOException e1) {
                LogUtil.logInfo("Could not connect " + this.nodename + " to ROS over TCP on port : " + this.COMport + " Error: " + e1);
                return null;
            }
        }
        try {
            outputStream = new PrintWriter(TCPConn.getOutputStream(), true);
            inputStream = new BufferedReader(new InputStreamReader(TCPConn.getInputStream()));
            tcpSocket = TCPConn;
            tcpOutputStream = TCPConn.getOutputStream();
            initializeConnection();
            isConnected = true;
            return TCPConn;
        } catch (Exception e) {
            LogUtil.logInfo("Error creating I/O ports for TCP communication for  " + this.nodename + " on port: " + this.COMport + " Error: " + e);
            return null;
        }

    }

    public void send_message(String buffer) {
        int len = (this.encode(buffer)).length;
        String send_string = String.format("%010d", len) + " " + buffer;
        outputStream.write(send_string);
        outputStream.flush();
    }

    @Override
    public boolean isConnected() {
        if (TCPConn == null) {
            LogUtil.logInfo("tcp_null_" + nodename, nodename + " TCP Socket check: Socket is null");
            return false;
        }

        try {
            if (TCPConn.isClosed()) {
                LogUtil.logInfo("tcp_closed_" + nodename, nodename + " TCP Socket check: Socket is closed");
                return false;
            }

            if (!TCPConn.isConnected()) {
                LogUtil.logInfo("tcp_disconnected_" + nodename, nodename + " TCP Socket check: Socket not connected");
                return false;
            }

            if (TCPConn.isInputShutdown() || TCPConn.isOutputShutdown()) {
                LogUtil.logInfo("tcp_shutdown_" + nodename, nodename + " TCP Socket check: Input or output is shutdown");
                return false;
            }

            // If we get here, connection is good - reset throttles
            LogUtil.resetLogThrottle("tcp_null_" + nodename);
            LogUtil.resetLogThrottle("tcp_closed_" + nodename);
            LogUtil.resetLogThrottle("tcp_disconnected_" + nodename);
            LogUtil.resetLogThrottle("tcp_shutdown_" + nodename);
            return true;
        } catch (Exception e) {
            LogUtil.logError("tcp_error_" + nodename, nodename + " TCP Socket check exception: " + e.getMessage());
            return false;
        }
    }

    @Override
    public String receive_message() {
        if (!isConnected()) {
            // Try to repair the connection
            try {
                LogUtil.logInfo(nodename + " TCP Socket lost, attempting reconnection");
                connect();
                if (!isConnected()) {
                    return null;
                }
            } catch (Exception e) {
                LogUtil.logInfo(nodename + " TCP Socket reconnection failed: " + e.getMessage());
                return null;
            }
        }

        try {
            // Set a timeout for socket operations
            TCPConn.setSoTimeout(1000); // 1 second timeout

            BufferedReader reader = new BufferedReader(new InputStreamReader(TCPConn.getInputStream()));
            String message = reader.readLine();
            processIncomingMessage(message);
            return message;
        } catch (SocketTimeoutException e) {
            // Timeout is normal, just return null
            return null;
        } catch (Exception e) {
            // Only log if it's not a timeout and not due to shutdown
            if (running && !(e instanceof InterruptedIOException)) {
                LogUtil.logInfo(nodename + " TCP receive error: " + e.getMessage());

                // Try to detect if connection is broken
                if (e instanceof SocketException ||
                        e instanceof EOFException ||
                        e.getMessage().contains("Connection reset")) {

                    LogUtil.logInfo(nodename + " TCP Connection appears broken, forcing reconnection");
                    try {
                        TCPConn.close();
                    } catch (Exception ex) {
                        // Ignore
                    }
                    TCPConn = null;
                }
            }
            return null;
        }
    }

    private void processIncomingMessage(String message) {
        if (message != null && message.startsWith("heartbeat")) {
            if (connectionManager != null) {
                connectionManager.handleIncomingHeartbeat();
                // Add debug logging occasionally
                LogUtil.logInfo("heartbeat_received", "TCP Heartbeat received");
            }
            return;
        }
        // Additional message processing logic can be added here
    }

    @Override
    public void close() {
        if (connectionManager != null) {
            connectionManager.stopHeartbeat();
        }
        LogUtil.logInfo(nodename + " TCP connection closing");
        running = false;

        if (TCPConn != null) {
            try {
                TCPConn.close();
                LogUtil.logInfo(nodename + " TCP socket closed successfully");
            } catch (Exception e) {
                LogUtil.logInfo(nodename + " TCP socket close error: " + e.getMessage());
            }
        }

        if (serverSocket != null) {
            try {
                serverSocket.close();
                LogUtil.logInfo(nodename + " TCP server socket closed successfully");
            } catch (Exception e) {
                LogUtil.logInfo(nodename + " TCP server socket close error: " + e.getMessage());
            }
        }
    }


    public String decode(byte[] data) {
        String message = new String(data, 0, data.length, UTF8_CHARSET);
        return message;
    }

    @Override
    public byte[] encode(String string) {
        return string.getBytes(UTF8_CHARSET);
    }

    @Override
    public void onMessageReceived(DatagramPacket packet) {
        // TCP doesn't use DatagramPacket, but we need to implement the interface
        // No-op for TCP
    }

}
