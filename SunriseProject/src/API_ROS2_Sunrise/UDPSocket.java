package API_ROS2_Sunrise;


import java.net.BindException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.charset.Charset;
import java.io.InterruptedIOException;
import java.net.SocketTimeoutException;

import API_ROS2_Sunrise.ISocket;


public class UDPSocket implements ISocket {
    private static final int SOCKET_TIMEOUT = 500;  // Reduce to match heartbeat interval
    private static final int MAX_RETRIES = 5;      // Increase retries for reliability

    private ConnectionManager connectionManager;
    
    // Add setter for ConnectionManager
    public void setConnectionManager(ConnectionManager connectionManager) {
        this.connectionManager = connectionManager;
    }
    
    public volatile boolean isConnected;
    DatagramSocket udpConn;
    DatagramPacket package_out;
    DatagramPacket package_in;
    private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");
    int COMport;
    static BindException b;
    String nodename;
    private boolean running;

    public UDPSocket(int port, String node_name) {
        isConnected = false;
        this.COMport = port;
        this.nodename = node_name;
        this.connectionManager = new ConnectionManager();
        udpConn = connect();
    }

    public DatagramSocket connect() {
        while (!isConnected) {
            try {
                LogUtil.logInfo("Connecting  " + this.nodename + " to ROS over UDP on port: " + COMport);
                int kuka_port = this.COMport;
                InetSocketAddress socket_address = new InetSocketAddress(kuka_port);

                String ros_host = "172.31.1.206"; // REMOTE PC
                int ros_port = this.COMport;
                InetAddress address = InetAddress.getByName(ros_host);

                udpConn = new DatagramSocket(null);
                udpConn.setReuseAddress(true);
                udpConn.bind(socket_address);

                // Initialize ConnectionManager with UDP details
                connectionManager.setUDPConnection(udpConn, address, ros_port);
                connectionManager.initializeHeartbeat();

                byte buf[] = "HALLA ROS".getBytes();
                byte buf1[] = new byte[1024];

                package_out = new DatagramPacket(buf, buf.length, address, ros_port);
                package_in = new DatagramPacket(buf1, buf1.length);

                // send() method
                udpConn.send(package_out);

                // receive() method
                udpConn.setSoTimeout(3000); // ms

                udpConn.receive(package_in);
                String s = decode(package_in);
                LogUtil.logInfo(this.nodename + " received packet data over UDP on port : " + COMport + " Message: " + s);

                if (s.length() < 1) {
                    udpConn.close();
                    isConnected = false;
                    LogUtil.logInfo(this.nodename + "  did not receive any message in 3 seconds, shutting off");
                    break;
                }
                udpConn.setSoTimeout(0);
                isConnected = true;
            } catch (Exception e1) {
                LogUtil.logInfo("ERROR connecting  " + this.nodename + " to ROS over UDP on port: " + this.COMport + " Error: " + e1);
                isConnected = false;
                close();
                b = new BindException();
                if (e1.getClass().equals(b.getClass())) {
                    e1.printStackTrace();
                    break;
                }
                break;
            }
        }
        return udpConn;
    }

    public String decode(DatagramPacket pack) {
        byte[] data = pack.getData();
        String message = new String(data, 0, pack.getLength(), UTF8_CHARSET);
        return message;

    }

    @Override
    public byte[] encode(String string) {
        return string.getBytes(UTF8_CHARSET);
    }

    @Override
    public void send_message(String msg) {
        byte[] bytlist = msg.getBytes(UTF8_CHARSET);
        package_out.setData(bytlist);
        package_out.setLength(bytlist.length);
        try {
            udpConn.send(package_out);
        } catch (Exception e) {
            LogUtil.logInfo(this.nodename + " could not send package over UDP on port: " + this.COMport + " error: " + e);
        }
    }

    @Override
    public String receive_message() {
        int retries = 0;
        while (retries < MAX_RETRIES) {
            if (!isConnected()) {
                LogUtil.logInfo(nodename + ": Socket disconnected, attempting reconnect");
                connect();
                if (!isConnected()) {
                    retries++;
                    continue;
                }
            }

            try {
                udpConn.setSoTimeout(SOCKET_TIMEOUT);
                byte[] buffer = new byte[1024];
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                udpConn.receive(packet);
                
                String message = decode(packet);
                processIncomingMessage(message);
                return message;
            } catch (SocketTimeoutException e) {
                // Expected timeout, continue listening
                return null;
            } catch (Exception e) {
                LogUtil.logError(nodename + " UDP receive error: " + e.getMessage());
                retries++;
            }
        }
        return null;
    }

    @Override
    public void close() {
        LogUtil.logInfo(nodename + " UDP connection closing");
        running = false;

        if (connectionManager != null) {
            connectionManager.stopHeartbeat();
        }

        if (udpConn != null && !udpConn.isClosed()) {
            try {
                udpConn.close();
                LogUtil.logInfo(nodename + " UDP socket closed successfully");
            } catch (Exception e) {
                LogUtil.logInfo(nodename + " UDP socket close error: " + e.getMessage());
            }
        }
    }

    @Override
    public boolean isConnected() {
        if (udpConn == null || udpConn.isClosed()) {
            LogUtil.logInfo(nodename + " UDP Socket check: Socket is null or closed");
            return false;
        }

        try {
            // For UDP, we don't have a true "connection" status like TCP
            // Instead, check if socket is bound and not closed
            if (!udpConn.isBound()) {
                LogUtil.logInfo(nodename + " UDP Socket check: Socket is not bound");
                return false;
            }

            // Additional check: try a non-destructive socket operation
            udpConn.getReceiveBufferSize(); // This will throw if socket is invalid
            return true;
        } catch (Exception e) {
            LogUtil.logInfo(nodename + " UDP Socket check exception: " + e.getMessage());
            return false;
        }
    }

    private void processIncomingMessage(String message) {
        if (message != null && message.startsWith("heartbeat")) {
            if (connectionManager != null) {
                connectionManager.handleIncomingHeartbeat();
            }
            return;
        }
        // Additional message processing logic can be added here
    }

    @Override
    public void onMessageReceived(DatagramPacket packet) {
        byte[] data = packet.getData();
        String message = new String(data, 0, packet.getLength());
        processIncomingMessage(message);
    }
}
