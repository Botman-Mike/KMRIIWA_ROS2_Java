package API_ROS2_Sunrise;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import com.kuka.task.ITaskLogger;
import javax.inject.Inject;

public class UDPSocket implements ISocket {
    private static final int DEFAULT_BUFFER_SIZE = 65535;
    private static final String DEFAULT_REMOTE_IP = "172.31.1.206";
    private static final int MAX_CONSECUTIVE_FAILURES = 3;
    private static final long RECONNECT_DELAY = 1000; // 1 second

    private volatile boolean isConnected = false;
    private DatagramSocket udpConn;
    private String nodename;
    private int COMport;
    private String remoteIP;
    private DatagramPacket package_out;
    private byte[] receiveBuffer;
    private int consecutiveFailures = 0;

    @Inject
    private ITaskLogger logger;

    public UDPSocket(int port, String node_name) {
        this(port, node_name, DEFAULT_REMOTE_IP);
    }

    public UDPSocket(int port, String node_name, String remoteIP) {
        this.COMport = port;
        this.nodename = node_name;
        this.remoteIP = remoteIP;
        this.receiveBuffer = new byte[DEFAULT_BUFFER_SIZE];
        connect();
    }

    public DatagramSocket connect() {
        while (!isConnected) {
            try {
                if (logger != null) logger.info("Connecting " + this.nodename + " to ROS over UDP on port: " + COMport);
                InetSocketAddress socket_address = new InetSocketAddress(COMport);
                InetAddress address = InetAddress.getByName(remoteIP);

                udpConn = new DatagramSocket(null);
                udpConn.setReuseAddress(true);
                udpConn.setSendBufferSize(DEFAULT_BUFFER_SIZE);
                udpConn.setReceiveBufferSize(DEFAULT_BUFFER_SIZE);
                udpConn.bind(socket_address);

                // Initialize output packet
                byte[] initialBuf = "INIT".getBytes();
                package_out = new DatagramPacket(initialBuf, initialBuf.length, address, COMport);
                
                isConnected = true;
                consecutiveFailures = 0;
                if (logger != null) logger.info(nodename + " UDP connection established");
                return udpConn;
            } catch (Exception e) {
                consecutiveFailures++;
                if (logger != null) logger.error(nodename + " UDP connection failed (attempt " + consecutiveFailures + "): " + e.getMessage());
                
                if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
                    if (logger != null) logger.error(nodename + " Max UDP connection attempts reached");
                    break;
                }
                
                try {
                    Thread.sleep(RECONNECT_DELAY);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
        return null;
    }

    public void send_message(String msg) {
        if (!isConnected() || udpConn == null) {
            if (logger != null) logger.warn(nodename + " Cannot send UDP message - not connected");
            return;
        }

        try {
            byte[] buf = msg.getBytes();
            package_out.setData(buf);
            package_out.setLength(buf.length);
            udpConn.send(package_out);
            consecutiveFailures = 0;
        } catch (Exception e) {
            consecutiveFailures++;
            if (logger != null) logger.error(nodename + " UDP send error: " + e.getMessage());
            
            if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
                if (logger != null) logger.warn(nodename + " Multiple UDP send failures, attempting reconnection");
                connect();
            }
        }
    }

    public String receive_message() {
        if (!isConnected() || udpConn == null) {
            return null;
        }

        try {
            DatagramPacket receivePacket = new DatagramPacket(receiveBuffer, receiveBuffer.length);
            udpConn.receive(receivePacket);
            consecutiveFailures = 0;
            return new String(receivePacket.getData(), 0, receivePacket.getLength());
        } catch (SocketTimeoutException e) {
            return null;
        } catch (Exception e) {
            consecutiveFailures++;
            if (logger != null) logger.error(nodename + " UDP receive error: " + e.getMessage());
            
            if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
                if (logger != null) logger.warn(nodename + " Multiple UDP receive failures, attempting reconnection");
                connect();
            }
            return null;
        }
    }

    public boolean isConnected() {
        return isConnected && udpConn != null && !udpConn.isClosed();
    }

    public void close() {
        if (udpConn != null) {
            try {
                udpConn.close();
                if (logger != null) logger.info(nodename + " UDP connection closed");
            } catch (Exception e) {
                if (logger != null) logger.error(nodename + " Error closing UDP connection: " + e.getMessage());
            }
        }
        isConnected = false;
    }

    // Implement ISocket heartbeat methods - UDP doesn't need active heartbeat
    public void sendHeartbeat() {} 
    public long getLastHeartbeat() { return System.currentTimeMillis(); }
    public void updateLastHeartbeat() {}
    public void startHeartbeatThread() {}
    public void stopHeartbeatThread() {}

    @Override
    public byte[] encode(String string) {
        try {
            return string.getBytes("UTF-8");
        } catch (java.io.UnsupportedEncodingException e) {
            if (logger != null) {
                logger.error(nodename + " Encoding error: " + e.getMessage());
            }
            return string.getBytes();
        }
    }

    @Override
    public String decode(byte[] data) {
        try {
            return new String(data, "UTF-8");
        } catch (java.io.UnsupportedEncodingException e) {
            if (logger != null) {
                logger.error(nodename + " Decoding error: " + e.getMessage());
            }
            return new String(data);
        }
    }
}
