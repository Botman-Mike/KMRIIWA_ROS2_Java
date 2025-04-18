package Test_Comms;

import java.net.*;

public class UDPSocket implements ISocket {
    private DatagramSocket UDPConn;
    private final int COMport;
    private final String nodename;
    private boolean running = true;
    private static final int BUFFER_SIZE = 65535;
    private InetAddress remoteAddress;

    public UDPSocket(int port, String node_name) {
        this.COMport = port;
        this.nodename = node_name;
        try {
            remoteAddress = InetAddress.getByName("172.31.1.147"); // Robot's IP
            UDPConn = new DatagramSocket();
            UDPConn.setReuseAddress(true);
            System.out.println(nodename + " UDP socket created on port " + port);
        } catch (Exception e) {
            System.err.println("Error creating UDP socket: " + e.getMessage());
        }
    }

    @Override
    public void send_message(String message) {
        if (!isConnected()) return;
        
        try {
            byte[] sendData = encode(message);
            DatagramPacket sendPacket = new DatagramPacket(
                sendData, sendData.length, remoteAddress, COMport);
            UDPConn.send(sendPacket);
        } catch (Exception e) {
            System.err.println(nodename + " UDP send error: " + e.getMessage());
        }
    }

    @Override
    public String receive_message() {
        if (!isConnected()) return null;

        try {
            byte[] receiveData = new byte[BUFFER_SIZE];
            DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
            UDPConn.setSoTimeout(1000);
            UDPConn.receive(receivePacket);
            return decode(receivePacket.getData()).trim();
        } catch (SocketTimeoutException e) {
            return null;
        } catch (Exception e) {
            if (running) {
                System.err.println(nodename + " UDP receive error: " + e.getMessage());
            }
            return null;
        }
    }

    @Override
    public boolean isConnected() {
        return UDPConn != null && !UDPConn.isClosed();
    }

    @Override
    public void close() {
        running = false;
        if (UDPConn != null) {
            UDPConn.close();
            System.out.println(nodename + " UDP socket closed");
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

    public void setReceiveBufferSize(int size) {
        try {
            if (UDPConn != null) {
                UDPConn.setReceiveBufferSize(size);
            }
        } catch (Exception e) {
            System.err.println("Could not set UDP receive buffer size: " + e.getMessage());
        }
    }
}