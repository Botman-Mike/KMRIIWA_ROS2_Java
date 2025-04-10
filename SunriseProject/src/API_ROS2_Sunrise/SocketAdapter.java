package API_ROS2_Sunrise;

import java.net.Socket;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.IOException;
import java.net.DatagramPacket;

public class SocketAdapter implements ISocket {
    private Socket socket;
    private BufferedReader reader;
    private PrintWriter writer;

    public SocketAdapter(Socket socket) {
        try {
            this.socket = socket;
            this.reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            this.writer = new PrintWriter(socket.getOutputStream(), true);
        } catch (IOException e) {
            LogUtil.logInfo("Error initializing socket adapter: " + e.getMessage());
        }
    }

    public SocketAdapter(String ipAddress, int port) {
        try {
            this.socket = new Socket(ipAddress, port);
            this.reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            this.writer = new PrintWriter(socket.getOutputStream(), true);
        } catch (IOException e) {
            LogUtil.logInfo("Error initializing socket adapter: " + e.getMessage());
        }
    }
    
    @Override
    public void close() {
        try {
            if (socket != null) socket.close();
            if (reader != null) reader.close();
            if (writer != null) writer.close();
        } catch (IOException e) {
            LogUtil.logInfo("Error closing socket: " + e.getMessage());
        }
    }
    
    @Override
    public void send_message(String msg) {
        if (writer != null) {
            writer.println(msg);
        }
    }
    
    @Override
    public String receive_message() {
        try {
            if (reader != null) {
                return reader.readLine();
            }
        } catch (IOException e) {
            LogUtil.logInfo("Error receiving message: " + e.getMessage());
        }
        return null;
    }
    
    @Override
    public byte[] encode(String string) {
        return string.getBytes();
    }
    
    @Override
    public boolean isConnected() {
        return socket != null && socket.isConnected() && !socket.isClosed();
    }
    
    @Override
    public void onMessageReceived(DatagramPacket packet) {
        // TCP socket adapter - no UDP packet handling needed
    }
}
