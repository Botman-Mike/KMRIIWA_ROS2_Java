package API_ROS2_Sunrise;

import java.net.DatagramPacket;

public interface ISocket {
	
	public void close();
	public void send_message(String msg);
	public String receive_message();
	public byte[] encode(String string);
	public boolean isConnected();

    /**
     * Handle received messages from the socket
     * @param packet The received datagram packet
     */
    void onMessageReceived(DatagramPacket packet);
}
