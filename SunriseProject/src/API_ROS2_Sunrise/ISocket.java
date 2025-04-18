package API_ROS2_Sunrise;

/**
 * Interface defining the contract for socket implementations in the ROS2-Sunrise communication layer.
 * Both TCP and UDP socket implementations must implement these methods.
 */
public interface ISocket {
    /**
     * Send a message through the socket
     * @param msg The message to send
     */
    void send_message(String msg);

    /**
     * Receive a message from the socket
     * @return The received message or null if no message available
     */
    String receive_message();

    /**
     * Check if the socket is currently connected
     * @return true if connected, false otherwise
     */
    boolean isConnected();

    /**
     * Close the socket connection and clean up resources
     */
    void close();

    /**
     * Send a heartbeat message to keep the connection alive
     */
    void sendHeartbeat();

    /**
     * Get the timestamp of the last received heartbeat
     * @return timestamp in milliseconds
     */
    long getLastHeartbeat();

    /**
     * Update the timestamp of the last received heartbeat
     */
    void updateLastHeartbeat();

    /**
     * Start the heartbeat monitoring thread
     */
    void startHeartbeatThread();

    /**
     * Stop the heartbeat monitoring thread
     */
    void stopHeartbeatThread();

    /**
     * Get bytes from a string to send over socket
     * @param string The string to encode
     * @return byte array representation
     */
    byte[] encode(String string);

    /**
     * Convert received bytes to string
     * @param data The received byte array
     * @return decoded string
     */
    String decode(byte[] data);
}
