package API_ROS2_Sunrise;




public interface ISocket {
	
	public void close();
	public void send_message(String msg);
	public String receive_message();
	public byte[] encode(String string);
	public boolean isConnected();

    /**
     * Sends a heartbeat message
     */
    void sendHeartbeat();
    
    /**
     * Gets last heartbeat time
     */
    public long getLastHeartbeat();
    
    /**
     * Sets last heartbeat time
     */
    public void updateLastHeartbeat();

    /**
     * Starts the heartbeat thread
     */
    void startHeartbeatThread();

    /**
     * Stops the heartbeat thread
     */
    void stopHeartbeatThread();
}
