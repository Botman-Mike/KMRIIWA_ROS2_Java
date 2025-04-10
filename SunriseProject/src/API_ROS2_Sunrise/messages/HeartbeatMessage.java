package API_ROS2_Sunrise.messages;

public class HeartbeatMessage {
    private long timestamp;
    private String status;

    public HeartbeatMessage() {
        this.timestamp = System.currentTimeMillis();
        this.status = "alive";
    }

    public byte[] toBytes() {
        String message = String.format("heartbeat,%d,%s", timestamp, status);
        return message.getBytes();
    }
}