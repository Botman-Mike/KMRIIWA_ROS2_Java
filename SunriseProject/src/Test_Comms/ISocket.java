package Test_Comms;

public interface ISocket {
    void send_message(String message);
    String receive_message();
    boolean isConnected();
    void close();
    byte[] encode(String string);
    String decode(byte[] data);
    void setReceiveBufferSize(int size);
}