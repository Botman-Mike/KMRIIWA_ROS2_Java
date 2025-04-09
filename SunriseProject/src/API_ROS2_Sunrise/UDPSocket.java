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


public class UDPSocket implements ISocket{
	
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
		udpConn = connect();
		this.nodename=node_name;
	}
	
	public DatagramSocket connect()
	{
		while (!isConnected){
			try{
				LogUtil.logInfo("Connecting  "+ this.nodename+ " to ROS over UDP on port: " + COMport); 
		    	int kuka_port = this.COMport; // change this if cannot bind error
		        InetSocketAddress socket_address = new InetSocketAddress(kuka_port);
		    	
		    	String ros_host = "172.31.1.206"; // REMOTE PC

		    	int ros_port = this.COMport;
		        InetAddress address = InetAddress.getByName(ros_host);

		        
		        udpConn = new DatagramSocket(null); 
		        udpConn.setReuseAddress(true);
		        udpConn.bind(socket_address);
		        
		   
		        
		        byte buf[] = "HALLA ROS".getBytes();
		        byte buf1[] = new byte[1024]; 
		        
		        package_out = new DatagramPacket(buf, buf.length, address, ros_port); 
		        package_in = new DatagramPacket(buf1, buf1.length); 
		      
		        // send() method 
		        udpConn.send(package_out); 
		  
		        // receive() method 
		        udpConn.setSoTimeout(3000); //ms

		        udpConn.receive(package_in); 
		        String s = decode(package_in);
		        LogUtil.logInfo(this.nodename+ " received packet data over UDP on port : " + COMport + " Message: " +s);  
		        
		        if(s.length()<1){
		    	   udpConn.close();
		       		isConnected=false;
		       		LogUtil.logInfo( this.nodename+ "  did not receive any message in 3 seconds, shutting off");
		       		break;
		       	 }
		        udpConn.setSoTimeout(0);
		        isConnected=true;
			}
			catch(Exception e1){
		        LogUtil.logInfo("ERROR connecting  "+ this.nodename+ " to ROS over UDP on port: " + this.COMport + " Error: " + e1);
		        isConnected=false;
		        close();
				b = new BindException();
		        if(e1.getClass().equals(b.getClass())){
		        	e1.printStackTrace();
		        	break;
		        }
		        break;
		        }
			}
		return udpConn;
	}
	
	public String decode(DatagramPacket pack)  {
    	byte[] data = pack.getData();
    	String message = new String(data,0,pack.getLength(), UTF8_CHARSET);
        return message;

    }

	@Override
    public byte[] encode(String string) {
        return string.getBytes(UTF8_CHARSET);
    }
    
	@Override
    public void send_message(String msg)
	{
    	byte[] bytlist = msg.getBytes(UTF8_CHARSET);
    	package_out.setData(bytlist);
    	package_out.setLength(bytlist.length);
     try {
			udpConn.send(package_out);
		} catch (Exception e) {
			LogUtil.logInfo( this.nodename+ " could not send package over UDP on port: "  + this.COMport + " error: " + e);
		}
	}
    
	@Override
	public String receive_message() {
	    if (!isConnected()) {
	        // Try to repair the connection
	        try {
	            LogUtil.logInfo(nodename + " UDP Socket lost, attempting reconnection");
	            connect();
	            if (!isConnected()) {
	                return null;
	            }
	        } catch (Exception e) {
	            LogUtil.logInfo(nodename + " UDP Socket reconnection failed: " + e.getMessage());
	            return null;
	        }
	    }
	    
	    try {
	        // Set a timeout for receive operations
	        udpConn.setSoTimeout(1000); // 1 second timeout
	        
	        byte[] buffer = new byte[1024];
	        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
	        
	        udpConn.receive(packet);
	        
	        String message = new String(packet.getData(), 0, packet.getLength());
	        return message;
	    } catch (SocketTimeoutException e) {
	        // Timeout is normal, just return null
	        return null;
	    } catch (Exception e) {
	        // Only log if it's not a timeout and not due to shutdown
	        if (running && !(e instanceof InterruptedIOException)) {
	            LogUtil.logInfo(nodename + " UDP receive error: " + e.getMessage());
	        }
	        return null;
	    }
	}
	
    @Override
	public void close() {
	    LogUtil.logInfo(nodename + " UDP connection closing");
	    running = false;
	    
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

}
