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
	
	public volatile boolean isConnected;
	private volatile boolean isHeartbeatRunning = true;
	private volatile boolean heartbeatRunning = false;
	private Thread heartbeatThread;
	DatagramSocket udpConn;
	DatagramPacket package_out;
	DatagramPacket package_in;
	private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");
	int COMport;
    static BindException b;
    String nodename;
    private boolean running;
    private long lastHeartbeat = System.currentTimeMillis();
    private static final long HEARTBEAT_TIMEOUT = 5000; // 5 second timeout

	
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
				System.out.println("Connecting  "+ this.nodename+ " to ROS over UDP on port: " + COMport); 
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
		        System.out.println(this.nodename+ " received packet data over UDP on port : " + COMport + " Message: " +s);  
		        
		        if(s.length()<1){
		    	   udpConn.close();
		       		isConnected=false;
		       		System.out.println( this.nodename+ "  did not receive any message in 3 seconds, shutting off");
		       		break;
		       	 }
		        udpConn.setSoTimeout(0);
		        isConnected=true;
			}
			catch(Exception e1){
		        System.out.println("ERROR connecting  "+ this.nodename+ " to ROS over UDP on port: " + this.COMport + " Error: " + e1);
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
			System.out.println( this.nodename+ " could not send package over UDP on port: "  + this.COMport + " error: " + e);
		}
	}
    
	@Override
	public String receive_message() {
	    if (!isConnected()) {
	        // Try to repair the connection
	        try {
	            System.out.println(nodename + " UDP Socket lost, attempting reconnection");
	            connect();
	            if (!isConnected()) {
	                return null;
	            }
	        } catch (Exception e) {
	            System.out.println(nodename + " UDP Socket reconnection failed: " + e.getMessage());
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
	        if (message != null && message.trim().equals("heartbeat")) {
	            updateLastHeartbeat();
	            return null; // Skip heartbeat messages
	        }
	        return message;
	    } catch (SocketTimeoutException e) {
	        // Timeout is normal, just return null
	        return null;
	    } catch (Exception e) {
	        // Only log if it's not a timeout and not due to shutdown
	        if (running && !(e instanceof InterruptedIOException)) {
	            System.out.println(nodename + " UDP receive error: " + e.getMessage());
	        }
	        return null;
	    }
	}
	
    @Override
	public void close() {
	    stopHeartbeatThread();
	    isHeartbeatRunning = false;
	    System.out.println(nodename + " UDP connection closing");
	    running = false;
	    
	    if (udpConn != null && !udpConn.isClosed()) {
	        try {
	            udpConn.close();
	            System.out.println(nodename + " UDP socket closed successfully");
	        } catch (Exception e) {
	            System.out.println(nodename + " UDP socket close error: " + e.getMessage());
	        }
	    }
	}

	@Override
	public boolean isConnected() {
	    if (udpConn == null || udpConn.isClosed()) {
	        System.out.println(nodename + " UDP Socket check: Socket is null or closed");
	        return false;
	    }
	    
	    try {
	        // For UDP, we don't have a true "connection" status like TCP
	        // Instead, check if socket is bound and not closed
	        if (!udpConn.isBound()) {
	            System.out.println(nodename + " UDP Socket check: Socket is not bound");
	            return false;
	        }
	        
	        // Additional check: try a non-destructive socket operation
	        udpConn.getReceiveBufferSize(); // This will throw if socket is invalid
	        return (System.currentTimeMillis() - lastHeartbeat) < HEARTBEAT_TIMEOUT;
	    } catch (Exception e) {
	        System.out.println(nodename + " UDP Socket check exception: " + e.getMessage());
	        return false;
	    }
	}

    @Override
    public long getLastHeartbeat() {
        return lastHeartbeat;
    }
    
    @Override
    public void updateLastHeartbeat() {
        lastHeartbeat = System.currentTimeMillis();
    }

    @Override
    public void sendHeartbeat() {
        if (!isConnected || udpConn == null) {
            return;
        }
        try {
            byte[] buffer = "heartbeat".getBytes(UTF8_CHARSET);
            package_out.setData(buffer);
            package_out.setLength(buffer.length);
            udpConn.send(package_out);
        } catch (Exception e) {
            System.out.println(nodename + " UDP heartbeat error: " + e.getMessage());
        }
    }

    @Override
    public void startHeartbeatThread() {
        if (heartbeatThread != null && heartbeatThread.isAlive()) {
            return;
        }
        
        heartbeatRunning = true;
        heartbeatThread = new Thread(new Runnable() {
            public void run() {
                while (heartbeatRunning && !Thread.currentThread().isInterrupted()) {
                    try {
                        sendHeartbeat();
                        Thread.sleep(5000);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                }
            }
        }, nodename + "-heartbeat");
        heartbeatThread.setDaemon(true);
        heartbeatThread.start();
    }

    public void stopHeartbeatThread() {
        heartbeatRunning = false;
        if (heartbeatThread != null) {
            heartbeatThread.interrupt();
            heartbeatThread = null;
        }
    }
}
