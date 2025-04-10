package API_ROS2_Sunrise;


import java.io.IOException;

import java.net.DatagramPacket;

import java.nio.charset.Charset;
import java.net.Socket;
import java.net.ServerSocket;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.EOFException;
import java.io.InterruptedIOException;

import java.net.SocketException;
import java.net.SocketTimeoutException;

import API_ROS2_Sunrise.ISocket;

public class TCPSocket implements ISocket{
	
	
	public volatile boolean isConnected;
	public Socket TCPConn;
	DatagramPacket package_out;
	DatagramPacket package_in;
	public PrintWriter outputStream;
	public BufferedReader inputStream;
	private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");
	int COMport;
	String nodename;
	private boolean running = true;
	private ServerSocket serverSocket;
	private long lastHeartbeat = System.currentTimeMillis();
	private static final long HEARTBEAT_TIMEOUT = 5000; // 5 second timeout
	private volatile boolean isHeartbeatRunning = true;
	private volatile boolean heartbeatRunning = false;
	private Thread heartbeatThread;
	
	public TCPSocket(int port, String node_name) {
		isConnected = false;
		COMport = port;
		this.nodename = node_name;
		TCPConn = connect();
	}
	
	public Socket connect()
	{
		while (true){
			try{
				String remotePC = "172.31.1.206";
				//String NUC = "192.168.10.120";

				TCPConn = new Socket(remotePC,COMport);
				TCPConn.setReuseAddress(true);
				System.out.println(this.nodename + " connecting to ROS over TCP on port: "+ COMport);
				break;
			}
			catch(IOException e1){
				System.out.println("Could not connect "+ this.nodename+ " to ROS over TCP on port : "+ this.COMport + " Error: " +e1);
			return null;
			}
		}
		try{
			outputStream = new PrintWriter(TCPConn.getOutputStream(),true);
			inputStream = new BufferedReader(new InputStreamReader(TCPConn.getInputStream()));
			isConnected=true;
			return TCPConn;
		}catch(Exception e){
			System.out.println("Error creating I/O ports for TCP communication for  "+ this.nodename+ " on port: "+ this.COMport + " Error: " +e);
			return null;
		}
		
		}
	
	public void send_message(String buffer){
		int len = (this.encode(buffer)).length;
		String send_string = String.format("%010d", len) + " "+buffer;
		outputStream.write(send_string);
		outputStream.flush();
	}
	
	@Override
	public boolean isConnected() {
	    if (TCPConn == null) {
	        System.out.println(nodename + " TCP Socket check: Socket is null");
	        return false;
	    }
	    
	    try {
	        // First, check if socket is marked as closed
	        if (TCPConn.isClosed()) {
	            System.out.println(nodename + " TCP Socket check: Socket is closed");
	            return false;
	        }
	        
	        // Check if the socket is connected
	        if (!TCPConn.isConnected()) {
	            System.out.println(nodename + " TCP Socket check: Socket not connected");
	            return false;
	        }
	        
	        // Check if connection is still alive by testing if it's input shutdown
	        if (TCPConn.isInputShutdown() || TCPConn.isOutputShutdown()) {
	            System.out.println(nodename + " TCP Socket check: Input or output is shutdown");
	            return false;
	        }
	        
	        return (System.currentTimeMillis() - lastHeartbeat) < HEARTBEAT_TIMEOUT;
	    } catch (Exception e) {
	        System.out.println(nodename + " TCP Socket check exception: " + e.getMessage());
	        return false;
	    }
	}

	@Override
	public String receive_message() {
	    if (!isConnected()) {
	        // Try to repair the connection
	        try {
	            System.out.println(nodename + " TCP Socket lost, attempting reconnection");
	            connect();
	            if (!isConnected()) {
	                return null;
	            }
	        } catch (Exception e) {
	            System.out.println(nodename + " TCP Socket reconnection failed: " + e.getMessage());
	            return null;
	        }
	    }
	    
	    try {
	        // Set a timeout for socket operations
	        TCPConn.setSoTimeout(1000); // 1 second timeout
	        
	        BufferedReader reader = new BufferedReader(new InputStreamReader(TCPConn.getInputStream()));
	        String message = reader.readLine();
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
	            System.out.println(nodename + " TCP receive error: " + e.getMessage());
	            
	            // Try to detect if connection is broken
	            if (e instanceof SocketException || 
	                e instanceof EOFException || 
	                e.getMessage().contains("Connection reset")) {
	                
	                System.out.println(nodename + " TCP Connection appears broken, forcing reconnection");
	                try {
	                    TCPConn.close();
	                } catch (Exception ex) {
	                    // Ignore
	                }
	                TCPConn = null;
	            }
	        }
	        return null;
	    }
	}

	@Override
	public void close() {
	    stopHeartbeatThread();
	    isHeartbeatRunning = false;
	    System.out.println(nodename + " TCP connection closing");
	    running = false;
	    
	    if (TCPConn != null) {
	        try {
	            TCPConn.close();
	            System.out.println(nodename + " TCP socket closed successfully");
	        } catch (Exception e) {
	            System.out.println(nodename + " TCP socket close error: " + e.getMessage());
	        }
	    }
	    
	    if (serverSocket != null) {
	        try {
	            serverSocket.close();
	            System.out.println(nodename + " TCP server socket closed successfully");
	        } catch (Exception e) {
	            System.out.println(nodename + " TCP server socket close error: " + e.getMessage());
	        }
	    }
	}
	
	@Override
	public void sendHeartbeat() {
	    if (!isConnected || outputStream == null) {
	        return;
	    }
	    String heartbeatMsg = "heartbeat";
	    String formattedMsg = String.format("%010d", heartbeatMsg.length()) + " " + heartbeatMsg;
	    try {
	        outputStream.write(formattedMsg);
	        outputStream.flush();
	    } catch (Exception e) {
	        System.out.println(nodename + " TCP heartbeat error: " + e.getMessage());
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

	public String decode(byte[] data) {
		String message = new String(data,0,data.length, UTF8_CHARSET);
        return message;
	}

	@Override
	public byte[] encode(String string) {
		return string.getBytes(UTF8_CHARSET);
	}
	
	@Override 
	public long getLastHeartbeat() {
	    return lastHeartbeat;
	}
	
	@Override
	public void updateLastHeartbeat() {
	    lastHeartbeat = System.currentTimeMillis(); 
	}

}
