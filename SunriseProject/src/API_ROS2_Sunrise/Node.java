// Copyright 2019 Nina Marie Wahl og Charlotte Heggem.
// Copyright 2019 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
package API_ROS2_Sunrise;

import API_ROS2_Sunrise.ISocket;
import API_ROS2_Sunrise.TCPSocket;
import API_ROS2_Sunrise.UDPSocket;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;

public abstract class Node extends Thread {
	
	// Runtime Variables
	private volatile static boolean shutdown;
	public volatile boolean closed = false;
	private static volatile boolean EmergencyStop;
	private volatile static boolean PathFinished = true;
	protected volatile static boolean isKMPmoving = false;
	private volatile static boolean isLBRmoving = false;
	private volatile static boolean isKMPconnected;
	private volatile static boolean isLBRconnected;

	// Socket
	protected ISocket socket;
	private String ConnectionType;
	private int port;
	public static int connection_timeout = 5000;

	// For KMP sensor reader:
	protected ISocket laser_socket;
	protected ISocket odometry_socket;
	private int KMP_laser_port;
	private int KMP_odometry_port;
	private String LaserConnectionType;
	private String OdometryConnectionType;
	
	protected String node_name;

	private long lastConnectionCheckTime = 0;
	private boolean lastKnownConnectionState = false;
	private int connectionFailCount = 0;

	protected DataController dataController;
	
	public Node(int port1, String Conn1, int port2, String Conn2, String node_name) {
		if (node_name == null) {
			System.out.println("This is a test, node_name is null in Node.");
		}
		this.KMP_laser_port = port1;
		this.KMP_odometry_port = port2;
		this.LaserConnectionType = Conn1;
		this.OdometryConnectionType = Conn2;
		this.node_name = node_name;
		setShutdown(false);
		setEmergencyStop(false);
		
		createSocket("Laser");
		createSocket("Odom");
	}
	
	public Node(int port, String Conn, String nodeName) {
		this.ConnectionType = Conn;
		this.port = port;
		this.node_name = nodeName;
		setShutdown(false);
		setEmergencyStop(false);
		
		createSocket();
	}
	
	public void createSocket() {
		if (this.ConnectionType == "TCP") {
			this.socket = new TCPSocket(this.port, this.node_name);
		} else {
			this.socket = new UDPSocket(this.port, this.node_name);
		}
	}
	
	public void createSocket(String Type) {
		if (Type == "Laser") {
			if (LaserConnectionType == "TCP") {
				this.laser_socket = new TCPSocket(KMP_laser_port, this.node_name);
			} else {
				this.laser_socket = new UDPSocket(KMP_laser_port, this.node_name);
			}
		} else if (Type == "Odom") {
			if (OdometryConnectionType == "TCP") {
				this.odometry_socket = new TCPSocket(KMP_odometry_port, this.node_name);
			} else {
				this.odometry_socket = new UDPSocket(KMP_odometry_port, this.node_name);
			}
		}
	}
	
	public boolean isSocketConnected() {
		return this.socket.isConnected();
	}
	
	public boolean isNodeRunning() {
		if (closed || getShutdown()) {
			return false;
		}
		
		long now = System.currentTimeMillis();
		// Only check socket connection every second to avoid overwhelming with checks
		if (now - lastConnectionCheckTime > 1000) {
			lastConnectionCheckTime = now;
			boolean currentConnected = this.socket != null && this.socket.isConnected();
			
			if (currentConnected) {
				// Reset failure count if connection is working
				connectionFailCount = 0;
				lastKnownConnectionState = true;
			} else {
				// Only count actual transitions to disconnected state
				if (lastKnownConnectionState) {
					System.out.println(node_name + ": Connection state changed to DISCONNECTED");
					connectionFailCount++;
					lastKnownConnectionState = false;
				}
				
				// Try to reconnect on first failure
				if (connectionFailCount == 1) {
					ensureSocketConnection();
				}
				
				// Only report node as not running if socket has been disconnected for some time
				// This prevents brief connection hiccups from stopping the node
				if (connectionFailCount >= 3) {
					System.out.println(node_name + ": Connection persistently lost, node will stop running");
					return false;
				}
			}
		}
		
		return !closed && !getShutdown();
	}
	
	public static void setEmergencyStop(boolean es) {
		EmergencyStop = es;
	}
	
	public boolean getEmergencyStop() {
		return EmergencyStop;
	}
	
	public void runmainthread() {
		this.run();
	}
	
	public boolean getShutdown() {
		return shutdown;
	}
	
	public void setShutdown(boolean in) {
		System.out.println("shutdown set by " + this.node_name + " to " + in);
		shutdown = in;
	}
	
	@Override
	public void run() {
	    try {
	        while (isNodeRunning()) {
	            // Check if thread has been interrupted
	            if (Thread.currentThread().isInterrupted()) {
	                System.out.println(node_name + " thread was interrupted, exiting");
	                break;
	            }
	            
	            try {
	                // Use a timeout on socket operations
	                String message = socket.receive_message();
	                
	                // Process the message if not null
	                if (message != null) {
	                    // Process message
	                }
	                
	                // Small sleep to prevent CPU thrashing
	                Thread.sleep(10);
	            } catch (InterruptedException ie) {
	                Thread.currentThread().interrupt(); // Restore interrupt status
	                System.out.println(node_name + " thread interrupted during sleep");
	                break;
	            } catch (Exception e) {
	                if (Thread.currentThread().isInterrupted() || getShutdown()) {
	                    break;
	                }
	                // Log error but continue running
	            }
	        }
	    } finally {
	        System.out.println(node_name + " thread ending");
	        // Close resources here
	        close();
	    }
	}
	
	public abstract void close();
	
	public boolean getisPathFinished() {
		return PathFinished;
	}
	
	public void setisPathFinished(boolean in) {
		PathFinished = in;
	}
	
	public boolean getisLBRMoving() {
		return isLBRmoving;
	}
	
	public void setisLBRMoving(boolean in) {
		isLBRmoving = in;
	}
	
	public boolean getisKMPMoving() {
		return isKMPmoving;
	}
	
	public void setisKMPMoving(boolean in) {
		isKMPmoving = in;
	}
	
	public boolean getisLBRConnected() {
		return isLBRconnected;
	}
	
	public void setisLBRConnected(boolean in) {
		isLBRconnected = in;
	}
	
	public boolean getisKMPConnected() {
		return isKMPconnected;
	}
	
	public void setisKMPConnected(boolean in) {
		isKMPconnected = in;
	}

	public void clearEmergencyStopIfSafe() {
	    // Only clear if the device is actually ready to move again
	    try {
	        if (this instanceof KMP_commander) {
	            KmpOmniMove kmp = ((KMP_commander)this).kmp;
	            if (kmp != null && kmp.isReadyToMove() && 
	                !kmp.getSafetyState().toString().contains("WARNING_FIELD")) {
	                setEmergencyStop(false);
	                System.out.println("Emergency stop cleared based on safety state");
	            }
	        }
	    } catch (Exception e) {
	        System.out.println("Error checking if emergency stop can be cleared: " + e.getMessage());
	    }
	}

	public void ensureSocketConnection() {
	    if (!isSocketConnected()) {
	        System.out.println(node_name + ": Socket connection lost, attempting to reconnect");
	        try {
	            // Close existing if needed
	            if (socket != null) {
	                try {
	                    socket.close();
	                } catch (Exception e) {
	                    // Ignore
	                }
	            }
	            
	            // Create new socket
	            createSocket();
	            
	            if (isSocketConnected()) {
	                System.out.println(node_name + ": Successfully reconnected socket");
	            } else {
	                System.out.println(node_name + ": Failed to reconnect socket");
	            }
	        } catch (Exception e) {
	            System.out.println(node_name + ": Exception during socket reconnection: " + e.getMessage());
	        }
	    }
	}

	public ISocket getSocket() {
	    return this.socket;
	}
	
	public void setDataController(DataController controller) {
	    this.dataController = controller;
	}
}
