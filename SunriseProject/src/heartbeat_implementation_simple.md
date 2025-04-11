# Simple Heartbeat Implementation for KUKA Robot - Master Branch

This document provides a minimal implementation guide for adding heartbeat functionality to your KUKA Sunrise Java application based on the master branch codebase at https://github.com/Botman-Mike/KMRIIWA_ROS2_Java/tree/master/SunriseProject/src/API_ROS2_Sunrise.

## Overview

The ROS2 communication system expects to receive periodic messages from the robot to maintain connection status. When no data is received within 10 seconds, the connection is marked as down. A simple heartbeat mechanism on the robot side will resolve this issue.

## Implementation Guidelines

### 1. For TCP Connections

#### Modify `TCPSocket.java`:

```java
// In TCPSocket.java
// Add a heartbeat method to the existing class
public void sendHeartbeat() {
    // The TCP protocol requires 10-digit message length + message
    String heartbeatMsg = "heartbeat";
    String formattedMsg = String.format("%010d", heartbeatMsg.length()) + heartbeatMsg;
    
    try {
        if (socket != null && socket.isConnected()) {
            PrintWriter out = new PrintWriter(socket.getOutputStream(), true);
            out.print(formattedMsg);
            out.flush();
        }
    } catch (Exception e) {
        System.out.println("Error sending heartbeat: " + e.getMessage());
    }
}

// Add a simple heartbeat thread starter method
public void startHeartbeatThread() {
    Thread heartbeatThread = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    sendHeartbeat();
                    Thread.sleep(5000); // Send every 5 seconds
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    System.out.println("Heartbeat thread error: " + e.getMessage());
                }
            }
        }
    });
    heartbeatThread.setDaemon(true);
    heartbeatThread.start();
}



2. For UDP Connections
Modify UDPSocket.java:

// In UDPSocket.java
// Add a heartbeat method to the existing class
public void sendHeartbeat() {
    String heartbeatMsg = "heartbeat";
    
    try {
        if (socket != null && !socket.isClosed()) {
            byte[] buffer = heartbeatMsg.getBytes();
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length, address, port);
            socket.send(packet);
        }
    } catch (Exception e) {
        System.out.println("Error sending UDP heartbeat: " + e.getMessage());
    }
}

// Add a simple heartbeat thread starter method
public void startHeartbeatThread() {
    Thread heartbeatThread = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    sendHeartbeat();
                    Thread.sleep(5000); // Send every 5 seconds
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    System.out.println("UDP Heartbeat thread error: " + e.getMessage());
                }
            }
        }
    });
    heartbeatThread.setDaemon(true);
    heartbeatThread.start();
}

3. Integration into Main Application
Modify your KMRiiwaSunriseApplication.java to start the heartbeat threads:




// In KMRiiwaSunriseApplication.java
// After initializing your TCP or UDP sockets, start heartbeat threads

// For TCP sockets (add after socket initialization)
if (kmpCommanderTCP != null) {
    kmpCommanderTCP.startHeartbeatThread();
}
if (lbrCommanderTCP != null) {
    lbrCommanderTCP.startHeartbeatThread();
}

// For UDP sockets (add after socket initialization)
if (kmpCommanderUDP != null) {
    kmpCommanderUDP.startHeartbeatThread();
}
if (lbrCommanderUDP != null) {
    lbrCommanderUDP.startHeartbeatThread();
}


Key Points
Simplicity: This implementation adds minimal code to your existing codebase.

Protocol Compatibility:

TCP heartbeats follow the required format with 10-digit length header
UDP heartbeats are simple string messages
Timing:

Sends heartbeats every 5 seconds
ROS timeout is 10 seconds, so this provides a safety margin
Thread Safety:

Uses daemon threads (won't prevent application shutdown)
Handles exceptions gracefully
Testing the Implementation
After adding the heartbeat functionality:

Start your ROS2 system
Start your KUKA application
Check ROS2 logs for "Connection established" messages
Verify that no "Connection is down" warnings appear after a successful connection
Common Issues
Message Format Error: If TCP heartbeats aren't working, double-check the message format (10-digit length + message).

Port Configuration: Ensure your sockets are connecting to the correct ports on the ROS2 system.

Thread Interactions: If the application becomes unstable, ensure the heartbeat threads aren't interfering with other operations.

This minimal implementation should keep your connections alive with the ROS2 system while adding minimal complexity to your codebase.

