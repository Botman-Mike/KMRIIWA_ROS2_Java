# KUKA-ROS2 Communication Protocol

## Connection Types
- TCP: Used for critical data requiring reliable delivery
  - Robot commands
  - Status updates
  - Heartbeat messages
- UDP: Used for high-frequency sensor data
  - LBR joint states/sensors
  - KMP laser scanner
  - KMP odometry

## Port Assignments
- 30001: KMP Status (TCP)
- 30002: KMP Command (TCP)
- 30003: KMP Laser (UDP)
- 30004: KMP Odometry (UDP) 
- 30005: LBR Command (TCP)
- 30006: LBR Status (TCP)
- 30007: LBR Sensors (UDP)

## Heartbeat Protocol
- TCP connections: 10-digit length prefix + "heartbeat"
- UDP connections: Simple "heartbeat" message
- Frequency: Every 5 seconds
- Timeout: 10 seconds

## Performance Optimizations
1. Separate UDP sockets for different sensor types
2. Increased UDP buffer sizes (65535 bytes)
3. Non-blocking socket operations
4. Thread priority management based on robot state

## Error Recovery
1. Automatic reconnection attempts
2. Graceful degradation during packet loss
3. Independent socket monitoring
4. Safety state integration
