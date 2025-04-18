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
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package API_ROS2_Sunrise;

import com.kuka.task.ITaskLogger;
import javax.inject.Inject;

/**
 * DataController handles direct socket communication with ROS2 for laser and odometry data.
 * No FDI dependencies. Includes heartbeat support.
 */
public class DataController {
    private ISocket laser_socket;
    private ISocket odometry_socket;
    private long ms_sent = System.currentTimeMillis();
    private HeartbeatThread heartbeatThread;
    private volatile boolean running = false;

    @Inject
    private ITaskLogger logger;

    public DataController(ISocket laser_socket, ISocket odometry_socket) {
        this.laser_socket = laser_socket;
        this.odometry_socket = odometry_socket;
    }

    // Example: send laser scan data to ROS
    public void sendLaserScan(String scanData) {
        if (laser_socket != null && laser_socket.isConnected()) {
            try {
                laser_socket.send_message(scanData);
            } catch (Exception e) {
                logger.error("Could not send laser data to ROS: " + e);
            }
        }
    }

    // Example: send odometry data to ROS
    public void sendOdometry(String odomData) {
        if (odometry_socket != null && odometry_socket.isConnected()) {
            try {
                long msg_time = System.currentTimeMillis();
                if (msg_time - ms_sent >= 50) {
                    ms_sent = msg_time;
                    odometry_socket.send_message(odomData);
                }
            } catch (Exception e) {
                logger.error("Could not send odometry data to ROS: " + e);
            }
        }
    }

    // Heartbeat support
    public void startHeartbeat() {
        if (heartbeatThread == null || !heartbeatThread.isAlive()) {
            running = true;
            heartbeatThread = new HeartbeatThread();
            heartbeatThread.start();
        }
    }

    public void stopHeartbeat() {
        running = false;
        if (heartbeatThread != null) {
            heartbeatThread.interrupt();
        }
    }

    private class HeartbeatThread extends Thread {
        @Override
        public void run() {
            while (running) {
                try {
                    if (laser_socket != null && laser_socket.isConnected()) {
                        laser_socket.sendHeartbeat();
                    }
                    if (odometry_socket != null && odometry_socket.isConnected()) {
                        odometry_socket.sendHeartbeat();
                    }
                    Thread.sleep(1000); // 1 second heartbeat
                } catch (InterruptedException e) {
                    break;
                } catch (Exception e) {
                    logger.warn("Heartbeat error: " + e);
                }
            }
        }
    }

    public void setLaserSocket(ISocket laser_socket2) {
        this.laser_socket = laser_socket2;
    }

    public void setOdometrySocket(ISocket odometry_socket2) {
        this.odometry_socket = odometry_socket2;
    }

    public void close() throws Exception {
        stopHeartbeat();
        if (laser_socket != null) {
            laser_socket.close();
        }
        if (odometry_socket != null) {
            odometry_socket.close();
        }
    }
}
