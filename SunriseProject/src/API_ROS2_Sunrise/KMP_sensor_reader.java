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
import java.util.logging.Logger;

public class KMP_sensor_reader extends Node {
    private static final Logger logger = Logger.getLogger(KMP_sensor_reader.class.getName());
    public DataController listener;
    private boolean closed = false;
    private ISocket laser_socket;
    private ISocket odometry_socket;
    private int connection_timeout = 2000;

    public KMP_sensor_reader(int laserport, int odomport, String LaserConnectionType, String OdometryConnectionType) {
        super(laserport, LaserConnectionType, odomport, OdometryConnectionType, "KMP sensor reader");
        logger.info("Initializing KMP_sensor_reader with Laser port: " + laserport + ", Odom port: " + odomport);
        createSocket("Laser");
        createSocket("Odom");
        logger.info("Laser socket created: " + (laser_socket != null));
        logger.info("Odometry socket created: " + (odometry_socket != null));
        listener = new DataController(laser_socket, odometry_socket);
        listener.startHeartbeat();
        if (!isLaserSocketConnected()) {
            logger.warning("Laser socket not connected at startup. Starting monitor thread.");
            new MonitorLaserConnectionThread().start();
        } else {
            logger.info("Laser socket connected at startup.");
        }
        if (!isOdometrySocketConnected()) {
            logger.warning("Odometry socket not connected at startup. Starting monitor thread.");
            new MonitorOdometryConnectionThread().start();
        } else {
            logger.info("Odometry socket connected at startup.");
        }
    }

    public void subscribe_kmp_laser_data() {
        // Implement your logic to read laser data from the socket and forward to DataController
        // Example: String scanData = ...;
        // listener.sendLaserScan(scanData);
    }

    public void subscribe_kmp_odometry_data() {
        // Implement your logic to read odometry data from the socket and forward to DataController
        // Example: String odomData = ...;
        // listener.sendOdometry(odomData);
    }

    @Override
    public void run() {
        if (isLaserSocketConnected()) {
            subscribe_kmp_laser_data();
        }
        if (isOdometrySocketConnected()) {
            subscribe_kmp_odometry_data();
        }
        while (!closed) {
            // Main loop for reading and forwarding data
            // You may want to poll sockets and call subscribe_kmp_laser_data/subscribe_kmp_odometry_data
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    @Override
    public void close() {
        closed = true;
        try {
            listener.close();
        } catch (Exception e) {
            logger.warning("Could not close DataController: " + e);
        }
        try {
            if (laser_socket != null) laser_socket.close();
        } catch (Exception b) {
            logger.warning("Can not close laser socket connection! : " + b);
        }
        try {
            if (odometry_socket != null) odometry_socket.close();
        } catch (Exception c) {
            logger.warning("Can not close odometry socket connection! : " + c);
        }
        logger.info("KMP sensor closed!");
    }

    public boolean isLaserSocketConnected() {
        boolean res = false;
        try {
            res = laser_socket != null && laser_socket.isConnected();
            logger.info("Laser socket connected: " + res);
        } catch (Exception e) {
            logger.severe("Error checking Laser socket connection: " + e);
        }
        return res;
    }

    public boolean isOdometrySocketConnected() {
        boolean res = false;
        try {
            res = odometry_socket != null && odometry_socket.isConnected();
            logger.info("Odometry socket connected: " + res);
        } catch (Exception e) {
            logger.severe("Error checking Odometry socket connection: " + e);
        }
        return res;
    }

    @Override
    public boolean isSocketConnected() {
        return (isOdometrySocketConnected() || isLaserSocketConnected());
    }

    public void createSocket(String Type) {
        if (Type.equals("Laser")) {
            logger.info("Creating Laser UDPSocket on port 30003");
            laser_socket = new UDPSocket(30003, "LaserSocket");
        } else if (Type.equals("Odom")) {
            logger.info("Creating Odometry UDPSocket on port 30004");
            odometry_socket = new UDPSocket(30004, "OdomSocket");
        }
    }

    public class MonitorLaserConnectionThread extends Thread {
        public void run() {
            logger.info("MonitorLaserConnectionThread started");
            while (!isLaserSocketConnected() && !closed) {
                logger.warning("Laser socket not connected. Attempting to create socket.");
                createSocket("Laser");
                if (isLaserSocketConnected()) {
                    logger.info("Laser socket connected in monitor thread.");
                    break;
                }
                try {
                    Thread.sleep(connection_timeout);
                } catch (InterruptedException e) {
                    logger.warning("MonitorLaserConnectionThread interrupted");
                    break;
                }
            }
            if (!closed) {
                listener.setLaserSocket(laser_socket);
                subscribe_kmp_laser_data();
                logger.info("Connection with KMP Laser Node OK!");
            }
        }
    }

    public class MonitorOdometryConnectionThread extends Thread {
        public void run() {
            logger.info("MonitorOdometryConnectionThread started");
            while (!isOdometrySocketConnected() && !closed) {
                logger.warning("Odometry socket not connected. Attempting to create socket.");
                createSocket("Odom");
                if (isOdometrySocketConnected()) {
                    logger.info("Odometry socket connected in monitor thread.");
                    break;
                }
                try {
                    Thread.sleep(connection_timeout);
                } catch (InterruptedException e) {
                    logger.warning("MonitorOdometryConnectionThread interrupted");
                    break;
                }
            }
            if (!closed) {
                listener.setOdometrySocket(odometry_socket);
                subscribe_kmp_odometry_data();
                logger.info("Connection with KMP Odometry Node OK!");
            }
        }
    }
}