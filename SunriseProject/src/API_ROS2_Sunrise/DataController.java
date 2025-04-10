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

// Implemented classes


// RoboticsAPI
import API_ROS2_Sunrise.ISocket;

import com.kuka.nav.fdi.DataConnectionListener;
import com.kuka.nav.fdi.DataListener;
import com.kuka.nav.fdi.data.CommandedVelocity;
import com.kuka.nav.fdi.data.Odometry;
import com.kuka.nav.fdi.data.RobotPose;
import com.kuka.nav.provider.LaserScan;

public class DataController implements DataListener, DataConnectionListener {

    public boolean fdi_isConnected;
    ISocket laser_socket;
    ISocket odometry_socket;
    long ms_sent = System.currentTimeMillis();
    Odometry odom;
    private ConnectionManager connectionManager;

    public DataController(ISocket laser_socket, ISocket odometry_socket) {
        this.fdi_isConnected = false;
        this.laser_socket = laser_socket;
        this.odometry_socket = odometry_socket;

        // Initialize ConnectionManager
        this.connectionManager = new ConnectionManager();

        // Set connection manager for sockets
        if (laser_socket instanceof TCPSocket) {
            ((TCPSocket) laser_socket).setConnectionManager(connectionManager);
        }
        if (odometry_socket instanceof TCPSocket) {
            ((TCPSocket) odometry_socket).setConnectionManager(connectionManager);
        }
        if (laser_socket instanceof UDPSocket) {
            ((UDPSocket) laser_socket).setConnectionManager(connectionManager);
        }
        if (odometry_socket instanceof UDPSocket) {
            ((UDPSocket) odometry_socket).setConnectionManager(connectionManager);
        }

        // Start heartbeat
        connectionManager.initializeHeartbeat();
    }

    @Override
    public void onNewCmdVelocity(CommandedVelocity arg0) {
        // TODO Auto-generated method stub

    }

    @Override
    public void onNewLaserData(LaserScan scan) {
        if (fdi_isConnected && this.laser_socket.isConnected()) {
            String scan_data = ">laserScan " + scan.getTimestamp() + " " + scan.getLaserId() + " " + scan.getRangesAsString();
            try {
                this.laser_socket.send_message(scan_data);
            } catch (Exception e) {
                LogUtil.logError("Could not send KMP laserdata to ROS: " + e);
            }
        }
    }

    @Override
    public void onNewOdometryData(Odometry odom) {
        long msg_time = System.currentTimeMillis();
        if (fdi_isConnected && this.odometry_socket.isConnected()) {
            try {
                if (msg_time - ms_sent >= 50) {
                    ms_sent = System.currentTimeMillis();
                    String odom_data = ">odometry " + odom.getTimestamp() + " " + odom.getPose().toString() + " " + odom.getVelocity().toString();
                    this.odometry_socket.send_message(odom_data);
                }
            } catch (Exception e) {
                LogUtil.logError("Could not send KMP odometry data to ROS: " + e);
            }
        }
    }

    @Override
    public void onNewRobotPoseData(RobotPose arg0) {
        // TODO Auto-generated method stub
    }

    @Override
    public void onConnectionClosed() {
        fdi_isConnected = false;
        LogUtil.logInfo("FDIConnection closed");
    }

    @Override
    public void onConnectionFailed(Exception arg0) {
        fdi_isConnected = false;
        LogUtil.logInfo("fdi_fail", "FDIConnection failed");
        LogUtil.logError("fdi_fail", arg0); // Using error level for exceptions
    }

    @Override
    public void onConnectionSuccessful() {
        fdi_isConnected = true;
        LogUtil.logInfo("FDIConnection successful");
        // Reset heartbeat state on successful connection
        if (connectionManager != null) {
            connectionManager.handleIncomingHeartbeat();
        }
    }

    @Override
    public void onConnectionTimeout() {
        LogUtil.logInfo("FDIConnection timeout");
    }

    @Override
    public void onReceiveError(Exception arg0) {
        LogUtil.logInfo("fdi_error", "FDIconnection - Received error");
        LogUtil.logError("fdi_error", arg0); // Using error level for exceptions
    }

    public void setLaserSocket(ISocket laser_socket2) {
        this.laser_socket = laser_socket2;
    }

    public void setOdometrySocket(ISocket odometry_socket2) {
        this.odometry_socket = odometry_socket2;
    }

    // Add to your cleanup/dispose method
    public void dispose() {
        if (connectionManager != null) {
            connectionManager.stopHeartbeat();
        }
    }
}
