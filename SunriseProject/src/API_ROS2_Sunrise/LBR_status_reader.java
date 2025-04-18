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

import com.kuka.roboticsAPI.deviceModel.LBR;

import java.util.logging.Logger;

public class LBR_status_reader extends Node {

    // Robot
    LBR lbr;
    private long last_sendtime = System.currentTimeMillis();
    private static final Logger logger = Logger.getLogger(LBR_status_reader.class.getName());

    public LBR_status_reader(int port, LBR robot, String ConnectionType) {
        super(port, ConnectionType, "LBR_status_reader"); // Set node_name in parent constructor
        this.lbr = robot;

        if (!(isSocketConnected())) {
            logger.info("Starting thread to connect LBR status node....");
            Thread monitorLBRStatusConnections = new MonitorLBRStatusConnectionsThread();
            monitorLBRStatusConnections.start();
        }
    }

    @Override
    public void run() {
        boolean pauseAlerted = false;
        while (isNodeRunning()) {
            if (Node.getShutdown()) break;
            if (Node.isPaused()) {
                if (!pauseAlerted) {
                    logger.info("=== SYSTEM PAUSED: All logging is now suppressed until resume ===");
                    pauseAlerted = true;
                }
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    break;
                }
                continue;
            } else {
                pauseAlerted = false;
            }
            if (System.currentTimeMillis() - last_sendtime > 30) {
                sendStatus();
            }
        }
    }

    private boolean getReadyToMove() {
        return lbr.isReadyToMove();
    }

    private String generateStatusString() {

        String toSend = ">lbr_statusdata ," + System.nanoTime() +
                ",ReadyToMove:" + getReadyToMove() +
                ",isLBRmoving:" + getisLBRMoving() +
                ",LBRsafetyStop:" + getEmergencyStop() +
                ",PathFinished:" + getisPathFinished();
        return toSend;
    }

    public void sendStatus() {
        String statusString = generateStatusString();
        last_sendtime = System.currentTimeMillis();

        if (isNodeRunning()) {
            try {
                this.socket.send_message(statusString);
                if (closed && !Node.isPaused()) {
                    logger.warning(this.node_name + " tried to send a message when application was closed");
                }
            } catch (Exception e) {
                if (!Node.isPaused()) {
                    logger.severe("Could not send " + this.node_name + " message to ROS: " + e);
                }
            }
        }
    }

    public class MonitorLBRStatusConnectionsThread extends Thread {
        public void run() {
            while (!(isSocketConnected()) && (!(closed))) {
                if (Thread.currentThread().isInterrupted()) {
                    logger.info("LBR status connection monitor thread interrupted, exiting");
                    break;
                }
                
                createSocket();
                if (isSocketConnected()) {
                    break;
                }
                try {
                    Thread.sleep(connection_timeout);
                } catch (InterruptedException e) {
                    logger.warning("LBR status connection monitor thread interrupted during sleep");
                    Thread.currentThread().interrupt(); // Restore interrupt status
                    break;
                }
            }
            if (!closed) {
                logger.info("Connection with LBR Status Node OK!");
                start(); // Use Thread's start() method instead of runmainthread
            }
        }
    }

    public void setLBRemergencyStop(boolean stop) {
        setEmergencyStop(stop);
    }

    @Override
    public void close() {
        closed = true;
        try {
            this.socket.close();
        } catch (Exception e) {
            logger.severe("Could not close LBR status connection: " + e);
        }
        logger.info("LBR status closed!");
    }

}
