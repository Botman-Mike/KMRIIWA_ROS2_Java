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

//RoboticsAPI
import com.kuka.roboticsAPI.deviceModel.OperationMode;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class KMP_status_reader extends Node {

    private static final Logger logger = LoggerFactory.getLogger(KMP_status_reader.class);

    // Robot
    KmpOmniMove kmp;

    // Status variables
    private OperationMode operation_mode = null;
    private Object isReadyToMove = null;
    private volatile boolean WarningField = false;
    private volatile boolean ProtectionField = false;
    private long last_sendtime = System.currentTimeMillis();

    public KMP_status_reader(int port, KmpOmniMove robot, String ConnectionType) {
        super(port, ConnectionType, "KMP status reader");

        this.kmp = robot;
        if (!(isSocketConnected())) {
            Thread monitorKMPStatusConnections = new MonitorKMPStatusConnectionsThread();
            monitorKMPStatusConnections.start();
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
                updateOperationMode();
                updateReadyToMove();
                updateWarningFieldState();
                updateProtectionFieldState();
                sendStatus();
            }

            if (!isSocketConnected() || (closed)) {
                break;
            }
        }
    }

    private void updateOperationMode() {
        this.operation_mode = kmp.getOperationMode();
    }

    private void updateReadyToMove() {
        this.isReadyToMove = kmp.isReadyToMove();
    }

    private void updateWarningFieldState() {
        try {
            // true = violation
            this.WarningField = kmp.getMobilePlatformSafetyState().isWarningFieldBreached();
        } catch (Exception e) {
        }
    }

    private void updateProtectionFieldState() {
        try {
            // true = violation
            this.ProtectionField = kmp.getMobilePlatformSafetyState().isSafetyFieldBreached();
        } catch (Exception e) {
        }
    }

    private String generateStatusString() {
        return ">kmp_statusdata ," + System.nanoTime() +
                ",OperationMode:" + this.operation_mode.toString() +
                ",ReadyToMove:" + this.isReadyToMove +
                ",WarningField:" + !this.WarningField +
                ",ProtectionField:" + !this.ProtectionField +
                ",isKMPmoving:" + getisKMPMoving() +
                ",KMPsafetyStop:" + getEmergencyStop();
    }

    public void sendStatus() {
        String toSend = this.generateStatusString();
        last_sendtime = System.currentTimeMillis();
        if (isNodeRunning()) {
            try {
                this.socket.send_message(toSend);
                if (closed && !Node.isPaused()) {
                    logger.warn("KMP status sender selv om han ikke faar lov");
                }
            } catch (Exception e) {
                if (!Node.isPaused()) {
                    logger.error("Could not send KMP status message to ROS: " + e);
                }
            }
        }
    }

    public class MonitorKMPStatusConnectionsThread extends Thread {
        public void run() {
            while (!(isSocketConnected()) && (!(closed))) {
                if (Thread.currentThread().isInterrupted()) {
                    logger.info("KMP status connection monitor thread interrupted, exiting");
                    break;
                }

                createSocket();
                if (isSocketConnected()) {
                    break;
                }
                try {
                    Thread.sleep(connection_timeout);
                } catch (InterruptedException e) {
                    logger.info("KMP status connection monitor thread interrupted during sleep");
                    Thread.currentThread().interrupt(); // Restore interrupt status
                    break;
                }
            }
            if (!closed) {
                logger.info("Connection with KMP Status Node OK!");
                runmainthread();
            }
        }
    }

    @Override
    public void close() {
        closed = true;
        try {
            this.socket.close();
        } catch (Exception e) {
            logger.error("Could not close KMP status connection: " + e);
        }
        if (!Node.isPaused()) {
            logger.info("KMP status closed!");
        }
    }
}