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
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

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

    // Reconnect scheduler and backoff
    private final ScheduledExecutorService reconnectScheduler = Executors.newSingleThreadScheduledExecutor();
    private static final long RECONNECT_BACKOFF_INITIAL_MS = 500;
    private static final long RECONNECT_BACKOFF_MAX_MS = 30000;
    private volatile long currentReconnectDelay = RECONNECT_BACKOFF_INITIAL_MS;

    public KMP_status_reader(int port, KmpOmniMove robot, String ConnectionType) {
        super(port, ConnectionType, "KMP status reader");
        assert port == 30001 : "Port mismatch for KMP_status_reader";
        logger.info(getClass().getSimpleName() + "|port " + port + " expected and initialized");
        this.kmp = robot;
        // schedule reconnect attempts
        scheduleReconnect();
        // Always start heartbeat thread for status socket
        getSocket().startHeartbeatThread();
    }

    /** Schedule next reconnect with exponential backoff */
    private void scheduleReconnect() {
        reconnectScheduler.schedule(new Runnable() {
            public void run() {
                try { reconnectTask(); } catch (Throwable t) { logger.error("Unexpected error in status reconnect", t);}            
            }
        }, currentReconnectDelay, TimeUnit.MILLISECONDS);
    }

    /** Attempt reconnect, reset backoff on success, apply jitter on failure */
    private void reconnectTask() {
        if (closed || Node.getShutdown()) return;
        if (Node.isPaused()) { scheduleReconnect(); return; }
        createSocket();
        if (socket instanceof TCPSocket) {
            try {
                ((TCPSocket) socket).TCPConn.setKeepAlive(true);
                ((TCPSocket) socket).TCPConn.setSoTimeout(5000);
            } catch (Exception ignore) {}
        }
        if (isSocketConnected()) {
            logger.info("Connection with KMP Status Node OK!");
            runmainthread();
            currentReconnectDelay = RECONNECT_BACKOFF_INITIAL_MS;
            logger.info("Status reconnect successful, backoff reset to " + currentReconnectDelay + " ms");
        } else {
            long base = currentReconnectDelay * 2;
            long jitter = (long)((Math.random()*0.4 - 0.2)*base);
            currentReconnectDelay = (int)Math.min(RECONNECT_BACKOFF_MAX_MS, base+jitter);
        }
        scheduleReconnect();
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
                // Add debug logs for header and body
                String body = toSend;
                String header = String.format("%010d", body.length());
                logger.debug(getClass().getSimpleName() + "|port " + port + " sending header: " + header);
                logger.debug(getClass().getSimpleName() + "|port " + port + " sending body: " + body);
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

    @Override
    public void close() {
        closed = true;
        // Stop heartbeat thread on close
        getSocket().stopHeartbeatThread();
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