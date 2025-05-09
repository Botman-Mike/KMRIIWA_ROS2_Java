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
import java.util.logging.Level;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class LBR_status_reader extends Node {

    // Robot
    LBR lbr;
    private long last_sendtime = System.currentTimeMillis();
    private static final Logger logger = Logger.getLogger(LBR_status_reader.class.getName());

    // Scheduler and backoff parameters
    private final ScheduledExecutorService reconnectScheduler = Executors.newSingleThreadScheduledExecutor();
    private static final long RECONNECT_BACKOFF_INITIAL_MS = 500;
    private static final long RECONNECT_BACKOFF_MAX_MS     = 30000;
    private volatile long currentReconnectDelay = RECONNECT_BACKOFF_INITIAL_MS;

    public LBR_status_reader(int port, LBR robot, String ConnectionType) {
        super(port, ConnectionType, "LBR_status_reader");
        assert port == 30006 : "Port mismatch for LBR_status_reader";
        logger.info(getClass().getSimpleName() + "|port " + port + " expected and initialized");
        this.lbr = robot;
        // start scheduler-based reconnect
        scheduleReconnect();
        // Always start heartbeat thread for status socket
        getSocket().startHeartbeatThread();
    }

    /** Schedule next reconnect with exponential backoff */
    private void scheduleReconnect() {
        reconnectScheduler.schedule(new Runnable() {
            public void run() {
                try { reconnectTask(); } catch (Throwable t) { logger.log(Level.SEVERE, "Unexpected error in LBR status reconnect", t); }
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
            logger.info("Connection with LBR Status Node OK!");
            // Restart heartbeat thread on successful reconnect
            getSocket().startHeartbeatThread();
            start();
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
                // Add debug logs for header and body
                String body = statusString;
                String header = String.format("%010d", body.length());
                logger.log(Level.FINE, getClass().getSimpleName() + "|port " + port + " sending header: " + header);
                logger.log(Level.FINE, getClass().getSimpleName() + "|port " + port + " sending body: " + body);
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

    // Old monitor thread removed; replaced by scheduler

    public void setLBRemergencyStop(boolean stop) {
        setEmergencyStop(stop);
    }

    @Override
    public void close() {
        reconnectScheduler.shutdownNow();
        // Stop heartbeat thread on close
        getSocket().stopHeartbeatThread();
        closed = true;
        try {
            this.socket.close();
        } catch (Exception e) {
            logger.severe("Could not close LBR status connection: " + e);
        }
        logger.info("LBR status closed!");
    }

}
