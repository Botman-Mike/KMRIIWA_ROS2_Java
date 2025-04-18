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

import com.kuka.roboticsAPI.deviceModel.OperationMode;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.Device;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import com.kuka.roboticsAPI.controllerModel.IControllerStateListener;
import com.kuka.roboticsAPI.controllerModel.DispatchedEventData;
import com.kuka.roboticsAPI.controllerModel.StatePortData;

public class SafetyStateListener implements IControllerStateListener {
    private static final Logger logger = LoggerFactory.getLogger(SafetyStateListener.class);

    private Controller controller;
    private LBR_commander lbr_commander;
    private KMP_commander kmp_commander;

    private volatile boolean isPaused = false;
    private long lastPauseLogTime = 0;
    private static final long PAUSE_LOG_INTERVAL_MS = 2000;

    public SafetyStateListener(Controller cont, LBR robot, LBR_commander lbrCmd, KMP_commander kmpCmd,
                              LBR_status_reader lbrStatus, KMP_status_reader kmpStatus) {
        this.controller = cont;
        this.lbr_commander = lbrCmd;
        this.kmp_commander = kmpCmd;
    }

    public void startSafetyStateListener() {
        controller.addControllerListener(this);
    }

    public void stop() {
        logger.info("SafetyStateListener: Stopping listener");
        try {
            controller.removeControllerListener(this);
            logger.info("SafetyStateListener: Listener stopped successfully");
        } catch (Exception e) {
            logger.error("SafetyStateListener: Error removing listener: " + e.getMessage());
        }
    }

    public void onSafetyStateChanged(Device device, SunriseSafetyState safetyState) {
        logger.info("Safety state changed on device: " + device.getName());
        logger.info("Full safety state: " + safetyState);

        boolean isPauseActive = false;

        // Check emergency stops
        if (safetyState.toString().contains("EmergencyStopInternal: ACTIVE") ||
            safetyState.toString().contains("EmergencyStopExternal: ACTIVE")) {
            logger.warn("Emergency stop is active!");
            isPauseActive = true;
        }

        // Check operator safety (pause button or similar)
        if (safetyState.toString().contains("OPERATOR_SAFETY_OPEN")) {
            logger.warn("Operator safety (Pause button) is active!");
            isPauseActive = true;
        }

        // Check safety stop signal (protective stop, e.g., Pause)
        if (safetyState.toString().contains("PROTECTIVE_STOP") || 
            safetyState.toString().contains("SAFETY_STOP")) {
            logger.warn("Safety stop signal active: " + safetyState);
            isPauseActive = true;
        }

        // Check operation mode (not in automatic mode)
        if (safetyState.getOperationMode() != OperationMode.AUT) {
            logger.warn("Robot is not in automatic mode: " + safetyState.getOperationMode());
            isPauseActive = true;
        }

        if (isPauseActive) {
            if (!isPaused) {
                logger.info("Pause detected - pausing all motion.");
                pauseAllMotion();
                isPaused = true;
                lastPauseLogTime = System.currentTimeMillis();
            } else {
                long now = System.currentTimeMillis();
                if (now - lastPauseLogTime > PAUSE_LOG_INTERVAL_MS) {
                    logger.info("System paused - motion remains stopped.");
                    lastPauseLogTime = now;
                }
            }
        } else if (isPaused) {
            logger.info("Pause cleared - resuming if safe.");
            resumeIfSafe();
            isPaused = false;
        }
    }

    public void onConnectionLost(Controller controller) {
        logger.error("Connection lost to controller: " + controller.getName());
    }

    @Override
    public void onShutdown(Controller controller) {
        logger.info("Controller shutdown: " + controller.getName());
    }

    @Override
    public void onIsReadyToMoveChanged(Device device, boolean readyToMove) {
        logger.info("Ready to move changed (" + device.getName() + "): " + readyToMove);
    }

    @Override
    public void onFieldBusDeviceIdentificationRequestReceived(String deviceIdentifier, DispatchedEventData eventData) {
        // This method is required by IControllerStateListener but not used in our implementation
        // Log the event for debugging purposes
        logger.debug("Field bus device identification request received for device: " + deviceIdentifier);
    }

    @Override
    public void onFieldBusDeviceConfigurationChangeReceived(String deviceIdentifier, DispatchedEventData eventData) {
        // This method is required by IControllerStateListener but not used in our implementation
        // Log the event for debugging purposes
        logger.debug("Field bus device configuration changed for device: " + deviceIdentifier);
    }

    private void pauseAllMotion() {
        try {
            if (kmp_commander != null) {
                kmp_commander.stopMotion();
            }
            if (lbr_commander != null) {
                lbr_commander.stopMotion();
            }
            logger.info("All motion paused due to safety event.");
        } catch (Exception e) {
            logger.error("Error pausing motion: " + e.getMessage());
        }
    }

    private void resumeIfSafe() {
        // Implement your logic to check if safe before resuming.
        logger.info("Resuming operations after safety clearance.");
    }

    @Override
    public void onStatePortChangeReceived(Controller controller, StatePortData statePort) {
        logger.debug("State port change received: " + statePort);
    }
}
