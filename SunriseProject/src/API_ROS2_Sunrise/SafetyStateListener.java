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

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.DispatchedEventData;
import com.kuka.roboticsAPI.controllerModel.StatePortData;
import com.kuka.roboticsAPI.controllerModel.sunrise.ISunriseControllerStateListener;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState;
import com.kuka.roboticsAPI.deviceModel.Device;

public class SafetyStateListener implements ISunriseControllerStateListener {
    Controller controller;
    LBR_commander lbr_commander;
    KMP_commander kmp_commander;
    LBR_status_reader lbr_status_reader;
    KMP_status_reader kmp_status_reader;

    public SafetyStateListener(Controller cont, LBR_commander lbr, KMP_commander kmp, LBR_status_reader lbr_status, KMP_status_reader kmp_status) {
        controller = cont;
        lbr_commander = lbr;
        kmp_commander = kmp;
        lbr_status_reader = lbr_status;
        kmp_status_reader = kmp_status;
    }

    public void startSafetyStateListener() {
        controller.addControllerListener(this);
    }

    public void stop() {
        LogUtil.logInfo("SafetyStateListener: Stopping listener");
        try {
            controller.removeControllerListener(this);
            LogUtil.logInfo("SafetyStateListener: Successfully removed controller listener");
        } catch (Exception e) {
            LogUtil.logInfo("SafetyStateListener: Error removing controller listener: " + e.getMessage());
        }
    }

    @Override
    public void onFieldBusDeviceConfigurationChangeReceived(String deviceName, DispatchedEventData eventData) {
        LogUtil.logInfo("SafetyStateListener: Field bus device configuration changed");
        LogUtil.logInfo("  Device name: " + deviceName);
        LogUtil.logInfo("  Event data: " + eventData);
        // Field bus configuration changes shouldn't affect our operation
        // but we should log them for debugging purposes
    }

    @Override
    public void onFieldBusDeviceIdentificationRequestReceived(String deviceName, DispatchedEventData eventData) {
        LogUtil.logInfo("SafetyStateListener: Field bus device identification requested");
        LogUtil.logInfo("  Device name: " + deviceName);
        LogUtil.logInfo("  Event data: " + eventData);
        // This is just for logging purposes - occurs when the controller
        // requests identification from field bus devices
    }

    @Override
    public void onIsReadyToMoveChanged(Device arg0, boolean arg1) {
        LogUtil.logInfo("READY TO MOVE! " + arg1 + " " + arg0);
    }

    @Override
    public void onShutdown(Controller controller) {
        LogUtil.logInfo("SafetyStateListener: Controller shutdown detected");
        LogUtil.logInfo("  Controller: " + controller.getName());

        // When controller is shutting down, we should ensure our application
        // also performs a clean shutdown - temporarily disabled for debugging
        if (kmp_commander != null) {
            LogUtil.logInfo("SafetyStateListener: KMP commander shutdown flag would be set here - temporarily disabled");
            // kmp_commander.setShutdown(true);  // Commented out to prevent triggering shutdown
        }

        if (lbr_commander != null) {
            LogUtil.logInfo("SafetyStateListener: LBR commander shutdown flag would be set here - temporarily disabled");
            // lbr_commander.setShutdown(true);  // Commented out to prevent triggering shutdown
        }
    }

    @Override
    public void onStatePortChangeReceived(Controller controller, StatePortData statePortData) {
        LogUtil.logInfo("SafetyStateListener: State port change received");
        LogUtil.logInfo("  Controller: " + controller.getName());
        LogUtil.logInfo("  Port name: " + statePortData.getPortName());
        // Note: The API does not provide a getValue() method.
    }

    @Override
    public void onConnectionLost(Controller controller) {
        LogUtil.logInfo("SafetyStateListener: CONNECTION LOST TO CONTROLLER");
        LogUtil.logInfo("  Controller: " + controller.getName());

        // Connection loss is a critical event - logging only for now
        LogUtil.logInfo("SafetyStateListener: Would set emergency stop due to lost connection - temporarily disabled");
        // Node.setEmergencyStop(true);  // Commented out to prevent triggering emergency stop

        // Also try to trigger shutdown on commanders if they're still accessible
        if (kmp_commander != null) {
            LogUtil.logInfo("SafetyStateListener: KMP commander shutdown flag would be set here - temporarily disabled");
            // kmp_commander.setShutdown(true);  // Commented out to prevent triggering shutdown
        }

        if (lbr_commander != null) {
            LogUtil.logInfo("SafetyStateListener: LBR commander shutdown flag would be set here - temporarily disabled");
            // lbr_commander.setShutdown(true);  // Commented out to prevent triggering shutdown
        }
    }

    @Override
    public void onSafetyStateChanged(Device device, SunriseSafetyState safetyState) {
        LogUtil.logInfo("safety_state", "Safety state change on device: " + device.getName() + 
                        "\nFull safety state: " + safetyState);
        
        if (safetyState.getSafetyStopSignal() == SunriseSafetyState.SafetyStopType.STOP1) {
            LogUtil.logInfo("safety_stop", "Safety STOP1 detected - safety scanner may have triggered");
            
            if (isLaserScannerWarning(safetyState)) {
                LogUtil.logInfo("scanner_warning", "Laser scanner warning field detected - pausing motion");
                pauseAllMotion();
            } else {
                LogUtil.logInfo("safety_critical", "Critical safety event - initiating controlled shutdown");
            }
        } else if (safetyState.getSafetyStopSignal() == SunriseSafetyState.SafetyStopType.NOSTOP) {
            LogUtil.logInfo("Safety stop cleared - operations can resume");
            resumeIfSafe();
        }
    }

    /**
     * Check if the safety event is likely from a laser scanner warning field
     */
    private boolean isLaserScannerWarning(SunriseSafetyState safetyState) {
        // Adapt this logic based on what safety state properties indicate scanner warnings
        // You may need to check specific properties or patterns in the safety state
        
        // Example check - adjust based on your scanner's behavior
        return safetyState.toString().contains("WARNING") || 
               safetyState.toString().contains("SCANNER");
    }

    /**
     * Pause motion without shutting down the system
     */
    private void pauseAllMotion() {
        try {
            // Stop motion but don't shut down commanders
            if (kmp_commander != null) {
                kmp_commander.stopMotion();
            }
            if (lbr_commander != null) {
                lbr_commander.stopMotion();
            }
            LogUtil.logInfo("All motion paused due to safety event");
        } catch (Exception e) {
            LogUtil.logInfo("Error pausing motion: " + e.getMessage());
        }
    }

    /**
     * Resume operations if safety conditions permit
     */
    private void resumeIfSafe() {
        // Add logic to check if it's safe to resume
        LogUtil.logInfo("Safety conditions cleared - motion can resume");
        // You may want to send a notification to your ROS nodes that they can resume operations
    }
}

