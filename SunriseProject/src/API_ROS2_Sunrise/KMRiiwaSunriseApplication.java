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

// Configuration
import javax.inject.Inject;
import javax.inject.Named;
import org.apache.log4j.BasicConfigurator;

// Implementated classes
// import API_ROS2_Sunrise.KMP_commander;
// import API_ROS2_Sunrise.KMP_sensor_reader;
// import API_ROS2_Sunrise.KMP_status_reader;
// import API_ROS2_Sunrise.LBR_commander;
// import API_ROS2_Sunrise.LBR_sensor_reader;
// import API_ROS2_Sunrise.LBR_status_reader;
// import API_ROS2_Sunrise.SafetyStateListener;

//RoboticsAPI
import com.kuka.roboticsAPI.annotations.*;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.deviceModel.LBR;
// import com.kuka.roboticsAPI.deviceModel.OperationMode;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;


import com.kuka.generated.ioAccess.ControlPanelIOGroup;
import com.kuka.task.ITaskManager;
// import com.kuka.task.RoboticsAPITask;

// AUT MODE: 3s, T1/T2/CRR: 2s
@ResumeAfterPauseEvent(delay = 0 ,  afterRepositioning = true)
public class KMRiiwaSunriseApplication extends RoboticsAPIApplication{
	
	// Runtime Variables
	private volatile boolean AppRunning;
	private IAutomaticResumeFunction resumeFunction;
	SafetyStateListener safetylistener;
	public static boolean StateChange = true;
	
	// Declare KMP
	@Inject
	@Named("KMR_omniMove_200_1")
	public KmpOmniMove kmp;
	public Controller controller;

	
	// Declare LBR
	@Inject
	@Named("LBR_iiwa_14_R820_1")
	public LBR lbr;

//	@Inject
//	@Named("TaskManager")
	private ITaskManager taskManager;

	//@Inject
	//@Named("name of tool")
	//public Tool tool;
	
	// Define UDP ports
	int KMP_status_port = 30001;
	int KMP_command_port = 30002;
	int KMP_laser_port = 30003;
	int KMP_odometry_port = 30004;
	int LBR_command_port = 30005;
	int LBR_status_port = 30006;
	int LBR_sensor_port = 30007;
	
	// Threading parameter
	String threading_prio = "LBR";
	
	// Connection types
	String TCPConnection = "TCP";
	String UDPConnection = "UDP";

	// Implemented node classes
	KMP_commander kmp_commander;
	LBR_commander lbr_commander;
	LBR_sensor_reader lbr_sensor_reader;
	KMP_sensor_reader kmp_sensor_reader;
	KMP_status_reader kmp_status_reader;
	LBR_status_reader lbr_status_reader;

	// Thread references
	private Thread kmp_commander_thread;
	private Thread kmp_status_thread;
	private Thread kmp_sensor_thread;
	private Thread lbr_commander_thread;
	private Thread lbr_status_thread;
	private Thread lbr_sensor_thread;

	// Check if application is paused:
	@Inject
	ControlPanelIOGroup ControlPanelIO;

	private final Object shutdownLock = new Object();
	private volatile boolean isShuttingDown = false;

	@Override
	public void initialize() {
	    try {
	        if (taskManager == null) {
	            taskManager = getTaskManager();
	        }
	    } catch (IllegalStateException e) {
	        System.out.println("WARNING: " + e.getMessage() + " Continuing without task functions.");
	        taskManager = null;
	    }
	    if(taskManager == null) { 
	        System.out.println("WARNING: TaskManager not injected! Continuing without it.");
	    } else {
	        System.out.println("Task manager initialized: " + taskManager);
	    }
	    
	    System.out.println("Initializing Robotics API Application");
	    BasicConfigurator.configure();
	    
	    // Configure robot
	    controller = getController("KUKA_Sunrise_Cabinet_1");
	    kmp = getContext().getDeviceFromType(KmpOmniMove.class);
	    lbr = getContext().getDeviceFromType(LBR.class);
	    
	    // Move to drive position
	    lbr.move(ptp(getApplicationData().getFrame("/DrivePos")).setJointVelocityRel(0.5));
	    
	    // Create nodes for communication
	    kmp_commander = new KMP_commander(KMP_command_port, kmp, TCPConnection);
	    lbr_commander = new LBR_commander(LBR_command_port, lbr, TCPConnection, getApplicationData().getFrame("/DrivePos"));
	    
	    // SafetyStateListener
	    safetylistener = new SafetyStateListener(controller, lbr_commander, kmp_commander, lbr_status_reader, kmp_status_reader);
	    safetylistener.startSafetyStateListener();
	    
	    // Check if a commander node is active...
	    long startTime = System.currentTimeMillis();
	    int shutDownAfterMs = 10000; 
	    while(!AppRunning) {
	        if(kmp_commander.isSocketConnected() || lbr_commander.isSocketConnected()){
	            AppRunning = true;
	            System.out.println("Application ready to run!");   
	            break;
	        } else if((System.currentTimeMillis() - startTime) > shutDownAfterMs){
	            System.out.println("Could not connect to a command node after " + shutDownAfterMs/1000 + "s. Shutting down.");   
	            shutdown_application();
	            break;
	        }               
	    }
	    // Establish remaining nodes if AppRunning...
	    if(AppRunning) {
	        kmp_status_reader = new KMP_status_reader(KMP_status_port, kmp, TCPConnection);
	        lbr_status_reader = new LBR_status_reader(LBR_status_port, lbr, TCPConnection);
	        lbr_sensor_reader = new LBR_sensor_reader(LBR_sensor_port, lbr, TCPConnection);
	        kmp_sensor_reader = new KMP_sensor_reader(KMP_laser_port, KMP_odometry_port, TCPConnection, TCPConnection);
	    }
	}
	

	public void shutdown_application() {
	    synchronized (shutdownLock) {
	        if (isShuttingDown) {
	            System.out.println("Shutdown already in progress");
	            return;
	        }
	        isShuttingDown = true;
	    }
	    
	    System.out.println("----- Shutting down Application -----");
	    
	    // 1. Stop the safety state listener
	    if (safetylistener != null) {
	        safetylistener.stop();
	    }
	    
	    // 2. Set the shutdown flags on all nodes
	    if (kmp_commander != null) { 
	        kmp_commander.setShutdown(true);
	    }
	    if (lbr_commander != null) { 
	        lbr_commander.setShutdown(true);
	    }
	    if (kmp_status_reader != null) { 
	        kmp_status_reader.setShutdown(true);
	    }
	    if (lbr_status_reader != null) { 
	        lbr_status_reader.setShutdown(true);
	    }
	    if (kmp_sensor_reader != null) { 
	        kmp_sensor_reader.setShutdown(true);
	    }
	    if (lbr_sensor_reader != null) { 
	        lbr_sensor_reader.setShutdown(true);
	    }
	    
	    // 3. Stop any ongoing motions (using your available API – for instance, cancelMotion())
	    if (kmp_commander != null) {
	        kmp_commander.stopMotion();
	    }
	    if (lbr_commander != null) {
	        lbr_commander.stopMotion();
	    }
	    
	    // 4. Interrupt all threads
	    interruptThread(kmp_commander_thread, "KMP commander");
	    interruptThread(lbr_commander_thread, "LBR commander");
	    interruptThread(kmp_status_thread, "KMP status");
	    interruptThread(lbr_status_thread, "LBR status");
	    interruptThread(kmp_sensor_thread, "KMP sensor");
	    interruptThread(lbr_sensor_thread, "LBR sensor");
	    
	    // 5. Allow brief pause then explicitly close all node sockets
	    try {
	        Thread.sleep(200);
	    } catch (InterruptedException e) {
	        Thread.currentThread().interrupt();
	    }
	    
	    closeNodeSafely(kmp_commander, "KMP commander");
	    closeNodeSafely(lbr_commander, "LBR commander");
	    closeNodeSafely(kmp_status_reader, "KMP status");
	    closeNodeSafely(kmp_sensor_reader, "KMP sensor");
	    closeNodeSafely(lbr_status_reader, "LBR status");
	    closeNodeSafely(lbr_sensor_reader, "LBR sensor");
	    
	    // 6. Wait for all threads to finish
	    waitForThread(kmp_commander_thread, "KMP commander", 2000);
	    waitForThread(lbr_commander_thread, "LBR commander", 2000);
	    waitForThread(kmp_status_thread, "KMP status", 1000);
	    waitForThread(lbr_status_thread, "LBR status", 1000);
	    waitForThread(kmp_sensor_thread, "KMP sensor", 1000);
	    waitForThread(lbr_sensor_thread, "LBR sensor", 1000);
	    
	    System.out.println("Application terminated");
	}

	private void interruptThread(Thread thread, String name) {
	    if (thread != null && thread.isAlive()) {
	        System.out.println("Interrupting " + name + " thread");
	        thread.interrupt();
	    }
	}

	private void waitForThread(Thread thread, String name, long timeoutMs) {
	    if (thread != null && thread.isAlive()) {
	        try {
	            thread.join(timeoutMs);
	            if (thread.isAlive()) {
	                System.out.println("WARNING: " + name + " thread did not terminate within timeout");
	                forceTerminate(thread, name);
	            }
	        } catch (InterruptedException e) {
	            System.out.println("Interrupted while waiting for " + name + " thread");
	        }
	    }
	}

	private void closeNodeSafely(Node node, String name) {
	    if (node != null) {
	        try {
	            node.close();
	        } catch (Exception e) {
	            System.out.println("Error closing " + name + ": " + e.getMessage());
	        }
	    }
	}
	
	private void forceTerminate(Thread thread, String name) {
	    if (thread != null && thread.isAlive()) {
	        System.out.println("WARNING: " + name + " thread still alive after timeout. Attempting force termination...");
	        
	        // Try one more interrupt before giving up
	        thread.interrupt();
	        try {
	            thread.join(1000);  // Give it one more second
	            
	            if (thread.isAlive()) {
	                System.out.println("SEVERE: " + name + " thread cannot be terminated gracefully");
	                // Thread cannot be forcibly terminated in a better way in Java
	                // We'll let the JVM handle it during shutdown
	            }
	        } catch (InterruptedException e) {
	            System.out.println("Interrupted while waiting final time for " + name + " thread");
	        }
	    }
	}
	
	@Override
	public void run() {
	    try {
	        resumeFunction = getTaskFunction(IAutomaticResumeFunction.class);
	        setAutomaticallyResumable(true);
	    } catch (Exception e) {
	        System.out.println("Error getting resume function: " + e.getMessage());
	    }

	    System.out.println("Running app!");
	    
	    // Start all connected nodes
	    kmp_commander.setPriority(Thread.MAX_PRIORITY);
	    lbr_commander.setPriority(Thread.MAX_PRIORITY);
	    
	    if (!(kmp_commander == null) && kmp_commander.isSocketConnected()) {
	        kmp_commander.start();
	        kmp_commander_thread = kmp_commander;
	    }
	    
	    if (!(kmp_status_reader == null) && kmp_status_reader.isSocketConnected()) {
	        kmp_status_reader.start();
	        kmp_status_thread = kmp_status_reader;
	    }
	    
	    if (!(lbr_commander == null) && lbr_commander.isSocketConnected()) {
	        lbr_commander.start();
	        lbr_commander_thread = lbr_commander;
	    }
	    
	    if (!(lbr_status_reader == null) && lbr_status_reader.isSocketConnected()) {
	        lbr_status_reader.start();
	        lbr_status_thread = lbr_status_reader;
	    }
	    
	    if (!(lbr_sensor_reader == null) && lbr_sensor_reader.isSocketConnected()) {
	        lbr_sensor_reader.start();
	        lbr_sensor_thread = lbr_sensor_reader;
	    }
	    
	    if (!(kmp_sensor_reader == null) && kmp_sensor_reader.isSocketConnected()) {
	        kmp_sensor_reader.start();
	        kmp_sensor_thread = kmp_sensor_reader;
	    }
	    
	    // Initialize mode monitoring variables
	    String lastMode = lbr.getOperationMode().toString();
	    long lastLogTime = System.currentTimeMillis();
//	    boolean wasPaused = false;
	    
	    // Check if the commanders are still alive periodically
	    long lastCommanderCheckTime = System.currentTimeMillis();
	    long lastSocketCheckTime = System.currentTimeMillis();
	    
	    while (AppRunning) {
	        // 1. Monitor if the commander threads are still alive
	        long currentTime = System.currentTimeMillis();
	        if (currentTime - lastCommanderCheckTime > 5000) { // Check every 5 seconds
	            lastCommanderCheckTime = currentTime;
	            
	            // Check KMP commander
	            if (kmp_commander_thread != null && !kmp_commander_thread.isAlive() && !isShuttingDown) {
	                System.out.println("WARNING: KMP commander thread died unexpectedly, attempting restart");
	                try {
	                    kmp_commander = new KMP_commander(KMP_command_port, kmp, TCPConnection);
	                    kmp_commander.setPriority(Thread.MAX_PRIORITY);
	                    kmp_commander.start();
	                    kmp_commander_thread = kmp_commander;
	                } catch (Exception e) {
	                    System.out.println("Failed to restart KMP commander: " + e.getMessage());
	                }
	            }
	            
	            // Check LBR commander
	            if (lbr_commander_thread != null && !lbr_commander_thread.isAlive() && !isShuttingDown) {
	                System.out.println("WARNING: LBR commander thread died unexpectedly, attempting restart");
	                try {
	                    lbr_commander = new LBR_commander(LBR_command_port, lbr, TCPConnection, getApplicationData().getFrame("/DrivePos"));
	                    lbr_commander.setPriority(Thread.MAX_PRIORITY);
	                    lbr_commander.start();
	                    lbr_commander_thread = lbr_commander;
	                } catch (Exception e) {
	                    System.out.println("Failed to restart LBR commander: " + e.getMessage());
	                }
	            }
	        }
	        
	        // Add to run() method in KMRiiwaSunriseApplication.java
	        // Call every few seconds, perhaps alongside commander thread checks
	        if (currentTime - lastSocketCheckTime > 5000) {
	            lastSocketCheckTime = currentTime;
	            monitorSocketConnections();
	        }
	        
	        // 2. Check if any of the commanders signaled for shutdown
	        if (kmp_commander != null && kmp_commander.getShutdown() ||
	            lbr_commander != null && lbr_commander.getShutdown()) {
	            System.out.println("Commander signaled for shutdown");
	            System.out.println("LBR commander shutdown: " + (lbr_commander != null ? lbr_commander.getShutdown() : "null"));
	            System.out.println("KMP commander shutdown: " + (kmp_commander != null ? kmp_commander.getShutdown() : "null"));
	            
	            // NEW DIAGNOSTIC CODE
	            System.out.println("DIAGNOSTIC: Tracking shutdown source");

	            // Check controller states
	            try {
	                // Try to use controller methods to see if it's still valid
	                Controller kmpController = kmp.getController();
	                boolean kmpValid = true;
	                try {
	                    String name = kmpController.getName();
	                    System.out.println("KMP controller appears valid (name: " + name + ")");
	                } catch (Exception e) {
	                    kmpValid = false;
	                    System.out.println("KMP controller appears invalid: " + e.getMessage());
	                }
	                System.out.println("KMP controller state: " + (kmpValid ? "valid" : "invalid"));
	            } catch (Exception e) {
	                System.out.println("KMP controller check error: " + e.getMessage());
	            }

	            try {
	                // Try to use controller methods to see if it's still valid  
	                Controller lbrController = lbr.getController();
	                boolean lbrValid = true;
	                try {
	                    String name = lbrController.getName();
	                    System.out.println("LBR controller appears valid (name: " + name + ")");
	                } catch (Exception e) {
	                    lbrValid = false;
	                    System.out.println("LBR controller appears invalid: " + e.getMessage());
	                }
	                System.out.println("LBR controller state: " + (lbrValid ? "valid" : "invalid"));
	            } catch (Exception e) {
	                System.out.println("LBR controller check error: " + e.getMessage());
	            }
	            // End DIAGNOSTIC CODE

	            AppRunning = false;
	            break;
	        }
	        
	        // 3. Check if the application is paused using StatusController
	        // Commenting out StatusController pause detection to avoid errors
	        /*
	        boolean isPaused = false;
	        try {
	            // Use the StatusController to check application state
	            if (statusController != null) {
	                String currentStatus = statusController.getStatusString("APPLICATION_STATE");
	                isPaused = currentStatus != null && 
	                          (currentStatus.contains("PAUSED") || 
	                           currentStatus.contains("Motion paused") ||
	                           currentStatus.contains("STOPPED"));
	                
	                if (isPaused != wasPaused) {
	                    System.out.println("Application pause state changed to: " + (isPaused ? "PAUSED" : "RUNNING"));
	                    System.out.println("Current status: " + currentStatus);
	                    
	                    // If we're paused by the user, be prepared to respond when they resume
	                    if (isPaused) {
	                        System.out.println("Application paused - waiting for resume button");
	                    }
	                }
	                wasPaused = isPaused;
	            } else {
	                System.out.println("StatusController is null, cannot check application state");
	            }
	        } catch (Exception e) {
	            System.out.println("Error checking pause state with StatusController: " + e.getMessage());
	        }
	        */
	        
	        // 4. Check for operation mode changes (with reduced logging)
	        String currentMode = lbr.getOperationMode().toString();
	        if (!currentMode.equals(lastMode)) {
	            System.out.println("Mode changed from " + lastMode + " to " + currentMode);
	            
	            // If we exit AUTO mode, this is a critical change requiring shutdown
	            if (lastMode.equals("AUTO") && !currentMode.equals("AUTO")) {
	                System.out.println("Exited AUTO mode - initiating shutdown");
	                AppRunning = false;
	                break;
	            }
	            lastMode = currentMode;
	        } else if (currentTime - lastLogTime > 60000) {  // Only log every minute instead of every 5 seconds
	            System.out.println("Current operating mode: " + currentMode);
	            lastLogTime = currentTime;
	        }
	        
	        // 5. Handle state changes (thread priorities)  
	        if (StateChange) {
	            System.out.println("State change.");
	            StateChange = false;
	            
	            if (!(threading_prio == "LBR") && lbr_commander != null && !lbr_commander.getisPathFinished()) {
	                threading_prio = "LBR";
	                if (kmp_status_reader != null) kmp_status_reader.setPriority(Thread.MIN_PRIORITY);
	                
	                System.out.println("Threading priority: " + threading_prio);
	                
	                if (lbr_sensor_reader != null) lbr_sensor_reader.setPriority(Thread.MAX_PRIORITY);
	                if (lbr_status_reader != null) lbr_status_reader.setPriority(Thread.MAX_PRIORITY);
	            } else if (!(threading_prio == "KMP") && lbr_commander != null && lbr_commander.getisPathFinished()) {
	                threading_prio = "KMP";
	                if (lbr_sensor_reader != null) lbr_sensor_reader.setPriority(Thread.MIN_PRIORITY);
	                if (lbr_status_reader != null) lbr_status_reader.setPriority(Thread.MIN_PRIORITY);
	                
	                System.out.println("Threading priority: " + threading_prio);
	                if (kmp_status_reader != null) kmp_status_reader.setPriority(Thread.MAX_PRIORITY);
	            }
	        }
	        
	        // Check safety states - don't shut down for scanner warnings
	        boolean safetyOk = handleSafetyStates();
	        if (!safetyOk) {
	            System.out.println("Critical safety violation - initiating shutdown");
	            AppRunning = false;
	            break;
	        }

	        // Check if safety scanner warning fields have cleared
	        if (kmp_commander != null && kmp_commander.getEmergencyStop()) {
	            try {
	                String safetyStateStr = kmp.getSafetyState().toString(); 
	                boolean scannerWarning = safetyStateStr.contains("WARNING_FIELD");
	                
	                if (!scannerWarning) {
	                    System.out.println("KMRiiwaSunriseApplication: Safety scanner warning cleared, resetting emergency flag");
						Node.setEmergencyStop(false);
	                    // Give UI feedback that system can move again
	                    System.out.println("Robot is ready to move again - scanner warning cleared");
	                } else {
	                    System.out.println("Robot still in scanner warning zone, cannot move");
	                }
	            } catch (Exception e) {
	                System.out.println("Error checking scanner state: " + e.getMessage());
	            }	
	        }
	        
	        // Sleep to prevent high CPU usage
	        try {
	            Thread.sleep(100);
	        } catch (InterruptedException e) {
	            Thread.currentThread().interrupt();
	            break;
	        }
	    }
	    
	    System.out.println("Shutdown message received in main application");
	    if (kmp_commander != null) {
	        System.out.println("KMP commander shutdown: " + kmp_commander.getShutdown());
	    }
	    if (lbr_commander != null) {
	        System.out.println("LBR commander shutdown: " + lbr_commander.getShutdown());
	    }
	    shutdown_application();
	}
	
	private void setAutomaticallyResumable(boolean enable)
	{
	    if(enable)
	    {
	        if (resumeFunction != null) {
	            resumeFunction.enableApplicationResuming(getClass().getCanonicalName());
	        } else {
	            System.out.println("WARNING: resumeFunction is null, cannot enable automatic resuming");
	        }
	        return;
	    }
	    
	    if (resumeFunction != null) {
	        resumeFunction.disableApplicationResuming(getClass().getCanonicalName());
	    } else {
	        System.out.println("WARNING: resumeFunction is null, cannot disable automatic resuming");
	    }
	}

	private boolean handleSafetyStates() {
	    try {
	        // Get the current safety state from both devices
	        String kmpSafetyState = kmp.getSafetyState().toString();
	        String lbrSafetyState = lbr.getSafetyState().toString();
	        
	        // Check for scanner warning fields
	        boolean scannerWarning = kmpSafetyState.contains("WARNING_FIELD") || 
	                                lbrSafetyState.contains("WARNING_FIELD");
	        
	        if (scannerWarning) {
	            System.out.println("Safety scanner warning field detected");
	            
	            // Pause motion but don't shut down
	            if (kmp_commander != null) {
	                kmp_commander.stopMotion();
	            }
	            if (lbr_commander != null) {
	                lbr_commander.stopMotion();
	            }
	            
	            // Wait for scanner warning to clear
	            // Don't return false which would trigger shutdown
	            return true;
	        }
	        
	        // No safety issues detected
	        return true;
	    } catch (Exception e) {
	        System.out.println("Error checking safety state: " + e.getMessage());
	        // Don't trigger shutdown for errors in safety state checking
	        return true;
	    }
	}
	
	private void monitorSocketConnections() {
	    System.out.println("=== Socket Connection Status ===");
	    if (kmp_commander != null) {
	        boolean socketOk = kmp_commander.isSocketConnected();
	        System.out.println("KMP commander socket: " + (socketOk ? "CONNECTED" : "DISCONNECTED"));
	    }
	    if (lbr_commander != null) {
	        boolean socketOk = lbr_commander.isSocketConnected();
	        System.out.println("LBR commander socket: " + (socketOk ? "CONNECTED" : "DISCONNECTED"));
	    }
	    // Add same checks for other nodes
	}

	public static void main(String[] args){
		KMRiiwaSunriseApplication app = new KMRiiwaSunriseApplication();
		app.runApplication();
	}
	
}
