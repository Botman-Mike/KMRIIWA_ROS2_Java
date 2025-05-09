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

//RoboticsAPI
import com.kuka.roboticsAPI.annotations.*;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.deviceModel.LBR;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.generated.ioAccess.ControlPanelIOGroup;
import com.kuka.task.ITaskManager;
import com.kuka.task.ITaskLogger;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;

@ResumeAfterPauseEvent(delay = 0 ,  afterRepositioning = true)
public class KMRiiwaSunriseApplication extends RoboticsAPIApplication {
	
	// Runtime Variables
	private volatile boolean AppRunning;
	private IAutomaticResumeFunction resumeFunction;
	SafetyStateListener safetylistener;
	private volatile boolean stateChange = true;
	
	// Field for startup grace-period tracking
	private volatile long startupTime;
    // Grace period duration to ignore early shutdowns
    private static final long STARTUP_GRACE_PERIOD_MS = 5000; // 5 seconds
    // Field to track custom shutdown requests
    private volatile boolean shutdownRequested;
	
	// Declare KMP
	@Inject
	@Named("KMR_omniMove_200_1")
	public KmpOmniMove kmp;
	public Controller controller;

	// Declare LBR
	@Inject
	@Named("LBR_iiwa_14_R820_1")
	public LBR lbr;

	private ITaskManager taskManager;

	@Inject
	private ITaskLogger logger;

	// Define UDP ports
	int KMP_status_port = 30001;
	int KMP_command_port = 30002;
	int KMP_laser_port = 30003;
	int KMP_odometry_port = 30004;
	int LBR_command_port = 30005;
	int LBR_status_port = 30006;
	int LBR_sensor_port = 30007;
	
	// Single TCP socket instance to manage all TCP connections
	private TCPSocket tcpSocket;
	
	// Threading parameter
	private String threadingPriority = "LBR";  // Using String instead of enum for Java 6
	private static final String PRIORITY_LBR = "LBR";
	private static final String PRIORITY_KMP = "KMP";
	
	// Connection types
	String TCPConnection = "TCP";
	String UDPConnection = "UDP";

	// Implemented node classes
	KMP_commander kmpCommanderRunnable;
	LBR_commander lbr_commander;
	LBR_sensor_reader lbr_sensor_reader;
	KMP_sensor_reader kmp_sensor_reader;
	KMP_status_reader kmp_status_reader;
	LBR_status_reader lbr_status_reader;

	// Thread references
	private Thread kmp_commander;
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

	// Add these declarations after other class variables
	private DataController kmpDataController;
	private DataController lbrDataController;
	private java.util.concurrent.ScheduledExecutorService executorService;

	// Add constants for commonly used values
	private static final int THREAD_SLEEP_TIME = 100; // 100ms
	
	// Add relevant getters/setters for non-static access
	public synchronized boolean getStateChange() {
		return stateChange; 
	}

	public synchronized void setStateChange(boolean state) {
		this.stateChange = state;
	}

	@Override
	public void initialize() {
	    try {
	        if (taskManager == null) {
	            taskManager = getTaskManager();
	        }
	    } catch (IllegalStateException e) {
	        logger.warn("WARNING: " + e.getMessage() + " Continuing without task functions.");
	        taskManager = null;
	    }
	    if(taskManager == null) { 
	        logger.warn("WARNING: TaskManager not injected! Continuing without it.");
	    } else {
	        logger.info("Task manager initialized: " + taskManager);
	    }
	    
	    logger.info("Initializing Robotics API Application");
	    BasicConfigurator.configure();
	    
	    // Configure robot
	    controller = getController("KUKA_Sunrise_Cabinet_1");
	    kmp = getContext().getDeviceFromType(KmpOmniMove.class);
	    lbr = getContext().getDeviceFromType(LBR.class);
	    
	    // Move to drive position
	    lbr.move(ptp(getApplicationData().getFrame("/DrivePos")).setJointVelocityRel(0.5));
	    
	    // Initialize the single TCPSocket instance with the default port (will be used when node classes are created)
	    tcpSocket = new TCPSocket(KMP_command_port, "MainSocket");
	    if (tcpSocket != null) {
	        logger.info("Main TCP Socket initialized - managing persistent connections to 172.31.1.206 (ports 30001, 30002, 30005, 30006)");
	        tcpSocket.startHeartbeatThread();
	    } else {
	        logger.error("FAILED to initialize main TCP Socket");
	    }
	    
	    // Create nodes for communication - passing the shared TCPSocket instance
	    kmpCommanderRunnable = new KMP_commander(KMP_command_port, kmp, TCPConnection);
	    lbr_commander = new LBR_commander(LBR_command_port, lbr, TCPConnection, getApplicationData().getFrame("/DrivePos"));
	    
	    // Initialize DataControllers with proper null checks
	    try {
	        if (kmpCommanderRunnable != null && kmpCommanderRunnable.isSocketConnected() && kmpCommanderRunnable.getSocket() != null) {
	            kmpDataController = new DataController(kmpCommanderRunnable.getSocket(), null);
	            kmpCommanderRunnable.setDataController(kmpDataController);
	        }
	        
	        if (lbr_commander != null && lbr_commander.isSocketConnected() && lbr_commander.getSocket() != null) {
	            lbrDataController = new DataController(lbr_commander.getSocket(), null);
	            lbr_commander.setDataController(lbrDataController);
	        }
	    } catch (Exception e) {
	        logger.error("Error initializing DataControllers: " + e.getMessage());
	    }
	    
	    // SafetyStateListener
	    safetylistener = new SafetyStateListener(controller, lbr, lbr_commander, kmpCommanderRunnable, lbr_status_reader, kmp_status_reader);
	    safetylistener.startSafetyStateListener();
	    
	    // Check if a commander node is active...
	    long startTime = System.currentTimeMillis();
	    int shutDownAfterMs = 600000; // 10 minutes in milliseconds
	    while(!AppRunning) {
	        if(kmpCommanderRunnable.isSocketConnected() || lbr_commander.isSocketConnected()){
	            AppRunning = true;
	            logger.info("Application ready to run!");   
	            break;
	        } else if((System.currentTimeMillis() - startTime) > shutDownAfterMs){
	            logger.warn("Could not connect to a command node after " + shutDownAfterMs/1000 + "s. Shutting down.");   
	            shutdown_application();
	            break;
	        }               
	    }
	    // Establish remaining nodes if AppRunning...
	    if(AppRunning) {
	        // Critical control and status data uses TCP for reliability
	        kmp_status_reader = new KMP_status_reader(KMP_status_port, kmp, TCPConnection);
	        lbr_status_reader = new LBR_status_reader(LBR_status_port, lbr, TCPConnection);
	        
	        // High-frequency sensor data uses UDP for better performance
	        lbr_sensor_reader = new LBR_sensor_reader(LBR_sensor_port, lbr, UDPConnection);
	        
	        // KMP sensor data uses separate UDP connections for laser and odometry
	        // to prevent blocking and allow independent packet handling
	        kmp_sensor_reader = new KMP_sensor_reader(
	            KMP_laser_port, 
	            KMP_odometry_port, 
	            UDPConnection,  // Laser scanner data 
	            UDPConnection   // Odometry data
	        );
	        
	        // Configure UDP buffer sizes for sensor nodes
	        if (lbr_sensor_reader != null && lbr_sensor_reader.getSocket() != null) {
	            logger.info("LBR sensor socket initialized");
	        }
	        
	        logger.info("Network configuration:");
	        logger.info("- TCP: Commands, Status (reliable delivery with persistent connections)");
	        logger.info("- UDP: Sensors, Laser, Odometry (optimized for speed)");
	    }

	    // NOTE: OSGi BundleContext/IApplicationControl service fetch is not implemented here because the API does not support restartApplication().
	}
	

	public void shutdown_application() {
	    synchronized (shutdownLock) {
	        if (isShuttingDown) {
	            logger.info("Shutdown already in progress");
	            return;
	        }
	        isShuttingDown = true;
	    }
	    
	    logger.info("----- Shutting down Application -----");
	    
	    // 1. Stop the safety state listener
	    if (safetylistener != null) {
	        safetylistener.stop();
	    }
	    
	    // 2. Set the shutdown flags on all nodes
	    if (kmpCommanderRunnable != null) { 
	        logger.info("Setting shutdown flag for KMP commander");
	        kmpCommanderRunnable.setShutdown(true);
	    }
	    if (lbr_commander != null) { 
	        logger.info("Setting shutdown flag for LBR commander");
	        lbr_commander.setShutdown(true);
	    }
	    if (kmp_status_reader != null) { 
	        logger.info("Setting shutdown flag for KMP status reader");
	        kmp_status_reader.setShutdown(true);
	    }
	    if (lbr_status_reader != null) { 
	        logger.info("Setting shutdown flag for LBR status reader");
	        lbr_status_reader.setShutdown(true);
	    }
	    if (kmp_sensor_reader != null) { 
	        logger.info("Setting shutdown flag for KMP sensor reader");
	        kmp_sensor_reader.setShutdown(true);
	    }
	    if (lbr_sensor_reader != null) { 
	        logger.info("Setting shutdown flag for LBR sensor reader");
	        lbr_sensor_reader.setShutdown(true);
	    }
	    
	    // Stop the shared TCP socket's heartbeat thread
	    if (tcpSocket != null) {
	        logger.info("Stopping main TCP Socket heartbeat thread");
	        tcpSocket.stopHeartbeatThread();
	    }
	    
	    // Add before closing nodes
	    cleanupDataController(kmpDataController, "KMP");
	    cleanupDataController(lbrDataController, "LBR");
	    
	    // 3. Stop any ongoing motions (using your available API – for instance, cancelMotion())
	    if (kmpCommanderRunnable != null) {
	        kmpCommanderRunnable.stopMotion();
	    }
	    if (lbr_commander != null) {
	        lbr_commander.stopMotion();
	    }
	    
	    // 4. Interrupt all threads
	    interruptThread(kmp_commander, "KMP commander");
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
	    
	    closeNodeSafely(kmpCommanderRunnable, "KMP commander");
	    closeNodeSafely(lbr_commander, "LBR commander");
	    closeNodeSafely(kmp_status_reader, "KMP status");
	    closeNodeSafely(kmp_sensor_reader, "KMP sensor");
	    closeNodeSafely(lbr_status_reader, "LBR status");
	    closeNodeSafely(lbr_sensor_reader, "LBR sensor");
	    
	    // Close the shared TCP socket
	    if (tcpSocket != null) {
	        logger.info("Closing main TCP Socket");
	        tcpSocket.close();
	        tcpSocket = null;
	    }
	    
	    // 6. Wait for all threads to finish
	    waitForThread(kmp_commander, "KMP commander", 2000);
	    waitForThread(lbr_commander_thread, "LBR commander", 2000);
	    waitForThread(kmp_status_thread, "KMP status", 1000);
	    waitForThread(lbr_status_thread, "LBR status", 1000);
	    waitForThread(kmp_sensor_thread, "KMP sensor", 1000);
	    waitForThread(lbr_sensor_thread, "LBR sensor", 1000);
	    
	    logger.info("Application terminated");
	}

	private void interruptThread(Thread thread, String name) {
	    if (thread != null && thread.isAlive()) {
	        logger.info("Interrupting " + name + " thread");
	        thread.interrupt();
	    }
	}

	private void waitForThread(Thread thread, String name, long timeoutMs) {
	    if (thread != null && thread.isAlive()) {
	        try {
	            logger.info("Waiting for thread to finish: " + name);
	            thread.join(timeoutMs);
	            if (thread.isAlive()) {
	                logger.warn("WARNING: " + name + " thread did not terminate within timeout");
	                forceTerminate(thread, name);
	            }
	        } catch (InterruptedException e) {
	            logger.warn("Interrupted while waiting for " + name + " thread");
	        }
	    }
	}

	private void closeNodeSafely(Node node, String name) {
	    if (node != null) {
	        try {
	            logger.info("Attempting to close node: " + name);
	            node.close();
	            logger.info("Successfully closed node: " + name);
	        } catch (Exception e) {
	            logger.error("Error closing " + name + ": " + e.getMessage(), e);
	        }
	    }
	}
	
	private void forceTerminate(Thread thread, String name) {
	    if (thread != null && thread.isAlive()) {
	        logger.warn("WARNING: " + name + " thread still alive after timeout. Attempting force termination...");
	        thread.interrupt();
	        try {
	            thread.join(1000);
	            if (thread.isAlive()) {
	                logger.error("SEVERE: " + name + " thread cannot be terminated gracefully");
	            }
	        } catch (InterruptedException e) {
	            logger.warn("Interrupted while waiting final time for " + name + " thread");
	        }
	    }
	}
	
	private void cleanupDataController(DataController controller, String name) {
	    if (controller != null) {
	        try {
	            controller.close(); // Implement close() in DataController if needed
	            logger.info("Closed " + name + " DataController");
	        } catch (Exception e) {
	            logger.error("Error closing " + name + " DataController: " + e.getMessage());
	        }
	    }
	}

	@Override
	public void run() {
	    // 1) Clear shutdown guard and restart grace-period clock
	    isShuttingDown = false;                // Clear KUKA shutdown guard
	    shutdownRequested = false;              // Clear any custom shutdown flag
	    AppRunning = true;                     // Ensure main loop will run
	    startupTime = System.currentTimeMillis();  // Fresh grace-period start
	    try {
	        resumeFunction = getTaskFunction(IAutomaticResumeFunction.class);
	        setAutomaticallyResumable(true);
	    } catch (Exception e) {
	        logger.error("Error getting resume function: " + e.getMessage());
	    }

	    logger.info("Running app!");
	    
	    // Start all connected nodes
	    kmpCommanderRunnable.setPriority(Thread.MAX_PRIORITY);
	    lbr_commander.setPriority(Thread.MAX_PRIORITY);
	    
	    if (kmpCommanderRunnable != null && kmpCommanderRunnable.isSocketConnected()) {
	        kmp_commander = new Thread(kmpCommanderRunnable, "KMP-Commander");
	        kmp_commander.setDaemon(true);
	        kmp_commander.start();
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
	    
	    // Check if the commanders are still alive periodically
	    long lastCommanderCheckTime = System.currentTimeMillis();
	    
	    // Initialize and schedule executorService for periodic tasks
	    if (executorService == null || executorService.isShutdown()) {
	        executorService = java.util.concurrent.Executors.newSingleThreadScheduledExecutor();
	        executorService.scheduleAtFixedRate(new Runnable() {
	            public void run() {
	                getLogger().info("[executorService] Periodic health check at " + new java.util.Date());
	                // Add any periodic logic here
	            }
	        }, 0, 10, java.util.concurrent.TimeUnit.SECONDS);
	    }

	    while (AppRunning) {
	        // Check external shutdown flag (for future use)
	        if (shutdownRequested) {
	            logger.info("External shutdownRequested flag detected, initiating shutdown");
	            isShuttingDown = true;
	        }
	        // 1. Monitor if the commander threads are still alive
	        long currentTime = System.currentTimeMillis();
	        if (currentTime - lastCommanderCheckTime > 5000) { // Check every 5 seconds
	            lastCommanderCheckTime = currentTime;
	            
	            // Check KMP commander
	            if (kmp_commander != null && !kmp_commander.isAlive() && !isShuttingDown) {
	                logger.warn("WARNING: KMP commander thread died unexpectedly, attempting restart");
	                try {
	                    kmpCommanderRunnable = new KMP_commander(KMP_command_port, kmp, TCPConnection);
	                    kmpCommanderRunnable.setPriority(Thread.MAX_PRIORITY);
	                    kmpCommanderRunnable.start();
	                    kmp_commander = kmpCommanderRunnable;
	                    logger.info("KMP commander thread restarted");
	                } catch (Exception e) {
	                    logger.error("Failed to restart KMP commander: " + e.getMessage(), e);
	                }
	            }
	            
	            // Check LBR commander
	            if (lbr_commander_thread != null && !lbr_commander_thread.isAlive() && !isShuttingDown) {
	                logger.warn("WARNING: LBR commander thread died unexpectedly, attempting restart");
	                try {
	                    lbr_commander = new LBR_commander(LBR_command_port, lbr, TCPConnection, getApplicationData().getFrame("/DrivePos"));
	                    lbr_commander.setPriority(Thread.MAX_PRIORITY);
	                    lbr_commander.start();
	                    lbr_commander_thread = lbr_commander;
	                    logger.info("LBR commander thread restarted");
	                } catch (Exception e) {
	                    logger.error("Failed to restart LBR commander: " + e.getMessage(), e);
	                }
	            }
	        }
	        
	        // 2. Check shutdown guard via isShuttingDown and grace period
	        if (isShuttingDown) {
	            long elapsed = System.currentTimeMillis() - startupTime;
	            if (elapsed > STARTUP_GRACE_PERIOD_MS) {
	                logger.info("Shutdown requested - exiting run loop after grace period");
	                AppRunning = false;
	            } else {
	                logger.info("Ignoring shutdown request during startup grace period (" + elapsed + "ms)");
	            }
	        }
	        
	        // 4. Check for operation mode changes (with reduced logging)
	        String currentMode = lbr.getOperationMode().toString();
	        if (!currentMode.equals(lastMode)) {
	            logger.info("Mode changed from " + lastMode + " to " + currentMode);
	            
	            // If we exit AUTO mode, this is a critical change requiring shutdown
	            if (lastMode.equals("AUTO") && !currentMode.equals("AUTO")) {
	                logger.warn("Exited AUTO mode - initiating shutdown");
	                AppRunning = false;
	                break;
	            }
	            lastMode = currentMode;
	        } else if (currentTime - lastLogTime > 60000) {  // Only log every minute instead of every 5 seconds
	            logger.info("Current operating mode: " + currentMode);
	            lastLogTime = currentTime;
	        }
	        
	        // 5. Handle state changes (thread priorities)  
	        if (getStateChange()) {
	            logger.info("State change.");
	            setStateChange(false);
	            
	            if (!(threadingPriority.equals(PRIORITY_LBR)) && lbr_commander != null && !lbr_commander.getisPathFinished()) {
	                threadingPriority = PRIORITY_LBR;
	                if (kmp_status_reader != null) {
	                    kmp_status_reader.setPriority(Thread.MIN_PRIORITY);
	                }
	                
	                logger.info("Threading priority: " + threadingPriority);
	                
	                if (lbr_sensor_reader != null) {
	                    lbr_sensor_reader.setPriority(Thread.MAX_PRIORITY);
	                }
	                if (lbr_status_reader != null) {
	                    lbr_status_reader.setPriority(Thread.MAX_PRIORITY);
	                }
	            } else if (!(threadingPriority.equals(PRIORITY_KMP)) && lbr_commander != null && lbr_commander.getisPathFinished()) {
	                threadingPriority = PRIORITY_KMP;
	                if (lbr_sensor_reader != null) {
	                    lbr_sensor_reader.setPriority(Thread.MIN_PRIORITY);
	                }
	                if (lbr_status_reader != null) {
	                    lbr_status_reader.setPriority(Thread.MIN_PRIORITY);
	                }
	                
	                logger.info("Threading priority: " + threadingPriority);
	                if (kmp_status_reader != null) {
	                    kmp_status_reader.setPriority(Thread.MAX_PRIORITY);
	                }
	            }
	        }
	        
	        // Check safety states - don't shut down for scanner warnings
	        boolean safetyOk = handleSafetyStates();
	        if (!safetyOk) {
	            logger.warn("Critical safety violation - initiating shutdown");
	            AppRunning = false;
	            break;
	        }

	        // Check if safety scanner warning fields have cleared
	        if (kmpCommanderRunnable != null && kmpCommanderRunnable.getEmergencyStop()) {
	            handleEmergencyStop();
	        }
	        
	        // Sleep to prevent high CPU usage
	        try {
	            Thread.sleep(THREAD_SLEEP_TIME);
	        } catch (InterruptedException e) {
	            Thread.currentThread().interrupt();
	            break;
	        }
	    }
	    
	    logger.info("Shutdown message received in main application");
	    logger.info("AppRunning = " + AppRunning + ", isShuttingDown = " + isShuttingDown);
	    shutdown_application();
	}
	
	private void setAutomaticallyResumable(boolean enable)
	{
	    if(enable)
	    {
	        if (resumeFunction != null) {
	            resumeFunction.enableApplicationResuming(getClass().getCanonicalName());
	        } else {
	            logger.warn("WARNING: resumeFunction is null, cannot enable automatic resuming");
	        }
	        return;
	    }
	    
	    if (resumeFunction != null) {
	        resumeFunction.disableApplicationResuming(getClass().getCanonicalName());
	    } else {
	        logger.warn("WARNING: resumeFunction is null, cannot disable automatic resuming");
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
	            logger.info("Safety scanner warning field detected");
	            
	            // Pause motion but don't shut down
	            if (kmpCommanderRunnable != null) {
	                kmpCommanderRunnable.stopMotion();
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
	        logger.error("Error checking safety state: " + e.getMessage());
	        // Don't trigger shutdown for errors in safety state checking
	        return true;
	    }
	}
	
	// Remove the monitorSocketConnections method - no longer needed with persistent socket

	private void handleEmergencyStop() {
	    if (kmpCommanderRunnable != null) {
	        kmpCommanderRunnable.setEmergencyStop(true);
	    }
	    if (lbr_commander != null) {
	        lbr_commander.setEmergencyStop(true); 
	    }
	}

	@Override
	public void dispose() {
	    // 1. Disable AutomaticResume background task
	    setAutomaticallyResumable(false);
	    // 2. Call super.dispose() immediately after
	    super.dispose();
	    // 3. Existing cleanup
	    for (Thread t : Thread.getAllStackTraces().keySet()) {
	        getLogger().info("💀 Still alive on shutdown: " + t.getName());
	    }
	    if (kmp_commander != null && kmp_commander.isAlive()) {
	        kmp_commander.interrupt();
	        try {
	            kmp_commander.join(10000);
	        } catch (InterruptedException e) {
	            Thread.currentThread().interrupt();
	        }
	    }
	    if (executorService != null) {
	        executorService.shutdownNow();
	        try {
	            executorService.awaitTermination(5, java.util.concurrent.TimeUnit.SECONDS);
	        } catch (InterruptedException e) {
	            Thread.currentThread().interrupt();
	        }
	    }
	    // 4. Hand control back to Sunrise’s framework
	    IApplicationControl ctrl = getApplicationControl();
	    if (ctrl != null) {
	        getLogger().info("🛑 Application shutdown via PositionAndGMSReferencing pattern");
	        ctrl.halt();
	    } else {
	        getLogger().warn("⚠️ Cannot get ApplicationControl, shutdown may not be clean");
	    }
	}

	public void triggerCleanRestart() {
	    getLogger().warn("⚠️ ApplicationControl.restartApplication() is not available in this API version. Manual restart required.");
	}

	// Stub for application ID (replace with actual logic if needed)
	public String getApplicationId() {
	    return "com.botman.kmriiwa.sunrise";
	}

	public static void main(String[] args) {
	    final KMRiiwaSunriseApplication app = new KMRiiwaSunriseApplication();
	    // Register JVM shutdown hook to cleanly shutdown KMP commander
	    Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
	        public void run() {
	            if (app.kmpCommanderRunnable != null) {
	                app.kmpCommanderRunnable.shutdown();
	            }
	        }
	    }));
	    app.runApplication();
	}
	
}
