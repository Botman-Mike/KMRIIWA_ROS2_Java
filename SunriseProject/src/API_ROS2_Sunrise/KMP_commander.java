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

import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class KMP_commander extends Node {

	// Logger
	private static final Logger logger = LoggerFactory.getLogger(KMP_commander.class);

	// Robot
	KmpOmniMove kmp;
	
	// Motion variables: KMP
	ICommandContainer KMP_currentMotion;
	double[] velocities = {0.0,0.0,0.0};
	KMPjogger kmp_jogger;
	long jogging_period  = 1L;
	
	// Implemented node classes
	LBR_sensor_reader lbr_sensor_reader;
	LBR_status_reader lbr_status_reader;
	KMP_sensor_reader kmp_sensor_reader;
	KMP_status_reader kmp_status_reader;

	// Added:
	// Startup
	boolean startup = true;
	// Removed redundant isNodeRunning field

	public KMP_commander(int port, KmpOmniMove kmp, String ConnectionType) {
		super(port, ConnectionType, "KMP_commander");
		this.kmp = kmp;
		this.kmp_jogger = new KMPjogger(kmp);
		
		// Ensure socket is initialized
		if (!isSocketConnected()) {
			createSocket();
			if (logger != null) {
				logger.info("KMP_commander: Initializing socket connection");
			}
			// Give socket time to establish
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
			}
		}
	}

	@Override
	public void run() {
	    boolean pauseAlerted = false;
	    if (startup) {
	        Thread emergencyStopThread = new MonitorEmergencyStopThread();
	        emergencyStopThread.start();
	        startup = false;
	    }
	    
	    // Added: Track startup time
	    long nodeStartTime = System.currentTimeMillis();
	    boolean isInStartupGracePeriod = true;
	    int messageCount = 0;
	    
	    while (isNodeRunning()) {
	        if (Node.getShutdown()) break;
	        if (Node.isPaused()) {
	            if (!pauseAlerted) {
	                logger.info("=== SYSTEM PAUSED: All logging is now suppressed until resume ===");
	                pauseAlerted = true;
	            }
	            try { Thread.sleep(100); } catch (InterruptedException e) { break; }
	            continue;
	        } else {
	            pauseAlerted = false;
	        }
	        
	        try {
	            // Check if startup grace period has ended
	            long currentTime = System.currentTimeMillis();
	            if (isInStartupGracePeriod && currentTime - nodeStartTime > STARTUP_GRACE_PERIOD_MS) {
	                isInStartupGracePeriod = false;
	                logger.info("KMP_commander: Startup grace period ended");
	            }
	            
	            String Commandstr = this.socket.receive_message();
	            if (Commandstr == null || Commandstr.trim().isEmpty()) {
	                continue;
	            }
	            
	            messageCount++;
	            
	            // During startup grace period, log ALL messages to help diagnose issues
	            if (isInStartupGracePeriod) {
	                logger.info("KMP_commander: [STARTUP] Message #" + messageCount + ": " + Commandstr);
	                
	                // Special handling for shutdown commands during startup
	                if (Commandstr.toLowerCase().contains("shutdown")) {
	                    logger.warn("KMP_commander: Ignoring shutdown command during startup grace period");
	                    continue; // Skip processing this command
	                }
	            } else {
	                // Normal logging once startup period is over
	                logger.debug("KMP_commander: Received command: " + Commandstr);
	            }
	            
	            String[] splt = Commandstr.split(" ");
	            if (!getShutdown() && !closed) {
	                if (splt.length > 0 && splt[0].equals("shutdown")) {
	                    // Don't process shutdown during startup grace period
	                    if (isInStartupGracePeriod) {
	                        logger.warn("KMP_commander: Ignoring shutdown command during startup grace period");
	                        continue;
	                    }
	                    
	                    logger.info("KMP_commander: Received explicit shutdown command");
	                    setShutdown(true);
	                    break;
	                }
	                
	                // Process velocity and position commands normally
	                if (splt.length > 0 && splt[0].equals("setTwist") && !getEmergencyStop()) {
	                    logger.debug("KMP_commander: Processing setTwist command");
	                    receivedRealCommand = true;
	                    setNewVelocity(Commandstr);
	                }
	                if (splt.length > 0 && splt[0].equals("setPose") && !getEmergencyStop()) {
	                    logger.debug("KMP_commander: Processing setPose command");
	                    receivedRealCommand = true;
	                    setNewPose(Commandstr);
	                }
	            } else {
	                logger.debug("KMP_commander: Command ignored - Shutdown: " + getShutdown() + ", Closed: " + closed);
	            }
	        } catch (Exception e) {
	            logger.error("KMP_commander: Error processing command: " + e.getMessage());
	            e.printStackTrace();
	        }
	    }
	    logger.info("KMPcommander no longer running");
	}
	
	public class MonitorEmergencyStopThread extends Thread {
	    private volatile boolean running = true;
	    private boolean previousEmergencyState = false;
	    
	    public void stopMonitoring() {
	        running = false;
	        this.interrupt(); // Add interruption when stopping
	    }
	    
	    public void run() {
	        while(isNodeRunning() && running && !Thread.currentThread().isInterrupted() && !getShutdown()) {
	            boolean currentEmergencyState = getEmergencyStop();
	            
	            // Check for emergency condition
	            if (currentEmergencyState && getisKMPMoving()) {
	                setisKMPMoving(false);
	                kmp_jogger.killJoggingExecution(getisKMPMoving());
	                logger.info("KMP_commander: Motion stopped due to safety event");
	                
	                // Don't trigger full shutdown for recoverable safety events
	                if (isRecoverableSafetyEvent()) {
	                    logger.info("KMP_commander: Recoverable safety event detected - waiting for conditions to clear");
	                } else if(!(KMP_currentMotion==null)) {
	                    KMP_currentMotion.cancel();
	                }
	            } 
	            
	            // Check for recovery from emergency condition
	            if (previousEmergencyState && !currentEmergencyState) {
	                logger.info("KMP_commander: Safety condition cleared, robot can move again");
	            }
	            
	            // Store state for next comparison
	            previousEmergencyState = currentEmergencyState;
	            
	            try {
	                Thread.sleep(50); // Reduced from 100ms to match LBR version
	            } catch (InterruptedException e) {
	                logger.info("KMP_commander: MonitorEmergencyStopThread interrupted, exiting thread");
	                Thread.currentThread().interrupt(); // Restore interrupt status
	                break;
	            }
	        }
	        logger.info("KMP_commander: MonitorEmergencyStopThread terminated normally");
	    }
	    
	    private boolean isRecoverableSafetyEvent() {
	        try {
	            if (kmp != null) {
	                String safetyStateStr = kmp.getSafetyState().toString();
	                return safetyStateStr.contains("WARNING_FIELD") || 
	                       safetyStateStr.contains("PROTECTIVE_FIELD");
	            }
	        } catch (Exception e) {
	            logger.error("Error checking safety state: " + e.getMessage());
	        }
	        return false;
	    }
	}
	
	public void setNewPose(String data){
		String []lineSplt = data.split(" ");
		if (lineSplt.length==4){
			
			double pose_x = Double.parseDouble(lineSplt[1]);
			double pose_y = Double.parseDouble(lineSplt[2]);
			double pose_theta = Double.parseDouble(lineSplt[3]);
			
			MobilePlatformRelativeMotion MRM = new MobilePlatformRelativeMotion(pose_x, pose_y, pose_theta);
			
			MRM.setVelocity(300, 10);
			MRM.setTimeout(100);
			MRM.setAcceleration(10, 10);

			if(kmp.isReadyToMove()) {
				logger.debug("moving");
				this.KMP_currentMotion =  kmp.moveAsync(MRM);
			}
			else {
				logger.info("Kmp is not ready to move!");
			}

		}else{
			logger.info("Unacceptable Mobile Platform Relative Velocity command!");
		}
	}

	
	public void setNewVelocity(String vel){
	    String []lineSplt = vel.split(" ");
	    logger.info("KMP_commander: Received velocity command: " + vel);  // Changed from debug to info for better visibility

	    if(lineSplt.length==4){
	        this.velocities[0] = Double.parseDouble(lineSplt[1]); // x
	        this.velocities[1] = Double.parseDouble(lineSplt[2]); // y
	        this.velocities[2] = Double.parseDouble(lineSplt[3]); // theta
	        logger.info("KMP_commander: Parsed velocities - x: " + velocities[0] + 
	                            ", y: " + velocities[1] + ", theta: " + velocities[2]);
	        
	        if(velocities[0] == 0 && velocities[1] == 0 && velocities[2] == 0) {
	            logger.info("KMP_commander: Zero velocity command received");
	            if(getisKMPMoving()) {
	                logger.info("KMP_commander: Robot is currently moving, stopping");
	                setisKMPMoving(false);
	                this.kmp_jogger.killJoggingExecution(true);
	                logger.info("KMP_commander: Motion stopped successfully without shutdown");
	            } else {
	                logger.info("KMP_commander: Robot already stopped");
	            }
	        } else {
	            logger.info("KMP_commander: Non-zero velocity command received");
	            logger.info("KMP_commander: Current states - IsMoving: " + getisKMPMoving() + 
	                              ", EmergencyStop: " + getEmergencyStop() + 
	                              ", ReadyToMove: " + kmp.isReadyToMove());
	            
	            if(getisKMPMoving() && !getEmergencyStop()) {
	                logger.info("KMP_commander: Robot already moving, updating velocities");
	                this.kmp_jogger.updateVelocities(this.velocities);
	            }
	            else if(!getisKMPMoving() && !getEmergencyStop() && kmp.isReadyToMove()) {
	                logger.info("KMP_commander: Starting robot movement");
	                setisKMPMoving(true);
	                
	                this.kmp_jogger.updateVelocities(this.velocities);
	                this.kmp_jogger.startJoggingExecution();
	            } else {
	                logger.info("KMP_commander: Cannot jog robot - ReadyToMove: " + kmp.isReadyToMove() + 
	                                  ", EmergencyStop: " + getEmergencyStop() + 
	                                  ", SafetyState: " + kmp.getSafetyState() + 
	                                  ", CalculatedReadyToMove: " + KmpOmniMove.calculateReadyToMove(kmp.getSafetyState()));
	                
	                // Check if application is in MOTIONPAUSED state in a safer way
	                try {
	                    // Try to get application state indirectly through the controller or other means
	                    if (Node.isPaused()) {
	                        logger.warn("KMP_commander: Application is paused - this will prevent movement");
	                        logger.warn("KMP_commander: To resume motion, resume the application via SmartPad");
	                    }
	                } catch (Exception e) {
	                    logger.warn("KMP_commander: Could not determine application state: " + e.getMessage());
	                }
	            }
	        }
	    } else {
	        logger.info("KMP_commander: Invalid velocity command format, expected 4 parts, got " + lineSplt.length);
	    }
	}
	
	public class MonitorKMPCommandConnectionsThread extends Thread {
		int timeout = 3000;
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				if (Thread.currentThread().isInterrupted()) {
					logger.info("KMP command connection monitor thread interrupted, exiting");
					break;
				}
				
				if (Node.getShutdown()) break;
				if (Node.isPaused()) {
				    try { Thread.sleep(100); } catch (InterruptedException e) {
						logger.info("KMP command connection monitor thread interrupted during pause wait");
						Thread.currentThread().interrupt();
						break;
					}
				    continue;
				}
				if(getisLBRConnected()) {
					timeout = 5000;
				}
				createSocket();
				if (isSocketConnected()){
					setisKMPConnected(true);
					break;
				}
				try {
					Thread.sleep(timeout);
				} catch (InterruptedException e) {
					logger.info("KMP command connection monitor thread interrupted during sleep");
					Thread.currentThread().interrupt(); // Restore interrupt status
					break;
				}
			}
			if(!closed){
				logger.info("Connection with KMP Command Node OK!");
				start(); // Use proper thread start
			}	
		}
	}

	public void stopMotion() {
	    if (kmp_jogger != null) {
	        try {
	            kmp_jogger.killJoggingExecution(getisKMPMoving());
	            setisKMPMoving(false);
	            logger.info("KMP_commander: Motion stopped.");
	        } catch (Exception e) {
	            logger.error("KMP_commander: Error stopping motion: " + e.getMessage());
	        }
	    } else {
	        logger.info("KMP_commander: No jogging instance found; nothing to stop.");
	    }
	}

	@Override
	public void close() {
	    closed = true;
	    
	    // First stop any ongoing motion
	    try {
	        if (kmp_jogger != null) {
	            kmp_jogger.killJoggingExecution(false); // Pass false to avoid recursive checks
	            logger.info("KMPJogger ended successfully");
	        }
	    } catch(Exception e) {
	        logger.error("Could not kill jogging execution: " + e.getMessage());
	    }

	    // Then close socket
	    try {
	        if (socket != null) {
	            socket.close();
	            logger.info("KMP commander TCP connection closing");
	        }
	    } catch(Exception e) {
	        logger.error("Could not close KMP commander connection: " + e);
	    }

	    // Set flags to ensure threads terminate
	    setShutdown(true);
	    isNodeRunning = false;
	    
	    // Interrupt any running threads
	    if (Thread.currentThread() != this) {
	        this.interrupt();
	    }

	    logger.info("KMP commander closed!");
	}
	
}