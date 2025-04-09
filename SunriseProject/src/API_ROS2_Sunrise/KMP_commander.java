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


import API_ROS2_Sunrise.KMPjogger;

import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;


public class KMP_commander extends Node{


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

	public KMP_commander(int port, KmpOmniMove robot, String ConnectionType) {
		super(port,ConnectionType, "KMP commander");
		this.kmp = robot;
		this.kmp_jogger = new KMPjogger((ICartesianJoggingSupport)kmp, jogging_period);
		
		
		if (!(isSocketConnected())) {
			Thread monitorKMPCommandConnections = new MonitorKMPCommandConnectionsThread();
			monitorKMPCommandConnections.start();
			}else {
				setisKMPConnected(true);
		}
	}

	@Override
	public void run() {
	    if (startup) {
	        Thread emergencyStopThread = new MonitorEmergencyStopThread();
	        emergencyStopThread.start();
	        startup = false;
	    }
	    
	    while (isNodeRunning()) {
	        try {
	            // Receive the command
	            String Commandstr = this.socket.receive_message();
	            if (Commandstr == null || Commandstr.trim().isEmpty()) {
	                // If no command is received, skip this iteration.
	                continue;
	            }
	            
	            LogUtil.logInfo("KMP_commander: Received command: " + Commandstr);
	            String[] splt = Commandstr.split(" ");
	            
	            if (!getShutdown() && !closed) {
	                if (splt.length > 0 && splt[0].equals("shutdown")) {
	                    LogUtil.logInfo("KMP_commander: Received explicit shutdown command");
	                    setShutdown(true);
	                    break;
	                }
	                if (splt.length > 0 && splt[0].equals("setTwist") && !getEmergencyStop()) {
	                    LogUtil.logInfo("KMP_commander: Processing setTwist command");
	                    setNewVelocity(Commandstr);
	                    // Don't set shutdown flag for normal commands
	                }
	                if (splt.length > 0 && splt[0].equals("setPose") && !getEmergencyStop()) {
	                    LogUtil.logInfo("KMP_commander: Processing setPose command");
	                    setNewPose(Commandstr);
	                    // Don't set shutdown flag for normal commands
	                }
	            } else {
	                LogUtil.logInfo("KMP_commander: Command ignored - Shutdown: " + getShutdown() + ", Closed: " + closed);
	            }
	        } catch (Exception e) {
	            LogUtil.logError("KMP_commander: Error processing command: " + e.getMessage());
	            e.printStackTrace();
	            // Only set shutdown on unrecoverable errors, not for normal operation
	            // setShutdown(true); // <-- Remove/comment this if it exists
	        }
	    }
	    LogUtil.logInfo("KMPcommander no longer running");
	}
	
	public class MonitorEmergencyStopThread extends Thread {
	    private volatile boolean running = true;
	    private boolean previousEmergencyState = false;
	    
	    public void stopMonitoring() {
	        running = false;
	    }
	    
	    public void run() {
	        while(isNodeRunning() && running) {
	            boolean currentEmergencyState = getEmergencyStop();
	            
	            // Check for emergency condition
	            if (currentEmergencyState && getisKMPMoving()) {
	                setisKMPMoving(false);
	                kmp_jogger.killJoggingExecution(getisKMPMoving());
	                LogUtil.logInfo("KMP_commander: Motion stopped due to safety event");
	                
	                // Don't trigger full shutdown for recoverable safety events
	                if (isRecoverableSafetyEvent()) {
	                    LogUtil.logInfo("KMP_commander: Recoverable safety event detected - waiting for conditions to clear");
	                } else if(!(KMP_currentMotion==null)) {
	                    KMP_currentMotion.cancel();
	                }
	            } 
	            
	            // Check for recovery from emergency condition
	            if (previousEmergencyState && !currentEmergencyState) {
	                LogUtil.logInfo("KMP_commander: Safety condition cleared, robot can move again");
	                // The emergency stop flag is now false again, so commands will work
	            }
	            
	            // Store state for next comparison
	            previousEmergencyState = currentEmergencyState;
	            
	            try {
	                Thread.sleep(100); // Check emergency state every 100ms
	            } catch (InterruptedException e) {
	                break;
	            }
	        }
	    }
	    
	    private boolean isRecoverableSafetyEvent() {
	        try {
	            if (kmp != null) {
	                String safetyStateStr = kmp.getSafetyState().toString();
	                return safetyStateStr.contains("WARNING_FIELD") || 
	                       safetyStateStr.contains("PROTECTIVE_FIELD");
	            }
	        } catch (Exception e) {
	            LogUtil.logError("Error checking safety state: " + e.getMessage());
	        }
	        return false;
	    }
	}
	
	public void setNewPose(String data){
		String []lineSplt = data.split(" ");
		if (lineSplt.length==4){
			
			// Declare state change
			API_ROS2_Sunrise.KMRiiwaSunriseApplication.StateChange = true;
			
			double pose_x = Double.parseDouble(lineSplt[1]);
			double pose_y = Double.parseDouble(lineSplt[2]);
			double pose_theta = Double.parseDouble(lineSplt[3]);
			
			MobilePlatformRelativeMotion MRM = new MobilePlatformRelativeMotion(pose_x, pose_y, pose_theta);
			
			MRM.setVelocity(300, 10);
			MRM.setTimeout(100);
			MRM.setAcceleration(10, 10);

			if(kmp.isReadyToMove()) {
				LogUtil.logInfo("moving");
				this.KMP_currentMotion =  kmp.moveAsync(MRM);
			}
			else {
				LogUtil.logInfo("Kmp is not ready to move!");
			}

		}else{
			LogUtil.logInfo("Unacceptable Mobile Platform Relative Velocity command!");

		}
	}

	
	public void setNewVelocity(String vel){
	    String []lineSplt = vel.split(" ");
	    LogUtil.logInfo("KMP_commander: Received velocity command: " + vel);

	    if(lineSplt.length==4){
	        this.velocities[0] = Double.parseDouble(lineSplt[1]); // x
	        this.velocities[1] = Double.parseDouble(lineSplt[2]); // y
	        this.velocities[2] = Double.parseDouble(lineSplt[3]); // theta
	        LogUtil.logInfo("KMP_commander: Parsed velocities - x: " + velocities[0] + 
	                          ", y: " + velocities[1] + ", theta: " + velocities[2]);
	        
	        if(velocities[0] == 0 && velocities[1] == 0 && velocities[2] == 0) {
	            LogUtil.logInfo("KMP_commander: Zero velocity command received");
	            if(getisKMPMoving()) {
	                LogUtil.logInfo("KMP_commander: Robot is currently moving, stopping");
	                setisKMPMoving(false);
	                this.kmp_jogger.killJoggingExecution(true);
	                // Do NOT set shutdown flag here - we only want to stop motion
	                LogUtil.logInfo("KMP_commander: Motion stopped successfully without shutdown");
	            } else {
	                LogUtil.logInfo("KMP_commander: Robot already stopped");
	            }
	        } else {
	            LogUtil.logInfo("KMP_commander: Non-zero velocity command received");
	            LogUtil.logInfo("KMP_commander: Current states - IsMoving: " + getisKMPMoving() + 
	                              ", EmergencyStop: " + getEmergencyStop() + 
	                              ", ReadyToMove: " + kmp.isReadyToMove());
	            
	            if(getisKMPMoving() && !getEmergencyStop()) {
	                LogUtil.logInfo("KMP_commander: Robot already moving, updating velocities");
	                this.kmp_jogger.updateVelocities(this.velocities);
	            }
	            else if(!getisKMPMoving() && !getEmergencyStop() && kmp.isReadyToMove()) {
	                LogUtil.logInfo("KMP_commander: Starting robot movement");
	                setisKMPMoving(true);
	                
	                // Declare state change.
	                API_ROS2_Sunrise.KMRiiwaSunriseApplication.StateChange = true;
	                
	                this.kmp_jogger.updateVelocities(this.velocities);
	                this.kmp_jogger.startJoggingExecution();
	            } else {
	                LogUtil.logInfo("KMP_commander: Cannot jog robot - ReadyToMove: " + kmp.isReadyToMove() + 
	                                  ", EmergencyStop: " + getEmergencyStop() + 
	                                  ", SafetyState: " + kmp.getSafetyState() + 
	                                  ", CalculatedReadyToMove: " + KmpOmniMove.calculateReadyToMove(kmp.getSafetyState()));
	            }
	        }
	    } else {
	        LogUtil.logInfo("KMP_commander: Invalid velocity command format, expected 4 parts, got " + lineSplt.length);
	    }
	}
	
	public class MonitorKMPCommandConnectionsThread extends Thread {
		int timeout = 3000;
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
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
					LogUtil.logInfo(node_name + " connection thread could not sleep");
				}
			}
			if(!closed){
				LogUtil.logInfo("Connection with KMP Command Node OK!");
				runmainthread();
				}	
		}
	}

	public void stopMotion() {
	    // Safely stop the jogging execution
	    if (kmp_jogger != null) {
	        kmp_jogger.killJoggingExecution(false);
	        setisKMPMoving(false);
	        LogUtil.logInfo("KMP_commander: Motion stopped.");
	    } else {
	        LogUtil.logInfo("KMP_commander: No jogging instance found; nothing to stop.");
	    }
	}

	@Override
	public void close() {
		closed = true;
		try{
			 this.kmp_jogger.killJoggingExecution(getisKMPMoving());
			 LogUtil.logInfo("KMPJogger ended successfully");
			 
		}catch(Exception e){
			LogUtil.logInfo("Could not kill jogging execution");
		}
		try{
			this.socket.close();
		}catch(Exception e){
				LogUtil.logInfo("Could not close KMP commander connection: " +e);
			}
		LogUtil.logInfo("KMP commander closed!");
 	}
	
}